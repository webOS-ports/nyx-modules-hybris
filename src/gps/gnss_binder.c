// Copyright (c) 2026 Herman van Hazendonk <github.com@herrie.org>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

/*
 * android.hardware.gnss IGnss client over libgbinder.
 *
 * Why binder and not libhybris: the legacy gps.h HAL reached through
 * hw_get_module() only exists pre-Treble. On the Halium bases LuneOS targets,
 * GNSS is a HIDL service on /dev/hwbinder, exactly as the lights HAL is (see
 * the note in led_controller.c). This module is therefore a HIDL client and
 * joins the ones LuneOS already runs - ofono-binder-plugin, bluebinder,
 * sensorfw, nfcd-binder-plugin.
 *
 * Interface versions: which IGnss version a device exposes is a property of the
 * GSI flashed to it rather than of the machine it was built from, so the choice
 * has to be made at runtime, never from cmake. This is the same reasoning that
 * governs the torch in nyx-modules/halium-arm64.cmake.
 *
 * We bind the @1.0 interface and use only @1.0 methods. hwservicemanager
 * registers a service under every descriptor in its inheritance chain, so a
 * device implementing @1.1 or @2.0 still answers to the @1.0 name and still
 * accepts @1.0 transactions - @1.1 and @2.0 append methods, they do not
 * renumber the ones they inherit. Sticking to @1.0 buys support for every
 * version from one code path. The trade is that @2.0-only additions
 * (setCallback_2_0's extra callbacks, GnssLocation's speed/bearing accuracy in
 * @2.0's ElapsedRealtime form) are not available; nothing nyx exposes needs
 * them today.
 *
 * The instance is looked up newest-first so the log names the version actually
 * in use, which is the first thing anyone debugging a device will want.
 */

#include "gnss_binder.h"

#include <string.h>

#include <gbinder.h>

#include <nyx/module/nyx_log.h>

#include "msgid.h"

#define GNSS_IFACE_1_0 "android.hardware.gnss@1.0::IGnss"
#define GNSS_CALLBACK_IFACE_1_0 "android.hardware.gnss@1.0::IGnssCallback"

#define AGNSS_IFACE_1_0 "android.hardware.gnss@1.0::IAGnss"
#define AGNSS_CALLBACK_IFACE_1_0 "android.hardware.gnss@1.0::IAGnssCallback"
#define AGNSS_RIL_IFACE_1_0 "android.hardware.gnss@1.0::IAGnssRil"
#define AGNSS_RIL_CALLBACK_IFACE_1_0 "android.hardware.gnss@1.0::IAGnssRilCallback"
#define GNSS_XTRA_IFACE_1_0 "android.hardware.gnss@1.0::IGnssXtra"
#define GNSS_XTRA_CALLBACK_IFACE_1_0 "android.hardware.gnss@1.0::IGnssXtraCallback"

/*
 * Transaction codes are the 1-based declaration order of the methods in
 * IGnss.hal / IGnssCallback.hal. Verified against
 * hardware/interfaces/gnss/1.0 at android-9.0.0_r61.
 */
enum gnss_tx {
	GNSS_TX_SET_CALLBACK = 1,
	GNSS_TX_START = 2,
	GNSS_TX_STOP = 3,
	GNSS_TX_CLEANUP = 4,
	GNSS_TX_INJECT_TIME = 5,
	GNSS_TX_INJECT_LOCATION = 6,
	GNSS_TX_DELETE_AIDING_DATA = 7,
	GNSS_TX_SET_POSITION_MODE = 8,
	GNSS_TX_GET_EXTENSION_AGNSS_RIL = 9,
	GNSS_TX_GET_EXTENSION_AGNSS = 11,
	GNSS_TX_GET_EXTENSION_XTRA = 15
};

enum agnss_tx {
	AGNSS_TX_SET_CALLBACK = 1,
	AGNSS_TX_DATA_CONN_CLOSED = 2,
	AGNSS_TX_DATA_CONN_FAILED = 3,
	AGNSS_TX_SET_SERVER = 4,
	AGNSS_TX_DATA_CONN_OPEN = 5
};

enum agnss_ril_tx {
	AGNSS_RIL_TX_SET_CALLBACK = 1,
	AGNSS_RIL_TX_SET_REF_LOCATION = 2,
	AGNSS_RIL_TX_SET_SET_ID = 3,
	AGNSS_RIL_TX_UPDATE_NETWORK_STATE = 4,
	AGNSS_RIL_TX_UPDATE_NETWORK_AVAILABILITY = 5
};

enum gnss_xtra_tx {
	GNSS_XTRA_TX_SET_CALLBACK = 1,
	GNSS_XTRA_TX_INJECT_XTRA_DATA = 2
};

enum agnss_callback_tx {
	/* Spelled with the double S in the 1.0 HAL; not a typo here. */
	AGNSS_CB_TX_STATUS_IPV4 = 1,
	AGNSS_CB_TX_STATUS_IPV6 = 2
};

enum agnss_ril_callback_tx {
	AGNSS_RIL_CB_TX_REQUEST_SET_ID = 1,
	AGNSS_RIL_CB_TX_REQUEST_REF_LOC = 2
};

enum gnss_xtra_callback_tx {
	GNSS_XTRA_CB_TX_DOWNLOAD_REQUEST = 1
};

enum gnss_callback_tx {
	GNSS_CB_TX_LOCATION = 1,
	GNSS_CB_TX_STATUS = 2,
	GNSS_CB_TX_SV_STATUS = 3,
	GNSS_CB_TX_NMEA = 4,
	GNSS_CB_TX_SET_CAPABILITIES = 5,
	GNSS_CB_TX_ACQUIRE_WAKELOCK = 6,
	GNSS_CB_TX_RELEASE_WAKELOCK = 7,
	GNSS_CB_TX_REQUEST_TIME = 8,
	GNSS_CB_TX_SET_SYSTEM_INFO = 9
};

/*
 * Wire layout of android.hardware.gnss@1.0::GnssLocation. HIDL lays structs out
 * with C++ rules, so a plain C struct with natural alignment matches: uint16 at
 * 0, doubles at 8/16/24, floats at 32..52, int64 at 56, 64 bytes total. The
 * static assert below keeps that honest.
 */
typedef struct {
	guint16 gnssLocationFlags;
	gdouble latitudeDegrees;
	gdouble longitudeDegrees;
	gdouble altitudeMeters;
	gfloat speedMetersPerSec;
	gfloat bearingDegrees;
	gfloat horizontalAccuracyMeters;
	gfloat verticalAccuracyMeters;
	gfloat speedAccuracyMetersPerSecond;
	gfloat bearingAccuracyDegrees;
	gint64 timestamp;
} GnssLocationHidl;

typedef struct {
	gint16 svid;
	guint8 constellation;
	gfloat cN0Dbhz;
	gfloat elevationDegrees;
	gfloat azimuthDegrees;
	gfloat carrierFrequencyHz;
	guint8 svFlags;
} GnssSvInfoHidl;

/* gnssSvList is a fixed-size array (GnssMax:SVS_COUNT), not a hidl_vec. */
typedef struct {
	guint32 numSvs;
	GnssSvInfoHidl gnssSvList[GNSS_BINDER_MAX_SVS];
} GnssSvStatusHidl;

/* android.hardware.gnss@1.0::IAGnssCallback.AGnssStatusIpV4 */
typedef struct {
	guint8 type;
	guint8 status;
	guint32 ipV4Addr;
} AGnssStatusIpV4Hidl;

/* android.hardware.gnss@1.0::IAGnssRil.AGnssRefLocationCellID */
typedef struct {
	guint8 type;
	guint16 mcc;
	guint16 mnc;
	guint16 lac;
	guint32 cid;
	guint16 tac;
	guint16 pcid;
} AGnssRefLocationCellIDHidl;

/* android.hardware.gnss@1.0::IAGnssRil.AGnssRefLocation */
typedef struct {
	guint8 type;
	AGnssRefLocationCellIDHidl cellID;
} AGnssRefLocationHidl;

G_STATIC_ASSERT(sizeof(AGnssStatusIpV4Hidl) == 8);
G_STATIC_ASSERT(sizeof(AGnssRefLocationCellIDHidl) == 16);
G_STATIC_ASSERT(sizeof(AGnssRefLocationHidl) == 20);
G_STATIC_ASSERT(sizeof(GnssLocationHidl) == 64);
G_STATIC_ASSERT(sizeof(GnssSvInfoHidl) == 24);
G_STATIC_ASSERT(sizeof(GnssSvStatusHidl) == 4 + (24 * GNSS_BINDER_MAX_SVS));

typedef struct {
	GBinderServiceManager *sm;
	GBinderRemoteObject *remote;
	GBinderClient *client;
	GBinderLocalObject *callback_object;

	/* Assistance extensions; any of these may legitimately stay NULL. */
	GBinderRemoteObject *agnss_remote;
	GBinderClient *agnss_client;
	GBinderLocalObject *agnss_callback_object;

	GBinderRemoteObject *ril_remote;
	GBinderClient *ril_client;
	GBinderLocalObject *ril_callback_object;

	GBinderRemoteObject *xtra_remote;
	GBinderClient *xtra_client;
	GBinderLocalObject *xtra_callback_object;

	gulong death_id;

	/*
	 * Fixed buffer rather than a g_strdup: providers_query() passes this
	 * pointer out through the nyx API, whose callers are entitled to assume
	 * it stays valid - nyx-modules' own GPS module only ever hands back
	 * string literals. Heap here would dangle the moment cleanup() ran.
	 */
	char provider_name[128];
	gboolean bound;

	gnss_binder_callbacks callbacks;
	void *user_data;
} gnss_binder_state;

static gnss_binder_state g_gnss;

/* Newest first, so the log reports the version the device really implements. */
static const char *const gnss_instances[] = {
	"android.hardware.gnss@2.1::IGnss/default",
	"android.hardware.gnss@2.0::IGnss/default",
	"android.hardware.gnss@1.1::IGnss/default",
	"android.hardware.gnss@1.0::IGnss/default"
};

static gboolean gnss_binder_transact_bool(guint32 code, GBinderLocalRequest *req)
{
	GBinderRemoteReply *reply;
	int status = -1;
	gboolean result = FALSE;

	if (!g_gnss.client) {
		gbinder_local_request_unref(req);
		return FALSE;
	}

	reply = gbinder_client_transact_sync_reply(g_gnss.client, code, req, &status);
	gbinder_local_request_unref(req);

	if (reply) {
		GBinderReader reader;
		gint32 hidl_status = -1;
		gboolean value = FALSE;

		gbinder_remote_reply_init_reader(reply, &reader);

		/*
		 * Every HIDL reply that generates a value is prefixed with the
		 * transport status word. Read and check it before the payload,
		 * or a failed call looks like a successful one returning
		 * garbage.
		 */
		if (gbinder_reader_read_int32(&reader, &hidl_status) &&
		    hidl_status == 0 &&
		    gbinder_reader_read_bool(&reader, &value)) {
			result = value;
		}

		gbinder_remote_reply_unref(reply);
	}

	if (status != GBINDER_STATUS_OK)
		nyx_error(MSGID_NYX_HYBRIS_GPS_TRANSACT_ERR, 0,
		          "GNSS transaction %u failed, binder status %d", code, status);

	return result;
}

static void gnss_binder_transact_oneway(guint32 code, GBinderLocalRequest *req)
{
	GBinderRemoteReply *reply;
	int status = -1;

	if (!g_gnss.client) {
		gbinder_local_request_unref(req);
		return;
	}

	/*
	 * cleanup() and deleteAidingData() generate no value, but they are not
	 * declared oneway either, so they are still ordinary synchronous
	 * transactions - just ones whose reply carries nothing but the status.
	 */
	reply = gbinder_client_transact_sync_reply(g_gnss.client, code, req, &status);
	gbinder_local_request_unref(req);

	if (reply)
		gbinder_remote_reply_unref(reply);

	if (status != GBINDER_STATUS_OK)
		nyx_error(MSGID_NYX_HYBRIS_GPS_TRANSACT_ERR, 0,
		          "GNSS transaction %u failed, binder status %d", code, status);
}

static void gnss_binder_convert_location(const GnssLocationHidl *in,
                                         gnss_binder_location *out)
{
	memset(out, 0, sizeof(*out));
	out->flags = in->gnssLocationFlags;
	out->latitude = in->latitudeDegrees;
	out->longitude = in->longitudeDegrees;
	out->altitude = in->altitudeMeters;
	out->speed = in->speedMetersPerSec;
	out->bearing = in->bearingDegrees;
	out->horizontal_accuracy = in->horizontalAccuracyMeters;
	out->vertical_accuracy = in->verticalAccuracyMeters;
	out->timestamp = in->timestamp;
}

static void gnss_binder_dispatch_location(GBinderReader *reader)
{
	const GnssLocationHidl *loc =
		gbinder_reader_read_hidl_struct(reader, GnssLocationHidl);
	gnss_binder_location out;

	if (!loc || !g_gnss.callbacks.location_cb)
		return;

	gnss_binder_convert_location(loc, &out);
	g_gnss.callbacks.location_cb(&out, g_gnss.user_data);
}

static void gnss_binder_dispatch_sv_status(GBinderReader *reader)
{
	const GnssSvStatusHidl *sv =
		gbinder_reader_read_hidl_struct(reader, GnssSvStatusHidl);
	gnss_binder_sv_status out;
	guint32 count;
	guint32 i;

	if (!sv || !g_gnss.callbacks.sv_status_cb)
		return;

	/* numSvs comes off the wire, so clamp it before indexing. */
	count = sv->numSvs;
	if (count > GNSS_BINDER_MAX_SVS)
		count = GNSS_BINDER_MAX_SVS;

	memset(&out, 0, sizeof(out));
	out.num_svs = count;

	for (i = 0; i < count; i++) {
		out.sv_list[i].svid = sv->gnssSvList[i].svid;
		out.sv_list[i].constellation = sv->gnssSvList[i].constellation;
		out.sv_list[i].c_n0_dbhz = sv->gnssSvList[i].cN0Dbhz;
		out.sv_list[i].elevation = sv->gnssSvList[i].elevationDegrees;
		out.sv_list[i].azimuth = sv->gnssSvList[i].azimuthDegrees;
		out.sv_list[i].flags = sv->gnssSvList[i].svFlags;
	}

	g_gnss.callbacks.sv_status_cb(&out, g_gnss.user_data);
}

static void gnss_binder_dispatch_nmea(GBinderReader *reader)
{
	guint64 timestamp = 0;
	const char *nmea;

	if (!gbinder_reader_read_uint64(reader, &timestamp))
		return;

	nmea = gbinder_reader_read_hidl_string_c(reader);

	if (nmea && g_gnss.callbacks.nmea_cb)
		g_gnss.callbacks.nmea_cb((int64_t) timestamp, nmea,
		                         (int) strlen(nmea), g_gnss.user_data);
}

static GBinderLocalReply *gnss_binder_callback_handler(GBinderLocalObject *obj,
                                                       GBinderRemoteRequest *req,
                                                       guint code, guint flags,
                                                       int *status, void *user_data)
{
	GBinderReader reader;
	const char *iface = gbinder_remote_request_interface(req);

	(void) obj;
	(void) flags;
	(void) user_data;

	*status = GBINDER_STATUS_OK;

	if (g_strcmp0(iface, GNSS_CALLBACK_IFACE_1_0)) {
		/*
		 * A @1.1 or @2.0 HAL may also call us on its own callback
		 * interface. We only registered the @1.0 one, so anything else
		 * is unexpected; log it rather than misparsing the payload
		 * against the wrong layout.
		 */
		nyx_debug("Ignoring GNSS callback on unexpected interface %s",
		          iface ? iface : "(none)");
		return NULL;
	}

	gbinder_remote_request_init_reader(req, &reader);

	switch (code) {
	case GNSS_CB_TX_LOCATION:
		gnss_binder_dispatch_location(&reader);
		break;
	case GNSS_CB_TX_STATUS: {
		gint32 value = 0;

		if (gbinder_reader_read_int32(&reader, &value) &&
		    g_gnss.callbacks.status_cb)
			g_gnss.callbacks.status_cb((uint16_t) value, g_gnss.user_data);
		break;
	}
	case GNSS_CB_TX_SV_STATUS:
		gnss_binder_dispatch_sv_status(&reader);
		break;
	case GNSS_CB_TX_NMEA:
		gnss_binder_dispatch_nmea(&reader);
		break;
	case GNSS_CB_TX_SET_CAPABILITIES: {
		gint32 value = 0;

		if (gbinder_reader_read_int32(&reader, &value) &&
		    g_gnss.callbacks.set_capabilities_cb)
			g_gnss.callbacks.set_capabilities_cb((uint32_t) value,
			                                     g_gnss.user_data);
		break;
	}
	case GNSS_CB_TX_ACQUIRE_WAKELOCK:
		if (g_gnss.callbacks.acquire_wakelock_cb)
			g_gnss.callbacks.acquire_wakelock_cb(g_gnss.user_data);
		break;
	case GNSS_CB_TX_RELEASE_WAKELOCK:
		if (g_gnss.callbacks.release_wakelock_cb)
			g_gnss.callbacks.release_wakelock_cb(g_gnss.user_data);
		break;
	case GNSS_CB_TX_REQUEST_TIME:
		if (g_gnss.callbacks.request_utc_time_cb)
			g_gnss.callbacks.request_utc_time_cb(g_gnss.user_data);
		break;
	case GNSS_CB_TX_SET_SYSTEM_INFO:
		/* GnssSystemInfo carries only yearOfHw; nyx has nowhere to put it. */
		break;
	default:
		nyx_debug("Unhandled GNSS callback transaction %u", code);
		break;
	}

	return NULL;
}

static void gnss_binder_death_handler(GBinderRemoteObject *obj, void *user_data)
{
	(void) obj;
	(void) user_data;

	/*
	 * The HAL died. Drop our side so a later start() reports failure
	 * cleanly instead of transacting on a dead object; recovery means a
	 * fresh nyx_gps_init, which is the location service's call to make.
	 */
	nyx_error(MSGID_NYX_HYBRIS_GPS_HAL_DIED, 0, "GNSS HAL died");
}

/*
 * The transact helpers above are hard-wired to the IGnss client; the extension
 * interfaces need the same shapes against a different client.
 */
static gboolean gnss_binder_client_transact_bool(GBinderClient *client,
                                                 guint32 code,
                                                 GBinderLocalRequest *req)
{
	GBinderRemoteReply *reply;
	int status = -1;
	gboolean result = FALSE;

	if (!client) {
		gbinder_local_request_unref(req);
		return FALSE;
	}

	reply = gbinder_client_transact_sync_reply(client, code, req, &status);
	gbinder_local_request_unref(req);

	if (reply) {
		GBinderReader reader;
		gint32 hidl_status = -1;
		gboolean value = FALSE;

		gbinder_remote_reply_init_reader(reply, &reader);

		if (gbinder_reader_read_int32(&reader, &hidl_status) &&
		    hidl_status == 0 &&
		    gbinder_reader_read_bool(&reader, &value)) {
			result = value;
		}

		gbinder_remote_reply_unref(reply);
	}

	if (status != GBINDER_STATUS_OK)
		nyx_error(MSGID_NYX_HYBRIS_GPS_TRANSACT_ERR, 0,
		          "GNSS extension transaction %u failed, binder status %d",
		          code, status);

	return result;
}

/*
 * setRefLocation and setCallback on the extensions generate no value, so their
 * replies carry only the transport status.
 */
static void gnss_binder_client_transact_void(GBinderClient *client,
                                             guint32 code,
                                             GBinderLocalRequest *req)
{
	GBinderRemoteReply *reply;
	int status = -1;

	if (!client) {
		gbinder_local_request_unref(req);
		return;
	}

	reply = gbinder_client_transact_sync_reply(client, code, req, &status);
	gbinder_local_request_unref(req);

	if (reply)
		gbinder_remote_reply_unref(reply);

	if (status != GBINDER_STATUS_OK)
		nyx_error(MSGID_NYX_HYBRIS_GPS_TRANSACT_ERR, 0,
		          "GNSS extension transaction %u failed, binder status %d",
		          code, status);
}

/*
 * Asks IGnss for one of its extension interfaces. A HAL that does not implement
 * the extension answers with a null binder rather than an error, so a NULL
 * result here is a normal outcome and not a failure to report.
 */
static GBinderRemoteObject *gnss_binder_get_extension(guint32 code,
                                                      const char *what)
{
	GBinderRemoteReply *reply;
	GBinderRemoteObject *obj = NULL;
	int status = -1;

	if (!g_gnss.client)
		return NULL;

	reply = gbinder_client_transact_sync_reply(g_gnss.client, code,
	                                           gbinder_client_new_request(g_gnss.client),
	                                           &status);

	if (reply) {
		GBinderReader reader;
		gint32 hidl_status = -1;

		gbinder_remote_reply_init_reader(reply, &reader);

		if (gbinder_reader_read_int32(&reader, &hidl_status) &&
		    hidl_status == 0) {
			GBinderRemoteObject *ext = gbinder_reader_read_object(&reader);

			if (ext)
				obj = ext;
		}

		gbinder_remote_reply_unref(reply);
	}

	if (!obj)
		nyx_info(MSGID_NYX_HYBRIS_GPS_NO_EXTENSION, 0,
		         "GNSS HAL provides no %s extension", what);

	return obj;
}

static GBinderLocalReply *gnss_binder_agnss_callback_handler(GBinderLocalObject *obj,
                                                             GBinderRemoteRequest *req,
                                                             guint code, guint flags,
                                                             int *status, void *user_data)
{
	GBinderReader reader;

	(void) obj;
	(void) flags;
	(void) user_data;

	*status = GBINDER_STATUS_OK;
	gbinder_remote_request_init_reader(req, &reader);

	if (code == AGNSS_CB_TX_STATUS_IPV4) {
		const AGnssStatusIpV4Hidl *st =
			gbinder_reader_read_hidl_struct(&reader, AGnssStatusIpV4Hidl);

		if (st && g_gnss.callbacks.agnss_status_cb)
			g_gnss.callbacks.agnss_status_cb(st->type, st->status,
			                                 st->ipV4Addr, g_gnss.user_data);
	} else if (code == AGNSS_CB_TX_STATUS_IPV6) {
		/*
		 * nyx_agps_status_t has an ipv6_addr field, but nothing downstream
		 * reads it and com.webos.service.location only ever acts on the
		 * status value, so report the status with a zero v4 address rather
		 * than decoding an address that would be discarded.
		 */
		const guint8 *st = gbinder_reader_read_hidl_struct1(&reader, 18);

		if (st && g_gnss.callbacks.agnss_status_cb)
			g_gnss.callbacks.agnss_status_cb(st[0], st[1], 0, g_gnss.user_data);
	}

	return NULL;
}

static GBinderLocalReply *gnss_binder_ril_callback_handler(GBinderLocalObject *obj,
                                                           GBinderRemoteRequest *req,
                                                           guint code, guint flags,
                                                           int *status, void *user_data)
{
	GBinderReader reader;

	(void) obj;
	(void) flags;
	(void) user_data;

	*status = GBINDER_STATUS_OK;
	gbinder_remote_request_init_reader(req, &reader);

	if (code == AGNSS_RIL_CB_TX_REQUEST_SET_ID) {
		gint32 flag = 0;

		if (gbinder_reader_read_int32(&reader, &flag) &&
		    g_gnss.callbacks.ril_request_set_id_cb)
			g_gnss.callbacks.ril_request_set_id_cb((uint32_t) flag,
			                                       g_gnss.user_data);
	} else if (code == AGNSS_RIL_CB_TX_REQUEST_REF_LOC) {
		if (g_gnss.callbacks.ril_request_ref_loc_cb)
			g_gnss.callbacks.ril_request_ref_loc_cb(g_gnss.user_data);
	}

	return NULL;
}

static GBinderLocalReply *gnss_binder_xtra_callback_handler(GBinderLocalObject *obj,
                                                            GBinderRemoteRequest *req,
                                                            guint code, guint flags,
                                                            int *status, void *user_data)
{
	(void) obj;
	(void) req;
	(void) flags;
	(void) user_data;

	*status = GBINDER_STATUS_OK;

	if (code == GNSS_XTRA_CB_TX_DOWNLOAD_REQUEST &&
	    g_gnss.callbacks.xtra_download_request_cb)
		g_gnss.callbacks.xtra_download_request_cb(g_gnss.user_data);

	return NULL;
}

/*
 * Brings up one extension: fetch the interface, register our callback object
 * with it, and keep the client only if both steps worked. Anything less is
 * torn down so the *_available() predicate stays truthful.
 */
static void gnss_binder_setup_extensions(void)
{
	/* A-GNSS (SUPL): server configuration and data-connection handshake. */
	g_gnss.agnss_remote = gnss_binder_get_extension(GNSS_TX_GET_EXTENSION_AGNSS,
	                                                "IAGnss");
	if (g_gnss.agnss_remote) {
		gbinder_remote_object_ref(g_gnss.agnss_remote);
		g_gnss.agnss_client = gbinder_client_new(g_gnss.agnss_remote,
		                                         AGNSS_IFACE_1_0);
		g_gnss.agnss_callback_object =
			gbinder_servicemanager_new_local_object(g_gnss.sm,
			                                        AGNSS_CALLBACK_IFACE_1_0,
			                                        gnss_binder_agnss_callback_handler,
			                                        NULL);

		if (g_gnss.agnss_client && g_gnss.agnss_callback_object) {
			GBinderLocalRequest *req =
				gbinder_client_new_request(g_gnss.agnss_client);

			gbinder_local_request_append_local_object(req,
			                                          g_gnss.agnss_callback_object);
			gnss_binder_client_transact_void(g_gnss.agnss_client,
			                                 AGNSS_TX_SET_CALLBACK, req);
		}
	}

	/* A-GNSS RIL: the side that actually benefits from a SIM. */
	g_gnss.ril_remote = gnss_binder_get_extension(GNSS_TX_GET_EXTENSION_AGNSS_RIL,
	                                              "IAGnssRil");
	if (g_gnss.ril_remote) {
		gbinder_remote_object_ref(g_gnss.ril_remote);
		g_gnss.ril_client = gbinder_client_new(g_gnss.ril_remote,
		                                       AGNSS_RIL_IFACE_1_0);
		g_gnss.ril_callback_object =
			gbinder_servicemanager_new_local_object(g_gnss.sm,
			                                        AGNSS_RIL_CALLBACK_IFACE_1_0,
			                                        gnss_binder_ril_callback_handler,
			                                        NULL);

		if (g_gnss.ril_client && g_gnss.ril_callback_object) {
			GBinderLocalRequest *req =
				gbinder_client_new_request(g_gnss.ril_client);

			gbinder_local_request_append_local_object(req,
			                                          g_gnss.ril_callback_object);
			gnss_binder_client_transact_void(g_gnss.ril_client,
			                                 AGNSS_RIL_TX_SET_CALLBACK, req);
		}
	}

	/* XTRA: predicted ephemeris, the largest cold-start TTFF saving. */
	g_gnss.xtra_remote = gnss_binder_get_extension(GNSS_TX_GET_EXTENSION_XTRA,
	                                               "IGnssXtra");
	if (g_gnss.xtra_remote) {
		gbinder_remote_object_ref(g_gnss.xtra_remote);
		g_gnss.xtra_client = gbinder_client_new(g_gnss.xtra_remote,
		                                        GNSS_XTRA_IFACE_1_0);
		g_gnss.xtra_callback_object =
			gbinder_servicemanager_new_local_object(g_gnss.sm,
			                                        GNSS_XTRA_CALLBACK_IFACE_1_0,
			                                        gnss_binder_xtra_callback_handler,
			                                        NULL);

		if (g_gnss.xtra_client && g_gnss.xtra_callback_object) {
			GBinderLocalRequest *req =
				gbinder_client_new_request(g_gnss.xtra_client);

			gbinder_local_request_append_local_object(req,
			                                          g_gnss.xtra_callback_object);
			gnss_binder_client_transact_bool(g_gnss.xtra_client,
			                                 GNSS_XTRA_TX_SET_CALLBACK, req);
		}
	}
}

/*
 * Looks the GNSS instance up newest-first, so the log names the version the
 * device really implements even though we go on to speak @1.0 to it.
 */
static gboolean gnss_binder_bind_service(void)
{
	gsize i;

	for (i = 0; i < G_N_ELEMENTS(gnss_instances); i++) {
		int status = -1;
		GBinderRemoteObject *remote =
			gbinder_servicemanager_get_service_sync(g_gnss.sm,
			                                        gnss_instances[i],
			                                        &status);

		if (remote) {
			/* get_service_sync returns an autoreleased reference. */
			g_gnss.remote = gbinder_remote_object_ref(remote);
			g_strlcpy(g_gnss.provider_name, gnss_instances[i],
			          sizeof(g_gnss.provider_name));
			g_gnss.bound = TRUE;
			nyx_info(MSGID_NYX_HYBRIS_GPS_HAL_FOUND, 0,
			         "Using GNSS HAL %s", gnss_instances[i]);
			return TRUE;
		}
	}

	return FALSE;
}

bool gnss_binder_init(const gnss_binder_callbacks *callbacks, void *user_data)
{
	GBinderLocalRequest *req;

	if (!callbacks)
		return false;

	if (g_gnss.client) {
		/* Already bound; just re-point the callbacks. */
		g_gnss.callbacks = *callbacks;
		g_gnss.user_data = user_data;
		return true;
	}

	g_gnss.callbacks = *callbacks;
	g_gnss.user_data = user_data;

	g_gnss.sm = gbinder_servicemanager_new("/dev/hwbinder");
	if (!g_gnss.sm) {
		nyx_error(MSGID_NYX_HYBRIS_GPS_NO_SERVICEMANAGER, 0,
		          "Failed to open /dev/hwbinder");
		return false;
	}

	if (!gnss_binder_bind_service()) {
		nyx_error(MSGID_NYX_HYBRIS_GPS_NO_HAL, 0,
		          "No android.hardware.gnss service found on /dev/hwbinder");
		gnss_binder_cleanup();
		return false;
	}

	/*
	 * Always talk @1.0 regardless of which instance answered - see the note
	 * at the top of this file on why that is both safe and sufficient.
	 */
	g_gnss.client = gbinder_client_new(g_gnss.remote, GNSS_IFACE_1_0);
	if (!g_gnss.client) {
		nyx_error(MSGID_NYX_HYBRIS_GPS_NO_HAL, 0, "Failed to create GNSS client");
		gnss_binder_cleanup();
		return false;
	}

	g_gnss.death_id = gbinder_remote_object_add_death_handler(g_gnss.remote,
	                                                          gnss_binder_death_handler,
	                                                          NULL);

	g_gnss.callback_object =
		gbinder_servicemanager_new_local_object(g_gnss.sm,
		                                        GNSS_CALLBACK_IFACE_1_0,
		                                        gnss_binder_callback_handler,
		                                        NULL);
	if (!g_gnss.callback_object) {
		nyx_error(MSGID_NYX_HYBRIS_GPS_NO_HAL, 0,
		          "Failed to create GNSS callback object");
		gnss_binder_cleanup();
		return false;
	}

	req = gbinder_client_new_request(g_gnss.client);
	gbinder_local_request_append_local_object(req, g_gnss.callback_object);

	if (!gnss_binder_transact_bool(GNSS_TX_SET_CALLBACK, req)) {
		nyx_error(MSGID_NYX_HYBRIS_GPS_SET_CALLBACK_ERR, 0,
		          "GNSS setCallback was rejected by %s", g_gnss.provider_name);
		gnss_binder_cleanup();
		return false;
	}

	/*
	 * Extensions come last: the core interface has to be usable before the
	 * assistance ones are worth asking for, and a HAL without them is still a
	 * perfectly good GNSS HAL.
	 */
	gnss_binder_setup_extensions();

	return true;
}

bool gnss_binder_start(void)
{
	if (!g_gnss.client)
		return false;

	return gnss_binder_transact_bool(GNSS_TX_START,
	                                 gbinder_client_new_request(g_gnss.client));
}

bool gnss_binder_stop(void)
{
	if (!g_gnss.client)
		return false;

	return gnss_binder_transact_bool(GNSS_TX_STOP,
	                                 gbinder_client_new_request(g_gnss.client));
}

bool gnss_binder_set_position_mode(uint32_t mode, uint32_t recurrence,
                                   uint32_t min_interval_ms,
                                   uint32_t preferred_accuracy_m,
                                   uint32_t preferred_time_ms)
{
	GBinderLocalRequest *req;

	if (!g_gnss.client)
		return false;

	req = gbinder_client_new_request(g_gnss.client);
	gbinder_local_request_append_int32(req, (gint32) mode);
	gbinder_local_request_append_int32(req, (gint32) recurrence);
	gbinder_local_request_append_int32(req, (gint32) min_interval_ms);
	gbinder_local_request_append_int32(req, (gint32) preferred_accuracy_m);
	gbinder_local_request_append_int32(req, (gint32) preferred_time_ms);

	return gnss_binder_transact_bool(GNSS_TX_SET_POSITION_MODE, req);
}

bool gnss_binder_inject_time(int64_t time_ms, int64_t time_reference_ms,
                             int32_t uncertainty_ms)
{
	GBinderLocalRequest *req;

	if (!g_gnss.client)
		return false;

	req = gbinder_client_new_request(g_gnss.client);
	gbinder_local_request_append_int64(req, time_ms);
	gbinder_local_request_append_int64(req, time_reference_ms);
	gbinder_local_request_append_int32(req, uncertainty_ms);

	return gnss_binder_transact_bool(GNSS_TX_INJECT_TIME, req);
}

bool gnss_binder_inject_location(double latitude, double longitude,
                                 float accuracy_m)
{
	GBinderLocalRequest *req;

	if (!g_gnss.client)
		return false;

	req = gbinder_client_new_request(g_gnss.client);
	gbinder_local_request_append_double(req, latitude);
	gbinder_local_request_append_double(req, longitude);
	gbinder_local_request_append_float(req, accuracy_m);

	return gnss_binder_transact_bool(GNSS_TX_INJECT_LOCATION, req);
}

void gnss_binder_delete_aiding_data(uint32_t flags)
{
	GBinderLocalRequest *req;

	if (!g_gnss.client)
		return;

	req = gbinder_client_new_request(g_gnss.client);
	gbinder_local_request_append_int32(req, (gint32) flags);

	gnss_binder_transact_oneway(GNSS_TX_DELETE_AIDING_DATA, req);
}

bool gnss_binder_agnss_available(void)
{
	return g_gnss.agnss_client != NULL;
}

bool gnss_binder_agnss_set_server(uint16_t type, const char *hostname, int port)
{
	GBinderLocalRequest *req;
	GBinderWriter writer;

	if (!g_gnss.agnss_client || !hostname)
		return false;

	req = gbinder_client_new_request(g_gnss.agnss_client);
	gbinder_local_request_init_writer(req, &writer);
	/* AGnssType is uint8_t but HIDL pads every primitive to 4 bytes. */
	gbinder_writer_append_int32(&writer, type);
	gbinder_writer_append_hidl_string(&writer, hostname);
	gbinder_writer_append_int32(&writer, port);

	return gnss_binder_client_transact_bool(g_gnss.agnss_client,
	                                        AGNSS_TX_SET_SERVER, req);
}

bool gnss_binder_agnss_data_conn_open(const char *apn, int16_t bearer_type)
{
	GBinderLocalRequest *req;
	GBinderWriter writer;

	if (!g_gnss.agnss_client || !apn)
		return false;

	req = gbinder_client_new_request(g_gnss.agnss_client);
	gbinder_local_request_init_writer(req, &writer);
	gbinder_writer_append_hidl_string(&writer, apn);
	gbinder_writer_append_int32(&writer, bearer_type);

	return gnss_binder_client_transact_bool(g_gnss.agnss_client,
	                                        AGNSS_TX_DATA_CONN_OPEN, req);
}

bool gnss_binder_agnss_data_conn_closed(void)
{
	if (!g_gnss.agnss_client)
		return false;

	return gnss_binder_client_transact_bool(g_gnss.agnss_client,
	                                        AGNSS_TX_DATA_CONN_CLOSED,
	                                        gbinder_client_new_request(g_gnss.agnss_client));
}

bool gnss_binder_agnss_data_conn_failed(void)
{
	if (!g_gnss.agnss_client)
		return false;

	return gnss_binder_client_transact_bool(g_gnss.agnss_client,
	                                        AGNSS_TX_DATA_CONN_FAILED,
	                                        gbinder_client_new_request(g_gnss.agnss_client));
}

bool gnss_binder_ril_available(void)
{
	return g_gnss.ril_client != NULL;
}

bool gnss_binder_ril_set_ref_location(uint16_t type, uint16_t mcc, uint16_t mnc,
                                      uint16_t lac, uint32_t cid)
{
	GBinderLocalRequest *req;
	GBinderWriter writer;
	AGnssRefLocationHidl *loc;

	if (!g_gnss.ril_client)
		return false;

	req = gbinder_client_new_request(g_gnss.ril_client);
	gbinder_local_request_init_writer(req, &writer);

	/*
	 * The struct is passed by value, which on the wire means a buffer object.
	 * It has to outlive the transaction, so let the writer own the copy.
	 */
	loc = gbinder_writer_new0(&writer, AGnssRefLocationHidl);
	loc->type = (guint8) type;
	loc->cellID.type = (guint8) type;
	loc->cellID.mcc = mcc;
	loc->cellID.mnc = mnc;
	loc->cellID.lac = lac;
	loc->cellID.cid = cid;

	gbinder_writer_append_buffer_object(&writer, loc, sizeof(*loc));

	gnss_binder_client_transact_void(g_gnss.ril_client,
	                                 AGNSS_RIL_TX_SET_REF_LOCATION, req);
	return true;
}

bool gnss_binder_ril_set_set_id(uint16_t type, const char *set_id)
{
	GBinderLocalRequest *req;
	GBinderWriter writer;

	if (!g_gnss.ril_client)
		return false;

	req = gbinder_client_new_request(g_gnss.ril_client);
	gbinder_local_request_init_writer(req, &writer);
	gbinder_writer_append_int32(&writer, type);
	gbinder_writer_append_hidl_string(&writer, set_id ? set_id : "");

	return gnss_binder_client_transact_bool(g_gnss.ril_client,
	                                        AGNSS_RIL_TX_SET_SET_ID, req);
}

bool gnss_binder_ril_update_network_state(bool connected, int type, bool roaming)
{
	GBinderLocalRequest *req;
	GBinderWriter writer;

	if (!g_gnss.ril_client)
		return false;

	req = gbinder_client_new_request(g_gnss.ril_client);
	gbinder_local_request_init_writer(req, &writer);
	gbinder_writer_append_bool(&writer, connected);
	gbinder_writer_append_int32(&writer, type);
	gbinder_writer_append_bool(&writer, roaming);

	return gnss_binder_client_transact_bool(g_gnss.ril_client,
	                                        AGNSS_RIL_TX_UPDATE_NETWORK_STATE, req);
}

bool gnss_binder_ril_update_network_availability(bool available, const char *apn)
{
	GBinderLocalRequest *req;
	GBinderWriter writer;

	if (!g_gnss.ril_client)
		return false;

	req = gbinder_client_new_request(g_gnss.ril_client);
	gbinder_local_request_init_writer(req, &writer);
	gbinder_writer_append_bool(&writer, available);
	gbinder_writer_append_hidl_string(&writer, apn ? apn : "");

	return gnss_binder_client_transact_bool(g_gnss.ril_client,
	                                        AGNSS_RIL_TX_UPDATE_NETWORK_AVAILABILITY,
	                                        req);
}

bool gnss_binder_xtra_available(void)
{
	return g_gnss.xtra_client != NULL;
}

bool gnss_binder_xtra_inject_data(const char *data, int length)
{
	GBinderLocalRequest *req;
	GBinderWriter writer;
	char *copy;
	gboolean ok;

	if (!g_gnss.xtra_client || !data || length <= 0)
		return false;

	/*
	 * injectXtraData takes a hidl_string, which is NUL-terminated on the wire,
	 * while nyx hands us a pointer and a length. The payload is binary
	 * ephemeris data rather than text, so copy it and terminate rather than
	 * assuming the caller's buffer already is.
	 */
	copy = g_malloc(length + 1);
	memcpy(copy, data, length);
	copy[length] = '\0';

	req = gbinder_client_new_request(g_gnss.xtra_client);
	gbinder_local_request_init_writer(req, &writer);
	gbinder_writer_append_hidl_string(&writer, copy);

	ok = gnss_binder_client_transact_bool(g_gnss.xtra_client,
	                                      GNSS_XTRA_TX_INJECT_XTRA_DATA, req);
	g_free(copy);

	return ok;
}

const char *gnss_binder_provider_name(void)
{
	return g_gnss.bound ? g_gnss.provider_name : NULL;
}

void gnss_binder_cleanup(void)
{
	/*
	 * Clear the callbacks first. Binder callbacks are delivered from
	 * whichever thread runs the main loop the service manager is attached
	 * to, so a transaction can already be in flight while we tear down;
	 * zeroing here means such a callback returns without dispatching
	 * instead of racing the unrefs below.
	 */
	memset(&g_gnss.callbacks, 0, sizeof(g_gnss.callbacks));
	g_gnss.user_data = NULL;
	g_gnss.bound = FALSE;

	if (g_gnss.client) {
		gnss_binder_transact_oneway(GNSS_TX_CLEANUP,
		                            gbinder_client_new_request(g_gnss.client));
		gbinder_client_unref(g_gnss.client);
		g_gnss.client = NULL;
	}

	if (g_gnss.agnss_client) {
		gbinder_client_unref(g_gnss.agnss_client);
		g_gnss.agnss_client = NULL;
	}
	if (g_gnss.ril_client) {
		gbinder_client_unref(g_gnss.ril_client);
		g_gnss.ril_client = NULL;
	}
	if (g_gnss.xtra_client) {
		gbinder_client_unref(g_gnss.xtra_client);
		g_gnss.xtra_client = NULL;
	}

	if (g_gnss.callback_object) {
		gbinder_local_object_drop(g_gnss.callback_object);
		g_gnss.callback_object = NULL;
	}
	if (g_gnss.agnss_callback_object) {
		gbinder_local_object_drop(g_gnss.agnss_callback_object);
		g_gnss.agnss_callback_object = NULL;
	}
	if (g_gnss.ril_callback_object) {
		gbinder_local_object_drop(g_gnss.ril_callback_object);
		g_gnss.ril_callback_object = NULL;
	}
	if (g_gnss.xtra_callback_object) {
		gbinder_local_object_drop(g_gnss.xtra_callback_object);
		g_gnss.xtra_callback_object = NULL;
	}

	if (g_gnss.agnss_remote) {
		gbinder_remote_object_unref(g_gnss.agnss_remote);
		g_gnss.agnss_remote = NULL;
	}
	if (g_gnss.ril_remote) {
		gbinder_remote_object_unref(g_gnss.ril_remote);
		g_gnss.ril_remote = NULL;
	}
	if (g_gnss.xtra_remote) {
		gbinder_remote_object_unref(g_gnss.xtra_remote);
		g_gnss.xtra_remote = NULL;
	}

	if (g_gnss.remote) {
		if (g_gnss.death_id) {
			gbinder_remote_object_remove_handler(g_gnss.remote, g_gnss.death_id);
			g_gnss.death_id = 0;
		}
		gbinder_remote_object_unref(g_gnss.remote);
		g_gnss.remote = NULL;
	}

	if (g_gnss.sm) {
		gbinder_servicemanager_unref(g_gnss.sm);
		g_gnss.sm = NULL;
	}

	g_gnss.provider_name[0] = '\0';
}
