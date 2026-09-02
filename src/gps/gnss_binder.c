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

#include <stdlib.h>
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
 * @2.0 renamed the assistance getters and made the @1.0 ones return a null
 * binder, so on a 2.x device the A-GPS extensions are only reachable through
 * these. IAGnss@2.0 is a fresh interface rather than a subclass of @1.0 - same
 * five methods in the same order, but dataConnOpen gained a leading network
 * handle - so it gets its own descriptor. IAGnssRil@2.0 does extend @1.0, and
 * everything used here is inherited, so it keeps the @1.0 descriptor; there is
 * deliberately no @2.0 name for it below.
 */
#define GNSS_IFACE_2_0 "android.hardware.gnss@2.0::IGnss"
#define AGNSS_IFACE_2_0 "android.hardware.gnss@2.0::IAGnss"
#define AGNSS_CALLBACK_IFACE_2_0 "android.hardware.gnss@2.0::IAGnssCallback"

/*
 * IGnssConfiguration's SUPL setters are all declared in @1.0, so - like
 * IAGnssRil - the object may come from the @2.0 getter but must be addressed
 * with the @1.0 descriptor.
 */
#define GNSS_CONFIG_IFACE_1_0 "android.hardware.gnss@1.0::IGnssConfiguration"

#define GNSS_DEBUG_IFACE_1_0 "android.hardware.gnss@1.0::IGnssDebug"

/* Note the separate package: visibility_control, not gnss. */
#define GNSS_VC_IFACE_1_0 \
	"android.hardware.gnss.visibility_control@1.0::IGnssVisibilityControl"
#define GNSS_VC_CALLBACK_IFACE_1_0 \
	"android.hardware.gnss.visibility_control@1.0::IGnssVisibilityControlCallback"

#define GNSS_NI_IFACE_1_0 "android.hardware.gnss@1.0::IGnssNi"
#define GNSS_NI_CALLBACK_IFACE_1_0 "android.hardware.gnss@1.0::IGnssNiCallback"
#define GNSS_GEOFENCING_IFACE_1_0 "android.hardware.gnss@1.0::IGnssGeofencing"
#define GNSS_GEOFENCE_CALLBACK_IFACE_1_0 "android.hardware.gnss@1.0::IGnssGeofenceCallback"

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
	GNSS_TX_GET_EXTENSION_GEOFENCING = 10,
	GNSS_TX_GET_EXTENSION_NI = 12,
	GNSS_TX_GET_EXTENSION_CONFIG = 16,
	GNSS_TX_GET_EXTENSION_DEBUG = 17,
	GNSS_TX_GET_EXTENSION_XTRA = 15,

	/*
	 * Transaction codes are assigned across the whole inheritance chain in
	 * declaration order: @1.0 occupies 1..18, @1.1 adds five methods at
	 * 19..23, and @2.0's own methods therefore start at 24.
	 */
	GNSS_TX_GET_EXTENSION_AGNSS_2_0 = 27,
	GNSS_TX_GET_EXTENSION_AGNSS_RIL_2_0 = 28,
	GNSS_TX_GET_EXTENSION_CONFIG_2_0 = 25,
	GNSS_TX_GET_EXTENSION_DEBUG_2_0 = 26,
	GNSS_TX_GET_EXTENSION_VISIBILITY_CONTROL = 31
};

enum gnss_debug_tx {
	GNSS_DEBUG_TX_GET_DEBUG_DATA = 1
};

enum gnss_vc_tx {
	GNSS_VC_TX_ENABLE_NFW_LOCATION_ACCESS = 1,
	GNSS_VC_TX_SET_CALLBACK = 2
};

enum gnss_vc_callback_tx {
	GNSS_VC_CB_TX_NFW_NOTIFY = 1,
	GNSS_VC_CB_TX_IS_IN_EMERGENCY_SESSION = 2
};

enum gnss_config_tx {
	GNSS_CONFIG_TX_SET_SUPL_ES = 1,
	GNSS_CONFIG_TX_SET_SUPL_VERSION = 2,
	GNSS_CONFIG_TX_SET_SUPL_MODE = 3,
	GNSS_CONFIG_TX_SET_GPS_LOCK = 4,
	GNSS_CONFIG_TX_SET_LPP_PROFILE = 5,
	GNSS_CONFIG_TX_SET_GLONASS_POS_PROTOCOL = 6,
	GNSS_CONFIG_TX_SET_EMERGENCY_SUPL_PDN = 7
};

/*
 * Defaults taken from AOSP's stock gps.conf, which is where the framework gets
 * them: SUPL 2.0, and both Mobile Station Based and Assisted modes offered so
 * the HAL can pick. Applied when a SUPL server is configured, because that is
 * the point at which the caller has said it wants SUPL at all.
 */
#define GNSS_SUPL_VERSION_2_0 0x00020000
#define GNSS_SUPL_MODE_MSB 0x01
#define GNSS_SUPL_MODE_MSA 0x02

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

enum gnss_ni_tx {
	GNSS_NI_TX_SET_CALLBACK = 1,
	GNSS_NI_TX_RESPOND = 2
};

enum gnss_geofencing_tx {
	GNSS_GEOFENCING_TX_SET_CALLBACK = 1,
	GNSS_GEOFENCING_TX_ADD = 2,
	GNSS_GEOFENCING_TX_PAUSE = 3,
	GNSS_GEOFENCING_TX_RESUME = 4,
	GNSS_GEOFENCING_TX_REMOVE = 5
};

enum gnss_ni_callback_tx {
	GNSS_NI_CB_TX_NOTIFY = 1
};

enum gnss_geofence_callback_tx {
	GNSS_GEOFENCE_CB_TX_TRANSITION = 1,
	GNSS_GEOFENCE_CB_TX_STATUS = 2,
	GNSS_GEOFENCE_CB_TX_ADD = 3,
	GNSS_GEOFENCE_CB_TX_REMOVE = 4,
	GNSS_GEOFENCE_CB_TX_PAUSE = 5,
	GNSS_GEOFENCE_CB_TX_RESUME = 6
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

/*
 * @2.0 collapsed the two address-specific callbacks into one that carries no
 * address at all, which is all nyx ever used anyway.
 */
enum agnss_callback_2_0_tx {
	AGNSS_CB_2_0_TX_STATUS = 1
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

/*
 * android.hardware.gnss@1.0::IGnssNiCallback.GnssNiNotification.
 *
 * The two hidl_string members are 16 bytes on both 32- and 64-bit builds -
 * hidl_pointer pads itself to 8 - so this layout is architecture independent,
 * which the size assert below pins down. The strings themselves are separate
 * buffer objects in the parcel and are read after the struct, not from it.
 */
typedef struct {
	gint32 notificationId;   /* 0 */
	guint8 niType;           /* 4, uint8_t enum */
	guint8 pad0[3];
	guint32 notifyFlags;     /* 8 */
	guint32 timeoutSecs;     /* 12 */
	guint8 defaultResponse;  /* 16, uint8_t enum */
	guint8 pad1[7];
	/*
	 * The two hidl_string members. Declared as guint64 pairs rather than
	 * byte arrays so the compiler gives them the 8-byte alignment a real
	 * hidl_string has - a guint8[16] aligns to 1 and would silently pull
	 * everything after it seven bytes forward. Their contents are never read
	 * from here; the strings arrive as separate buffer objects.
	 */
	guint64 requestorId[2];         /* 24 */
	guint64 notificationMessage[2]; /* 40 */
	gint32 requestorIdEncoding;     /* 56 */
	gint32 notificationIdEncoding;  /* 60 */
} GnssNiNotificationHidl;

/* android.hardware.gnss@1.0::IGnssDebug structures. */
typedef struct {
	guint8 valid;
	guint8 pad0[7];
	gdouble latitudeDegrees;
	gdouble longitudeDegrees;
	gfloat altitudeMeters;
	gfloat speedMetersPerSecs;
	gfloat bearingDegrees;
	guint8 pad1[4];
	gdouble horizontalAccuracyMeters;
	gdouble verticalAccuracyMeters;
	gdouble speedAccuracyMetersPerSecond;
	gdouble bearingAccuracyDegrees;
	gfloat ageSeconds;
	guint8 pad2[4];
} PositionDebugHidl;

typedef struct {
	gint64 timeEstimate;
	gfloat timeUncertaintyNs;
	gfloat frequencyUncertaintyNsPerSec;
} TimeDebugHidl;

typedef struct {
	gint16 svid;
	guint8 constellation;
	guint8 ephemerisType;
	guint8 ephemerisSource;
	guint8 ephemerisHealth;
	guint8 pad0[2];
	gfloat ephemerisAgeSeconds;
	guint8 serverPredictionIsAvailable;
	guint8 pad1[3];
	gfloat serverPredictionAgeSeconds;
} SatelliteDataHidl;

typedef struct {
	PositionDebugHidl position;
	TimeDebugHidl time;
	guint64 satelliteDataArray[2]; /* hidl_vec, read separately */
} DebugDataHidl;

/*
 * IGnssVisibilityControlCallback.NfwNotification. Three hidl_strings, read
 * from the parcel after the struct in field order, exactly as the NI
 * notification is.
 */
typedef struct {
	guint64 proxyAppPackageName[2];
	guint8 protocolStack;
	guint8 pad0[7];
	guint64 otherProtocolStackName[2];
	guint8 requestor;
	guint8 pad1[7];
	guint64 requestorId[2];
	guint8 responseType;
	guint8 inEmergencyMode;
	guint8 isCachedLocation;
	guint8 pad2[5];
} NfwNotificationHidl;

G_STATIC_ASSERT(sizeof(PositionDebugHidl) == 80);
G_STATIC_ASSERT(sizeof(TimeDebugHidl) == 16);
G_STATIC_ASSERT(sizeof(SatelliteDataHidl) == 20);
G_STATIC_ASSERT(sizeof(DebugDataHidl) == 112);
G_STATIC_ASSERT(sizeof(NfwNotificationHidl) == 72);
G_STATIC_ASSERT(sizeof(GnssNiNotificationHidl) == 64);
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

	GBinderRemoteObject *ni_remote;
	GBinderClient *ni_client;
	GBinderLocalObject *ni_callback_object;

	GBinderRemoteObject *geofence_remote;
	GBinderClient *geofence_client;
	GBinderLocalObject *geofence_callback_object;

	/* No callback object: IGnssConfiguration is setters only. */
	GBinderRemoteObject *config_remote;
	GBinderClient *config_client;

	GBinderRemoteObject *debug_remote;
	GBinderClient *debug_client;

	GBinderRemoteObject *vc_remote;
	GBinderClient *vc_client;
	GBinderLocalObject *vc_callback_object;

	gulong death_id;

	/*
	 * Fixed buffer rather than a g_strdup: providers_query() passes this
	 * pointer out through the nyx API, whose callers are entitled to assume
	 * it stays valid - nyx-modules' own GPS module only ever hands back
	 * string literals. Heap here would dangle the moment cleanup() ran.
	 */
	char provider_name[128];
	gboolean bound;

	/*
	 * The HAL dropped off the bus. Set from the death handler, which runs on
	 * the binder event thread, so it only records the fact; the teardown and
	 * the rebind happen on the next init() from the caller's own thread.
	 */
	gboolean hal_died;

	/* Major version of the bound IGnss, parsed from the instance name. */
	int hal_major;
	/* Only created on a 2.x HAL, purely to reach the renamed getters. */
	GBinderClient *client_2_0;
	gboolean agnss_is_2_0;

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
	 * This runs on the binder event thread, so record the death rather than
	 * tearing the binding down here - cleanup() unrefs the very objects this
	 * callback arrived through. The next init() sees the flag and rebinds,
	 * instead of handing back the dead client the way it used to: init()
	 * returned true on a stale binding and every later call silently did
	 * nothing.
	 */
	g_gnss.hal_died = TRUE;

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
		          "%s transaction %u failed, binder status %d",
		          gbinder_client_interface(client), code, status);

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
		          "%s transaction %u failed, binder status %d",
		          gbinder_client_interface(client), code, status);
}

/*
 * Asks IGnss for one of its extension interfaces. A HAL that does not implement
 * the extension answers with a null binder rather than an error, so a NULL
 * result here is a normal outcome and not a failure to report.
 */
static GBinderRemoteObject *gnss_binder_get_extension(GBinderClient *client,
                                                      guint32 code,
                                                      const char *what)
{
	GBinderRemoteReply *reply;
	GBinderRemoteObject *obj = NULL;
	int status = -1;

	if (!client)
		return NULL;

	reply = gbinder_client_transact_sync_reply(client, code,
	                                           gbinder_client_new_request(client),
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

	if (g_gnss.agnss_is_2_0) {
		/*
		 * @2.0::IAGnssCallback has a single agnsssStatusCb(type, status)
		 * taking two plain enums rather than a struct with an address.
		 */
		if (code == AGNSS_CB_2_0_TX_STATUS) {
			gint32 type = 0;
			gint32 st = 0;

			if (gbinder_reader_read_int32(&reader, &type) &&
			    gbinder_reader_read_int32(&reader, &st) &&
			    g_gnss.callbacks.agnss_status_cb)
				g_gnss.callbacks.agnss_status_cb((uint16_t) type,
				                                 (uint16_t) st, 0,
				                                 g_gnss.user_data);
		}
		return NULL;
	}

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

static GBinderLocalReply *gnss_binder_ni_callback_handler(GBinderLocalObject *obj,
                                                          GBinderRemoteRequest *req,
                                                          guint code, guint flags,
                                                          int *status, void *user_data)
{
	GBinderReader reader;

	(void) obj;
	(void) flags;
	(void) user_data;

	*status = GBINDER_STATUS_OK;

	if (code != GNSS_NI_CB_TX_NOTIFY || !g_gnss.callbacks.ni_notify_cb)
		return NULL;

	gbinder_remote_request_init_reader(req, &reader);

	{
		const GnssNiNotificationHidl *n =
			gbinder_reader_read_hidl_struct(&reader, GnssNiNotificationHidl);
		gnss_binder_ni_notification out;

		if (!n)
			return NULL;

		memset(&out, 0, sizeof(out));
		out.notification_id = n->notificationId;
		out.ni_type = n->niType;
		out.notify_flags = n->notifyFlags;
		out.timeout_secs = n->timeoutSecs;
		out.default_response = n->defaultResponse;
		out.requestor_id_encoding = n->requestorIdEncoding;
		out.notification_id_encoding = n->notificationIdEncoding;

		/*
		 * A hidl_string inside a struct is written as its own buffer object
		 * following the parent, in field order, so the pointers embedded in
		 * the struct above are meaningless to us and the text has to be read
		 * from the parcel instead. Order matters: requestorId then
		 * notificationMessage.
		 */
		out.requestor_id = gbinder_reader_read_hidl_string_c(&reader);
		out.notification_message = gbinder_reader_read_hidl_string_c(&reader);

		g_gnss.callbacks.ni_notify_cb(&out, g_gnss.user_data);
	}

	return NULL;
}

/*
 * addGeofence, removeGeofence, pauseGeofence and resumeGeofence all report back
 * as (geofenceId, GeofenceStatus), so they share one decode. The status values
 * are already the NYX_GEOFENCER_* ones - both sides use OPERATION_SUCCESS 0 and
 * the same negative error codes - so they pass straight through.
 */
static void gnss_binder_dispatch_geofence_result(GBinderReader *reader, guint code)
{
	gint32 geofence_id = 0;
	gint32 st = 0;

	if (!gbinder_reader_read_int32(reader, &geofence_id) ||
	    !gbinder_reader_read_int32(reader, &st))
		return;

	switch (code) {
	case GNSS_GEOFENCE_CB_TX_ADD:
		if (g_gnss.callbacks.geofence_add_cb)
			g_gnss.callbacks.geofence_add_cb(geofence_id, st, g_gnss.user_data);
		break;
	case GNSS_GEOFENCE_CB_TX_REMOVE:
		if (g_gnss.callbacks.geofence_remove_cb)
			g_gnss.callbacks.geofence_remove_cb(geofence_id, st, g_gnss.user_data);
		break;
	case GNSS_GEOFENCE_CB_TX_PAUSE:
		if (g_gnss.callbacks.geofence_pause_cb)
			g_gnss.callbacks.geofence_pause_cb(geofence_id, st, g_gnss.user_data);
		break;
	default:
		if (g_gnss.callbacks.geofence_resume_cb)
			g_gnss.callbacks.geofence_resume_cb(geofence_id, st, g_gnss.user_data);
		break;
	}
}

static GBinderLocalReply *gnss_binder_geofence_callback_handler(GBinderLocalObject *obj,
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

	switch (code) {
	case GNSS_GEOFENCE_CB_TX_TRANSITION: {
		const GnssLocationHidl *loc;
		gint32 geofence_id = 0;
		gint32 transition = 0;
		guint64 timestamp = 0;
		gnss_binder_location out;

		if (!gbinder_reader_read_int32(&reader, &geofence_id))
			break;

		loc = gbinder_reader_read_hidl_struct(&reader, GnssLocationHidl);

		if (!gbinder_reader_read_int32(&reader, &transition) ||
		    !gbinder_reader_read_uint64(&reader, &timestamp))
			break;

		if (!g_gnss.callbacks.geofence_transition_cb)
			break;

		if (loc) {
			gnss_binder_convert_location(loc, &out);
			g_gnss.callbacks.geofence_transition_cb(geofence_id, &out,
			                                        transition,
			                                        (int64_t) timestamp,
			                                        g_gnss.user_data);
		} else {
			g_gnss.callbacks.geofence_transition_cb(geofence_id, NULL,
			                                        transition,
			                                        (int64_t) timestamp,
			                                        g_gnss.user_data);
		}
		break;
	}
	case GNSS_GEOFENCE_CB_TX_STATUS: {
		const GnssLocationHidl *loc;
		gint32 st = 0;
		gnss_binder_location out;

		if (!gbinder_reader_read_int32(&reader, &st))
			break;

		loc = gbinder_reader_read_hidl_struct(&reader, GnssLocationHidl);

		if (!g_gnss.callbacks.geofence_status_cb)
			break;

		if (loc) {
			gnss_binder_convert_location(loc, &out);
			g_gnss.callbacks.geofence_status_cb(st, &out, g_gnss.user_data);
		} else {
			g_gnss.callbacks.geofence_status_cb(st, NULL, g_gnss.user_data);
		}
		break;
	}
	case GNSS_GEOFENCE_CB_TX_ADD:
	case GNSS_GEOFENCE_CB_TX_REMOVE:
	case GNSS_GEOFENCE_CB_TX_PAUSE:
	case GNSS_GEOFENCE_CB_TX_RESUME:
		gnss_binder_dispatch_geofence_result(&reader, code);
		break;
	default:
		break;
	}

	return NULL;
}

/*
 * Brings up one extension: fetch the interface, register our callback object
 * with it, and keep the client only if both steps worked. Anything less is
 * torn down so the *_available() predicate stays truthful.
 */
static GBinderLocalReply *gnss_binder_vc_callback_handler(GBinderLocalObject *obj,
                                                          GBinderRemoteRequest *req,
                                                          guint code, guint flags,
                                                          int *status, void *user_data)
{
	GBinderReader reader;

	(void) flags;
	(void) user_data;

	*status = GBINDER_STATUS_OK;

	if (code == GNSS_VC_CB_TX_IS_IN_EMERGENCY_SESSION) {
		/*
		 * Unlike every other callback here this one generates a value, so it
		 * must be answered or the HAL blocks waiting. LuneOS has no emergency
		 * call state plumbed to this module, so answer false rather than
		 * claim an emergency session that would bypass the user's consent.
		 */
		GBinderLocalReply *reply = gbinder_local_object_new_reply(obj);
		GBinderWriter writer;

		gbinder_local_reply_init_writer(reply, &writer);
		gbinder_writer_append_int32(&writer, 0); /* hidl status: OK */
		gbinder_writer_append_bool(&writer, FALSE);
		return reply;
	}

	if (code != GNSS_VC_CB_TX_NFW_NOTIFY || !g_gnss.callbacks.nfw_notify_cb)
		return NULL;

	gbinder_remote_request_init_reader(req, &reader);

	{
		const NfwNotificationHidl *n =
			gbinder_reader_read_hidl_struct(&reader, NfwNotificationHidl);
		gnss_binder_nfw_notification out;

		if (!n)
			return NULL;

		memset(&out, 0, sizeof(out));
		out.protocol_stack = n->protocolStack;
		out.requestor = n->requestor;
		out.response_type = n->responseType;
		out.in_emergency_mode = n->inEmergencyMode != 0;
		out.is_cached_location = n->isCachedLocation != 0;

		/* Field order: proxyAppPackageName, otherProtocolStackName, requestorId. */
		out.proxy_app_package_name = gbinder_reader_read_hidl_string_c(&reader);
		out.other_protocol_stack_name = gbinder_reader_read_hidl_string_c(&reader);
		out.requestor_id = gbinder_reader_read_hidl_string_c(&reader);

		g_gnss.callbacks.nfw_notify_cb(&out, g_gnss.user_data);
	}

	return NULL;
}

/*
 * One extension: fetch the interface, wrap it in a client, register our
 * callback object with it. The client is kept only if every step worked, so the
 * *_available() predicates stay truthful. A HAL that does not implement an
 * extension answers getExtension* with a null binder rather than an error, so
 * that outcome is reported as absence, not failure.
 */
static void gnss_binder_setup_extension(GBinderClient *from, guint32 get_code,
                                        const char *what, const char *iface,
                                        const char *cb_iface,
                                        GBinderLocalTransactFunc handler,
                                        guint32 set_callback_code,
                                        gboolean set_callback_returns_bool,
                                        GBinderRemoteObject **remote_out,
                                        GBinderClient **client_out,
                                        GBinderLocalObject **cb_out)
{
	GBinderRemoteObject *remote = gnss_binder_get_extension(from, get_code, what);
	GBinderLocalRequest *req;

	if (!remote)
		return;

	*remote_out = gbinder_remote_object_ref(remote);
	*client_out = gbinder_client_new(remote, iface);
	*cb_out = gbinder_servicemanager_new_local_object(g_gnss.sm, cb_iface,
	                                                  handler, NULL);

	if (!*client_out || !*cb_out)
		return;

	req = gbinder_client_new_request(*client_out);
	gbinder_local_request_append_local_object(req, *cb_out);

	if (set_callback_returns_bool)
		gnss_binder_client_transact_bool(*client_out, set_callback_code, req);
	else
		gnss_binder_client_transact_void(*client_out, set_callback_code, req);
}

static void gnss_binder_setup_extensions(void)
{
	/*
	 * On a 2.x HAL the assistance getters live on the @2.0 interface and the
	 * @1.0 ones are specified to return a null binder, so asking through the
	 * @1.0 client would silently look like "no A-GPS on this device" - which
	 * is exactly what it did look like before this was handled.
	 */
	if (g_gnss.hal_major >= 2) {
		g_gnss.client_2_0 = gbinder_client_new(g_gnss.remote, GNSS_IFACE_2_0);
		g_gnss.agnss_is_2_0 = TRUE;
	}

	/* A-GNSS (SUPL): server configuration and data-connection handshake. */
	gnss_binder_setup_extension(g_gnss.agnss_is_2_0 ? g_gnss.client_2_0 : g_gnss.client,
	                            g_gnss.agnss_is_2_0 ?
	                            GNSS_TX_GET_EXTENSION_AGNSS_2_0 :
	                            GNSS_TX_GET_EXTENSION_AGNSS,
	                            g_gnss.agnss_is_2_0 ? "IAGnss@2.0" : "IAGnss",
	                            g_gnss.agnss_is_2_0 ? AGNSS_IFACE_2_0 : AGNSS_IFACE_1_0,
	                            g_gnss.agnss_is_2_0 ? AGNSS_CALLBACK_IFACE_2_0 :
	                            AGNSS_CALLBACK_IFACE_1_0,
	                            gnss_binder_agnss_callback_handler,
	                            AGNSS_TX_SET_CALLBACK, FALSE,
	                            &g_gnss.agnss_remote, &g_gnss.agnss_client,
	                            &g_gnss.agnss_callback_object);

	/*
	 * A-GNSS RIL: the side that actually benefits from a SIM.
	 *
	 * @2.0::IAGnssRil extends @1.0, and every method this module calls is one
	 * of the five it inherits. Inherited methods must be addressed with the
	 * descriptor of the interface that *declared* them, not the derived one:
	 * the generated server dispatches those codes down to the @1.0 base, whose
	 * onTransact does enforceInterface(@1.0::IAGnssRil::descriptor). Sending
	 * @2.0::IAGnssRil there fails that check and the transaction comes back
	 * BAD_TYPE (0x80000001), which is exactly what MTK returned. The object is
	 * still the one obtained from getExtensionAGnssRil_2_0; only the name on
	 * the wire has to be the base. Same rule the IGnss client already follows
	 * by speaking @1.0 to a @2.1 object.
	 */
	gnss_binder_setup_extension(g_gnss.agnss_is_2_0 ? g_gnss.client_2_0 : g_gnss.client,
	                            g_gnss.agnss_is_2_0 ?
	                            GNSS_TX_GET_EXTENSION_AGNSS_RIL_2_0 :
	                            GNSS_TX_GET_EXTENSION_AGNSS_RIL,
	                            g_gnss.agnss_is_2_0 ? "IAGnssRil@2.0" : "IAGnssRil",
	                            AGNSS_RIL_IFACE_1_0,
	                            AGNSS_RIL_CALLBACK_IFACE_1_0,
	                            gnss_binder_ril_callback_handler,
	                            AGNSS_RIL_TX_SET_CALLBACK, FALSE,
	                            &g_gnss.ril_remote, &g_gnss.ril_client,
	                            &g_gnss.ril_callback_object);

	/*
	 * NI: network-initiated location requests. @2.0 requires
	 * getExtensionGnssNi() to return null - its replacement is the different
	 * IGnssVisibilityControl model - so on a 2.x HAL this is expected to be
	 * absent rather than broken.
	 */
	gnss_binder_setup_extension(g_gnss.client, GNSS_TX_GET_EXTENSION_NI,
	                            "IGnssNi", GNSS_NI_IFACE_1_0,
	                            GNSS_NI_CALLBACK_IFACE_1_0,
	                            gnss_binder_ni_callback_handler,
	                            GNSS_NI_TX_SET_CALLBACK, FALSE,
	                            &g_gnss.ni_remote, &g_gnss.ni_client,
	                            &g_gnss.ni_callback_object);

	/* Geofencing: the chip watches the fences so nothing has to poll GPS. */
	gnss_binder_setup_extension(g_gnss.client, GNSS_TX_GET_EXTENSION_GEOFENCING,
	                            "IGnssGeofencing", GNSS_GEOFENCING_IFACE_1_0,
	                            GNSS_GEOFENCE_CALLBACK_IFACE_1_0,
	                            gnss_binder_geofence_callback_handler,
	                            GNSS_GEOFENCING_TX_SET_CALLBACK, FALSE,
	                            &g_gnss.geofence_remote, &g_gnss.geofence_client,
	                            &g_gnss.geofence_callback_object);

	/*
	 * IGnssConfiguration: SUPL version and mode. Setters only, so there is no
	 * callback object to register and the shared helper does not fit.
	 */
	g_gnss.config_remote = gnss_binder_get_extension(g_gnss.agnss_is_2_0 ?
	                                                 g_gnss.client_2_0 : g_gnss.client,
	                                                 g_gnss.agnss_is_2_0 ?
	                                                 GNSS_TX_GET_EXTENSION_CONFIG_2_0 :
	                                                 GNSS_TX_GET_EXTENSION_CONFIG,
	                                                 "IGnssConfiguration");
	if (g_gnss.config_remote) {
		gbinder_remote_object_ref(g_gnss.config_remote);
		g_gnss.config_client = gbinder_client_new(g_gnss.config_remote,
		                                          GNSS_CONFIG_IFACE_1_0);
	}

	/*
	 * IGnssDebug. getDebugData is declared in @1.0, so even when the object
	 * comes from getExtensionGnssDebug_2_0 it is addressed as @1.0::IGnssDebug.
	 */
	g_gnss.debug_remote = gnss_binder_get_extension(g_gnss.agnss_is_2_0 ?
	                                                g_gnss.client_2_0 : g_gnss.client,
	                                                g_gnss.agnss_is_2_0 ?
	                                                GNSS_TX_GET_EXTENSION_DEBUG_2_0 :
	                                                GNSS_TX_GET_EXTENSION_DEBUG,
	                                                "IGnssDebug");
	if (g_gnss.debug_remote) {
		gbinder_remote_object_ref(g_gnss.debug_remote);
		g_gnss.debug_client = gbinder_client_new(g_gnss.debug_remote,
		                                         GNSS_DEBUG_IFACE_1_0);
	}

	/*
	 * IGnssVisibilityControl exists only from @2.0 - it is the replacement for
	 * the NI model that @2.0 retires - so it is not looked for on a 1.x HAL.
	 */
	if (g_gnss.agnss_is_2_0)
		gnss_binder_setup_extension(g_gnss.client_2_0,
		                            GNSS_TX_GET_EXTENSION_VISIBILITY_CONTROL,
		                            "IGnssVisibilityControl", GNSS_VC_IFACE_1_0,
		                            GNSS_VC_CALLBACK_IFACE_1_0,
		                            gnss_binder_vc_callback_handler,
		                            GNSS_VC_TX_SET_CALLBACK, TRUE,
		                            &g_gnss.vc_remote, &g_gnss.vc_client,
		                            &g_gnss.vc_callback_object);

	/* XTRA: predicted ephemeris, the largest cold-start TTFF saving. */
	gnss_binder_setup_extension(g_gnss.client, GNSS_TX_GET_EXTENSION_XTRA,
	                            "IGnssXtra", GNSS_XTRA_IFACE_1_0,
	                            GNSS_XTRA_CALLBACK_IFACE_1_0,
	                            gnss_binder_xtra_callback_handler,
	                            GNSS_XTRA_TX_SET_CALLBACK, TRUE,
	                            &g_gnss.xtra_remote, &g_gnss.xtra_client,
	                            &g_gnss.xtra_callback_object);
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

			/*
			 * "android.hardware.gnss@<major>.<minor>::IGnss/default" - the
			 * major version decides which assistance getters exist, so take
			 * it from the name that actually answered rather than probing.
			 */
			{
				const char *at = strchr(gnss_instances[i], '@');

				g_gnss.hal_major = at ?
					(int) strtol(at + 1, NULL, 10) : 1;
			}

			nyx_info(MSGID_NYX_HYBRIS_GPS_HAL_FOUND, 0,
			         "Using GNSS HAL %s", gnss_instances[i]);
			return TRUE;
		}
	}

	return FALSE;
}

/*
 * setCallback is not the one-time handshake it looks like. On MediaTek the HAL
 * passes our callback down to mnld, and mnld exits at the end of every session;
 * whatever brings it back comes back holding nothing. The HAL then goes on
 * producing NMEA with nobody to hand it to and reports no error at all - the
 * subscriber simply gets silence, which is a great deal harder to diagnose than
 * a failure would have been. Re-registering costs one transaction per start,
 * and is what a freshly launched location service does anyway: that was the
 * only configuration in which any of this ever worked.
 */
static gboolean gnss_binder_register_callback(void)
{
	GBinderLocalRequest *req;

	if (!g_gnss.client || !g_gnss.callback_object)
		return FALSE;

	req = gbinder_client_new_request(g_gnss.client);
	gbinder_local_request_append_local_object(req, g_gnss.callback_object);

	return gnss_binder_transact_bool(GNSS_TX_SET_CALLBACK, req);
}

bool gnss_binder_init(const gnss_binder_callbacks *callbacks, void *user_data)
{
	if (!callbacks)
		return false;

	/*
	 * A binding whose HAL has since died is worse than no binding at all:
	 * every transaction on it fails, yet the early return below would still
	 * report success. Tear it down and bind again from scratch.
	 */
	if (g_gnss.hal_died)
		gnss_binder_cleanup();

	if (g_gnss.client) {
		/*
		 * Already bound, so re-point the callbacks - but also make sure
		 * the HAL still holds ours, because re-pointing function
		 * pointers on our side means nothing if the HAL has forgotten
		 * where it is meant to send anything.
		 */
		g_gnss.callbacks = *callbacks;
		g_gnss.user_data = user_data;

		if (!gnss_binder_register_callback())
			nyx_warn(MSGID_NYX_HYBRIS_GPS_SET_CALLBACK_ERR, 0,
			         "Could not re-register the GNSS callback with %s",
			         g_gnss.provider_name);

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

	if (!gnss_binder_register_callback()) {
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

	/*
	 * Unconditionally, not just when a stop invalidated it. The failure this
	 * fixes had no stop in it at all: the service initialised the HAL at
	 * boot and started GPS 73s later, by which point the HAL was no longer
	 * holding the callback that init had handed it. Tracking our own stops
	 * cannot see that happen.
	 *
	 * Registering immediately before start is exactly the sequence a freshly
	 * launched location service produces, which is the only one that has
	 * ever worked on either device - so this makes every session look like
	 * the good case rather than inventing a new one. A rejection is not
	 * fatal: a HAL that refuses a second setCallback is one that is still
	 * holding the first.
	 */
	if (!gnss_binder_register_callback())
		nyx_warn(MSGID_NYX_HYBRIS_GPS_SET_CALLBACK_ERR, 0,
		         "Could not re-register the GNSS callback with %s before start",
		         g_gnss.provider_name);

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

	/*
	 * Configuring a SUPL server is the caller declaring it wants SUPL, so bring
	 * the SUPL parameters up to something usable at the same time. Without a
	 * version the HAL is entitled to assume SUPL 1.0, which most carriers no
	 * longer answer. Failures here are not fatal - the server setting below is
	 * what actually matters - so they are logged by the transact helper and
	 * otherwise ignored.
	 */
	if (g_gnss.config_client) {
		GBinderLocalRequest *creq;
		GBinderWriter cw;

		creq = gbinder_client_new_request(g_gnss.config_client);
		gbinder_local_request_init_writer(creq, &cw);
		gbinder_writer_append_int32(&cw, GNSS_SUPL_VERSION_2_0);
		gnss_binder_client_transact_bool(g_gnss.config_client,
		                                 GNSS_CONFIG_TX_SET_SUPL_VERSION, creq);

		creq = gbinder_client_new_request(g_gnss.config_client);
		gbinder_local_request_init_writer(creq, &cw);
		gbinder_writer_append_int32(&cw, GNSS_SUPL_MODE_MSB | GNSS_SUPL_MODE_MSA);
		gnss_binder_client_transact_bool(g_gnss.config_client,
		                                 GNSS_CONFIG_TX_SET_SUPL_MODE, creq);
	}

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

	/*
	 * @2.0::dataConnOpen takes a leading net_handle_t naming the network to
	 * use. nyx has no equivalent - it only ever passes an APN - so send
	 * NETWORK_UNSPECIFIED (0) and let the HAL pick, which is what the handle
	 * means when the framework has no specific network in mind.
	 */
	if (g_gnss.agnss_is_2_0)
		gbinder_writer_append_int64(&writer, 0);

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

bool gnss_binder_ni_available(void)
{
	return g_gnss.ni_client != NULL;
}

bool gnss_binder_ni_respond(int32_t notification_id, int32_t user_response)
{
	GBinderLocalRequest *req;
	GBinderWriter writer;

	if (!g_gnss.ni_client)
		return false;

	req = gbinder_client_new_request(g_gnss.ni_client);
	gbinder_local_request_init_writer(req, &writer);
	gbinder_writer_append_int32(&writer, notification_id);
	gbinder_writer_append_int32(&writer, user_response);

	gnss_binder_client_transact_void(g_gnss.ni_client, GNSS_NI_TX_RESPOND, req);
	return true;
}

bool gnss_binder_geofence_available(void)
{
	return g_gnss.geofence_client != NULL;
}

bool gnss_binder_geofence_add(int32_t geofence_id, double latitude,
                              double longitude, double radius_meters,
                              int32_t last_transition,
                              int32_t monitor_transitions,
                              int32_t notification_responsiveness_ms,
                              int32_t unknown_timer_ms)
{
	GBinderLocalRequest *req;
	GBinderWriter writer;

	if (!g_gnss.geofence_client)
		return false;

	req = gbinder_client_new_request(g_gnss.geofence_client);
	gbinder_local_request_init_writer(req, &writer);
	gbinder_writer_append_int32(&writer, geofence_id);
	gbinder_writer_append_double(&writer, latitude);
	gbinder_writer_append_double(&writer, longitude);
	gbinder_writer_append_double(&writer, radius_meters);
	gbinder_writer_append_int32(&writer, last_transition);
	gbinder_writer_append_int32(&writer, monitor_transitions);
	gbinder_writer_append_int32(&writer, notification_responsiveness_ms);
	gbinder_writer_append_int32(&writer, unknown_timer_ms);

	/* addGeofence generates no value; the outcome arrives on gnssGeofenceAddCb. */
	gnss_binder_client_transact_void(g_gnss.geofence_client,
	                                 GNSS_GEOFENCING_TX_ADD, req);
	return true;
}

bool gnss_binder_geofence_remove(int32_t geofence_id)
{
	GBinderLocalRequest *req;
	GBinderWriter writer;

	if (!g_gnss.geofence_client)
		return false;

	req = gbinder_client_new_request(g_gnss.geofence_client);
	gbinder_local_request_init_writer(req, &writer);
	gbinder_writer_append_int32(&writer, geofence_id);

	gnss_binder_client_transact_void(g_gnss.geofence_client,
	                                 GNSS_GEOFENCING_TX_REMOVE, req);
	return true;
}

bool gnss_binder_geofence_pause(int32_t geofence_id)
{
	GBinderLocalRequest *req;
	GBinderWriter writer;

	if (!g_gnss.geofence_client)
		return false;

	req = gbinder_client_new_request(g_gnss.geofence_client);
	gbinder_local_request_init_writer(req, &writer);
	gbinder_writer_append_int32(&writer, geofence_id);

	gnss_binder_client_transact_void(g_gnss.geofence_client,
	                                 GNSS_GEOFENCING_TX_PAUSE, req);
	return true;
}

bool gnss_binder_geofence_resume(int32_t geofence_id, int32_t monitor_transitions)
{
	GBinderLocalRequest *req;
	GBinderWriter writer;

	if (!g_gnss.geofence_client)
		return false;

	req = gbinder_client_new_request(g_gnss.geofence_client);
	gbinder_local_request_init_writer(req, &writer);
	gbinder_writer_append_int32(&writer, geofence_id);
	gbinder_writer_append_int32(&writer, monitor_transitions);

	gnss_binder_client_transact_void(g_gnss.geofence_client,
	                                 GNSS_GEOFENCING_TX_RESUME, req);
	return true;
}

bool gnss_binder_debug_available(void)
{
	return g_gnss.debug_client != NULL;
}

bool gnss_binder_nfw_available(void)
{
	return g_gnss.vc_client != NULL;
}

bool gnss_binder_get_debug_data(char *dest, size_t dest_len)
{
	GBinderRemoteReply *reply;
	int status = -1;
	gboolean ok = FALSE;

	if (!g_gnss.debug_client || !dest || dest_len == 0)
		return false;

	reply = gbinder_client_transact_sync_reply(g_gnss.debug_client,
	                                           GNSS_DEBUG_TX_GET_DEBUG_DATA,
	                                           gbinder_client_new_request(g_gnss.debug_client),
	                                           &status);
	if (!reply)
		return false;

	{
		GBinderReader reader;
		gint32 hidl_status = -1;
		const DebugDataHidl *d;

		gbinder_remote_reply_init_reader(reply, &reader);

		if (gbinder_reader_read_int32(&reader, &hidl_status) && hidl_status == 0 &&
		    (d = gbinder_reader_read_hidl_struct(&reader, DebugDataHidl)) != NULL) {
			const SatelliteDataHidl *sats;
			gsize count = 0;
			gsize used;

			used = (gsize) g_snprintf(dest, dest_len,
			         "position: %s lat=%.6f lon=%.6f alt=%.1fm "
			         "hacc=%.1fm vacc=%.1fm age=%.1fs\n"
			         "time: estimate=%lld uncertainty=%.0fns drift=%.3fns/s\n",
			         d->position.valid ? "valid" : "invalid",
			         d->position.latitudeDegrees, d->position.longitudeDegrees,
			         (double) d->position.altitudeMeters,
			         d->position.horizontalAccuracyMeters,
			         d->position.verticalAccuracyMeters,
			         (double) d->position.ageSeconds,
			         (long long) d->time.timeEstimate,
			         (double) d->time.timeUncertaintyNs,
			         (double) d->time.frequencyUncertaintyNsPerSec);

			/*
			 * The satellite array is a hidl_vec following the struct. Its
			 * element layout depends on enum widths the HAL does not pin down,
			 * so a mismatch makes this return NULL rather than misparse - in
			 * which case the position and time above are still reported.
			 */
			sats = gbinder_reader_read_hidl_type_vec(&reader, SatelliteDataHidl,
			                                         &count);

			if (sats && used < dest_len) {
				gsize i;

				used += (gsize) g_snprintf(dest + used, dest_len - used,
				                           "satellites: %u\n", (unsigned) count);

				for (i = 0; i < count && used < dest_len; i++)
					used += (gsize) g_snprintf(dest + used, dest_len - used,
					          "  svid=%d constellation=%u ephemeris=%u "
					          "source=%u health=%u age=%.0fs\n",
					          sats[i].svid, sats[i].constellation,
					          sats[i].ephemerisType, sats[i].ephemerisSource,
					          sats[i].ephemerisHealth,
					          (double) sats[i].ephemerisAgeSeconds);
			} else if (used < dest_len) {
				g_snprintf(dest + used, dest_len - used,
				           "satellites: not reported\n");
			}

			ok = TRUE;
		}

		gbinder_remote_reply_unref(reply);
	}

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
	g_gnss.hal_died = FALSE;

	if (g_gnss.client) {
		gnss_binder_transact_oneway(GNSS_TX_CLEANUP,
		                            gbinder_client_new_request(g_gnss.client));
		gbinder_client_unref(g_gnss.client);
		g_gnss.client = NULL;
	}

	if (g_gnss.client_2_0) {
		gbinder_client_unref(g_gnss.client_2_0);
		g_gnss.client_2_0 = NULL;
	}
	g_gnss.agnss_is_2_0 = FALSE;
	g_gnss.hal_major = 0;

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
	if (g_gnss.ni_client) {
		gbinder_client_unref(g_gnss.ni_client);
		g_gnss.ni_client = NULL;
	}
	if (g_gnss.geofence_client) {
		gbinder_client_unref(g_gnss.geofence_client);
		g_gnss.geofence_client = NULL;
	}
	if (g_gnss.config_client) {
		gbinder_client_unref(g_gnss.config_client);
		g_gnss.config_client = NULL;
	}
	if (g_gnss.debug_client) {
		gbinder_client_unref(g_gnss.debug_client);
		g_gnss.debug_client = NULL;
	}
	if (g_gnss.vc_client) {
		gbinder_client_unref(g_gnss.vc_client);
		g_gnss.vc_client = NULL;
	}
	if (g_gnss.vc_callback_object) {
		gbinder_local_object_drop(g_gnss.vc_callback_object);
		g_gnss.vc_callback_object = NULL;
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
	if (g_gnss.ni_callback_object) {
		gbinder_local_object_drop(g_gnss.ni_callback_object);
		g_gnss.ni_callback_object = NULL;
	}
	if (g_gnss.geofence_callback_object) {
		gbinder_local_object_drop(g_gnss.geofence_callback_object);
		g_gnss.geofence_callback_object = NULL;
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
	if (g_gnss.ni_remote) {
		gbinder_remote_object_unref(g_gnss.ni_remote);
		g_gnss.ni_remote = NULL;
	}
	if (g_gnss.geofence_remote) {
		gbinder_remote_object_unref(g_gnss.geofence_remote);
		g_gnss.geofence_remote = NULL;
	}
	if (g_gnss.config_remote) {
		gbinder_remote_object_unref(g_gnss.config_remote);
		g_gnss.config_remote = NULL;
	}
	if (g_gnss.debug_remote) {
		gbinder_remote_object_unref(g_gnss.debug_remote);
		g_gnss.debug_remote = NULL;
	}
	if (g_gnss.vc_remote) {
		gbinder_remote_object_unref(g_gnss.vc_remote);
		g_gnss.vc_remote = NULL;
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
