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
 * NYX_DEVICE_GPS backed by the Android GNSS HAL over binder.
 *
 * This is the hybris counterpart to nyx-modules/src/gps, which drives a serial
 * NMEA receiver and is the right module for PinePhone. Only one of the two is
 * built for a given machine, selected by NYXMOD_OW_GPS.
 *
 * The translation here is almost a memcpy because nyx's GPS API was modelled on
 * the Android GPS HAL: NYX_GPS_POSITION_MODE_*, NYX_GPS_STATUS_*,
 * NYX_GPS_CAPABILITY_* and the delete-aiding-data bits all carry the same
 * numeric values as their Android counterparts. Rather than trust that, the
 * static asserts below check it at compile time against
 * <android/hardware/gnss-base.h>, so a future divergence is a build error
 * instead of a device that silently reports the wrong thing.
 *
 * Two places where the mapping is genuinely not one-to-one, and so is done by
 * hand:
 *
 *   - Satellite counts. HIDL reports up to 64 SVs, nyx has room for 32.
 *   - Satellite masks. The legacy HAL and nyx describe ephemeris/almanac/
 *     used-in-fix as PRN-indexed uint32 bitmasks; HIDL moved those to per-SV
 *     flags. We rebuild the masks from the flags.
 */

#include <stdlib.h>
#include <string.h>

#include <nyx/nyx_module.h>
#include <nyx/module/nyx_utils.h>
#include <nyx/module/nyx_log.h>

#include <android/hardware/gnss-base.h>

#include "msgid.h"
#include "gnss_binder.h"

NYX_DECLARE_MODULE(NYX_DEVICE_GPS, "Gps");

/*
 * nyx and Android must agree on these, because we pass them straight through.
 */
G_STATIC_ASSERT(NYX_GPS_POSITION_MODE_STANDALONE == GPS_POSITION_MODE_STANDALONE);
G_STATIC_ASSERT(NYX_GPS_POSITION_MODE_MS_BASED == GPS_POSITION_MODE_MS_BASED);
G_STATIC_ASSERT(NYX_GPS_POSITION_MODE_MS_ASSISTED == GPS_POSITION_MODE_MS_ASSISTED);
G_STATIC_ASSERT(NYX_GPS_POSITION_RECURRENCE_PERIODIC == GPS_POSITION_RECURRENCE_PERIODIC);
G_STATIC_ASSERT(NYX_GPS_POSITION_RECURRENCE_SINGLE == GPS_POSITION_RECURRENCE_SINGLE);
G_STATIC_ASSERT(NYX_GPS_STATUS_NONE == GPS_STATUS_NONE);
G_STATIC_ASSERT(NYX_GPS_STATUS_SESSION_BEGIN == GPS_STATUS_SESSION_BEGIN);
G_STATIC_ASSERT(NYX_GPS_STATUS_SESSION_END == GPS_STATUS_SESSION_END);
G_STATIC_ASSERT(NYX_GPS_STATUS_ENGINE_ON == GPS_STATUS_ENGINE_ON);
G_STATIC_ASSERT(NYX_GPS_STATUS_ENGINE_OFF == GPS_STATUS_ENGINE_OFF);
G_STATIC_ASSERT(NYX_GPS_CAPABILITY_SCHEDULING == GPS_CAPABILITY_SCHEDULING);
G_STATIC_ASSERT(NYX_GPS_CAPABILITY_MSB == GPS_CAPABILITY_MSB);
G_STATIC_ASSERT(NYX_GPS_CAPABILITY_MSA == GPS_CAPABILITY_MSA);
G_STATIC_ASSERT(NYX_GPS_CAPABILITY_SINGLE_SHOT == GPS_CAPABILITY_SINGLE_SHOT);
G_STATIC_ASSERT(NYX_GPS_CAPABILITY_ON_DEMAND_TIME == GPS_CAPABILITY_ON_DEMAND_TIME);
G_STATIC_ASSERT(NYX_GPS_LOCATION_HAS_LAT_LONG == GPS_LOCATION_HAS_LAT_LONG);
G_STATIC_ASSERT(NYX_GPS_LOCATION_HAS_ALTITUDE == GPS_LOCATION_HAS_ALTITUDE);
G_STATIC_ASSERT(NYX_GPS_LOCATION_HAS_SPEED == GPS_LOCATION_HAS_SPEED);
G_STATIC_ASSERT(NYX_GPS_LOCATION_HAS_BEARING == GPS_LOCATION_HAS_BEARING);
G_STATIC_ASSERT(NYX_AGPS_TYPE_SUPL == AGPS_TYPE_SUPL);
G_STATIC_ASSERT(NYX_AGPS_TYPE_C2K == AGPS_TYPE_C2K);
G_STATIC_ASSERT(NYX_AGPS_SET_ID_TYPE_IMSI == AGPS_SETID_TYPE_IMSI);
G_STATIC_ASSERT(NYX_AGPS_SET_ID_TYPE_MSISDN == AGPS_SETID_TYPE_MSISDM);
G_STATIC_ASSERT(NYX_AGPS_REQUEST_DATA_CONN == GNSS_REQUEST_AGNSS_DATA_CONN);
G_STATIC_ASSERT(NYX_AGPS_RELEASE_DATA_CONN == GNSS_RELEASE_AGNSS_DATA_CONN);
G_STATIC_ASSERT(NYX_AGPS_DATA_CONNECTED == GNSS_AGNSS_DATA_CONNECTED);
G_STATIC_ASSERT(NYX_AGPS_DATA_CONN_DONE == GNSS_AGNSS_DATA_CONN_DONE);
G_STATIC_ASSERT(NYX_AGPS_DATA_CONN_FAILED == GNSS_AGNSS_DATA_CONN_FAILED);
G_STATIC_ASSERT(NYX_AGPS_REF_LOCATION_TYPE_GSM_CELLID == AGPS_REF_LOCATION_TYPE_GSM_CELLID);
G_STATIC_ASSERT(NYX_AGPS_REF_LOCATION_TYPE_UMTS_CELLID == AGPS_REF_LOCATION_TYPE_UMTS_CELLID);
G_STATIC_ASSERT(NYX_AGPS_RIL_NETWORK_TYPE_MOBILE == AGPS_RIL_NETWORK_TYPE_MOBILE);
G_STATIC_ASSERT(NYX_AGPS_RIL_NETWORK_TYPE_WIFI == AGPS_RIL_NETWORK_TYPE_WIFI);
G_STATIC_ASSERT(NYX_AGPS_RIL_REQUEST_SETID_IMSI == AGPS_RIL_REQUEST_SETID_IMSI);
G_STATIC_ASSERT(NYX_AGPS_RIL_REQUEST_SETID_MSISDN == AGPS_RIL_REQUEST_SETID_MSISDN);

typedef struct methodStringPair {
	module_method_t mType;
	const char *mString;
} methodStringPair_t;

/*
 * Only the methods this module actually implements are registered. nyx answers
 * NYX_ERROR_NOT_IMPLEMENTED for anything unregistered, which is the truthful
 * result for the A-GPS, NI, XTRA and geofencing families - they need the
 * IAGnss/IGnssNi/IGnssXtra/IGnssGeofencing extension interfaces, which this
 * module does not fetch yet. Registering them as no-ops that return success
 * would make com.webos.service.location believe SUPL was configured when it
 * was not.
 */
static const methodStringPair_t mapMethodString[] = {
	{NYX_GPS_INIT_MODULE_METHOD,               "init"},
	{NYX_GPS_QUERY_PROVIDERS_MODULE_METHOD,    "providers_query"},
	{NYX_GPS_START_MODULE_METHOD,              "start"},
	{NYX_GPS_STOP_MODULE_METHOD,               "stop"},
	{NYX_GPS_CLEANUP_MODULE_METHOD,            "cleanup"},
	{NYX_GPS_INJECT_TIME_MODULE_METHOD,        "inject_time"},
	{NYX_GPS_INJECT_LOCATION_MODULE_METHOD,    "inject_location"},
	{NYX_GPS_DELETE_AIDING_DATA_MODULE_METHOD, "delete_aiding_data"},
	{NYX_GPS_SET_POSITION_MODE_MODULE_METHOD,  "set_position_mode"}
};

/*
 * Assistance methods. These cannot be registered conditionally on the extension
 * actually existing: nyx_module_register_method needs the nyx_instance_t that
 * only nyx_module_open has, and the HAL is not bound until init(). So they are
 * always registered and each one answers NYX_ERROR_NOT_IMPLEMENTED when its
 * extension was not obtained - the caller still gets a truthful answer, just at
 * call time rather than at registration time.
 */
static const methodStringPair_t mapAssistMethodString[] = {
	{NYX_GPS_SET_SERVER_MODULE_METHOD,                  "set_server"},
	{NYX_GPS_DATA_CONN_OPEN_MODULE_METHOD,              "data_conn_open"},
	{NYX_GPS_DATA_CONN_CLOSED_MODULE_METHOD,            "data_conn_closed"},
	{NYX_GPS_DATA_CONN_FAILED_MODULE_METHOD,            "data_conn_failed"},
	{NYX_GPS_SET_REF_LOCATION_MODULE_METHOD,            "set_ref_location"},
	{NYX_GPS_SET_SET_ID_MODULE_METHOD,                  "set_set_id"},
	{NYX_GPS_UPDATE_NETWORK_STATE_MODULE_METHOD,        "update_network_state"},
	{NYX_GPS_UPDATE_NETWORK_AVAILABILITY_MODULE_METHOD, "update_network_availability"},
	{NYX_GPS_INJECT_XTRA_DATA_MODULE_METHOD,            "inject_xtra_data"}
};

static nyx_device_t *nyx_dev = NULL;

static nyx_gps_callbacks_t *nyx_gps_cbs = NULL;
static nyx_gps_xtra_callbacks_t *nyx_gps_xtra_cbs = NULL;
static nyx_agps_callbacks_t *nyx_agps_cbs = NULL;
static nyx_agps_ril_callbacks_t *nyx_agps_ril_cbs = NULL;

static nyx_agps_status_t g_agps_status;

/* Reused across callbacks so a fix does not allocate on the binder thread. */
static nyx_gps_location_t g_location;
static nyx_gps_status_t g_status;
static nyx_gps_sv_status_t g_sv_status;

static void gps_location_cb(const gnss_binder_location *location, void *user_data)
{
	(void) user_data;

	if (!nyx_gps_cbs || !nyx_gps_cbs->location_cb)
		return;

	memset(&g_location, 0, sizeof(g_location));
	g_location.size = sizeof(g_location);
	g_location.flags = location->flags;
	g_location.latitude = location->latitude;
	g_location.longitude = location->longitude;
	g_location.altitude = location->altitude;
	g_location.speed = location->speed;
	g_location.bearing = location->bearing;
	g_location.accuracy = location->horizontal_accuracy;
	g_location.vertical_accuracy = location->vertical_accuracy;
	g_location.timestamp = location->timestamp;

	/*
	 * HIDL splits what nyx calls "accuracy" into horizontal and vertical
	 * halves with separate flags. nyx has only NYX_GPS_LOCATION_HAS_ACCURACY
	 * for the horizontal one, so map that flag across explicitly; the
	 * numeric identity asserted above covers the rest.
	 */
	if (location->flags & GPS_LOCATION_HAS_HORIZONTAL_ACCURACY)
		g_location.flags |= NYX_GPS_LOCATION_HAS_ACCURACY;
	if (location->flags & GPS_LOCATION_HAS_VERTICAL_ACCURACY)
		g_location.flags |= NYX_GPS_LOCATION_HAS_VERTICAL_ACCURACY;

	nyx_gps_cbs->location_cb(&g_location, nyx_gps_cbs->user_data);
}

static void gps_status_cb(uint16_t status, void *user_data)
{
	(void) user_data;

	if (!nyx_gps_cbs || !nyx_gps_cbs->status_cb)
		return;

	memset(&g_status, 0, sizeof(g_status));
	g_status.size = sizeof(g_status);
	g_status.status = status;

	nyx_gps_cbs->status_cb(&g_status, nyx_gps_cbs->user_data);
}

static void gps_sv_status_cb(const gnss_binder_sv_status *sv_status, void *user_data)
{
	uint32_t i;
	uint32_t out = 0;

	(void) user_data;

	if (!nyx_gps_cbs || !nyx_gps_cbs->sv_status_cb)
		return;

	memset(&g_sv_status, 0, sizeof(g_sv_status));
	g_sv_status.size = sizeof(g_sv_status);

	for (i = 0; i < sv_status->num_svs && out < NYX_GPS_MAX_SVS; i++) {
		const gnss_binder_sv_info *in = &sv_status->sv_list[i];

		/*
		 * nyx carries a bare PRN with no constellation field, and its
		 * ephemeris/almanac/used-in-fix masks are indexed by PRN over
		 * 32 bits. Only GPS satellites (svid 1..32) can be described
		 * without ambiguity, so anything else is dropped rather than
		 * aliased onto a GPS PRN and reported as a satellite the device
		 * is not tracking.
		 */
		if (in->constellation != GNSS_CONSTELLATION_GPS)
			continue;

		if (in->svid < 1 || in->svid > 32)
			continue;

		g_sv_status.sv_list[out].size = sizeof(g_sv_status.sv_list[out]);
		g_sv_status.sv_list[out].prn = in->svid;
		g_sv_status.sv_list[out].snr = in->c_n0_dbhz;
		g_sv_status.sv_list[out].elevation = in->elevation;
		g_sv_status.sv_list[out].azimuth = in->azimuth;

		if (in->flags & GNSS_SV_FLAGS_HAS_EPHEMERIS_DATA)
			g_sv_status.ephemeris_mask |= (1U << (in->svid - 1));
		if (in->flags & GNSS_SV_FLAGS_HAS_ALMANAC_DATA)
			g_sv_status.almanac_mask |= (1U << (in->svid - 1));
		if (in->flags & GNSS_SV_FLAGS_USED_IN_FIX)
			g_sv_status.used_in_fix_mask |= (1U << (in->svid - 1));

		out++;
	}

	g_sv_status.num_svs = (int) out;

	nyx_gps_cbs->sv_status_cb(&g_sv_status, nyx_gps_cbs->user_data);
}

static void gps_nmea_cb(int64_t timestamp, const char *nmea, int length,
                        void *user_data)
{
	(void) user_data;

	if (!nyx_gps_cbs || !nyx_gps_cbs->nmea_cb)
		return;

	nyx_gps_cbs->nmea_cb(timestamp, nmea, length, nyx_gps_cbs->user_data);
}

static void gps_set_capabilities_cb(uint32_t capabilities, void *user_data)
{
	(void) user_data;

	if (!nyx_gps_cbs || !nyx_gps_cbs->set_capabilities_cb)
		return;

	nyx_gps_cbs->set_capabilities_cb(capabilities, nyx_gps_cbs->user_data);
}

static void gps_acquire_wakelock_cb(void *user_data)
{
	(void) user_data;

	if (nyx_gps_cbs && nyx_gps_cbs->acquire_wakelock_cb)
		nyx_gps_cbs->acquire_wakelock_cb(nyx_gps_cbs->user_data);
}

static void gps_release_wakelock_cb(void *user_data)
{
	(void) user_data;

	if (nyx_gps_cbs && nyx_gps_cbs->release_wakelock_cb)
		nyx_gps_cbs->release_wakelock_cb(nyx_gps_cbs->user_data);
}

static void gps_request_utc_time_cb(void *user_data)
{
	(void) user_data;

	if (nyx_gps_cbs && nyx_gps_cbs->request_utc_time_cb)
		nyx_gps_cbs->request_utc_time_cb(nyx_gps_cbs->user_data);
}

/*
 * nyx's APN bearer type follows the legacy GPS HAL numbering
 * (AGPS_APN_BEARER_IPV4 == 0), while HIDL's ApnIpType inserted INVALID at 0 and
 * shifted the rest up by one. They are therefore NOT interchangeable: passing a
 * nyx value straight through would send IPv4 to the HAL as INVALID. This is the
 * one place in the module where the two enumerations genuinely disagree, which
 * is why it is a conversion rather than a static assert.
 */
static int32_t gps_apn_ip_type_to_hidl(nyx_agps_bearer_type_t bearer)
{
	switch (bearer) {
	case NYX_AGPS_APN_BEARER_IPV4:
		return APN_IP_IPV4;
	case NYX_AGPS_APN_BEARER_IPV6:
		return APN_IP_IPV6;
	case NYX_AGPS_APN_BEARER_IPV4V6:
		return APN_IP_IPV4V6;
	default:
		return APN_IP_INVALID;
	}
}

static void gps_agnss_status_cb(uint16_t type, uint16_t status,
                                uint32_t ipv4_addr, void *user_data)
{
	(void) user_data;

	if (!nyx_agps_cbs || !nyx_agps_cbs->status_cb)
		return;

	memset(&g_agps_status, 0, sizeof(g_agps_status));
	g_agps_status.size = sizeof(g_agps_status);
	/* nyx_agps_type_t is signed; the HIDL values are 1 and 2, so this is
	 * value-preserving, but be explicit about the narrowing. */
	g_agps_status.type = (nyx_agps_type_t) type;
	g_agps_status.status = status;
	g_agps_status.ipv4_addr = (int) ipv4_addr;

	nyx_agps_cbs->status_cb(&g_agps_status, nyx_agps_cbs->user_data);
}

static void gps_ril_request_set_id_cb(uint32_t flags, void *user_data)
{
	(void) user_data;

	if (nyx_agps_ril_cbs && nyx_agps_ril_cbs->ril_request_set_id_cb)
		nyx_agps_ril_cbs->ril_request_set_id_cb(flags,
		                                        nyx_agps_ril_cbs->user_data);
}

static void gps_ril_request_ref_loc_cb(void *user_data)
{
	(void) user_data;

	/*
	 * IAGnssRilCallback::requestRefLocCb carries no argument, while nyx's
	 * callback takes a flags word. The only reference-location kind this
	 * module can supply is a cell ID, so say so rather than passing zero.
	 */
	if (nyx_agps_ril_cbs && nyx_agps_ril_cbs->ril_request_ref_loc_cb)
		nyx_agps_ril_cbs->ril_request_ref_loc_cb(NYX_AGPS_RIL_REQUEST_REFLOC_CELLID,
		                                         nyx_agps_ril_cbs->user_data);
}

static void gps_xtra_download_request_cb(void *user_data)
{
	(void) user_data;

	if (nyx_gps_xtra_cbs && nyx_gps_xtra_cbs->xtra_download_request_cb)
		nyx_gps_xtra_cbs->xtra_download_request_cb(nyx_gps_xtra_cbs->user_data);
}

static const gnss_binder_callbacks gnss_cbs = {
	.location_cb = gps_location_cb,
	.status_cb = gps_status_cb,
	.sv_status_cb = gps_sv_status_cb,
	.nmea_cb = gps_nmea_cb,
	.set_capabilities_cb = gps_set_capabilities_cb,
	.acquire_wakelock_cb = gps_acquire_wakelock_cb,
	.release_wakelock_cb = gps_release_wakelock_cb,
	.request_utc_time_cb = gps_request_utc_time_cb,
	.agnss_status_cb = gps_agnss_status_cb,
	.ril_request_set_id_cb = gps_ril_request_set_id_cb,
	.ril_request_ref_loc_cb = gps_ril_request_ref_loc_cb,
	.xtra_download_request_cb = gps_xtra_download_request_cb
};

nyx_error_t init(nyx_device_handle_t handle,
                 nyx_gps_callbacks_t *gps_cbs,
                 nyx_gps_xtra_callbacks_t *xtra_cbs,
                 nyx_agps_callbacks_t *agps_cbs,
                 nyx_gps_ni_callbacks_t *gps_ni_cbs,
                 nyx_agps_ril_callbacks_t *agps_ril_cbs,
                 nyx_gps_geofence_callbacks_t *geofence_cbs)
{
	/*
	 * NI and geofencing are still not implemented - they need IGnssNi and
	 * IGnssGeofencing, which this module does not fetch - so their callbacks
	 * are deliberately dropped rather than stored, matching the fact that
	 * none of their methods are registered.
	 */
	(void) gps_ni_cbs;
	(void) geofence_cbs;

	if (nyx_dev == NULL)
		return NYX_ERROR_DEVICE_NOT_EXIST;

	if (handle != nyx_dev || gps_cbs == NULL)
		return NYX_ERROR_INVALID_HANDLE;

	nyx_gps_cbs = gps_cbs;
	nyx_gps_xtra_cbs = xtra_cbs;
	nyx_agps_cbs = agps_cbs;
	nyx_agps_ril_cbs = agps_ril_cbs;

	if (!gnss_binder_init(&gnss_cbs, NULL)) {
		nyx_gps_cbs = NULL;
		nyx_gps_xtra_cbs = NULL;
		nyx_agps_cbs = NULL;
		nyx_agps_ril_cbs = NULL;
		return NYX_ERROR_DEVICE_UNAVAILABLE;
	}

	return NYX_ERROR_NONE;
}

nyx_error_t providers_query(nyx_device_handle_t handle,
                            nyx_gps_providers_query_t query,
                            const char **dest)
{
	if (handle != nyx_dev || dest == NULL)
		return NYX_ERROR_INVALID_HANDLE;

	switch (query) {
	case NYX_GPS_PROVIDER_NAME:
		*dest = gnss_binder_provider_name();
		if (*dest == NULL)
			*dest = "none";
		return NYX_ERROR_NONE;
	case NYX_GPS_PROVIDER_STATUS:
		*dest = gnss_binder_provider_name() ? "started" : "stopped";
		return NYX_ERROR_NONE;
	default:
		return NYX_ERROR_INVALID_VALUE;
	}
}

nyx_error_t start(nyx_device_handle_t handle)
{
	if (handle != nyx_dev)
		return NYX_ERROR_INVALID_HANDLE;

	return gnss_binder_start() ? NYX_ERROR_NONE : NYX_ERROR_GENERIC;
}

nyx_error_t stop(nyx_device_handle_t handle)
{
	if (handle != nyx_dev)
		return NYX_ERROR_INVALID_HANDLE;

	return gnss_binder_stop() ? NYX_ERROR_NONE : NYX_ERROR_GENERIC;
}

nyx_error_t cleanup(nyx_device_handle_t handle)
{
	if (handle != nyx_dev)
		return NYX_ERROR_INVALID_HANDLE;

	gnss_binder_cleanup();
	nyx_gps_cbs = NULL;
	nyx_gps_xtra_cbs = NULL;
	nyx_agps_cbs = NULL;
	nyx_agps_ril_cbs = NULL;

	return NYX_ERROR_NONE;
}

nyx_error_t inject_time(nyx_device_handle_t handle, int64_t time,
                        int64_t timeReference, int uncertainty)
{
	if (handle != nyx_dev)
		return NYX_ERROR_INVALID_HANDLE;

	return gnss_binder_inject_time(time, timeReference, uncertainty) ?
	       NYX_ERROR_NONE : NYX_ERROR_GENERIC;
}

nyx_error_t inject_location(nyx_device_handle_t handle, double latitude,
                            double longitude, float accuracy)
{
	if (handle != nyx_dev)
		return NYX_ERROR_INVALID_HANDLE;

	return gnss_binder_inject_location(latitude, longitude, accuracy) ?
	       NYX_ERROR_NONE : NYX_ERROR_GENERIC;
}

nyx_error_t delete_aiding_data(nyx_device_handle_t handle,
                               nyx_gps_aiding_data_t flags)
{
	if (handle != nyx_dev)
		return NYX_ERROR_INVALID_HANDLE;

	gnss_binder_delete_aiding_data((uint32_t) flags);

	return NYX_ERROR_NONE;
}

nyx_error_t set_position_mode(nyx_device_handle_t handle,
                              nyx_gps_position_mode_t mode,
                              nyx_gps_position_recurrence_t recurrence,
                              uint32_t min_interval,
                              uint32_t preferred_accuracy,
                              uint32_t preferred_time)
{
	if (handle != nyx_dev)
		return NYX_ERROR_INVALID_HANDLE;

	return gnss_binder_set_position_mode(mode, recurrence, min_interval,
	                                     preferred_accuracy, preferred_time) ?
	       NYX_ERROR_NONE : NYX_ERROR_GENERIC;
}

nyx_error_t set_server(nyx_device_handle_t handle, nyx_agps_type_t type,
                       const char *hostname, int port)
{
	if (handle != nyx_dev)
		return NYX_ERROR_INVALID_HANDLE;

	if (!gnss_binder_agnss_available())
		return NYX_ERROR_NOT_IMPLEMENTED;

	return gnss_binder_agnss_set_server((uint16_t) type, hostname, port) ?
	       NYX_ERROR_NONE : NYX_ERROR_GENERIC;
}

nyx_error_t data_conn_open(nyx_device_handle_t handle, nyx_agps_type_t agpsType,
                           const char *apn, nyx_agps_bearer_type_t bearerType)
{
	if (handle != nyx_dev)
		return NYX_ERROR_INVALID_HANDLE;

	if (!gnss_binder_agnss_available())
		return NYX_ERROR_NOT_IMPLEMENTED;

	/*
	 * IAGnss@1.0::dataConnOpen takes only the APN and its IP type; the AGnss
	 * type is not part of the call, so agpsType is accepted for API
	 * compatibility and has nothing to map onto.
	 */
	(void) agpsType;

	return gnss_binder_agnss_data_conn_open(apn,
	                                       (int16_t) gps_apn_ip_type_to_hidl(bearerType)) ?
	       NYX_ERROR_NONE : NYX_ERROR_GENERIC;
}

nyx_error_t data_conn_closed(nyx_device_handle_t handle, nyx_agps_type_t agpsType)
{
	if (handle != nyx_dev)
		return NYX_ERROR_INVALID_HANDLE;

	if (!gnss_binder_agnss_available())
		return NYX_ERROR_NOT_IMPLEMENTED;

	(void) agpsType;

	return gnss_binder_agnss_data_conn_closed() ?
	       NYX_ERROR_NONE : NYX_ERROR_GENERIC;
}

nyx_error_t data_conn_failed(nyx_device_handle_t handle, nyx_agps_type_t agpsType)
{
	if (handle != nyx_dev)
		return NYX_ERROR_INVALID_HANDLE;

	if (!gnss_binder_agnss_available())
		return NYX_ERROR_NOT_IMPLEMENTED;

	(void) agpsType;

	return gnss_binder_agnss_data_conn_failed() ?
	       NYX_ERROR_NONE : NYX_ERROR_GENERIC;
}

nyx_error_t set_ref_location(nyx_device_handle_t handle,
                             const nyx_agps_ref_location_t *agps_reflocation,
                             size_t sz_struct)
{
	if (handle != nyx_dev)
		return NYX_ERROR_INVALID_HANDLE;

	if (!agps_reflocation || sz_struct < sizeof(*agps_reflocation))
		return NYX_ERROR_INVALID_VALUE;

	if (!gnss_binder_ril_available())
		return NYX_ERROR_NOT_IMPLEMENTED;

	/*
	 * Only the cell-ID form can be forwarded. nyx also allows a MAC-address
	 * reference location, which IAGnssRil@1.0 has no representation for.
	 */
	if (agps_reflocation->type != NYX_AGPS_REF_LOCATION_TYPE_GSM_CELLID &&
	    agps_reflocation->type != NYX_AGPS_REF_LOCATION_TYPE_UMTS_CELLID)
		return NYX_ERROR_NOT_IMPLEMENTED;

	return gnss_binder_ril_set_ref_location(agps_reflocation->type,
	                                        agps_reflocation->u.cellID.mcc,
	                                        agps_reflocation->u.cellID.mnc,
	                                        agps_reflocation->u.cellID.lac,
	                                        agps_reflocation->u.cellID.cid) ?
	       NYX_ERROR_NONE : NYX_ERROR_GENERIC;
}

nyx_error_t set_set_id(nyx_device_handle_t handle, nyx_agps_set_id_type_t type,
                       const char *set_id)
{
	if (handle != nyx_dev)
		return NYX_ERROR_INVALID_HANDLE;

	if (!gnss_binder_ril_available())
		return NYX_ERROR_NOT_IMPLEMENTED;

	return gnss_binder_ril_set_set_id((uint16_t) type, set_id) ?
	       NYX_ERROR_NONE : NYX_ERROR_GENERIC;
}

nyx_error_t update_network_state(nyx_device_handle_t handle, int connected,
                                 int type, int roaming, const char *extra_info)
{
	if (handle != nyx_dev)
		return NYX_ERROR_INVALID_HANDLE;

	if (!gnss_binder_ril_available())
		return NYX_ERROR_NOT_IMPLEMENTED;

	/* IAGnssRil@1.0::updateNetworkState has no field for extra_info. */
	(void) extra_info;

	return gnss_binder_ril_update_network_state(connected != 0, type,
	                                            roaming != 0) ?
	       NYX_ERROR_NONE : NYX_ERROR_GENERIC;
}

nyx_error_t update_network_availability(nyx_device_handle_t handle, int available,
                                        const char *apn)
{
	if (handle != nyx_dev)
		return NYX_ERROR_INVALID_HANDLE;

	if (!gnss_binder_ril_available())
		return NYX_ERROR_NOT_IMPLEMENTED;

	return gnss_binder_ril_update_network_availability(available != 0, apn) ?
	       NYX_ERROR_NONE : NYX_ERROR_GENERIC;
}

nyx_error_t inject_xtra_data(nyx_device_handle_t handle, char *data, int length)
{
	if (handle != nyx_dev)
		return NYX_ERROR_INVALID_HANDLE;

	if (!gnss_binder_xtra_available())
		return NYX_ERROR_NOT_IMPLEMENTED;

	return gnss_binder_xtra_inject_data(data, length) ?
	       NYX_ERROR_NONE : NYX_ERROR_GENERIC;
}

nyx_error_t nyx_module_open(nyx_instance_t instance, nyx_device_t **device_ptr)
{
	nyx_error_t error;
	size_t i;

	if (device_ptr == NULL)
		return NYX_ERROR_INVALID_VALUE;

	if (nyx_dev) {
		*device_ptr = nyx_dev;
		return NYX_ERROR_NONE;
	}

	nyx_dev = (nyx_device_t *) calloc(1, sizeof(nyx_device_t));
	if (!nyx_dev)
		return NYX_ERROR_OUT_OF_MEMORY;

	error = nyx_module_set_description(instance, nyx_dev,
	                                   "Module to drive the GPS via the Android GNSS HAL");
	if (error != NYX_ERROR_NONE)
		goto ERROR_HANDLER;

	error = nyx_module_set_name(instance, nyx_dev, "GPS");
	if (error != NYX_ERROR_NONE)
		goto ERROR_HANDLER;

	for (i = 0; i < G_N_ELEMENTS(mapMethodString); i++) {
		error = nyx_module_register_method(instance, nyx_dev,
		                                   mapMethodString[i].mType,
		                                   mapMethodString[i].mString);
		if (error != NYX_ERROR_NONE) {
			nyx_error(MSGID_NYX_HYBRIS_GPS_METHOD_REGISTER_ERR, 0,
			          "Failed to register GPS nyx module method %s",
			          mapMethodString[i].mString);
			goto ERROR_HANDLER;
		}
	}

	for (i = 0; i < G_N_ELEMENTS(mapAssistMethodString); i++) {
		error = nyx_module_register_method(instance, nyx_dev,
		                                   mapAssistMethodString[i].mType,
		                                   mapAssistMethodString[i].mString);
		if (error != NYX_ERROR_NONE) {
			nyx_error(MSGID_NYX_HYBRIS_GPS_METHOD_REGISTER_ERR, 0,
			          "Failed to register GPS nyx module method %s",
			          mapAssistMethodString[i].mString);
			goto ERROR_HANDLER;
		}
	}

	/*
	 * The HAL is bound in init() rather than here. nyx_module_open runs when
	 * anything opens the device, which on a Halium boot can be before
	 * hwservicemanager is serving; binding lazily lets the location service
	 * retry by calling nyx_gps_init again instead of the module being
	 * permanently unavailable because it lost a startup race.
	 */

	*device_ptr = nyx_dev;

	return NYX_ERROR_NONE;

ERROR_HANDLER:
	free(nyx_dev);
	nyx_dev = NULL;

	return error;
}

nyx_error_t nyx_module_close(nyx_device_t *device)
{
	if (device == NULL || device != nyx_dev)
		return NYX_ERROR_INVALID_VALUE;

	gnss_binder_cleanup();
	nyx_gps_cbs = NULL;
	nyx_gps_xtra_cbs = NULL;
	nyx_agps_cbs = NULL;
	nyx_agps_ril_cbs = NULL;

	free(nyx_dev);
	nyx_dev = NULL;

	return NYX_ERROR_NONE;
}
