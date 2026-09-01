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
 * Backend seam for the hybris GPS nyx module.
 *
 * gps.c speaks nyx and knows nothing about binder; this header is everything it
 * needs from the Android side. The shape deliberately mirrors the legacy
 * GpsInterface/GpsCallbacks pair that nyx's GPS API was modelled on, so the
 * adapter in gps.c stays a straight field-for-field translation.
 *
 * Values passed across this seam use the Android/HIDL constants from
 * <android/hardware/gnss-base.h>, which are numerically identical to their
 * NYX_GPS_* counterparts - that identity is asserted at compile time in gps.c
 * rather than assumed here.
 */

#ifndef _GNSS_BINDER_H_
#define _GNSS_BINDER_H_

#include <glib.h>
#include <stdbool.h>
#include <stdint.h>

G_BEGIN_DECLS

/* Mirrors android.hardware.gnss@1.0::GnssLocation, already converted to host
 * types. Not the wire struct - see gnss_binder.c for that. */
typedef struct {
	uint16_t flags;
	double latitude;
	double longitude;
	double altitude;
	float speed;
	float bearing;
	float horizontal_accuracy;
	float vertical_accuracy;
	int64_t timestamp;
} gnss_binder_location;

typedef struct {
	int16_t svid;
	uint8_t constellation;
	float c_n0_dbhz;
	float elevation;
	float azimuth;
	uint8_t flags;
} gnss_binder_sv_info;

/* GnssMax:SVS_COUNT from android.hardware.gnss@1.0. Note this is twice
 * NYX_GPS_MAX_SVS, so gps.c has to clamp when converting. */
#define GNSS_BINDER_MAX_SVS 64

typedef struct {
	uint32_t num_svs;
	gnss_binder_sv_info sv_list[GNSS_BINDER_MAX_SVS];
} gnss_binder_sv_status;

typedef struct {
	void (*location_cb)(const gnss_binder_location *location, void *user_data);
	void (*status_cb)(uint16_t status, void *user_data);
	void (*sv_status_cb)(const gnss_binder_sv_status *sv_status, void *user_data);
	void (*nmea_cb)(int64_t timestamp, const char *nmea, int length, void *user_data);
	void (*set_capabilities_cb)(uint32_t capabilities, void *user_data);
	void (*acquire_wakelock_cb)(void *user_data);
	void (*release_wakelock_cb)(void *user_data);
	void (*request_utc_time_cb)(void *user_data);
} gnss_binder_callbacks;

/*
 * Binds the GNSS HAL and registers the callback object with it. Returns false
 * if no GNSS service is reachable, which is the normal outcome on a device
 * whose GSI ships no GNSS HAL - the caller should treat that as
 * NYX_ERROR_DEVICE_UNAVAILABLE rather than as a fault.
 */
bool gnss_binder_init(const gnss_binder_callbacks *callbacks, void *user_data);

bool gnss_binder_start(void);
bool gnss_binder_stop(void);
void gnss_binder_cleanup(void);

bool gnss_binder_set_position_mode(uint32_t mode, uint32_t recurrence,
                                   uint32_t min_interval_ms,
                                   uint32_t preferred_accuracy_m,
                                   uint32_t preferred_time_ms);

bool gnss_binder_inject_time(int64_t time_ms, int64_t time_reference_ms,
                             int32_t uncertainty_ms);
bool gnss_binder_inject_location(double latitude, double longitude,
                                 float accuracy_m);
void gnss_binder_delete_aiding_data(uint32_t flags);

/* Human-readable name of the bound HAL, e.g.
 * "android.hardware.gnss@1.1::IGnss/default", or NULL when not bound. Used to
 * answer NYX_GPS_PROVIDER_NAME. */
const char *gnss_binder_provider_name(void);

G_END_DECLS

#endif /* _GNSS_BINDER_H_ */
