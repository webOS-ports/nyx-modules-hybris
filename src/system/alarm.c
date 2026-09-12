// Copyright (c) 2014 Simon Busch <morphis@gravedo.de>
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
*************************************************************************
* @file alarm.c
*
* @brief Convenience functions to interact with the Android Alarm driver.
*************************************************************************
*/

#include <linux/rtc.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <stdbool.h>
#include <glib.h>
#include <nyx/nyx_module.h>
#include <nyx/module/nyx_log.h>
#include "msgid.h"
#include "alarm.h"
#include "android_alarm.h"

/**
 * @addtogroup RTCAlarms
 * @{
 */

static int32_t alarm_fd = -1;

static time_t curr_expiry = 0;

/*
 * /dev/alarm is the legacy Android alarm-dev driver. Kernels that dropped
 * drivers/staging/android/alarm-dev.c simply do not have it - upstream removed
 * it in 3.10, and vendor trees diverge even at the same version (a Pixel 3a
 * 4.9 still ships it, a Mi A1 4.9 does not). Its absence is a valid
 * configuration, not an error.
 *
 * Everything here is a *supplementary* deep-sleep hint: rtc.c arms the real
 * wakeup through /dev/rtc0 with RTC_WKALM_SET and only calls us in addition,
 * to "make sure we really wake up when in deep sleep". So when the node is
 * missing, the correct behaviour is to no-op quietly.
 *
 * Without this guard android_alarm_open() failed, left alarm_fd at -1, and
 * every later call still issued ioctl(-1, ...) -> EBADF. On a Mi A1 that meant
 * one "Could not open rtc driver. 2" at startup followed by "Failed to clear
 * alarm" on every RTC watchdog tick, forever.
 */
static bool android_alarm_available(void)
{
	return alarm_fd >= 0;
}

/**
 * @brief Open Android Alarm device.
 *
 */
bool android_alarm_open(void)
{
	if (alarm_fd >= 0)
		return true;

	alarm_fd = open("/dev/alarm", O_RDWR);
	if (alarm_fd < 0) {
		if (errno == ENOENT) {
			/* No alarm-dev on this kernel; rtc.c drives the real
			 * wakeup through /dev/rtc0. Not an error. */
			g_debug("No /dev/alarm on this kernel - Android alarm hints disabled");
		} else {
			g_critical("Could not open /dev/alarm. %d", errno);
		}

		return false;
	}

	return true;
}

/**
* @brief Close Android Alarm device.
*/
void android_alarm_close(void)
{
	if (alarm_fd >= 0)
	{
		close(alarm_fd);
		alarm_fd = -1;
	}
}

/**
* @brief Read the RTC time from the Android Alarm driver.
*/

bool android_alarm_read(struct tm *tm_time)
{
	nyx_debug("%s", __FUNCTION__);

	if (!tm_time)
		return false;

	if (!android_alarm_available())
		return false;

	struct timespec alarm_time = { .tv_sec = 0, .tv_nsec = 0 };

	int32_t ret = ioctl(alarm_fd, ANDROID_ALARM_GET_TIME(ANDROID_ALARM_RTC), &alarm_time);
	if (ret < 0) {
		nyx_warn(MSGID_NYX_HYBRIS_ANDROID_ALARM_GET_TIME_ERR, 0, "ANDROID_ALARM_GET_TIME(ANDROID_ALARM_SYSTEMTIME) ioctl %d", errno);
		return false;
	}

	if (localtime_r(&alarm_time.tv_sec, tm_time) == NULL)
		return false;

	return true;
}

/**
* @brief Read the RTC time and convert it in time_t.
*/

time_t android_alarm_time(time_t *time)
{
	struct tm tm;
	time_t t;

	g_debug("%s", __FUNCTION__);

	if (!android_alarm_read(&tm))
		return -1;

	t = timegm(&tm);

	g_debug("%s: after android_alarm_read %ld", __FUNCTION__, (long) t);

	if (time)
		*time = t;

	return t;
}

/**
* @brief Sets an rtc alarm to fire.
*
* Alarm expiry will be floored at 2 seconds in the future
* (i.e. if expiry = now + 1, alarm will fire at now + 2).
*
* @param  expiry
*
* @retval
*/

bool android_alarm_set(time_t expiry)
{
	time_t now = 0;
	struct timespec wakeup_time = { .tv_sec = 0, .tv_nsec = 0 };
	int rc;

	g_debug("%s", __FUNCTION__);

	if (!android_alarm_available())
		return false;

	if (expiry == curr_expiry)
		return true;

	android_alarm_time(&now);

	if (expiry < now + 2) {
		g_debug("%s: expiry = now + 2", __FUNCTION__);
		expiry = now + 2;
	}

	wakeup_time.tv_sec = expiry;

	rc = ioctl(alarm_fd, ANDROID_ALARM_SET(ANDROID_ALARM_RTC_WAKEUP), &wakeup_time);
	if (rc != 0) {
		g_warning("Failed to set wakeup alarm at %ld (err %d)", expiry, rc);
		return false;
	}

	curr_expiry = expiry;

	return true;
}

/**
* @brief Clear the RTC alarm, if its set.
*/

bool android_alarm_clear(void)
{
	g_debug("%s: clearing...", __FUNCTION__);

	if (!android_alarm_available())
		return false;

	if (ioctl(alarm_fd, ANDROID_ALARM_CLEAR(ANDROID_ALARM_RTC_WAKEUP)) != 0) {
		g_warning("Failed to clear alarm");
		return false;
	}

	curr_expiry = 0;

	return true;
}

/* @} END OF RTCAlarms */
