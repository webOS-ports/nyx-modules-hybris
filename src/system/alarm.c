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

int32_t alarm_fd = -1;

static time_t curr_expiry = 0;

/**
 * @brief Open Android Alarm device.
 *
 */
bool android_alarm_open()
{
	if (alarm_fd >= 0)
		return true;

	alarm_fd = open("/dev/alarm", O_RDWR);
	if (alarm_fd < 0) {
		g_critical("Could not open rtc driver. %d", errno);
		return false;
	}

	return true;
}

/**
* @brief Close Android Alarm device.
*/
void android_alarm_close()
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

	struct timespec alarm_time = { .tv_sec = 0, .tv_nsec = 0 };

	int32_t ret = ioctl(alarm_fd, ANDROID_ALARM_GET_TIME(ANDROID_ALARM_RTC), &alarm_time);
	if (ret < 0) {
		nyx_warn(MSGID_NYX_HYBRIS_ANDROID_ALARM_GET_TIME_ERR, 0, "ANDROID_ALARM_GET_TIME(ANDROID_ALARM_SYSTEMTIME) ioctl %d", errno);
		return false;
	}

	if (localtime_r(&alarm_time.tv_sec, tm_time) == 0)
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

	printf("%s\n", __FUNCTION__);

	if (!android_alarm_read(&tm))
		return -1;

	t = timegm(&tm);

	printf("%s after android_alarm_read %ld\n", __FUNCTION__);

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

	printf("%s\n", __FUNCTION__);

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

	if (ioctl(alarm_fd, ANDROID_ALARM_CLEAR(ANDROID_ALARM_RTC_WAKEUP)) != 0) {
		g_warning("Failed to clear alarm");
		return false;
	}

	return true;
}

/* @} END OF RTCAlarms */
