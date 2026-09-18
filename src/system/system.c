/* @@@LICENSE
*
*      Copyright (c) 2010-2013 LG Electronics, Inc.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
* LICENSE@@@ */

/*
************************************************
* @file system.c
************************************************
*/

#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <stdbool.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <sys/un.h>
#include <glib.h>
#include "rtc.h"
#include <nyx/nyx_module.h>
#include <nyx/common/nyx_macros.h>
#include <nyx/module/nyx_utils.h>
#include "msgid.h"

static nyx_device_t *nyxDev;
static nyx_device_callback_function_t alarm_fired_callback = NULL;

NYX_DECLARE_MODULE(NYX_DEVICE_SYSTEM, "System");

static void AlarmFiredCB(void)
{
	if (alarm_fired_callback)
	{
		alarm_fired_callback(nyxDev, NYX_CALLBACK_STATUS_DONE, NULL);
	}
}

nyx_error_t nyx_module_open(nyx_instance_t i, nyx_device_t **d)
{
	if (nyxDev)
	{
		nyx_info(MSGID_NYX_HYBRIS_SYSTEM_MODULE_OPEN_ERR, 0, "System module already open.");
		return NYX_ERROR_NONE;
	}

	nyxDev = (nyx_device_t *)calloc(1, sizeof(nyx_device_t));

	if (NULL == nyxDev)
	{
		return NYX_ERROR_OUT_OF_MEMORY;
	}

	nyx_module_register_method(i, (nyx_device_t *)nyxDev,
	                           NYX_SYSTEM_SET_ALARM_MODULE_METHOD,
	                           "system_set_alarm");

	nyx_module_register_method(i, (nyx_device_t *)nyxDev,
	                           NYX_SYSTEM_QUERY_NEXT_ALARM_MODULE_METHOD,
	                           "system_query_next_alarm");

	nyx_module_register_method(i, (nyx_device_t *)nyxDev,
	                           NYX_SYSTEM_QUERY_RTC_TIME_MODULE_METHOD,
	                           "system_query_rtc_time");

	nyx_module_register_method(i, (nyx_device_t *)nyxDev,
	                           NYX_SYSTEM_SUSPEND_MODULE_METHOD,
	                           "system_suspend");

	nyx_module_register_method(i, (nyx_device_t *)nyxDev,
	                           NYX_SYSTEM_SUSPEND_ASYNC_MODULE_METHOD,
	                           "system_suspend_async");

	nyx_module_register_method(i, (nyx_device_t *)nyxDev,
	                           NYX_SYSTEM_RESUME_MODULE_METHOD,
	                           "system_resume");

	nyx_module_register_method(i, (nyx_device_t *)nyxDev,
	                           NYX_SYSTEM_SHUTDOWN_MODULE_METHOD,
	                           "system_shutdown");

	nyx_module_register_method(i, (nyx_device_t *)nyxDev,
	                           NYX_SYSTEM_REBOOT_MODULE_METHOD,
	                           "system_reboot");

	nyx_module_register_method(i, (nyx_device_t *)nyxDev,
	                           NYX_SYSTEM_ERASE_PARTITION_MODULE_METHOD,
	                           "system_erase_partition");

	*d = (nyx_device_t *)nyxDev;
	return NYX_ERROR_NONE;
}

nyx_error_t nyx_module_close(nyx_device_t *d)
{
	rtc_close();
	return NYX_ERROR_NONE;
}

nyx_error_t system_set_alarm(nyx_device_handle_t handle, time_t time,
                             nyx_device_callback_function_t callback_func, void *context)
{
	if (handle != nyxDev)
	{
		return NYX_ERROR_INVALID_HANDLE;
	}

	if (rtc_open() == 0)
	{
		return NYX_ERROR_INVALID_OPERATION;
	}

	if (!time)
	{
		rtc_clear_alarm();
	}
	else
	{
		if (rtc_set_alarm_time(time) == 0)
		{
			return NYX_ERROR_INVALID_OPERATION;
		}

		if (callback_func)
		{
			alarm_fired_callback = callback_func;
			rtc_add_watch(AlarmFiredCB);
		}
		else
		{
			alarm_fired_callback = NULL;
			rtc_clear_watch();
		}
	}

	return NYX_ERROR_NONE;
}

nyx_error_t system_query_next_alarm(nyx_device_handle_t handle, time_t *time)
{
	if (handle != nyxDev)
	{
		return NYX_ERROR_INVALID_HANDLE;
	}

	if (rtc_open() == 0)
	{
		return NYX_ERROR_INVALID_OPERATION;
	}

	if (!rtc_read_alarm_time(time))
	{
		return NYX_ERROR_INVALID_OPERATION;
	}

	return NYX_ERROR_NONE;
}

nyx_error_t system_query_rtc_time(nyx_device_handle_t handle, time_t *time)
{
	if (handle != nyxDev)
	{
		return NYX_ERROR_INVALID_HANDLE;
	}

	if (rtc_open() == 0)
	{
		return NYX_ERROR_INVALID_OPERATION;
	}

	if (rtc_time(time) < 0)
	{
		return NYX_ERROR_INVALID_OPERATION;
	}

	return NYX_ERROR_NONE;
}


/*
 * Suspend: one-shot, blocking, with the wakeup_count handshake.
 *
 * sleepd calls nyx_system_suspend_async() from MachineSleep() on its own
 * suspend thread and treats the call as the whole sleep: when it returns the
 * state machine goes straight to kernel-resume (resume signal, MachineWakeup,
 * idle check rescheduled). Both suspend entry points therefore share this one
 * body and block until the kernel has resumed or refused to enter suspend.
 *
 * The handshake mirrors what Android's SystemSuspend does and works the same
 * on kernels with and without PM_AUTOSLEEP:
 *
 *   1. read /sys/power/wakeup_count. The read blocks while any wakeup source
 *      is active - that includes every kernel wakelock in /sys/power/wake_lock,
 *      which is how sleepd activities, IPC clients and the display manager veto
 *      the suspend between the userspace vote and the kernel write. The kernel
 *      logs the active sources ("PM: active wakeup source: ...") while waiting.
 *   2. write the value back. EBUSY (or EINVAL on older kernels) means a wakeup
 *      event raced with us: report "not suspended" and let sleepd retry after
 *      after_resume_idle_ms. That is the retry loop; there is none here.
 *   3. write "mem" to /sys/power/state. Returns 0 once the kernel has resumed;
 *      -EBUSY if a wakeup arrived during entry (also "not suspended").
 *
 * /sys/power/autosleep is never armed: an opportunistic re-suspend loop the
 * kernel runs on its own leaves sleepd unable to tell wake from sleep and,
 * measured on a PinePhone Pro, turns the device into a zombie that re-suspends
 * before userspace can take a wakelock. system_resume() only disarms it, in
 * case something else did.
 *
 * *success = false with NYX_ERROR_NONE means "retry later"; an NYX error is
 * reserved for a bad handle.
 */

#define SYSFS_POWER_STATE   "/sys/power/state"
#define SYSFS_WAKEUP_COUNT  "/sys/power/wakeup_count"
#define SYSFS_AUTOSLEEP     "/sys/power/autosleep"

/* Returns 0, or -errno. */
static int write_sysfs_string(const char *path, const char *value)
{
	ssize_t written;
	int fd = open(path, O_WRONLY | O_CLOEXEC);

	if (fd < 0)
	{
		return -errno;
	}

	written = write(fd, value, strlen(value));

	if (written < 0)
	{
		int err = errno;
		close(fd);
		return -err;
	}

	close(fd);
	return 0;
}

/*
 * Reads the current wakeup_count into buf, stripped of the trailing newline.
 * Returns 0 on success, -errno on failure. Blocks while a wakeup source is
 * active (see above); the kernel returns EINTR if a signal interrupts the wait.
 */
static int read_wakeup_count(char *buf, size_t len)
{
	ssize_t n;
	int fd = open(SYSFS_WAKEUP_COUNT, O_RDONLY | O_CLOEXEC);

	if (fd < 0)
	{
		return -errno;
	}

	n = read(fd, buf, len - 1);

	if (n < 0)
	{
		int err = errno;
		close(fd);
		return -err;
	}

	close(fd);

	while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == ' '))
	{
		n--;
	}

	buf[n] = '\0';
	return (n > 0) ? 0 : -EIO;
}

static double boottime_now(void)
{
	struct timespec ts;

	if (clock_gettime(CLOCK_BOOTTIME, &ts) != 0)
	{
		return 0.0;
	}

	return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

static nyx_error_t suspend_blocking(nyx_device_handle_t handle, bool *success)
{
	char count[32];
	double t0;
	int ret;

	if (handle != nyxDev)
	{
		return NYX_ERROR_INVALID_HANDLE;
	}

	if (success)
	{
		*success = false;
	}

	ret = read_wakeup_count(count, sizeof(count));

	if (ret == -ENOENT)
	{
		/* No wakeup_count on this kernel: no handshake possible, suspend blind. */
		nyx_info(MSGID_NYX_HYBRIS_SYSTEM_SUSPEND, 0,
		         "no " SYSFS_WAKEUP_COUNT ", suspending without the handshake");
	}
	else if (ret < 0)
	{
		nyx_info(MSGID_NYX_HYBRIS_SYSTEM_SUSPEND, 0,
		         "not suspended: reading " SYSFS_WAKEUP_COUNT " failed: %s (%d)",
		         strerror(-ret), -ret);
		return NYX_ERROR_NONE;
	}
	else
	{
		ret = write_sysfs_string(SYSFS_WAKEUP_COUNT, count);

		if (ret < 0)
		{
			/* EBUSY (EINVAL on old kernels): a wakeup event raced us. */
			nyx_info(MSGID_NYX_HYBRIS_SYSTEM_SUSPEND, 0,
			         "not suspended: wakeup_count %s changed under us: %s (%d)",
			         count, strerror(-ret), -ret);
			return NYX_ERROR_NONE;
		}
	}

	t0 = boottime_now();
	ret = write_sysfs_string(SYSFS_POWER_STATE, "mem");

	if (ret < 0)
	{
		nyx_info(MSGID_NYX_HYBRIS_SYSTEM_SUSPEND, 0,
		         "not suspended: writing mem to " SYSFS_POWER_STATE " failed: %s (%d)",
		         strerror(-ret), -ret);
		return NYX_ERROR_NONE;
	}

	nyx_info(MSGID_NYX_HYBRIS_SYSTEM_SUSPEND, 0,
	         "suspended and resumed after %.1f s", boottime_now() - t0);

	if (success)
	{
		*success = true;
	}

	return NYX_ERROR_NONE;
}

nyx_error_t system_suspend(nyx_device_handle_t handle, bool *success)
{
	return suspend_blocking(handle, success);
}

nyx_error_t system_suspend_async(nyx_device_handle_t handle, bool *success)
{
	return suspend_blocking(handle, success);
}

/*
 * Nothing to undo after a one-shot suspend. Disarm autosleep defensively, only
 * where the node exists: an image whose previous nyx build armed it, or anything
 * else that did, would otherwise leave the device unable to stay awake.
 */
nyx_error_t system_resume(nyx_device_handle_t handle, bool *success)
{
	int ret;

	if (handle != nyxDev)
	{
		return NYX_ERROR_INVALID_HANDLE;
	}

	if (access(SYSFS_AUTOSLEEP, W_OK) == 0)
	{
		ret = write_sysfs_string(SYSFS_AUTOSLEEP, "off");

		if (ret < 0)
		{
			nyx_info(MSGID_NYX_HYBRIS_SYSTEM_SUSPEND, 0,
			         "disarming " SYSFS_AUTOSLEEP " failed: %s (%d)",
			         strerror(-ret), -ret);
		}
	}

	if (success)
	{
		*success = true;
	}

	return NYX_ERROR_NONE;
}


/* Runs a shutdown/reboot command and reports whether it succeeded. On
 * success the command typically does not return control to us at all; if
 * it does fail, the device is still running and the caller needs to know. */
static nyx_error_t run_shutdown_command(const char *command)
{
	int status = system(command);

	if (status == -1 || !WIFEXITED(status) || WEXITSTATUS(status) != 0)
	{
		nyx_error(MSGID_NYX_HYBRIS_SYSTEM_SHUTDOWN_CMD_ERR, 0,
		          "Failed to run '%s' (status %d)", command, status);
		return NYX_ERROR_GENERIC;
	}

	return NYX_ERROR_NONE;
}

nyx_error_t system_shutdown(nyx_device_handle_t handle ,
                            nyx_system_shutdown_type_t type, const char *reason)
{
	if (handle != nyxDev)
	{
		return NYX_ERROR_INVALID_HANDLE;
	}

	switch (type)
	{
		case NYX_SYSTEM_EMERG_SHUTDOWN:
			return run_shutdown_command("halt -f");

		case NYX_SYSTEM_NORMAL_SHUTDOWN:
		case NYX_SYSTEM_TEST_SHUTDOWN:
		default:
			return run_shutdown_command("shutdown -h now");
	}
}


nyx_error_t system_reboot(nyx_device_handle_t handle ,
                          nyx_system_shutdown_type_t type, const char *reason)
{
	if (handle != nyxDev)
	{
		return NYX_ERROR_INVALID_HANDLE;
	}

	switch (type)
	{
		case NYX_SYSTEM_EMERG_SHUTDOWN:
			return run_shutdown_command("reboot -f");

		case NYX_SYSTEM_NORMAL_SHUTDOWN:
		case NYX_SYSTEM_TEST_SHUTDOWN:
		default:
			return run_shutdown_command("reboot");
	}
}


nyx_error_t system_erase_partition(nyx_device_handle_t handle,
                                   nyx_system_erase_type_t type)
{
	return NYX_ERROR_NOT_IMPLEMENTED;
}
