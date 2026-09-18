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
 *      the suspend between the userspace vote and the kernel write. Unlike
 *      Android we do not wait indefinitely: nothing in LuneOS holds a wakelock
 *      for "the screen is on", so a suspend that parked here for as long as,
 *      say, the USB controller's wakeup source stays active (the whole time a
 *      cable is plugged in) would fire the instant the cable is pulled, with
 *      the user looking at the screen. The read is bounded to
 *      WAKEUP_COUNT_WAIT_MS - long enough to absorb the short wakelocks an
 *      interrupt handler holds, and on the scale of sleepd's own retry
 *      interval (after_resume_idle_ms, 1 s by default). Past that we report
 *      "not suspended" naming the sources still active and let sleepd re-run
 *      its policy before trying again.
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

/* How long to wait for the kernel's wakeup sources to go quiet. */
#define WAKEUP_COUNT_WAIT_MS 1000

struct wakeup_count_read
{
	pthread_mutex_t lock;
	pthread_cond_t cond;
	bool done;
	int result;     /* 0, or -errno */
	char buf[32];   /* the count, newline stripped */
};

static void close_fd_cleanup(void *arg)
{
	close(*(int *)arg);
}

/*
 * Helper thread: the sysfs read blocks (interruptibly) while a wakeup source
 * is active, so it runs here and the caller times it out with pthread_cancel.
 * read() is the cancellation point; the fd is closed by the cleanup handler
 * if the thread is unwound there.
 */
static void *wakeup_count_reader(void *arg)
{
	struct wakeup_count_read *r = arg;
	int ret;
	int fd = open(SYSFS_WAKEUP_COUNT, O_RDONLY | O_CLOEXEC);

	if (fd < 0)
	{
		ret = -errno;
	}
	else
	{
		ssize_t n;
		int old_state;

		pthread_cleanup_push(close_fd_cleanup, &fd);
		n = read(fd, r->buf, sizeof(r->buf) - 1);
		ret = (n < 0) ? -errno : 0;
		/* Past the blocking read: finish and report even if a cancel is pending. */
		pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, &old_state);
		pthread_cleanup_pop(1);

		if (ret == 0)
		{
			while (n > 0 && (r->buf[n - 1] == '\n' || r->buf[n - 1] == ' '))
			{
				n--;
			}

			r->buf[n] = '\0';
			ret = (n > 0) ? 0 : -EIO;
		}
	}

	pthread_mutex_lock(&r->lock);
	r->result = ret;
	r->done = true;
	pthread_cond_signal(&r->cond);
	pthread_mutex_unlock(&r->lock);
	return NULL;
}

/*
 * Reads the current wakeup_count into buf, waiting at most wait_ms for the
 * kernel's wakeup sources to go quiet. Returns 0 on success, -ETIMEDOUT if a
 * source was still active when the wait ran out, or -errno.
 */
static int read_wakeup_count(char *buf, size_t len, unsigned int wait_ms)
{
	struct wakeup_count_read r = { .done = false, .result = -EIO, .buf = "" };
	pthread_condattr_t cattr;
	pthread_t tid;
	struct timespec deadline;
	int ret;

	pthread_mutex_init(&r.lock, NULL);
	pthread_condattr_init(&cattr);
	pthread_condattr_setclock(&cattr, CLOCK_MONOTONIC);
	pthread_cond_init(&r.cond, &cattr);
	pthread_condattr_destroy(&cattr);

	ret = pthread_create(&tid, NULL, wakeup_count_reader, &r);

	if (ret != 0)
	{
		pthread_cond_destroy(&r.cond);
		pthread_mutex_destroy(&r.lock);
		return -ret;
	}

	clock_gettime(CLOCK_MONOTONIC, &deadline);
	deadline.tv_sec += wait_ms / 1000;
	deadline.tv_nsec += (long)(wait_ms % 1000) * 1000000L;

	if (deadline.tv_nsec >= 1000000000L)
	{
		deadline.tv_sec++;
		deadline.tv_nsec -= 1000000000L;
	}

	pthread_mutex_lock(&r.lock);

	while (!r.done)
	{
		if (pthread_cond_timedwait(&r.cond, &r.lock, &deadline) == ETIMEDOUT)
		{
			break;
		}
	}

	pthread_mutex_unlock(&r.lock);

	if (!r.done)
	{
		pthread_cancel(tid);
	}

	pthread_join(tid, NULL);
	pthread_cond_destroy(&r.cond);
	pthread_mutex_destroy(&r.lock);

	/* r.done may have flipped between the timeout and the cancel. */
	if (!r.done)
	{
		return -ETIMEDOUT;
	}

	if (r.result == 0)
	{
		g_strlcpy(buf, r.buf, len);
	}

	return r.result;
}

/*
 * Names the wakeup sources that are active right now, comma separated, for
 * the log. Best effort: /sys/class/wakeup (5.4+) first, then the debugfs
 * table older kernels have; an empty string if neither is readable.
 */
static void active_wakeup_sources(char *out, size_t len)
{
	const char *dir_path = "/sys/class/wakeup";
	GDir *dir;
	FILE *f;
	char line[512];

	out[0] = '\0';
	dir = g_dir_open(dir_path, 0, NULL);

	if (dir)
	{
		const char *entry;

		while ((entry = g_dir_read_name(dir)) != NULL)
		{
			char *path = g_build_filename(dir_path, entry, "active_time_ms", NULL);
			char *contents = NULL;
			char *name = NULL;

			if (g_file_get_contents(path, &contents, NULL, NULL) &&
			        g_ascii_strtoll(contents, NULL, 10) > 0)
			{
				char *name_path = g_build_filename(dir_path, entry, "name", NULL);
				g_file_get_contents(name_path, &name, NULL, NULL);
				g_free(name_path);
			}

			if (name)
			{
				g_strchomp(name);
				g_strlcat(out, out[0] ? "," : "", len);
				g_strlcat(out, name, len);
			}

			g_free(name);
			g_free(contents);
			g_free(path);
		}

		g_dir_close(dir);
		return;
	}

	f = fopen("/sys/kernel/debug/wakeup_sources", "r");

	if (!f)
	{
		return;
	}

	/* name active_count event_count wakeup_count expire_count active_since ... */
	while (fgets(line, sizeof(line), f))
	{
		char name[128];
		unsigned long long active_since;

		if (sscanf(line, "%127s %*u %*u %*u %*u %llu", name, &active_since) == 2 &&
		        active_since != 0)
		{
			g_strlcat(out, out[0] ? "," : "", len);
			g_strlcat(out, name, len);
		}
	}

	fclose(f);
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

	ret = read_wakeup_count(count, sizeof(count), WAKEUP_COUNT_WAIT_MS);

	if (ret == -ETIMEDOUT)
	{
		char active[256];

		active_wakeup_sources(active, sizeof(active));
		nyx_info(MSGID_NYX_HYBRIS_SYSTEM_SUSPEND, 0,
		         "not suspended: wakeup source still active after %u ms: %s",
		         WAKEUP_COUNT_WAIT_MS, active[0] ? active : "(unknown)");
		return NYX_ERROR_NONE;
	}
	else if (ret == -ENOENT)
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
