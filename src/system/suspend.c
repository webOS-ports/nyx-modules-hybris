/* @@@LICENSE
*
* Copyright (c) 2014 Simon Busch.
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

#include <glib.h>
#include <stdbool.h>
#include <libsuspend.h>
#include <stdio.h>
#include <string.h>

#include "resume_handler.h"

#define WAKEUP_SOURCE_PATH		"/tmp/wakeup_source"

GThread *wakeup_thread = NULL;
GMutex suspend_mutex;
GCond suspend_cond;
bool suspended = false;
GMainContext *suspend_context = NULL;
GMainLoop *suspend_main_loop = NULL;

bool is_system_suspended(void)
{
	return suspended;
}

void wakeup_system(const char *reason, const char *wakelock_to_release)
{
	libsuspend_exit_suspend();

	if (wakelock_to_release)
		libsuspend_release_wake_lock(wakelock_to_release);

	g_message("Waking up the system with reason '%s'", reason);

	g_file_set_contents(WAKEUP_SOURCE_PATH, reason, strlen(reason), NULL);

	g_cond_signal(&suspend_cond);
}

gpointer wakeup_thread_cb(gpointer user_data)
{
	g_message("Starting the wakeup thread ...");

	power_key_resume_handler_init();

	suspend_context = g_main_context_new();

	suspend_main_loop = g_main_loop_new(suspend_context, FALSE);

	g_main_loop_run(suspend_main_loop);

	power_key_resume_handler_release();

	return NULL;
}

bool suspend_init(void)
{
	g_message("Initialization suspend");

	if (wakeup_thread != NULL)
		return false;

	wakeup_thread = g_thread_new(NULL, wakeup_thread_cb, NULL);

	g_cond_init(&suspend_cond);

	libsuspend_init(0);

	return true;
}

bool suspend_enter(void)
{
	g_message("Entering suspend");

	suspended = true;

	libsuspend_prepare_suspend();

	libsuspend_enter_suspend();

	g_cond_wait(&suspend_cond, &suspend_mutex);

	suspended = false;

	g_message("Leaving suspend");

	return true;
}

void suspend_release(void)
{
	g_message("Release suspend");

	g_thread_exit(wakeup_thread);
}
