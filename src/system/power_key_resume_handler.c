/*
 *  Copyright (C) 2012 Simon Busch <morphis@gravedo.de>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>

#include <glib.h>

#include <linux/input.h>
#include <libevdev/libevdev.h>

#include <libsuspend.h>

#include "resume_handler.h"

static int input_source_fd = 0;
static GIOChannel *channel = NULL;
static int readwatch = 0;

gboolean _handle_input_event(GIOChannel *channel, GIOCondition condition, gpointer data)
{
	int bytesread;
	int wakeup = 0;
	struct input_event ev;

	if ((condition  & G_IO_IN) != G_IO_IN)
		return TRUE;

	if (!is_system_suspended())
		return TRUE;

	libsuspend_acquire_wake_lock("wakelockd_handle_input_event");

	bytesread = read(input_source_fd, &ev, sizeof(struct input_event));
	if (bytesread == 0) {
		g_warning("Got some input event but could not read anything -> waking up the system!");
		wakeup = 1;
	}
	else if (ev.code == KEY_POWER) {
		g_debug("Got power key input event -> waking up the system!");
		wakeup = 1;
	}
	else {
		g_debug("Got some other input event but not the power key -> NOT wakeing up");
	}

	if (wakeup)
		wakeup_system("power_key", "wakelockd_handle_input_event");
	else
		libsuspend_release_wake_lock("wakelockd_handle_input_event");

	return TRUE;
}

int power_key_resume_handler_init(void)
{
	const char *node_path;
    char *full_path;
	GDir *input_dir;
	int rc = 1;
	struct libevdev *dev = NULL;

	g_message("Setting up power key resume handler ...");

	input_dir = g_dir_open("/dev/input", 0, NULL);
	if (!input_dir) {
		g_warning("Failed to reach /dev/input directory");
		return -ENODEV;
	}

	while ((node_path = g_dir_read_name(input_dir)) != NULL) {
		full_path = g_strdup_printf("/dev/input/%s", node_path);

		if (g_file_test(node_path, G_FILE_TEST_IS_DIR))
			continue;

		g_message("Opening %s", full_path);

		input_source_fd = open(full_path, O_RDONLY|O_NONBLOCK);
		g_free(full_path);

		if (input_source_fd < 0)
			continue;

		rc = libevdev_new_from_fd(input_source_fd, &dev);
		if (rc < 0) {
			fprintf(stderr, "Failed to init libevdev (%s)\n", strerror(-rc));
			close(input_source_fd);
			input_source_fd = -1;
			continue;
		}

		if (libevdev_has_event_code(dev, EV_KEY, KEY_POWER)) {
			libevdev_free(dev);
			break;
		}

		libevdev_free(dev);
		close(input_source_fd);
		input_source_fd = -1;
	}

	if (input_source_fd < 0)
		return -ENODEV;

	channel = g_io_channel_unix_new(input_source_fd);
	g_io_channel_set_encoding(channel, NULL, NULL);
	readwatch = g_io_add_watch(channel, G_IO_IN | G_IO_HUP | G_IO_NVAL, _handle_input_event, NULL);

	return 0;
}

void power_key_resume_handler_release(void)
{
	if (readwatch > 0)
		g_source_remove(readwatch);

	if (channel != NULL)
		g_io_channel_unref(channel);

	if (input_source_fd > 0)
		close(input_source_fd);
}

// vim:ts=4:sw=4:noexpandtab
