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


// NYX_DEVICE_INFO_WIFI_ADDR: connman's hardware address record for the
// interface named by wifi.interface first, the interface's own address
// (SIOCGIFHWADDR) only until there is one. libhybris' property_get() and
// the ioctl are stubbed below.

#include <errno.h>
#include <fcntl.h>
#include <glib.h>
#include <glib/gstdio.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <net/if.h>

#include <nyx/nyx_module.h>
#include <nyx/module/nyx_utils.h>

// The record directory, and the owner a test run can create files as
static char *test_dir;
#define WIFI_HWADDR_DIR   test_dir
#define WIFI_HWADDR_OWNER geteuid()

// What SIOCGIFHWADDR answers
static unsigned char live_mac[6];
static int live_fail;
static int ioctl_calls;
static char ioctl_ifname[IFNAMSIZ];
int test_ioctl(int fd, unsigned long request, ...);
#define ioctl test_ioctl

// Pull in the units under test
#include "../wifi_hwaddr.c"
#include "../device_info.c"

#undef ioctl

int test_ioctl(int fd, unsigned long request, ...)
{
	struct ifreq *ifr;
	va_list ap;

	(void) fd;
	va_start(ap, request);
	ifr = va_arg(ap, struct ifreq *);
	va_end(ap);

	g_assert_true(request == SIOCGIFHWADDR);
	g_strlcpy(ioctl_ifname, ifr->ifr_name, sizeof(ioctl_ifname));
	ioctl_calls++;
	if (live_fail) {
		errno = ENODEV;
		return -1;
	}
	memcpy(ifr->ifr_hwaddr.sa_data, live_mac, sizeof(live_mac));
	return 0;
}

// libhybris: only wifi.interface is set
static char wifi_interface[PROP_VALUE_MAX];

int property_get(const char *key, char *value, const char *default_value)
{
	const char *v = default_value ? default_value : "";

	if (strcmp(key, "wifi.interface") == 0)
		v = wifi_interface;
	g_strlcpy(value, v, PROP_VALUE_MAX);
	return (int) strlen(value);
}

nyx_error_t nyx_module_register_method(nyx_instance_t instance, nyx_device_t *device_in_ptr,
                                       module_method_t method, const char *symbol_str)
{
	(void) instance;
	(void) device_in_ptr;
	(void) method;
	(void) symbol_str;
	return NYX_ERROR_NONE;
}

static nyx_device_t *dev;

static void set_record(const char *ifname, const char *content, mode_t mode)
{
	char *name = g_strconcat("wifi-hwaddr-", ifname, NULL);
	char *path = g_build_filename(test_dir, name, NULL);

	g_unlink(path);
	if (content) {
		int fd = open(path, O_WRONLY | O_CREAT | O_EXCL, 0600);
		g_assert_true(fd >= 0);
		g_assert_true(write(fd, content, strlen(content)) == (ssize_t) strlen(content));
		close(fd);
		g_assert_true(chmod(path, mode) == 0);
	}
	g_free(path);
	g_free(name);
}

// A fresh device and fresh stubs for every case
static void setup(void)
{
	static const unsigned char mac[6] = { 0x02, 0xaa, 0xbb, 0xcc, 0xdd, 0x01 };

	if (dev)
		g_assert_true(nyx_module_close(dev) == NYX_ERROR_NONE);
	dev = NULL;
	g_assert_true(nyx_module_open(NULL, &dev) == NYX_ERROR_NONE);
	set_record("wlan0", NULL, 0);
	set_record("wlan1", NULL, 0);
	g_strlcpy(wifi_interface, "wlan0", sizeof(wifi_interface));
	memcpy(live_mac, mac, sizeof(mac));
	live_fail = 0;
	ioctl_calls = 0;
	ioctl_ifname[0] = '\0';
}

static const char *wifi_addr(nyx_error_t expected)
{
	const char *dest = NULL;

	g_assert_true(device_info_query(dev, NYX_DEVICE_INFO_WIFI_ADDR, &dest) == expected);
	g_assert_nonnull(dest);
	return dest;
}

static void test_record_used(void)
{
	setup();
	set_record("wlan0", "02:11:22:33:44:55 1\n", 0644);
	g_assert_cmpstr(wifi_addr(NYX_ERROR_NONE), ==, "02:11:22:33:44:55");
	g_assert_cmpint(ioctl_calls, ==, 0);
}

static void test_record_follows_property(void)
{
	setup();
	g_strlcpy(wifi_interface, "wlan1", sizeof(wifi_interface));
	set_record("wlan0", "02:11:22:33:44:55 1\n", 0644);
	set_record("wlan1", "02:66:77:88:99:aa 1\n", 0644);
	g_assert_cmpstr(wifi_addr(NYX_ERROR_NONE), ==, "02:66:77:88:99:AA");
}

static void test_no_record_uses_interface(void)
{
	setup();
	g_assert_cmpstr(wifi_addr(NYX_ERROR_NONE), ==, "02:AA:BB:CC:DD:01");
	g_assert_cmpint(ioctl_calls, ==, 1);
	g_assert_cmpstr(ioctl_ifname, ==, "wlan0");
}

static void test_interface_follows_property(void)
{
	setup();
	g_strlcpy(wifi_interface, "wlan1", sizeof(wifi_interface));
	set_record("wlan0", "02:11:22:33:44:55 1\n", 0644);
	g_assert_cmpstr(wifi_addr(NYX_ERROR_NONE), ==, "02:AA:BB:CC:DD:01");
	g_assert_cmpstr(ioctl_ifname, ==, "wlan1");
}

static void test_bad_record_uses_interface(void)
{
	setup();
	set_record("wlan0", "02:11:22:33:44:55 7\n", 0644);
	g_assert_cmpstr(wifi_addr(NYX_ERROR_NONE), ==, "02:AA:BB:CC:DD:01");
	g_assert_cmpint(ioctl_calls, ==, 1);

	setup();
	set_record("wlan0", "02:11:22:33:44:55 1\n", 0666);
	g_assert_cmpstr(wifi_addr(NYX_ERROR_NONE), ==, "02:AA:BB:CC:DD:01");
}

// The interface carries a network's own address while connected, so
// without a record it is asked again each time rather than kept.
static void test_interface_not_kept(void)
{
	setup();
	g_assert_cmpstr(wifi_addr(NYX_ERROR_NONE), ==, "02:AA:BB:CC:DD:01");
	live_mac[5] = 0x02;
	g_assert_cmpstr(wifi_addr(NYX_ERROR_NONE), ==, "02:AA:BB:CC:DD:02");
	g_assert_cmpint(ioctl_calls, ==, 2);
}

static void test_record_appearing_later(void)
{
	setup();
	g_assert_cmpstr(wifi_addr(NYX_ERROR_NONE), ==, "02:AA:BB:CC:DD:01");
	set_record("wlan0", "02:11:22:33:44:55 1\n", 0644);
	g_assert_cmpstr(wifi_addr(NYX_ERROR_NONE), ==, "02:11:22:33:44:55");
}

static void test_record_kept_once_read(void)
{
	setup();
	set_record("wlan0", "02:11:22:33:44:55 1\n", 0644);
	g_assert_cmpstr(wifi_addr(NYX_ERROR_NONE), ==, "02:11:22:33:44:55");
	set_record("wlan0", NULL, 0);
	g_assert_cmpstr(wifi_addr(NYX_ERROR_NONE), ==, "02:11:22:33:44:55");
	g_assert_cmpint(ioctl_calls, ==, 0);
}

static void test_pointer_stays_valid(void)
{
	const char *first, *second;

	setup();
	first = wifi_addr(NYX_ERROR_NONE);
	set_record("wlan0", "02:11:22:33:44:55 1\n", 0644);
	second = wifi_addr(NYX_ERROR_NONE);
	g_assert_true(first == second);
	g_assert_cmpstr(first, ==, "02:11:22:33:44:55");
}

static void test_no_wifi_interface(void)
{
	setup();
	wifi_interface[0] = '\0';
	set_record("wlan0", "02:11:22:33:44:55 1\n", 0644);
	g_assert_cmpstr(wifi_addr(NYX_ERROR_NONE), ==, "");
	g_assert_cmpint(ioctl_calls, ==, 0);
}

static void test_interface_failure(void)
{
	setup();
	live_fail = 1;
	g_assert_cmpstr(wifi_addr(NYX_ERROR_INVALID_OPERATION), ==, "");
	live_fail = 0;
	g_assert_cmpstr(wifi_addr(NYX_ERROR_NONE), ==, "02:AA:BB:CC:DD:01");
}

static void test_get_info(void)
{
	char buf[18], small[17];

	setup();
	set_record("wlan0", "02:11:22:33:44:55 1\n", 0644);
	g_assert_true(device_info_get_info(dev, NYX_DEVICE_INFO_WIFI_ADDR,
	                                   buf, sizeof(buf)) == NYX_ERROR_NONE);
	g_assert_cmpstr(buf, ==, "02:11:22:33:44:55");
	g_assert_true(device_info_get_info(dev, NYX_DEVICE_INFO_WIFI_ADDR,
	                                   small, sizeof(small)) == NYX_ERROR_VALUE_OUT_OF_RANGE);
}

int main(int argc, char **argv)
{
	int ret;

	g_test_init(&argc, &argv, NULL);
	test_dir = g_dir_make_tmp("test-device-info-XXXXXX", NULL);
	g_assert_nonnull(test_dir);
	g_assert_true(chmod(test_dir, 0755) == 0);

	g_test_add_func("/device_info/wifi_addr/record_used", test_record_used);
	g_test_add_func("/device_info/wifi_addr/record_follows_property",
	                test_record_follows_property);
	g_test_add_func("/device_info/wifi_addr/no_record_uses_interface",
	                test_no_record_uses_interface);
	g_test_add_func("/device_info/wifi_addr/interface_follows_property",
	                test_interface_follows_property);
	g_test_add_func("/device_info/wifi_addr/bad_record_uses_interface",
	                test_bad_record_uses_interface);
	g_test_add_func("/device_info/wifi_addr/interface_not_kept", test_interface_not_kept);
	g_test_add_func("/device_info/wifi_addr/record_appearing_later",
	                test_record_appearing_later);
	g_test_add_func("/device_info/wifi_addr/record_kept_once_read",
	                test_record_kept_once_read);
	g_test_add_func("/device_info/wifi_addr/pointer_stays_valid", test_pointer_stays_valid);
	g_test_add_func("/device_info/wifi_addr/no_wifi_interface", test_no_wifi_interface);
	g_test_add_func("/device_info/wifi_addr/interface_failure", test_interface_failure);
	g_test_add_func("/device_info/wifi_addr/get_info", test_get_info);
	ret = g_test_run();

	if (dev)
		nyx_module_close(dev);
	set_record("wlan0", NULL, 0);
	set_record("wlan1", NULL, 0);
	g_rmdir(test_dir);
	g_free(test_dir);
	return ret;
}
