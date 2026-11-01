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


// wifi_hwaddr.c: the reader of connman's hardware address record.
// fstat() and read() are routed through the hooks below so that an owner,
// a short or grown read and EINTR can be simulated.

#include <errno.h>
#include <fcntl.h>
#include <glib.h>
#include <glib/gstdio.h>
#include <limits.h>
#include <net/if.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static long fake_reg_uid = -1;	// >= 0: st_uid reported for regular files
static long read_limit = -1;	// >= 0: at most this many bytes per read
static int read_eintr;		// reads that fail with EINTR first
static int read_extra;		// one more byte than the file has
static int read_calls;

static int test_fstat(int fd, struct stat *st);
static ssize_t test_read(int fd, void *buf, size_t n);
#define fstat test_fstat
#define read  test_read

// Pull in the unit under test
#include "../wifi_hwaddr.c"

#undef fstat
#undef read

static int test_fstat(int fd, struct stat *st)
{
	int r = fstat(fd, st);

	if (r == 0 && S_ISREG(st->st_mode) && fake_reg_uid >= 0)
		st->st_uid = (uid_t) fake_reg_uid;
	return r;
}

static ssize_t test_read(int fd, void *buf, size_t n)
{
	ssize_t r;

	read_calls++;
	if (read_eintr > 0) {
		read_eintr--;
		errno = EINTR;
		return -1;
	}
	if (read_limit >= 0 && n > (size_t) read_limit)
		n = (size_t) read_limit;
	r = read(fd, buf, n);
	if (read_extra && r >= 0 && (size_t) r < n) {
		((char *) buf)[r] = 'x';
		r++;
	}
	return r;
}

static char *dir;

static char *record_path(const char *ifname)
{
	return g_strdup_printf("%s/wifi-hwaddr-%s", dir, ifname);
}

static void reset(void)
{
	GDir *d = g_dir_open(dir, 0, NULL);
	const char *name;

	while (d && (name = g_dir_read_name(d)) != NULL) {
		char *p = g_build_filename(dir, name, NULL);
		if (g_rmdir(p) != 0)
			g_unlink(p);
		g_free(p);
	}
	if (d)
		g_dir_close(d);
	g_assert(chmod(dir, 0755) == 0);
	fake_reg_uid = -1;
	read_limit = -1;
	read_eintr = 0;
	read_extra = 0;
	read_calls = 0;
}

static void put(const char *ifname, const char *content, size_t len, mode_t mode)
{
	char *p = record_path(ifname);
	int fd = open(p, O_WRONLY | O_CREAT | O_EXCL, 0600);

	g_assert(fd >= 0);
	g_assert(write(fd, content, len) == (ssize_t) len);
	close(fd);
	g_assert(chmod(p, mode) == 0);
	g_free(p);
}

static void record(const char *content)
{
	reset();
	put("wlan0", content, strlen(content), 0644);
}

static int get(char *out)
{
	return wifi_hwaddr_read(dir, "wlan0", geteuid(), out, WIFI_HWADDR_STR_LEN);
}

// A failure must leave out alone.
static void expect_error(int expected)
{
	char out[WIFI_HWADDR_STR_LEN] = "unchanged";

	g_assert_cmpint(get(out), ==, expected);
	g_assert_cmpstr(out, ==, "unchanged");
}

static void test_valid(void)
{
	char out[WIFI_HWADDR_STR_LEN];

	record("02:1a:2b:3c:4d:5e 1\n");
	g_assert_cmpint(get(out), ==, 0);
	g_assert_cmpstr(out, ==, "02:1A:2B:3C:4D:5E");

	record("00:00:00:00:00:01 0\n");
	g_assert_cmpint(get(out), ==, 0);
	g_assert_cmpstr(out, ==, "00:00:00:00:00:01");
	g_assert_cmpint(read_calls, ==, 1);
}

static void test_missing(void)
{
	char *missing = g_build_filename(dir, "nope", NULL);
	char out[WIFI_HWADDR_STR_LEN];

	reset();
	expect_error(-ENOENT);
	g_assert_cmpint(wifi_hwaddr_read(missing, "wlan0", geteuid(), out, sizeof(out)), ==, -ENOENT);
	put("wlan1", "02:11:22:33:44:55 1\n", 20, 0644);
	expect_error(-ENOENT);
	g_free(missing);
}

static void test_not_followed(void)
{
	char *target = record_path("target"), *p = record_path("wlan0");
	char *alias = g_strconcat(dir, "-link", NULL);
	char out[WIFI_HWADDR_STR_LEN];

	reset();
	put("target", "02:11:22:33:44:55 1\n", 20, 0644);
	g_assert(symlink(target, p) == 0);
	expect_error(-EINVAL);

	g_unlink(p);
	g_assert(link(target, p) == 0);	// a hard link
	expect_error(-EINVAL);

	g_unlink(p);
	g_assert(g_mkdir(p, 0755) == 0);
	expect_error(-EINVAL);

	g_rmdir(p);
	g_assert(mkfifo(p, 0644) == 0);	// must not block
	expect_error(-EINVAL);

	record("02:11:22:33:44:55 1\n");
	g_unlink(alias);
	g_assert(symlink(dir, alias) == 0);
	g_assert_cmpint(wifi_hwaddr_read(alias, "wlan0", geteuid(), out, sizeof(out)), ==, -EINVAL);
	g_unlink(alias);
	g_free(target);
	g_free(p);
	g_free(alias);
}

static void test_owner_and_mode(void)
{
	char out[WIFI_HWADDR_STR_LEN];
	char *p;

	record("02:11:22:33:44:55 1\n");
	p = record_path("wlan0");
	g_assert(chmod(p, 0664) == 0);
	expect_error(-EINVAL);
	g_assert(chmod(p, 0646) == 0);
	expect_error(-EINVAL);
	g_assert(chmod(p, 0600) == 0);
	g_assert_cmpint(get(out), ==, 0);

	g_assert(chmod(dir, 0775) == 0);
	expect_error(-EINVAL);
	g_assert(chmod(dir, 0757) == 0);
	expect_error(-EINVAL);
	g_assert(chmod(dir, 0755) == 0);

	g_assert_cmpint(wifi_hwaddr_read(dir, "wlan0", geteuid() + 1, out, sizeof(out)), ==, -EINVAL);
	fake_reg_uid = (long) geteuid() + 1;
	expect_error(-EINVAL);
	g_free(p);
}

static void test_size(void)
{
	static const char *const bad[] = {
		"", "02:11:22:33:44:55 1", "02:11:22:33:44:55 1\n\n",
		"2:11:22:33:44:55 1\n", "02:11:22:33:44:55  1\n",
	};
	char big[4096];
	size_t i;

	for (i = 0; i < G_N_ELEMENTS(bad); i++) {
		record(bad[i]);
		expect_error(-EINVAL);
	}
	memset(big, 'a', sizeof(big));
	reset();
	put("wlan0", big, sizeof(big), 0644);
	expect_error(-EINVAL);

	record("02:11:22:33:44:55 1\n");
	read_extra = 1;		// grew after fstat()
	expect_error(-EINVAL);

	record("02:11:22:33:44:55 1\n");
	read_limit = 19;	// a short read is not retried into a record
	expect_error(-EINVAL);

	reset();
	put("wlan0", "02:11:22:33:44:55 1\nxx", 22, 0644);
	read_limit = 20;	// a longer file whose read stops at a record
	expect_error(-EINVAL);
}

static void test_eintr(void)
{
	char out[WIFI_HWADDR_STR_LEN];

	record("02:11:22:33:44:55 1\n");
	read_eintr = 2;
	g_assert_cmpint(get(out), ==, 0);
	g_assert_cmpint(read_calls, ==, 3);
}

static void test_format(void)
{
	static const char *const bad[] = {
		"02:1A:2B:3C:4D:5E 1\n", "02-11-22-33-44-55 1\n",
		"02:11:22:33:44:55:1\n", "02:11:22:33:44:55 2\n",
		"02:11:22:33:44:55 1 ", "0g:11:22:33:44:55 1\n",
		" 2:11:22:33:44:55 1\n", "+2:11:22:33:44:55 1\n",
		"02:11:22:33:44:55\n1\n",
		"01:11:22:33:44:55 1\n", "ff:ff:ff:ff:ff:ff 1\n",
		"00:00:00:00:00:00 1\n",
	};
	size_t i;

	for (i = 0; i < G_N_ELEMENTS(bad); i++) {
		g_assert_cmpuint(strlen(bad[i]), ==, 20);
		record(bad[i]);
		expect_error(-EINVAL);
	}
	reset();
	put("wlan0", "02:11\00022:33:44:55 1\n", 20, 0644);
	expect_error(-EINVAL);
}

static void test_arguments(void)
{
	static const char *const names[] = { "", "a/b", ".", "..", "0123456789abcdef" };
	char out[WIFI_HWADDR_STR_LEN] = "unchanged";
	char ok[64];
	size_t i;

	record("02:11:22:33:44:55 1\n");
	for (i = 0; i < G_N_ELEMENTS(names); i++)
		g_assert_cmpint(wifi_hwaddr_read(dir, names[i], geteuid(), out, sizeof(out)), ==, -EINVAL);
	g_assert_cmpint(wifi_hwaddr_read(dir, NULL, geteuid(), out, sizeof(out)), ==, -EINVAL);
	g_assert_cmpint(wifi_hwaddr_read(NULL, "wlan0", geteuid(), out, sizeof(out)), ==, -EINVAL);
	g_assert_cmpint(wifi_hwaddr_read(dir, "wlan0", geteuid(), out, WIFI_HWADDR_STR_LEN - 1), ==, -EINVAL);
	g_assert_cmpstr(out, ==, "unchanged");
	g_assert_cmpint(wifi_hwaddr_read(dir, "wlan0", geteuid(), ok, sizeof(ok)), ==, 0);

	reset();
	put("0123456789abcde", "02:11:22:33:44:55 1\n", 20, 0644);
	g_assert_cmpint(wifi_hwaddr_read(dir, "0123456789abcde", geteuid(), ok, sizeof(ok)), ==, 0);
}

static int live_calls, live_result;

static int fake_live(const char *ifname, char *out, unsigned int len)
{
	g_assert_cmpstr(ifname, ==, "wlan0");
	live_calls++;
	if (live_result)
		return live_result;
	g_strlcpy(out, "AA:BB:CC:DD:EE:FF", len);
	return 0;
}

static void test_get(void)
{
	char out[32];

	record("02:11:22:33:44:55 1\n");
	live_calls = 0; live_result = 0;
	g_assert_cmpint(wifi_hwaddr_get(dir, "wlan0", geteuid(), out, sizeof(out), fake_live), ==, WIFI_HWADDR_FROM_RECORD);
	g_assert_cmpstr(out, ==, "02:11:22:33:44:55");
	g_assert_cmpint(live_calls, ==, 0);

	record("01:11:22:33:44:55 1\n");
	g_assert_cmpint(wifi_hwaddr_get(dir, "wlan0", geteuid(), out, sizeof(out), fake_live), ==, WIFI_HWADDR_FROM_LIVE);
	g_assert_cmpstr(out, ==, "AA:BB:CC:DD:EE:FF");
	g_assert_cmpint(live_calls, ==, 1);

	reset();
	live_result = -1;
	g_assert_cmpint(wifi_hwaddr_get(dir, "wlan0", geteuid(), out, sizeof(out), fake_live), <, 0);
	g_assert_cmpint(wifi_hwaddr_get(dir, "wlan0", geteuid(), out, sizeof(out), NULL), <, 0);
	g_assert_cmpint(wifi_hwaddr_get(dir, "wlan0", geteuid(), out, 0, fake_live), <, 0);
	g_assert_cmpint(live_calls, ==, 2);
}

int main(int argc, char **argv)
{
	int ret;

	g_test_init(&argc, &argv, NULL);
	dir = g_dir_make_tmp("test-wifi-hwaddr-XXXXXX", NULL);
	g_assert(dir != NULL);

	g_test_add_func("/wifi_hwaddr/valid", test_valid);
	g_test_add_func("/wifi_hwaddr/missing", test_missing);
	g_test_add_func("/wifi_hwaddr/not_followed", test_not_followed);
	g_test_add_func("/wifi_hwaddr/owner_and_mode", test_owner_and_mode);
	g_test_add_func("/wifi_hwaddr/size", test_size);
	g_test_add_func("/wifi_hwaddr/eintr", test_eintr);
	g_test_add_func("/wifi_hwaddr/format", test_format);
	g_test_add_func("/wifi_hwaddr/arguments", test_arguments);
	g_test_add_func("/wifi_hwaddr/get", test_get);
	ret = g_test_run();

	reset();
	g_rmdir(dir);
	g_free(dir);
	return ret;
}
