/*
 * Copyright (c) 2026 Herman van Hazendonk <github.com@herrie.org>
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
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdint.h>
#include <net/if.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "wifi_hwaddr.h"

#define RECORD_LEN	20
#define MAC_LEN		6

static int valid_ifname(const char *ifname)
{
	size_t len;

	if (ifname == NULL)
		return 0;

	len = strnlen(ifname, IFNAMSIZ);
	if (len == 0 || len >= IFNAMSIZ)
		return 0;

	if (strchr(ifname, '/') != NULL || strcmp(ifname, ".") == 0 ||
	    strcmp(ifname, "..") == 0)
		return 0;

	return 1;
}

/* Only what connman writes: lower-case hex. */
static int hex_value(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	return -1;
}

static int parse_record(const char *buf, size_t len, unsigned char *mac)
{
	unsigned char any = 0;
	size_t i;

	if (len != RECORD_LEN)
		return -EINVAL;

	for (i = 0; i < MAC_LEN; i++) {
		int hi = hex_value(buf[i * 3]);
		int lo = hex_value(buf[i * 3 + 1]);

		if (hi < 0 || lo < 0)
			return -EINVAL;
		if (i < MAC_LEN - 1 && buf[i * 3 + 2] != ':')
			return -EINVAL;

		mac[i] = (unsigned char) ((hi << 4) | lo);
		any |= mac[i];
	}

	if (buf[17] != ' ' || (buf[18] != '0' && buf[18] != '1') ||
	    buf[19] != '\n')
		return -EINVAL;

	/* A hardware address is unicast, and never all zeroes. */
	if ((mac[0] & 0x01) != 0 || any == 0)
		return -EINVAL;

	return 0;
}

static int trusted(const struct stat *st, uid_t owner)
{
	return st->st_uid == owner && (st->st_mode & (S_IWGRP | S_IWOTH)) == 0;
}

static int read_record(int dfd, const char *name, uid_t owner,
		       unsigned char *mac)
{
	char buf[RECORD_LEN + 1];
	struct stat st;
	ssize_t n;
	int fd;

	fd = openat(dfd, name, O_RDONLY | O_NOFOLLOW | O_NONBLOCK |
		    O_NOCTTY | O_CLOEXEC);
	if (fd < 0)
		return errno == ENOENT ? -ENOENT : -EINVAL;

	if (fstat(fd, &st) < 0 || !S_ISREG(st.st_mode) ||
	    !trusted(&st, owner) || st.st_nlink != 1 ||
	    st.st_size != RECORD_LEN) {
		close(fd);
		return -EINVAL;
	}

	/* One byte more than a record, so a file that grew is noticed. */
	do {
		n = read(fd, buf, sizeof(buf));
	} while (n < 0 && errno == EINTR);
	close(fd);

	if (n != RECORD_LEN)
		return -EINVAL;

	return parse_record(buf, (size_t) n, mac);
}

int wifi_hwaddr_read(const char *dir, const char *ifname, uid_t owner,
		     char *out, size_t out_len)
{
	char name[sizeof(WIFI_HWADDR_PREFIX) + IFNAMSIZ];
	unsigned char mac[MAC_LEN];
	struct stat st;
	int dfd, err, len;

	if (dir == NULL || out == NULL || out_len < WIFI_HWADDR_STR_LEN ||
	    !valid_ifname(ifname))
		return -EINVAL;

	len = snprintf(name, sizeof(name), "%s%s", WIFI_HWADDR_PREFIX, ifname);
	if (len < 0 || (size_t) len >= sizeof(name))
		return -EINVAL;

	dfd = open(dir, O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC);
	if (dfd < 0)
		return errno == ENOENT ? -ENOENT : -EINVAL;

	if (fstat(dfd, &st) < 0 || !S_ISDIR(st.st_mode) || !trusted(&st, owner)) {
		close(dfd);
		return -EINVAL;
	}

	err = read_record(dfd, name, owner, mac);
	close(dfd);
	if (err < 0)
		return err;

	len = snprintf(out, out_len, "%02X:%02X:%02X:%02X:%02X:%02X",
		       mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	if (len != WIFI_HWADDR_STR_LEN - 1)
		return -EINVAL;

	return 0;
}

int wifi_hwaddr_get(const char *dir, const char *ifname, uid_t owner,
		    char *out, size_t out_len, wifi_hwaddr_live_fn live)
{
	if (wifi_hwaddr_read(dir, ifname, owner, out, out_len) == 0)
		return WIFI_HWADDR_FROM_RECORD;

	if (live == NULL || ifname == NULL || out == NULL || out_len == 0)
		return -EINVAL;
#if SIZE_MAX > UINT_MAX
	if (out_len > UINT_MAX)
		return -EINVAL;
#endif

	if (live(ifname, out, (unsigned int) out_len) != 0)
		return -ENODEV;

	return WIFI_HWADDR_FROM_LIVE;
}
