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

#ifndef WIFI_HWADDR_H
#define WIFI_HWADDR_H

#include <stddef.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * With WiFiMACRandomization on, connman joins every network with an
 * address of its own, so the interface address is not the device's while
 * it is connected. connman records the hardware address once per boot,
 * before it changes anything, in
 *
 *   /run/connman/wifi-hwaddr-<ifname>
 *
 * a root-owned regular file, never group- or world-writable, of exactly
 * 20 bytes: six lower-case two-digit hex octets separated by ':', a
 * space, '0' or '1' (whether the driver lets the address change) and
 * '\n', e.g. "02:11:22:33:44:55 1\n". The format is documented as an
 * interface in connman's src/wifi-mac.h (LuneOS connman patch 0008).
 * This file is shared, unchanged, by nyx-modules, nyx-modules-hybris and
 * webos-connman-adapter.
 */
#ifndef WIFI_HWADDR_DIR
#define WIFI_HWADDR_DIR		"/run/connman"
#endif
#ifndef WIFI_HWADDR_OWNER
#define WIFI_HWADDR_OWNER	((uid_t) 0)
#endif
#define WIFI_HWADDR_PREFIX	"wifi-hwaddr-"

/* "AA:BB:CC:DD:EE:FF" and its NUL */
#define WIFI_HWADDR_STR_LEN	18

/*
 * Read the hardware address of ifname that connman recorded under dir,
 * and write it to out as "AA:BB:CC:DD:EE:FF" (upper case, like the
 * SIOCGIFHWADDR lookups it stands in for). The directory and the record
 * must be owned by owner and not group- or world-writable, the record
 * must be a regular file (never followed through a symlink) with one link
 * and exactly the format above, and the address must be a unicast one
 * other than 00:00:00:00:00:00.
 *
 * Returns 0, -ENOENT when there is no record, -EINVAL when the arguments
 * or the record are not acceptable, or another negative errno. out is
 * only written on success.
 */
int wifi_hwaddr_read(const char *dir, const char *ifname, uid_t owner,
		     char *out, size_t out_len);

/* A live lookup, e.g. SIOCGIFHWADDR: 0 and out filled, or non-zero. */
typedef int (*wifi_hwaddr_live_fn)(const char *ifname, char *out,
				   unsigned int out_len);

#define WIFI_HWADDR_FROM_RECORD	0
#define WIFI_HWADDR_FROM_LIVE	1

/*
 * The hardware address of ifname: connman's record when there is a valid
 * one, otherwise whatever live() finds. Returns WIFI_HWADDR_FROM_RECORD,
 * WIFI_HWADDR_FROM_LIVE, or a negative value when neither worked.
 */
int wifi_hwaddr_get(const char *dir, const char *ifname, uid_t owner,
		    char *out, size_t out_len, wifi_hwaddr_live_fn live);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_HWADDR_H */
