# Copyright (c) 2026 Herman van Hazendonk <github.com@herrie.org>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

include_directories(.)

# This module reaches the GNSS HAL over binder rather than through libhybris,
# so it needs gbinder rather than -lhardware. See the note at the top of
# gnss_binder.c.
pkg_check_modules(GBINDER REQUIRED libgbinder)
include_directories(${GBINDER_INCLUDE_DIRS})
webos_add_compiler_flags(ALL ${GBINDER_CFLAGS_OTHER})

pkg_check_modules(GLIBUTIL REQUIRED libglibutil)
include_directories(${GLIBUTIL_INCLUDE_DIRS})
webos_add_compiler_flags(ALL ${GLIBUTIL_CFLAGS_OTHER})

# The target name decides the file name, and nyx-lib composes the path it
# dlopen()s as nyx<type><id>.module - it does not scan for providers. The id
# comes from com.webos.service.location's CHIPSET_ID, which defaults to "Main",
# so this has to build nyxGpsMain.module exactly as nyx-modules' serial module
# does. The two never coexist: NYXMOD_OW_GPS selects one or the other.
webos_build_nyx_module(GpsMain
                       SOURCES gps.c gnss_binder.c
                       LIBRARIES ${GLIB2_LDFLAGS} ${PMLOG_LDFLAGS} ${NYXLIB_LDFLAGS}
                                 ${GBINDER_LDFLAGS} ${GLIBUTIL_LDFLAGS})
