# Copyright (c) 2010-2018 LG Electronics, Inc.
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

cmake_minimum_required(VERSION 2.8.7)

project(nyx-modules-hybris)

include(FindPkgConfig)

include(webOS/webOS)

webos_modules_init(1 6 2)
webos_component(1 0 0)
webos_include_install_paths()

pkg_check_modules(PMLOG REQUIRED PmLogLib)
include_directories(${PMLOG_INCLUDE_DIRS})
webos_add_compiler_flags(ALL ${PMLOG_CFLAGS_OTHER})

webos_nyx_module_provider(HYBRIS DEVICEINFO HAPTICS LEDCONTROLLER SYSTEM GPS)

# src/CMakeLists.txt decides what to build from the *other* provider's flags -
# it builds a module exactly when nyx-modules does not - so parse those too.
#
# They used to arrive in a machine.cmake copied in from nyx-modules' files
# directory, which meant the two recipes could describe the same machine
# differently and either ship a module twice or not at all. Both now read
# NYX_MODULES_REQUIRED, from one shared .inc.
#
# NYXMOD_OW_LEDTORCH is deliberately absent: _webos_nyx_parse_modules() sets a
# listed module FALSE when it is not in the list, and FALSE here means "the
# hybris camera-service torch", which pulls in droidmedia. Leaving it unparsed
# keeps it undefined, which src/CMakeLists.txt defaults to TRUE - the sysfs
# torch in nyx-modules - the same way nyx-modules defaults it.
webos_nyx_module_provider(OW DEVICEINFO SYSTEM LED HAPTICS GPS)
webos_test_provider(GLIB_TEST)
webos_include_install_paths()

include_directories(include/internal)
add_subdirectory(src)
