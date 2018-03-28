# @@@LICENSE
#
# Copyright (c) 2013 Simon Busch <morphis@gravedo.de>
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
# LICENSE@@@

include_directories(.)

webos_build_nyx_module(DeviceInfoMain 
                       SOURCES device_info.c
                       LIBRARIES ${GLIB2_LDFLAGS} ${PMLOG_LDFLAGS} ${LIBCRYPTO_LDFLAGS} -landroid-properties)
# FIXME once we have updated to a newer libhybris version we should use the pkgconfig file
# for libandroid-properties
# target_link_libraries(DeviceInfoMain ${ANDROID_PROPERTIES_LDFLAGS} ${LIBCRYPTO_LDFLAGS})
#target_link_libraries(DeviceInfoMain -landroid-properties ${LIBCRYPTO_LDFLAGS})
