# @@@LICENSE
#
#      Copyright (c) 2010-2012 Hewlett-Packard Development Company, L.P.
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

webos_build_nyx_module(MicroControllerLEDsDefault 
                       SOURCES led_controller.c
                       LIBRARIES  ${GLIB2_LDFLAGS} ${PMLOG_LDFLAGS} ${NYXLIB_LDFLAGS} -lhardware
                       # This module loads the Android lights HAL through libhybris,
                       # which pulls Android .so's into their own linker namespace.
                       # nyx_device_close() dlclose()s the nyx module afterwards, and
                       # unloading it while that Android-side state still references it
                       # segfaults - every close on the LED controller died this way on
                       # tissot, after the light had already been set. -z nodelete makes
                       # the dlclose a no-op for this module, which costs one mapping and
                       # is the usual remedy for a library that cannot be safely unloaded.
                       # The native modules do not need it: they load no Android code and
                       # close cleanly.
                       -Wl,-z,nodelete)
