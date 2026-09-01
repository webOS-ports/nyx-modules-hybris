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

/*
 * Stand-in for <android/hardware/lights.h> on header sets that no longer ship
 * it.
 *
 * Android 14 replaced the legacy lights HAL with the android.hardware.light
 * AIDL interface and stopped installing the header; android-headers-halium 14.0
 * has 43 hardware headers and this is not one of them. 9.0, 11.0 and 16.0 all
 * still ship it, and their copies are byte-identical, so the interface itself
 * has not moved - only its availability at build time.
 *
 * What matters here is the ABI of the vendor blob we dlopen, not which header
 * AOSP chose to install, so declaring it ourselves is safe. The declarations
 * below are copied from that header and must stay layout-compatible with it:
 * struct light_state_t is passed straight into the blob's set_light().
 *
 * If a device really has no legacy lights HAL, hw_get_module() returns NULL at
 * runtime and led_controller.c takes its existing sysfs path - the same thing
 * already happens on any device whose HAL cannot be loaded, so nothing new is
 * required of the caller.
 *
 * Deliberately carries the upstream include guard: if some other header pulls
 * in the real lights.h, whichever arrives second is skipped instead of
 * redefining these types.
 */

#ifndef ANDROID_LIGHTS_INTERFACE_H
#define ANDROID_LIGHTS_INTERFACE_H

#include <stdint.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include <hardware/hardware.h>

__BEGIN_DECLS

#define LIGHTS_HARDWARE_MODULE_ID "lights"

/* Logical, not physical, lights. */
#define LIGHT_ID_BACKLIGHT          "backlight"
#define LIGHT_ID_KEYBOARD           "keyboard"
#define LIGHT_ID_BUTTONS            "buttons"
#define LIGHT_ID_BATTERY            "battery"
#define LIGHT_ID_NOTIFICATIONS      "notifications"
#define LIGHT_ID_ATTENTION          "attention"

/* Flash modes for light_state_t.flashMode. */
#define LIGHT_FLASH_NONE            0
#define LIGHT_FLASH_TIMED           1
#define LIGHT_FLASH_HARDWARE        2

/* Policies for light_state_t.brightnessMode. */
#define BRIGHTNESS_MODE_USER        0
#define BRIGHTNESS_MODE_SENSOR      1

struct light_state_t {
    /* Colour of the LED in ARGB. The high byte is ignored; callers set 0xff. */
    unsigned int color;

    int flashMode;      /* see the LIGHT_FLASH_* constants */
    int flashOnMS;
    int flashOffMS;

    int brightnessMode; /* see the BRIGHTNESS_MODE_* constants */
};

struct light_device_t {
    struct hw_device_t common;

    /* Returns 0 on success, an error code on failure. */
    int (*set_light)(struct light_device_t *dev,
                     struct light_state_t const *state);
};

__END_DECLS

#endif  // ANDROID_LIGHTS_INTERFACE_H
