// Copyright (c) 2013 Simon Busch <morphis@gravedo.de>
// Copyright (c) 2018 Herman van Hazendonk <github.com@herrie.org>
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

#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <glib.h>
/*
 * <android/system/window.h> used to be included here and contributed nothing -
 * this module names no symbol from it, and lights.h pulls in
 * hardware/hardware.h for hw_get_module() on its own. It was not harmless:
 * on Android 14 it reaches AHardwareBuffer declarations written with the Clang
 * _Nullable attribute, which GCC rejects outright.
 */
#if defined(__has_include)
#  if __has_include(<android/hardware/lights.h>)
#    include <android/hardware/lights.h>
#  else
     /* Android 14 stopped shipping it; see the header for why this is safe. */
#    include "lights_compat.h"
#  endif
#else
#  include <android/hardware/lights.h>
#endif
#include <nyx/nyx_module.h>
#include <nyx/module/nyx_utils.h>
#include <nyx/module/nyx_log.h>
#include "msgid.h"

NYX_DECLARE_MODULE(NYX_DEVICE_LED_CONTROLLER, "LedControllers");

/*
 * On newer Halium bases the display backlight is no longer reachable through the
 * legacy Android lights hw_module (it is a HIDL/AIDL service, or an MTK vendor
 * HAL that hw_get_module() cannot dlopen inside the hybris namespace). When the
 * lights module fails to load we fall back to the kernel LED-class backlight,
 * which MTK (and many others) expose here. The incoming brightness_lcd is a
 * 0..100 percentage (see luna-sysmgr NyxLedControl), scaled to the node's
 * max_brightness.
 *
 * This used to say the path could be overridden per-machine from cmake. It
 * cannot any more: the machine.cmake files are gone, and the shared
 * NYX_MODULES_REQUIRED list that replaced them names providers, not paths. The
 * override that remains is -DBACKLIGHT_SYSFS_PATH via EXTRA_OECMAKE in the
 * recipe. No machine uses it, and a second spelling of this node would be
 * better added to a probe list here, the way the keypad below does it.
 */
#ifndef BACKLIGHT_SYSFS_PATH
#define BACKLIGHT_SYSFS_PATH "/sys/class/leds/lcd-backlight/brightness"
#endif
#ifndef BACKLIGHT_MAX_SYSFS_PATH
#define BACKLIGHT_MAX_SYSFS_PATH "/sys/class/leds/lcd-backlight/max_brightness"
#endif

/*
 * Same story for the keyboard backlight of a device that has a physical
 * keyboard (BlackBerry KEY2 and friends). It is a second light in the same
 * effect - nyx_led_controller_effect_t carries brightness_lcd and
 * brightness_keypad side by side, and luna-displaymanager fills both on every
 * backlightOn()/backlightOff() - so it is driven from the same place, not from
 * a separate device.
 *
 * This one is probed at runtime from a list of names rather than compiled in
 * per machine, because there is no longer anywhere per-machine to compile it:
 * the machine.cmake files that carried device paths are gone, replaced by a
 * NYX_MODULES_REQUIRED list in one shared .inc that says which provider owns a
 * module and nothing about paths. A machine that needs something else can still
 * pass -DKEYPAD_SYSFS_PATH through EXTRA_OECMAKE, but it should not have to:
 * the two spellings below cover what devices actually use, and a third belongs
 * in this list rather than in a machine-specific define.
 *
 * "keyboard-backlight" is the name AOSP's reference lights HAL uses for
 * LIGHT_ID_KEYBOARD, and it is what the athena (KEY2) device tree registers for
 * both of the LED-class drivers that can own that light (qcom,leds-smg31323 and
 * awinic,aw2027_led). "kbd_backlight" is the upstream-Linux spelling, used by
 * the laptop and Chromebook LED drivers.
 *
 * Deliberately not in the list: button-backlight. That is the capacitive
 * navigation row, a different light that happens to sit next to this one in
 * athena's device tree, and driving it from the keyboard brightness would light
 * the wrong thing on every device that has both.
 */
#ifdef KEYPAD_SYSFS_PATH
#ifndef KEYPAD_MAX_SYSFS_PATH
#error "KEYPAD_SYSFS_PATH needs a KEYPAD_MAX_SYSFS_PATH alongside it"
#endif
static const char *const keypad_nodes[][2] = {
    { KEYPAD_SYSFS_PATH, KEYPAD_MAX_SYSFS_PATH },
};
#else
static const char *const keypad_nodes[][2] = {
    { "/sys/class/leds/keyboard-backlight/brightness",
      "/sys/class/leds/keyboard-backlight/max_brightness" },
    { "/sys/class/leds/kbd_backlight/brightness",
      "/sys/class/leds/kbd_backlight/max_brightness" },
};
#endif

static const struct hw_module_t *lights_module = NULL;
static struct light_device_t *backlight_device = NULL;
static struct light_device_t *notifications_device = NULL;
static struct light_device_t *keypad_device = NULL;

static bool use_sysfs_backlight = false;
static int sysfs_backlight_max = 255;
/* Which of keypad_nodes[] this device turned out to have; NULL for none. */
static const char *sysfs_keypad_path = NULL;
static int sysfs_keypad_max = 255;

static int sysfs_read_int(const char *path, int fallback)
{
    char buf[32];
    char *end = NULL;
    long value;
    FILE *f = fopen(path, "r");

    if (!f)
        return fallback;

    if (fgets(buf, sizeof(buf), f) == NULL)
    {
        (void) fclose(f);
        return fallback;
    }

    (void) fclose(f);

    value = strtol(buf, &end, 10);
    if (end == buf || value < INT_MIN || value > INT_MAX)
        return fallback;

    return (int) value;
}

static bool sysfs_light_available(const char *path)
{
    FILE *f = fopen(path, "w");

    if (!f)
        return false;

    (void) fclose(f);
    return true;
}

/* level is a 0..100 percentage; scale it onto the node's max_brightness. */
static bool sysfs_light_set(const char *path, int max, int level)
{
    int pct = (level < 0) ? 0 : (level > 100) ? 100 : level;
    int value = (pct * max + 50) / 100;
    FILE *f = fopen(path, "w");

    if (!f)
    {
        nyx_error(MSGID_NYX_HYBRIS_LED_BRIGHTNESS_LEV_ERR, 0,
                  "Failed to open %s for backlight (level %i)", path, level);
        return false;
    }

    if (fprintf(f, "%d", value) < 0)
    {
        nyx_error(MSGID_NYX_HYBRIS_LED_BRIGHTNESS_LEV_ERR, 0,
                  "Failed to write %s for backlight (level %i)", path, level);
        (void) fclose(f);
        return false;
    }

    if (fclose(f) != 0)
    {
        nyx_error(MSGID_NYX_HYBRIS_LED_BRIGHTNESS_LEV_ERR, 0,
                  "Failed to write %s for backlight (level %i)", path, level);
        return false;
    }

    return true;
}

/* Probe one sysfs LED-class light, remembering its max_brightness. */
static bool sysfs_light_probe(const char *path, const char *max_path, int *max)
{
    if (!sysfs_light_available(path))
        return false;

    *max = sysfs_read_int(max_path, 255);
    if (*max <= 0)
        *max = 255;

    return true;
}

/* First of keypad_nodes[] this device has, if any. */
static bool sysfs_keypad_probe(void)
{
    size_t i;

    for (i = 0; i < sizeof(keypad_nodes) / sizeof(keypad_nodes[0]); i++)
    {
        if (!sysfs_light_probe(keypad_nodes[i][0], keypad_nodes[i][1], &sysfs_keypad_max))
            continue;

        sysfs_keypad_path = keypad_nodes[i][0];
        return true;
    }

    return false;
}

static int light_device_open(const struct hw_module_t* module, const char *id,
                             struct light_device_t** device)
{
    return module->methods->open(module, id, (struct hw_device_t**)device);
}

static void light_device_close(const struct light_device_t *device)
{
    /*
     * common.close is supplied by the vendor blob, and plenty of lights HALs
     * leave it NULL - the legacy HAL never required it. Calling it
     * unconditionally segfaults, and does so on the way out, long after the
     * light has been set: on tissot every nyx_device_close() on the LED
     * controller died here, which read as an unrelated crash rather than as
     * "this HAL has no close".
     */
    if (device->common.close)
    {
        device->common.close((struct hw_device_t *) device);
    }
}

static bool hybris_module_lights_load(void)
{
    static bool done = false;

    if (!done)
    {
        done = true;
        hw_get_module(LIGHTS_HARDWARE_MODULE_ID, &lights_module);
        if (!lights_module)
            nyx_error(MSGID_NYX_HYBRIS_LED_ANDROID_LIGHT_MOD_ERR , 0, "Could not load android hardware lights module");
    }

    return lights_module != NULL;
}

/*
 * optional: a light the HAL is allowed not to have. LIGHT_ID_KEYBOARD is one -
 * most devices have no keyboard to light - so a failure there is a fact about
 * the device, not an error, and must not be logged as one.
 */
static struct light_device_t* hybris_light_init(const char *id, bool optional)
{
    struct light_device_t *device = NULL ;

    if (!hybris_module_lights_load())
        return NULL;

    light_device_open(lights_module, id, &device);
    if (!device) {
        if (optional)
            nyx_debug("No light device for id %s; this device has none", id);
        else
            nyx_error(MSGID_NYX_HYBRIS_LED_ANDROID_LIGHT_DEV_ERR, 0, "Failed to open light device (id %s)", id);
        return NULL;
    }

    return device;
}

static void hybris_light_release(const struct light_device_t *device)
{
    if (!device)
        return;

    light_device_close(device);
}

static bool hybris_light_set_brightness(struct light_device_t *device, int level)
{
    unsigned normalized_level = (level < 0) ? 0 : (level > 255) ? 255 : level;
    struct light_state_t state;

    if (!device)
        return false;

    memset(&state, 0, sizeof(state));
    state.color = (0xff << 24) | (normalized_level << 16) | (normalized_level << 8) | (normalized_level << 0);
    state.flashMode = LIGHT_FLASH_NONE;
    state.flashOnMS = 0;
    state.flashOffMS = 0;
    state.brightnessMode = BRIGHTNESS_MODE_USER;

    nyx_debug("Set light brightness to %i (%i) ...", normalized_level, level);

    if (device->set_light(device, &state) < 0)
    {
        nyx_error(MSGID_NYX_HYBRIS_LED_BRIGHTNESS_LEV_ERR, 0, "Failed to set brightness for light (level %i)", level);
        return false;
    }

    return true;
}

static bool hybris_light_set_pattern(struct light_device_t *device, int r, int g, int b, int ms_on, int ms_off)
{
    struct light_state_t state;

    if (!device)
        return false;

    memset(&state, 0, sizeof(state));
    state.color = (0xff << 24) | (r << 16) | (g << 8) | (b << 0);
    state.brightnessMode = BRIGHTNESS_MODE_USER;

    if (ms_on > 0 && ms_off > 0)
    {
        state.flashMode = LIGHT_FLASH_TIMED;
        state.flashOnMS = ms_on;
        state.flashOffMS = ms_off;
    }
    else
    {
        state.flashMode = LIGHT_FLASH_NONE;
        state.flashOnMS = 0;
        state.flashOffMS = 0;
    }

    if (device->set_light(device, &state) < 0)
        return false;

    return true;
}

nyx_error_t nyx_module_open (nyx_instance_t i, nyx_device_t** d)
{
    nyx_device_t *nyxDev = (nyx_device_t*)calloc(1, sizeof(nyx_device_t));
    if (NULL == nyxDev)
        return NYX_ERROR_OUT_OF_MEMORY;

    nyx_module_register_method(i, (nyx_device_t*)nyxDev, NYX_LED_CONTROLLER_EXECUTE_EFFECT_MODULE_METHOD,
        "led_controller_execute_effect");

    nyx_module_register_method(i, (nyx_device_t*)nyxDev, NYX_LED_CONTROLLER_GET_STATE_MODULE_METHOD,
        "led_controller_get_state");

    *d = (nyx_device_t*)nyxDev;

    if (!hybris_module_lights_load())
    {
        /*
         * No usable Android lights HAL. If the kernel exposes LED-class
         * backlights, drive them directly from sysfs so the brightness slider
         * and the keyboard light still work; the notification LED is simply
         * unavailable in that case.
         *
         * The two are probed independently: a device can have one without the
         * other, and on a keyboard device the keyboard light is reason enough
         * to open this module even if the display backlight lives elsewhere.
         */
        use_sysfs_backlight = sysfs_light_probe(BACKLIGHT_SYSFS_PATH,
                                                BACKLIGHT_MAX_SYSFS_PATH,
                                                &sysfs_backlight_max);
        if (use_sysfs_backlight)
            nyx_debug("Lights HAL unavailable; using sysfs backlight %s (max %d)",
                      BACKLIGHT_SYSFS_PATH, sysfs_backlight_max);

        if (sysfs_keypad_probe())
            nyx_debug("Lights HAL unavailable; using sysfs keypad backlight %s (max %d)",
                      sysfs_keypad_path, sysfs_keypad_max);

        if (use_sysfs_backlight || sysfs_keypad_path)
            return NYX_ERROR_NONE;

        nyx_error(MSGID_NYX_HYBRIS_LED_HAL_MOD_OPEN_ERR, 0, "Failed to open lights hardware abstraction module");
        return NYX_ERROR_DEVICE_UNAVAILABLE;
    }

    backlight_device = hybris_light_init(LIGHT_ID_BACKLIGHT, false);
    if (!backlight_device)
    {
        nyx_error(MSGID_NYX_HYBRIS_LED_BACKLIGHT_DEV_ERR, 0, "Failed to create a backlight device");
        return NYX_ERROR_DEVICE_UNAVAILABLE;
    }

    /*
     * The keyboard light is optional on both counts: most devices have no
     * keyboard, and a HAL that lights the display may still leave the keyboard
     * to the kernel. Fall back to sysfs for it alone rather than failing, which
     * is why this is probed even on the HAL path.
     */
    keypad_device = hybris_light_init(LIGHT_ID_KEYBOARD, true);
    if (!keypad_device && sysfs_keypad_probe())
        nyx_debug("No keyboard light in the lights HAL; using sysfs %s (max %d)",
                  sysfs_keypad_path, sysfs_keypad_max);

    /* open and obtain handle to notification LED */
    notifications_device = hybris_light_init(LIGHT_ID_NOTIFICATIONS, false);
    if(notifications_device == NULL)
    {
        nyx_error(MSGID_NYX_HYBRIS_LED_CONTROLLER_DEV_ERR, 0, "Failed to create an LED-controller notification device");
        return NYX_ERROR_DEVICE_UNAVAILABLE;
    }

    return NYX_ERROR_NONE;
}

nyx_error_t nyx_module_close (nyx_device_t* d)
{
    free(d);

    hybris_light_release(notifications_device);
    notifications_device = NULL;

    hybris_light_release(keypad_device);
    keypad_device = NULL;

    hybris_light_release(backlight_device);
    backlight_device = NULL;

    return NYX_ERROR_NONE;
}

static nyx_error_t handle_backlight_effect(nyx_device_handle_t handle, nyx_led_controller_effect_t effect)
{
    nyx_callback_status_t status = NYX_CALLBACK_STATUS_DONE;
    int32_t brightness = 0;
    int32_t keypad_brightness = 0;

    switch(effect.required.effect)
    {
    case NYX_LED_CONTROLLER_EFFECT_LED_SET:
        brightness = effect.backlight.brightness_lcd;
        keypad_brightness = effect.backlight.brightness_keypad;

        nyx_debug("Adjusting backlight: brightness %i, keypad %i",
                  brightness, keypad_brightness);

        if (use_sysfs_backlight)
        {
            if (!sysfs_light_set(BACKLIGHT_SYSFS_PATH, sysfs_backlight_max, brightness))
            {
                status = NYX_CALLBACK_STATUS_FAILED;
                goto done;
            }
        }
        else if (!hybris_light_set_brightness(backlight_device, brightness))
        {
            status = NYX_CALLBACK_STATUS_FAILED;
            goto done;
        }

        /*
         * A device with no keyboard light has neither of these set, and the
         * keypad brightness luna-displaymanager computes for it is simply
         * dropped, as it was before this light existed here.
         *
         * A negative value means "leave this light as it is" - that is what
         * nyx-test-ledcontroller documents for --keypad and what it sends by
         * default, so honouring it keeps a display-brightness test from
         * switching the keyboard off as a side effect. luna-displaymanager
         * never sends one: backlightOn() computes a 0..100 percentage and
         * backlightOff() asks for 0, which does turn the keyboard off with the
         * screen. Note that brightness_lcd above deliberately does NOT get this
         * treatment: backlightOff() passes -1 for the display and relies on it
         * reaching zero before the panel is powered down, so a negative there
         * has always meant off and changing that is a separate question.
         */
        if (keypad_brightness < 0)
            break;

        if (sysfs_keypad_path)
        {
            if (!sysfs_light_set(sysfs_keypad_path, sysfs_keypad_max, keypad_brightness))
            {
                status = NYX_CALLBACK_STATUS_FAILED;
                goto done;
            }
        }
        else if (keypad_device && !hybris_light_set_brightness(keypad_device, keypad_brightness))
        {
            status = NYX_CALLBACK_STATUS_FAILED;
            goto done;
        }

        break;
    default:
        break;
    }

done:
    effect.backlight.callback(handle, status, effect.backlight.callback_context);

    return NYX_ERROR_NONE;
}



/*
 * Resolve an effect's colour into the three channels the lights HAL wants.
 *
 * A caller that set no colour gets the greyscale this module has always
 * produced, so CoreNaviLeds and every other brightness-only caller is
 * unaffected. A caller that set one gets it scaled by brightness, which is the
 * contract documented in nyx_led_controller_core_configuration.h: colour is the
 * hue, brightness stays the intensity control.
 */
static void resolve_colour(nyx_led_controller_core_configuration_handle_t config,
                           int brightness, int *r, int *g, int *b)
{
    static const nyx_led_controller_parameter_type_t param[3] = {
        NYX_LED_CONTROLLER_CORE_EFFECT_COLOUR_RED,
        NYX_LED_CONTROLLER_CORE_EFFECT_COLOUR_GREEN,
        NYX_LED_CONTROLLER_CORE_EFFECT_COLOUR_BLUE,
    };
    /*
     * Seeded before the call, not after a failed one. nyx-lib's get_param ends
     * its switch with "default: break;" and returns NYX_ERROR_NONE for a
     * parameter it does not recognise, without touching *value - so against a
     * nyx-lib older than the one that added these, the channels would keep
     * whatever was on the stack, any_set would go true on garbage, and garbage
     * would be scaled and handed to set_light as a colour.
     */
    int32_t channel[3] = {
        NYX_LED_CONTROLLER_CORE_COLOUR_UNSET,
        NYX_LED_CONTROLLER_CORE_COLOUR_UNSET,
        NYX_LED_CONTROLLER_CORE_COLOUR_UNSET,
    };
    bool any_set = false;
    int i;

    for (i = 0; i < 3; i++)
    {
        if (nyx_led_controller_core_configuration_get_param(config, param[i],
                                                           &channel[i]) != NYX_ERROR_NONE)
            channel[i] = NYX_LED_CONTROLLER_CORE_COLOUR_UNSET;

        if (channel[i] != NYX_LED_CONTROLLER_CORE_COLOUR_UNSET)
            any_set = true;
    }

    if (!any_set)
    {
        /* No colour asked for: what this module did before colour existed. */
        channel[0] = channel[1] = channel[2] = brightness;
    }
    else
    {
        for (i = 0; i < 3; i++)
        {
            /* One channel named and the others silent means the others are off. */
            if (channel[i] == NYX_LED_CONTROLLER_CORE_COLOUR_UNSET)
                channel[i] = 0;

            channel[i] = channel[i] * brightness / NYX_LED_CONTROLLER_CORE_COLOUR_MAX;
        }
    }

    /*
     * Clamp rather than trust the arithmetic: set_light packs each of these
     * into one byte next to the alpha byte, so an out-of-range channel would
     * not just be too bright, it would corrupt the value.
     */
    for (i = 0; i < 3; i++)
    {
        if (channel[i] < 0)
            channel[i] = 0;
        else if (channel[i] > NYX_LED_CONTROLLER_CORE_COLOUR_MAX)
            channel[i] = NYX_LED_CONTROLLER_CORE_COLOUR_MAX;
    }

    *r = channel[0];
    *g = channel[1];
    *b = channel[2];
}

static nyx_error_t handle_notification_effect(nyx_device_handle_t handle, nyx_led_controller_effect_t effect)
{
    nyx_error_t err = NYX_ERROR_NONE; 
    bool hybris_err = true ;
    int32_t led_on = 0 , led_off = 0 , brightness = 0 ;
    int red = 0 , green = 0 , blue = 0 ;

    /* Sanity Check input params */
    if( handle == NULL ) 
    {
        nyx_error(MSGID_NYX_HYBRIS_LED_INVALID_HANDLE_ERR, 0, "Handle to LED device - for notification-effect = NULL");
        err = NYX_ERROR_INVALID_HANDLE ; 
        goto err_notification_handle ;
    }

    if( effect.core_configuration == NULL ) 
    {
        nyx_error(MSGID_NYX_HYBRIS_LED_INVALID_VALUE_ERR, 0, "LED Core configuration argument = NULL");
        err = NYX_ERROR_INVALID_VALUE ; 
        goto err_notification_handle ;
    }

    switch(effect.required.effect)
    {
        case NYX_LED_CONTROLLER_EFFECT_LED_SET:
            err = nyx_led_controller_core_configuration_get_param( effect.core_configuration 
                                                                   , NYX_LED_CONTROLLER_CORE_EFFECT_BRIGHTNESS
                                                                   , &brightness);
            if( err != NYX_ERROR_NONE )
            {
                nyx_debug("Could not resolve brightness level");
                goto err_notification_handle ;
            }
  
            resolve_colour(effect.core_configuration, brightness, &red, &green, &blue);

            nyx_debug("setting LED colour = [%d,%d,%d] (brightness %d).Duty-cycle=100%%"
                                                  , red, green, blue, brightness);

            hybris_err = hybris_light_set_pattern(notifications_device
                                                  , red, green, blue , 0, 0); 
            if( hybris_err == false ) 
            {
                err = NYX_ERROR_INVALID_OPERATION ;
                goto err_notification_handle ;
            }
            break;
        case NYX_LED_CONTROLLER_EFFECT_LED_PULSATE:
            err = nyx_led_controller_core_configuration_get_param( effect.core_configuration 
                                                                   , NYX_LED_CONTROLLER_CORE_EFFECT_FADE_IN
                                                                   , &led_on);
            if( err != NYX_ERROR_NONE )
            {
                nyx_debug("Could not resolve pulse fade-in time");
                goto err_notification_handle ;
            }

            err = nyx_led_controller_core_configuration_get_param( effect.core_configuration 
                                                                   , NYX_LED_CONTROLLER_CORE_EFFECT_FADE_OUT
                                                                   , &led_off);
            if( err != NYX_ERROR_NONE )
            {
                nyx_debug("Could not resolve pulse fade-out time");
                goto err_notification_handle ;
            }

            err = nyx_led_controller_core_configuration_get_param( effect.core_configuration 
                                                                   , NYX_LED_CONTROLLER_CORE_EFFECT_BRIGHTNESS
                                                                   , &brightness);
            if( err != NYX_ERROR_NONE )
            {
                nyx_debug("Could not resolve pulse brightness level");
                goto err_notification_handle ;
            }
  
            resolve_colour(effect.core_configuration, brightness, &red, &green, &blue);

            nyx_debug("setting LED colour [%d,%d,%d] (brightness %d) to pulse on[ms]=%d , off[ms]=%d"
                                                               , red, green, blue, brightness
                                                               , led_on , led_off);

            hybris_err = hybris_light_set_pattern(notifications_device
                                                  , red, green, blue
                                                  , led_on, led_off); 
            if( hybris_err == false ) 
            {
                err = NYX_ERROR_INVALID_OPERATION ;
                goto err_notification_handle ;
            }
            break;
        default:
            break;
    }

err_notification_handle:
    return err;
}

nyx_error_t led_controller_execute_effect(nyx_device_handle_t handle, nyx_led_controller_effect_t effect)
{
    switch (effect.required.led) {
    case NYX_LED_CONTROLLER_BACKLIGHT_LEDS:
        return handle_backlight_effect(handle, effect);
    case NYX_LED_CONTROLLER_CENTER_LED:
        return handle_notification_effect(handle, effect);
    default:
        break;
    }

    return NYX_ERROR_DEVICE_UNAVAILABLE;
}

nyx_error_t led_controller_get_state(nyx_device_handle_t handle, nyx_led_controller_led_t led, nyx_led_controller_state_t *state)
{
    return NYX_ERROR_DEVICE_UNAVAILABLE;
}
