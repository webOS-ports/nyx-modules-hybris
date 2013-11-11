/* @@@LICENSE
*
* Copyright (c) 2013 Simon Busch <morphis@gravedo.de>
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
* LICENSE@@@ */

#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <glib.h>

#include <android/system/window.h>
#include <android/hardware/lights.h>

#include <nyx/nyx_module.h>
#include <nyx/module/nyx_utils.h>

NYX_DECLARE_MODULE(NYX_DEVICE_LED_CONTROLLER, "LedControllers");

static const struct hw_module_t *lights_module = 0;
static struct light_device_t *backlight_device = 0;

unsigned int backlight_brightness = 0;
bool backlight_power = true;

static int light_device_open(const struct hw_module_t* module, const char *id,
                             struct light_device_t** device)
{
    return module->methods->open(module, id, (struct hw_device_t**)device);
}

static void light_device_close(const struct light_device_t *device)
{
    device->common.close((struct hw_device_t*) device);
}

static bool hybris_module_lights_load(void)
{
    static bool done = false;

    if (!done)
    {
        done = true;
        hw_get_module(LIGHTS_HARDWARE_MODULE_ID, &lights_module);
        if (!lights_module)
            nyx_error("Could not load android hardware lights module");
    }

    return lights_module != 0;
}

static struct light_device_t* hybris_light_init(const char *id)
{
    struct light_device_t *device;

    if (!hybris_module_lights_load())
        return false;

    light_device_open(lights_module, LIGHT_ID_BACKLIGHT, &device);
    if (!device) {
        nyx_error("Failed to open light device (id %i)", id);
        return false;
    }

    return device;
}

static void hybris_light_release(struct light_device_t *device)
{
    if (!device)
        return;

    light_device_close(device);
    device = 0;
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
        nyx_error("Failed to set brightness for light (level %i)", level);
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
        state.flashMode = LIGHT_FLASH_HARDWARE;
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
    nyx_device_t *nyxDev = (nyx_device_t*)calloc(sizeof(nyx_device_t), 1);
    if (NULL == nyxDev)
        return NYX_ERROR_OUT_OF_MEMORY;

    nyx_module_register_method(i, (nyx_device_t*)nyxDev, NYX_LED_CONTROLLER_EXECUTE_EFFECT_MODULE_METHOD,
        "led_controller_execute_effect");

    nyx_module_register_method(i, (nyx_device_t*)nyxDev, NYX_LED_CONTROLLER_GET_STATE_MODULE_METHOD,
        "led_controller_get_state");

    *d = (nyx_device_t*)nyxDev;

    if (!hybris_module_lights_load())
    {
        nyx_error("Failed to open lights hardware abstraction module");
        return NYX_ERROR_DEVICE_UNAVAILABLE;
    }

    backlight_device = hybris_light_init(LIGHT_ID_BACKLIGHT);
    if (!backlight_device)
    {
        nyx_error("Failed to create a backlight device");
        return NYX_ERROR_DEVICE_UNAVAILABLE;
    }

    // set initial backlight brightness so we now where we start
    backlight_brightness = 50;
    hybris_light_set_brightness(backlight_device, backlight_brightness);

    return NYX_ERROR_NONE;
}

nyx_error_t nyx_module_close (nyx_device_t* d)
{
    free(d);

    hybris_light_release(backlight_device);
    backlight_device = 0;

    return NYX_ERROR_NONE;
}

static nyx_error_t handle_backlight_effect(nyx_device_handle_t handle, nyx_led_controller_effect_t effect)
{
    nyx_callback_status_t status = NYX_CALLBACK_STATUS_DONE;
    unsigned int brightness = 0;

    switch(effect.required.effect)
    {
    case NYX_LED_CONTROLLER_EFFECT_LED_SET:
        brightness = effect.backlight.brightness_lcd;

        nyx_debug("Adjusting backlight: brightness %i power %s",
                  brightness, backlight_power ? "on" : "off");

        if (!hybris_light_set_brightness(backlight_device, brightness))
        {
            status = NYX_CALLBACK_STATUS_FAILED;
            goto done;
        }

        backlight_brightness = brightness;

        break;
    default:
        break;
    }

done:
    effect.backlight.callback(handle, status, effect.backlight.callback_context);

    return NYX_ERROR_NONE;
}

nyx_error_t led_controller_execute_effect(nyx_device_handle_t handle, nyx_led_controller_effect_t effect)
{
    switch (effect.required.led) {
    case NYX_LED_CONTROLLER_BACKLIGHT_LEDS:
        return handle_backlight_effect(handle, effect);
    default:
        break;
    }

    return NYX_ERROR_DEVICE_UNAVAILABLE;
}

nyx_error_t led_controller_get_state(nyx_device_handle_t handle, nyx_led_controller_led_t led, nyx_led_controller_state_t *state)
{
    switch (led) {
    case NYX_LED_CONTROLLER_BACKLIGHT_LEDS:
        *state = backlight_brightness > 0 ? NYX_LED_CONTROLLER_STATE_ON : NYX_LED_CONTROLLER_STATE_OFF;
        return NYX_ERROR_NONE;
    default:
        break;
    }

    return NYX_ERROR_DEVICE_UNAVAILABLE;
}
