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
 * NYX_DEVICE_LED "Torch" via the Android camera service.
 *
 * The hybris counterpart to the sysfs torch module in nyx-modules. On a Halium
 * device the /sys/class/leds torch nodes belong to the Android side and are
 * usually not writable from here, so the camera service is the route that
 * actually works - it is what SailfishOS drives, through the same droidmedia
 * entry point.
 *
 * droidmedia's hybris.c resolves droid_media_camera_set_torch_mode() out of the
 * Android-side libdroidmedia.so with android_dlsym(), so this stays plain C on
 * the glibc side and never touches the Android C++ ABI.
 *
 * The API is boolean - Android's ICameraService::setTorchMode has no intensity -
 * so brightness collapses to on/off. nyx_led.h asks for a percentage, so any
 * non-zero value turns the torch on and get_brightness answers 0 or 100 rather
 * than inventing a level the hardware never had.
 */

#include <stdbool.h>
#include <stdlib.h>

#include <droidmedia.h>
#include <droidmediacamera.h>

#include <nyx/nyx_module.h>
#include <nyx/module/nyx_log.h>

#include "msgid.h"

NYX_DECLARE_MODULE(NYX_DEVICE_LED, "Torch");

static bool droidmedia_ready = false;
static bool torch_on = false;

nyx_error_t nyx_module_open(nyx_instance_t i, nyx_device_t **d)
{
	nyx_device_t *dev;

	/*
	 * Initialising here rather than on first use keeps a failure visible at
	 * open time: a caller that gets a handle can then rely on set_brightness
	 * reaching the camera service.
	 */
	droidmedia_ready = droid_media_init();

	if (!droidmedia_ready)
	{
		nyx_error(MSGID_NYX_HYBRIS_TORCH_INIT_ERR, 0,
		          "droid_media_init() failed; is libdroidmedia.so present on the Android side?");
		return NYX_ERROR_DEVICE_UNAVAILABLE;
	}

	/*
	 * A successful init is not enough to conclude there is a torch here. It only
	 * reports that the libhybris glue came up - droid_media_init() returns true
	 * as soon as __init_glue() succeeds - and says nothing about whether the
	 * Android-side libdroidmedia.so is new enough to have a torch entry point.
	 *
	 * droid_media_camera_set_torch_mode() is deliberately resolved with
	 * __try_resolve_sym rather than the HYBRIS_WRAPPER macros, which abort on a
	 * missing symbol. On a GSI older than 16.0 the symbol does not exist, so it
	 * quietly returns false instead. Unprobed, this module would therefore open
	 * cleanly and then fail every call it was ever given - which torchd cannot
	 * tell apart from a torch that is present but broken, when the truth is that
	 * this device has no camera-service torch route at all.
	 *
	 * Probing with an explicit "off" settles it. A missing symbol answers false
	 * without reaching the camera service, so the older-GSI case is decided
	 * exactly; and off is the state we want to start from regardless, so on a
	 * device where the route does work the probe costs nothing and lights
	 * nothing.
	 *
	 * Reopening will not give a different answer: droidmedia caches the lookup
	 * in a function-local static, so the first attempt in this process decides
	 * for the lifetime of the process.
	 */
	if (!droid_media_camera_set_torch_mode(false))
	{
		nyx_error(MSGID_NYX_HYBRIS_TORCH_UNSUPPORTED_ERR, 0,
		          "no torch via the camera service; libdroidmedia.so is most likely "
		          "older than the revision that added droid_media_camera_set_torch_mode");
		return NYX_ERROR_DEVICE_UNAVAILABLE;
	}

	torch_on = false;

	dev = (nyx_device_t *)calloc(1, sizeof(nyx_device_t));

	if (NULL == dev)
	{
		return NYX_ERROR_OUT_OF_MEMORY;
	}

	nyx_module_register_method(i, dev, NYX_LED_SET_BRIGHTNESS_MODULE_METHOD,
	                           "led_set_brightness");
	nyx_module_register_method(i, dev, NYX_LED_GET_BRIGHTNESS_MODULE_METHOD,
	                           "led_get_brightness");

	*d = dev;
	return NYX_ERROR_NONE;
}

nyx_error_t nyx_module_close(nyx_device_t *d)
{
	/*
	 * Leaving the torch burning after the last handle closes would outlive
	 * whatever asked for it, with no way left to turn it off.
	 */
	if (torch_on)
	{
		droid_media_camera_set_torch_mode(false);
		torch_on = false;
	}

	free(d);
	return NYX_ERROR_NONE;
}

nyx_error_t led_set_brightness(nyx_device_handle_t handle, int32_t brightness)
{
	bool enable;

	if (NULL == handle)
	{
		return NYX_ERROR_INVALID_HANDLE;
	}

	if (brightness < 0 || brightness > 100)
	{
		return NYX_ERROR_INVALID_VALUE;
	}

	enable = (brightness > 0);

	if (!droid_media_camera_set_torch_mode(enable))
	{
		nyx_error(MSGID_NYX_HYBRIS_TORCH_SET_ERR, 0,
		          "droid_media_camera_set_torch_mode(%d) failed", enable);
		return NYX_ERROR_INVALID_OPERATION;
	}

	torch_on = enable;
	return NYX_ERROR_NONE;
}

nyx_error_t led_get_brightness(nyx_device_handle_t handle,
                               int32_t *brightness_out_ptr)
{
	if (NULL == handle)
	{
		return NYX_ERROR_INVALID_HANDLE;
	}

	if (NULL == brightness_out_ptr)
	{
		return NYX_ERROR_INVALID_VALUE;
	}

	/*
	 * The camera service exposes no getter, so this reports what we last set.
	 * That is wrong only if something outside LuneOS drove the torch, which on
	 * a device where the camera HAL is ours to drive does not happen.
	 */
	*brightness_out_ptr = torch_on ? 100 : 0;
	return NYX_ERROR_NONE;
}
