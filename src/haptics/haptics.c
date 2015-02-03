/* @@@LICENSE
*
* Copyright (c) 2013 Simon Busch <morphis@gravedo.de>
* Copyright (c) 2015 Nikolay Nizov <nizovn@gmail.com>
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

#include <nyx/nyx_module.h>
#include <android/hardware_legacy/vibrator.h>
#include <glib.h>
#include <stdlib.h>

#define VIBRATOR_PAUSE 25 //ms

gboolean vibrate_cb (gpointer user_data);
void vibrate_stop_timeout();

nyx_haptics_device_t *nyxDev = NULL;
nyx_haptics_configuration_t *nyxConf = NULL;

NYX_DECLARE_MODULE(NYX_DEVICE_HAPTICS, "Haptics")

nyx_error_t nyx_module_open (nyx_instance_t instance, nyx_device_t** device_ptr)
{
	if (!vibrator_exists())
		return NYX_ERROR_DEVICE_NOT_EXIST;

	if (nyxDev) {
		nyx_info("Haptics module already open");
		*device_ptr = (nyx_device_t *)nyxDev;
		return NYX_ERROR_NONE;
	}

	nyxDev = (nyx_haptics_device_t*)calloc(sizeof(nyx_haptics_device_t), 1);
	if (NULL == nyxDev)
		return NYX_ERROR_OUT_OF_MEMORY;
	nyxConf = (nyx_haptics_configuration_t*)calloc(sizeof(nyx_haptics_configuration_t), 1);
	if (NULL == nyxConf)
		return NYX_ERROR_OUT_OF_MEMORY;

	nyx_module_register_method(instance, (nyx_device_t*) nyxDev,
		NYX_HAPTICS_VIBRATE_MODULE_METHOD, "vibrate");
	nyx_module_register_method(instance, (nyx_device_t*) nyxDev,
		NYX_HAPTICS_CANCEL_MODULE_METHOD, "cancel");
	nyx_module_register_method(instance, (nyx_device_t*) nyxDev,
		NYX_HAPTICS_CANCEL_ALL_MODULE_METHOD, "cancel_all");

	*device_ptr = (nyx_device_t*)nyxDev;

	return NYX_ERROR_NONE;
}

nyx_error_t nyx_module_close (nyx_device_t* device) {
	vibrate_stop_timeout();
	vibrator_off();
	free(nyxDev);
	free(nyxConf);
	return NYX_ERROR_NONE;
}

nyx_error_t vibrate (nyx_device_handle_t handle, nyx_haptics_configuration_t configuration) {
	if (configuration.type != NYX_HAPTICS_EFFECT_UNDEFINED)
		return NYX_ERROR_NOT_IMPLEMENTED;
	if ((configuration.period < 0) || (configuration.duration < 0))
		return NYX_ERROR_INVALID_VALUE;

	vibrate_stop_timeout();

	if (configuration.period == 0)
		configuration.period = 1;
	if (configuration.duration == 0)
		configuration.duration = 2147483647L;
	*nyxConf = configuration;

	if (nyxConf->period > VIBRATOR_PAUSE) {
		vibrator_on(nyxConf->period - VIBRATOR_PAUSE);
		nyxDev->haptic_effect_id = g_timeout_add (nyxConf->period, (GSourceFunc) vibrate_cb, NULL);
	}

	return NYX_ERROR_NONE;
}

nyx_error_t cancel (nyx_device_handle_t handle, int32_t haptics_id) {
	vibrate_stop_timeout();
	vibrator_off();
	return NYX_ERROR_NONE;
}

nyx_error_t cancel_all (nyx_device_handle_t handle) {
	vibrate_stop_timeout();
	vibrator_off();
	return NYX_ERROR_NONE;
}

gboolean vibrate_cb (gpointer user_data) {
	nyxConf->duration -= nyxConf->period;
	if (nyxConf->duration <= 0) {
		nyxDev->haptic_effect_id = 0;
		return false;
	}
	vibrator_off();
	vibrator_on(nyxConf->period - VIBRATOR_PAUSE);
	return true;
}

void vibrate_stop_timeout() {
	if (nyxDev->haptic_effect_id > 0) {
		g_source_remove(nyxDev->haptic_effect_id);
		nyxDev->haptic_effect_id = 0;
	}
}
