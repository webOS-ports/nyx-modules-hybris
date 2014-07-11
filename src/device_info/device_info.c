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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <glib.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <openssl/sha.h>
#include <nyx/nyx_module.h>
#include <nyx/module/nyx_utils.h>
#include <hybris/properties/properties.h>

struct device_info {
	nyx_device_t base;
	char *device_name;
	char *board_type;
	char *nduid;
	char *wifi_mac_address;
	char *bt_mac_address;
	char *modem_present;
};

#define MAC_ADDRESS_NUM_OCTETS		6

nyx_error_t device_info_query(nyx_device_handle_t device,
				nyx_device_info_type_t query, const char **dest)
{
	struct device_info *dinfo;
	nyx_error_t error = NYX_ERROR_NONE;
	char value[PROP_VALUE_MAX];
	unsigned char hash[SHA_DIGEST_LENGTH];
	char *p;
	int n, fd, sock;
	struct ifreq req;

	if (device == NULL)
		return NYX_ERROR_INVALID_VALUE;

	dinfo = (struct device_info*) device;

	*dest = "";

	switch (query) {
	case NYX_DEVICE_INFO_BATT_CH:
	case NYX_DEVICE_INFO_BATT_RSP:
	case NYX_DEVICE_INFO_HARDWARE_ID:
	case NYX_DEVICE_INFO_HARDWARE_REVISION:
	case NYX_DEVICE_INFO_INSTALLER:
	case NYX_DEVICE_INFO_KEYBOARD_TYPE:
	case NYX_DEVICE_INFO_LAST_RESET_TYPE:
	case NYX_DEVICE_INFO_PRODUCT_ID:
	case NYX_DEVICE_INFO_RADIO_TYPE:
	case NYX_DEVICE_INFO_SERIAL_NUMBER:
	case NYX_DEVICE_INFO_STORAGE_FREE:
	case NYX_DEVICE_INFO_RAM_SIZE:
	case NYX_DEVICE_INFO_STORAGE_SIZE:
		error = NYX_ERROR_NOT_IMPLEMENTED;
		break;

	case NYX_DEVICE_INFO_MODEM_PRESENT:
		if (dinfo->modem_present == NULL) {
			property_get("rild.libpath", value, "");
			if (strlen(value) == 0)
				dinfo->modem_present = g_strdup("false");
			else
				dinfo->modem_present = g_strdup("true");
		}

		*dest = dinfo->modem_present;

		break;

	case NYX_DEVICE_INFO_BT_ADDR:
		if (dinfo->bt_mac_address == NULL) {
			property_get("ro.bt.bdaddr_path", value, "");
			if (strlen(value) == 0) {
				error = NYX_ERROR_INVALID_OPERATION;
				break;
			}

			fd = open(value, O_RDONLY);
			if (fd < 0) {
				error = NYX_ERROR_INVALID_OPERATION;
				break;
			}

			dinfo->bt_mac_address = (char*) malloc(sizeof(char) * (MAC_ADDRESS_NUM_OCTETS * 3));
			read(fd, dinfo->bt_mac_address, (MAC_ADDRESS_NUM_OCTETS * 3));
			close(fd);
		}

		*dest = dinfo->bt_mac_address;

		break;

	case NYX_DEVICE_INFO_BOARD_TYPE:
		if (dinfo->board_type == NULL) {
			property_get("ro.product.board", value, "");
			if (strlen(value) == 0) {
				error = NYX_ERROR_INVALID_OPERATION;
				break;
			}
			dinfo->board_type = strdup(value);
		}

		*dest = dinfo->board_type;

		break;

	case NYX_DEVICE_INFO_WIFI_ADDR:
		if (dinfo->wifi_mac_address == NULL) {
			property_get("wifi.interface", value, "");
			if (strlen(value) == 0) {
				error = NYX_ERROR_INVALID_OPERATION;
				break;
			}

			sock = socket(AF_INET, SOCK_DGRAM, 0);
			if (socket < 0) {
				error = NYX_ERROR_INVALID_OPERATION;
				break;
			}

			strncpy(req.ifr_name, value, IFNAMSIZ);
			if (ioctl(sock, SIOCGIFHWADDR, &req) < 0) {
				close(sock);
				error = NYX_ERROR_INVALID_OPERATION;
				break;
			}

			dinfo->wifi_mac_address = (char*) malloc(sizeof(char) * 32);
			for (n = 0; n < MAC_ADDRESS_NUM_OCTETS; n++) {
				sprintf(&dinfo->wifi_mac_address[n * 3], "%02X%s",
						 (unsigned char) req.ifr_hwaddr.sa_data[n],
						 (n < (MAC_ADDRESS_NUM_OCTETS - 1)) ? ":" : "");
			}

			close(sock);
		}

		*dest = dinfo->wifi_mac_address;

		break;

	case NYX_DEVICE_INFO_DEVICE_NAME:
		if (dinfo->device_name == NULL) {
			property_get("ro.product.device", value, "");
			if (strlen(value) == 0) {
				error = NYX_ERROR_INVALID_OPERATION;
				break;
			}
			dinfo->device_name = strdup(value);
		}

		*dest = dinfo->device_name;

		break;

	case NYX_DEVICE_INFO_NDUID:
		if (dinfo->nduid == NULL) {
			property_get("ro.serialno", value, "");
			if (strlen(value) == 0) {
				error = NYX_ERROR_INVALID_OPERATION;
				break;
			}
			SHA1((unsigned char*) value, strlen(value), hash);
			dinfo->nduid = (char*) malloc(sizeof(char) * (SHA_DIGEST_LENGTH * 2));
			p = dinfo->nduid;
			for (n = 0; n < SHA_DIGEST_LENGTH; n++) {
				snprintf(p, 3, "%02x", hash[n]);
				p += 2;
			}
			*p = '\0';
		}

		*dest = dinfo->nduid;
		break;

	default:
		error = NYX_ERROR_INVALID_VALUE;
		break;
	}

	return error;
}

nyx_error_t device_info_get_info(nyx_device_handle_t device, nyx_device_info_type_t type,
		char *dest, size_t dest_len)
{
	const char *value = NULL;
	nyx_error_t error;

	if (dest == NULL || dest_len == 0)
		return NYX_ERROR_INVALID_VALUE;

	error = device_info_query(device, type, &value);
	if (error != NYX_ERROR_NONE)
		return error;

	if (strlen(value) >= dest_len)
		return NYX_ERROR_VALUE_OUT_OF_RANGE;

	strncpy(dest, value, dest_len);
	dest[dest_len - 1] = '\0';

	return NYX_ERROR_NONE;
}

nyx_error_t nyx_module_open(nyx_instance_t instance, nyx_device_t **device)
{
	struct device_info *dinfo;
	nyx_error_t error = NYX_ERROR_NONE;

	if (device == NULL)
		return NYX_ERROR_INVALID_VALUE;

	dinfo = (struct device_info*) calloc(sizeof(struct device_info), 1);
	if (dinfo == NULL) {
		error = NYX_ERROR_OUT_OF_MEMORY;
		goto error;
	}

	nyx_module_register_method(instance, (nyx_device_t*) dinfo,
			NYX_DEVICE_INFO_GET_INFO_MODULE_METHOD, "device_info_get_info");
	nyx_module_register_method(instance, (nyx_device_t*) dinfo,
			NYX_DEVICE_INFO_QUERY_MODULE_METHOD, "device_info_query");

	*device = (nyx_device_t*) dinfo;

	return NYX_ERROR_NONE;

error:
	free(dinfo);
	*device = NULL;
	return error;
}

nyx_error_t nyx_module_close(nyx_device_handle_t device)
{
	struct device_info *dinfo;

	if (device == NULL)
		return NYX_ERROR_INVALID_VALUE;

	dinfo = (struct device_info*) device;

	if (dinfo->nduid)
		free(dinfo->nduid);

	if (dinfo->device_name)
		free(dinfo->device_name);

	if (dinfo->board_type)
		free(dinfo->board_type);

	if (dinfo->wifi_mac_address)
		free(dinfo->wifi_mac_address);

	if (dinfo->bt_mac_address)
		free(dinfo->bt_mac_address);

	if (dinfo->modem_present)
		free(dinfo->modem_present);

	free(dinfo);

	return NYX_ERROR_NONE;
}

NYX_DECLARE_MODULE(NYX_DEVICE_DEVICE_INFO, "DeviceInfo")
