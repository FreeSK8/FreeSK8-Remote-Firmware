#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include <string.h>

#include "user-settings.h"

#define STORAGE_NAMESPACE "storage"

size_t settings_size = sizeof(user_settings_t);

esp_err_t save_user_settings(user_settings_t * my_user_settings)
{
	nvs_handle my_handle;
	esp_err_t err;

	// Open
	err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
	if (err != ESP_OK) return err;

	// Write value
	err = nvs_set_blob(my_handle, "user_settings", my_user_settings, settings_size);

	if (err != ESP_OK) return err;

	// Commit
	err = nvs_commit(my_handle);
	if (err != ESP_OK) return err;

	// Close
	nvs_close(my_handle);

	ESP_LOGI(__FUNCTION__, "User Settings Saved Successfully");

	return ESP_OK;
}

esp_err_t load_user_settings(user_settings_t * my_user_settings)
{
	user_settings_t loaded_settings;
	nvs_handle my_handle;
	esp_err_t err;

	// Open
	err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
	if (err != ESP_OK) {
		ESP_LOGE(__FUNCTION__, "nvs_open failed");
		return err;
	}

	// Read blob
	size_t required_size = 0;  // value will default to 0, if not set yet in NVS
	// obtain required memory space to store blob being read from NVS
	err = nvs_get_blob(my_handle, "user_settings", NULL, &required_size);
	if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

	if (required_size == 0) {
		printf("Nothing saved yet!\n");
		return ESP_FAIL;
	} else if (required_size != settings_size) {
		ESP_LOGW(__FUNCTION__, "Settings size mismatch: Saved=%d Expected=%d", required_size, settings_size);
		return ESP_FAIL;
	} else {
		err = nvs_get_blob(my_handle, "user_settings", &loaded_settings, &required_size);
		if (err != ESP_OK) {
			return err;
		}

		if (loaded_settings.settings_version != SETTINGS_VERSION)
		{
			ESP_LOGW(__FUNCTION__, "Settings version mismatch: Received=%d Expected=%d", loaded_settings.settings_version, SETTINGS_VERSION);
			return ESP_FAIL;
		}
	}

	// Close
	nvs_close(my_handle);

    memcpy(my_user_settings, &loaded_settings, sizeof(user_settings_t));

	ESP_LOGI(__FUNCTION__, "User Settings Loaded Successfully");

	return ESP_OK;
}
