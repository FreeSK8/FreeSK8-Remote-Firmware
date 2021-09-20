#ifndef USERSETTINGS_H_
#define USERSETTINGS_H_

#define SETTINGS_VERSION 2

typedef enum {
    SETTING_PIEZO = 0,
    SETTING_BUZZER,
    SETTING_SPEED,
    SETTING_TEMP,
    SETTING_THROTTLE,
    SETTING_MODEL,
    SETTING_LEFTY,
} SETTINGS_INDEX;

typedef enum {
	MODEL_UNDEFINED = 0,
	MODEL_ALBERT,
	MODEL_BRUCE,
	MODEL_CLINT,
} REMOTE_MODELS;

typedef struct {
	uint8_t settings_version;
	bool disable_piezo;
	bool disable_buzzer;
	bool display_mph;
	bool dispaly_fahrenheit;
	bool throttle_reverse;
	uint8_t remote_model; //1=Albert,2=Bruce,3=Clint
	bool left_handed;
} user_settings_t;

esp_err_t save_user_settings(user_settings_t * my_user_settings);
esp_err_t load_user_settings(user_settings_t * my_user_settings);

#endif