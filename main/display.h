#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "lib/st7789/st7789.h"
#include "lib/st7789/fontx.h"
#include "lib/st7789/bmpfile.h"
#include "lib/st7789/pngle.h"
#include "lib/st7789/decode_image.h"

#include "lib/vesc/datatypes.h"

#include "user-settings.h"

extern bool remote_in_pairing_mode;
extern char str_pairing_1[15];
extern char str_pairing_2[15];
extern char str_pairing_3[15];
extern char str_pairing_4[15];

extern uint16_t adc_raw_joystick;
extern uint16_t adc_raw_joystick_2;
extern uint16_t adc_raw_battery_level;
extern uint16_t adc_raw_rssi;
extern uint8_t joystick_value_mapped;

extern int gpio_switch_1;
extern int gpio_switch_2;
extern int gpio_switch_3;
extern int gpio_switch_detect;
extern int gpio_usb_detect;

extern TELEMETRY_DATA esc_telemetry;
extern TELEMETRY_DATA esc_telemetry_last_fault;

volatile bool alert_show; // Is it time to display an alert
volatile bool alert_visible; // Has alert been displayed
volatile bool alert_clear; // Has alert been cleared by user input

TickType_t drawScreenDeveloper(TFT_t * dev, FontxFile *fx, int width, int height);
TickType_t drawScreenPairing(TFT_t * dev, FontxFile *fx, int width, int height);
TickType_t drawScreenPrimary(TFT_t * dev, FontxFile *fx, int width, int height);
TickType_t drawAlert(TFT_t * dev, FontxFile *fx, uint16_t p_color, char * title, char * line1, char * line2, char * line3, char * line4);
TickType_t drawSetupMenu(TFT_t * dev, FontxFile *fx, user_settings_t *user_settings, uint8_t current_index);
TickType_t JPEGTest(TFT_t * dev, char * file, int width, int height, int offset_x, int offset_y);
void resetPreviousValues();

#endif