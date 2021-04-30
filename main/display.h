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

TickType_t ArrowTest(TFT_t * dev, FontxFile *fx, int width, int height);
TickType_t JPEGTest(TFT_t * dev, char * file, int width, int height);

#endif