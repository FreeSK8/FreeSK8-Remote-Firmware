#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "button.h"

#define TAG "BUTTON"

typedef struct {
  uint8_t pin;
  bool inverted;
  uint16_t history;
  uint32_t down_time;
  uint32_t next_long_time;
  uint32_t single_click_time;
  bool single_click;
  bool double_click;
} debounce_t;

int pin_count = -1;
debounce_t * debounce;
QueueHandle_t queue;

static void update_button(debounce_t *d) {
    d->history = (d->history << 1) | gpio_get_level(d->pin);
}

#define MASK   0b1111000000111111
static bool button_rose(debounce_t *d) {
    if ((d->history & MASK) == 0b0000000000111111) {
        d->history = 0xffff;
        return 1;
    }
    return 0;
}
static bool button_fell(debounce_t *d) {
    if ((d->history & MASK) == 0b1111000000000000) {
        d->history = 0x0000;
        return 1;
    }
    return 0;
}
static bool button_down(debounce_t *d) {
    if (d->inverted) return button_fell(d);
    return button_rose(d);
}
static bool button_up(debounce_t *d) {
    if (d->inverted) return button_rose(d);
    return button_fell(d);
}

#define LONG_PRESS_DURATION (2000)
#define LONG_PRESS_REPEAT (1000)
#define DOUBLE_PRESS_TIMEOUT (250)

static uint32_t millis() {
    return esp_timer_get_time() / 1000;
}

static void send_event(debounce_t db, int ev) {
    button_event_t event = {
        .pin = db.pin,
        .event = ev,
    };
    xQueueSend(queue, &event, portMAX_DELAY);
}

static void button_task(void *pvParameter)
{
    for (;;) {
        for (int idx=0; idx<pin_count; idx++) {
            update_button(&debounce[idx]);
            if (debounce[idx].down_time && millis() >= debounce[idx].next_long_time) {
                debounce[idx].single_click = false;
                debounce[idx].double_click = false;
                //ESP_LOGI(TAG, "%d LONG", debounce[idx].pin);
                debounce[idx].next_long_time = debounce[idx].next_long_time + LONG_PRESS_REPEAT;
                send_event(debounce[idx], BUTTON_HELD);
            } else if (button_down(&debounce[idx]) && debounce[idx].down_time == 0) {
                if (millis() - debounce[idx].single_click_time < DOUBLE_PRESS_TIMEOUT && debounce[idx].single_click)
                {
                    debounce[idx].single_click = false;
                    debounce[idx].double_click = true;
                }
                else
                {
                    debounce[idx].single_click = true;
                    debounce[idx].single_click_time = millis();
                }
                ESP_LOGI(TAG, "%d DOWN", debounce[idx].pin);
                debounce[idx].down_time = millis();
                debounce[idx].next_long_time = debounce[idx].down_time + LONG_PRESS_DURATION;               
            } else if (button_up(&debounce[idx])) {
                debounce[idx].down_time = 0;
                ESP_LOGI(TAG, "%d UP", debounce[idx].pin);
                send_event(debounce[idx], BUTTON_UP);
            }
        }
        for (int idx=0; idx<pin_count; idx++) {
            // Check if we've single clicked
            // Have waiting until a doulbe click is not possible
            // Have released the button
            if (debounce[idx].single_click && millis() - debounce[idx].single_click_time > DOUBLE_PRESS_TIMEOUT && debounce[idx].down_time == 0)
            {
                debounce[idx].single_click_time = 0;
                debounce[idx].single_click = false;
                ESP_LOGI(TAG, "%d SINGLE CLICK", debounce[idx].pin);
                send_event(debounce[idx], BUTTON_DOWN);
            }
            else if (debounce[idx].double_click && debounce[idx].down_time == 0)
            {
                debounce[idx].single_click_time = 0;
                debounce[idx].double_click = false;
                ESP_LOGI(TAG, "%d DOUBLE CLICK", debounce[idx].pin);
                send_event(debounce[idx], BUTTON_DOUBLE_CLICK);
            }
        }
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}


QueueHandle_t button_init(unsigned long long pin_select, unsigned long long pin_select_pullups)
{
    if (pin_count != -1) {
        ESP_LOGI(TAG, "Already initialized");
        return NULL;
    }

    // Configure the pins
    gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 1;
    io_conf.pin_bit_mask = pin_select;
    gpio_config(&io_conf);

    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 1;
    io_conf.pin_bit_mask = pin_select_pullups;
    gpio_config(&io_conf);

    // Scan the pin map to determine number of pins
    pin_count = 0;
    for (int pin=0; pin<=39; pin++) {
        if (((1ULL<<pin) & pin_select) || ((1ULL<<pin) & pin_select_pullups)) {
            pin_count++;
        }
    }

    // Initialize global state and queue
    debounce = calloc(pin_count, sizeof(debounce_t));
    queue = xQueueCreate(4, sizeof(button_event_t));

    // Scan the pin map to determine each pin number, populate the state
    uint32_t idx = 0;
    for (int pin=0; pin<=39; pin++) {
        if (((1ULL<<pin) & pin_select) || ((1ULL<<pin) & pin_select_pullups)) {
            ESP_LOGI(TAG, "Registering button input: %d", pin);
            debounce[idx].pin = pin;
            debounce[idx].down_time = 0;
            debounce[idx].inverted = (pin != 27); //TODO: only pin 27 is not inverted?
            if (debounce[idx].inverted) debounce[idx].history = 0xffff;
            idx++;
        }
    }

    // Spawn a task to monitor the pins
    xTaskCreate(&button_task, "button_task", 4096, NULL, 10, NULL);

    return queue;
}
