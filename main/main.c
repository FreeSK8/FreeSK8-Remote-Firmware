#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"

#include "lib/ADS1015/src/ADS1015.h"

#include "lib/st7789/st7789.h"
#include "lib/st7789/fontx.h"
#include "lib/st7789/bmpfile.h"
#include "lib/st7789/pngle.h"
#include "lib/st7789/decode_image.h"

#include "esp-i2c.h"
#include "display.h"

#include "lib/haptic/haptic.h"

const char * version = "0.1.2";

#define ADC_BATTERY_MIN 525

int gyro_x, gyro_y, gyro_z;
float accel_g_x, accel_g_x_delta;
float accel_g_y, accel_g_y_delta;
float accel_g_z, accel_g_z_delta;

bool is_remote_idle = true; //NOTE: Controlled by IMU
bool display_second_screen = false;
bool display_blank_now = false;

bool is_throttle_idle = false;
bool is_throttle_locked = false;

TickType_t esc_last_responded = 0;

/* User Settings */
#include "user-settings.h"
uint8_t user_settings_index = 0;
bool remote_in_setup_mode = false;
user_settings_t my_user_settings;
/* User Settings */

/* Piezo */
#include "lib/melody/melody.h"

#include "driver/ledc.h"
#if CONFIG_IDF_TARGET_ESP32
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (19)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#endif
#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_TEST_CH_NUM       (1)
#define LEDC_TEST_DUTY         (4000)
#define LEDC_TEST_FADE_TIME    (3000)
static void piezo_test(void *arg)
{
    int ch;

    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 5000,                      // frequency of PWM signal
        .speed_mode = LEDC_LS_MODE,           // timer mode
        .timer_num = LEDC_LS_TIMER,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);
#ifdef CONFIG_IDF_TARGET_ESP32
    // Prepare and set configuration of timer1 for low speed channels
    ledc_timer.speed_mode = LEDC_HS_MODE;
    ledc_timer.timer_num = LEDC_HS_TIMER;
    ledc_timer_config(&ledc_timer);
#endif
    /*
     * Prepare individual configuration
     * for each channel of LED Controller
     * by selecting:
     * - controller's channel number
     * - output duty cycle, set initially to 0
     * - GPIO number where LED is connected to
     * - speed mode, either high or low
     * - timer servicing selected channel
     *   Note: if different channels use one timer,
     *         then frequency and bit_num of these channels
     *         will be the same
     */
    ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
        {
            .channel    = LEDC_HS_CH0_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH0_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER
        }
    };

    // Set LED Controller with previously prepared configuration
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }

    // Initialize fade service.
    ledc_fade_func_install(0);

	if (!my_user_settings.disable_piezo) melody_play(MELODY_LOG_START, true);
	haptic_play(MELODY_LOG_START, true);
	TickType_t startTick = xTaskGetTickCount();
	TickType_t endTick, diffTick;
	bool was_remote_idle = false;

    while (1) {
		// Low Battery Alert
		if (adc_raw_battery_level < ADC_BATTERY_MIN || adc_raw_battery_level == ADS1015_ERROR)
		{
			melody_play(MELODY_GOTCHI_FAULT, false); //NOTE: disregard user preference
			//NOTE: No Haptics PLEASE
		}

		// Idle Throttle Alert
		if (is_throttle_idle) {
			melody_play(MELODY_BLE_FAIL, false); //NOTE: disregard user preference
			//NOTE: No Haptics
		}

		// Idle IMU Alert
		if (is_remote_idle)
		{
			if (!was_remote_idle)
			{
				was_remote_idle = true;
				startTick = xTaskGetTickCount();
			}
			if (!gpio_usb_detect)
			{
				// Check if we've been idle for more than 5 minutes
				endTick = xTaskGetTickCount();
				diffTick = endTick - startTick;
				if (diffTick*portTICK_RATE_MS > 5 * 60 * 1000)
				{
					if (!my_user_settings.disable_piezo) melody_play(MELODY_ESC_FAULT, false);
				}
			}
		}
		else 
		{
			was_remote_idle = false;
		}

		// Play melody if enabled
		//             or when battery is low
		//             or when remote is left unattended
		melody_step();
		
		vTaskDelay(10/portTICK_PERIOD_MS);
    }
}
/* Piezo */

/* IMU */
//https://github.com/gabrielbvicari/esp32-i2c_rw/tree/f532d6a554dc2f2daa3954b597072ecb48354688
#include "lib/MPU6050/mpu6050.h"
/* IMU */


/* GPIO */
#include "freertos/queue.h"
#include "driver/gpio.h"

#include "button.h"

#define GPIO_OUTPUT_IO_0    2  //LED
#define GPIO_OUTPUT_IO_1    32 //MCU_LATCH
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1))

#define GPIO_INPUT_IO_0     33 //SW3
#define GPIO_INPUT_IO_1     34 //SW1
#define GPIO_INPUT_IO_2     35 //SW2
#define GPIO_INPUT_IO_3		27 //SWITCH_DETECT
#define GPIO_INPUT_IO_4     23 //USB_PRES
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1) | (1ULL<<GPIO_INPUT_IO_2) | (1ULL<<GPIO_INPUT_IO_3))
#define GPIO_INPUT_PINS_UP  ((1ULL<<GPIO_INPUT_IO_4))

int gpio_switch_1 = 0;
int gpio_switch_2 = 0;
int gpio_switch_3 = 0;
int gpio_switch_detect = 0;
int gpio_usb_detect = 0;
static void gpio_input_task(void* arg)
{
    button_event_t ev;
	QueueHandle_t button_events = button_init(GPIO_INPUT_PIN_SEL, GPIO_INPUT_PINS_UP);

	gpio_switch_3 = !gpio_get_level(GPIO_INPUT_IO_0);
	gpio_switch_1 = !gpio_get_level(GPIO_INPUT_IO_1);
	gpio_switch_2 = !gpio_get_level(GPIO_INPUT_IO_2);
	gpio_switch_detect = !gpio_get_level(GPIO_INPUT_IO_3);
	gpio_usb_detect = !gpio_get_level(GPIO_INPUT_IO_4);

	// Check if we are to enter pairing mode
	if (!gpio_get_level(GPIO_INPUT_IO_0))
	{
		remote_in_pairing_mode = true;
	}

	while (true) {
		if (xQueueReceive(button_events, &ev, 1000/portTICK_PERIOD_MS)) {
			if ((ev.pin == GPIO_INPUT_IO_0) && (ev.event == BUTTON_DOWN)) {
				// User Button (SW3 on HW v1.2 PCB)
				if (remote_in_setup_mode)
				{
					switch(user_settings_index) 
					{
						case SETTING_PIEZO:
							my_user_settings.disable_piezo = !my_user_settings.disable_piezo;
						break;
						case SETTING_BUZZER:
							my_user_settings.disable_buzzer = !my_user_settings.disable_buzzer;
						break;
						case SETTING_SPEED:
							my_user_settings.display_mph = !my_user_settings.display_mph;
						break;
						case SETTING_TEMP:
							my_user_settings.dispaly_fahrenheit = !my_user_settings.dispaly_fahrenheit;
						break;
						case SETTING_THROTTLE:
							my_user_settings.throttle_reverse = !my_user_settings.throttle_reverse;
						break;
						case SETTING_MODEL:
							if (++my_user_settings.remote_model > MODEL_CLINT) my_user_settings.remote_model = MODEL_ALBERT;
						break;
					}
					save_user_settings(&my_user_settings);
					continue;
				}

				// No action desired when throttle is locked
				if (is_throttle_locked) continue;

				// Clear alert or change display
				if (alert_visible) alert_clear = true;
				else
				{
					//Switch between display views
					display_second_screen = !display_second_screen;
					display_blank_now = true;
				}
			}
			if ((ev.pin == GPIO_INPUT_IO_0) && (ev.event == BUTTON_DOUBLE_CLICK)) {
				// SW3 on HW v1.2 PCB
				is_throttle_locked = !is_throttle_locked; // Toggle throttle lock
				if (is_throttle_locked)
				{
					display_blank_now = true; // Clear display
					display_second_screen = false; // Request primary screen with throttle locked
					if (!my_user_settings.disable_piezo) melody_play(MELODY_GPS_LOST, true);
					haptic_play(MELODY_GPS_LOST, true);
				} else {
					display_blank_now = true; // Clear the display if we turn off throttle lock
					if (!my_user_settings.disable_piezo) melody_play(MELODY_GPS_LOCK, true);
					haptic_play(MELODY_GPS_LOCK, true);
				}
				ESP_LOGI(__FUNCTION__, "Throttle lock is %d", is_throttle_locked);
			}
			if ((ev.pin == GPIO_INPUT_IO_1) && (ev.event == BUTTON_DOWN)) {
				// SW1 on HW v1.2 PCB
			}
			if ((ev.pin == GPIO_INPUT_IO_2) && (ev.event == BUTTON_DOWN)) {
				// SW2 on HW v1.2 PCB
			}
			if ((ev.pin == GPIO_INPUT_IO_3) && (ev.event == BUTTON_HELD)) {
				if (!my_user_settings.disable_piezo) melody_play(MELODY_LOG_STOP, true);
				haptic_play(MELODY_LOG_STOP, true);
				ESP_LOGI(__FUNCTION__, "Setting MCU_LATCH to 0");
				/// Turn Power switch LED off
				gpio_set_level(GPIO_OUTPUT_IO_0, 0);
				/// Turn battery power off
				gpio_set_level(GPIO_OUTPUT_IO_1, 0);
			}
			if (ev.pin == GPIO_INPUT_IO_4) {
				gpio_usb_detect = (ev.event != BUTTON_UP);
			}
		}
	}
}
static void gpio_init_remote()
{
	gpio_config_t io_conf;

	/// OUTPUTS

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

	// Turn the battery power on
	gpio_set_level(GPIO_OUTPUT_IO_1, 1);

	/// Turn LED on with a blink
	gpio_set_level(GPIO_OUTPUT_IO_0, 1);
	vTaskDelay(500/portTICK_PERIOD_MS);
	gpio_set_level(GPIO_OUTPUT_IO_0, 0);
	vTaskDelay(500/portTICK_PERIOD_MS);
	gpio_set_level(GPIO_OUTPUT_IO_0, 1);

    //start gpio task
    xTaskCreate(gpio_input_task, "gpio_task", 2048, NULL, 10, NULL);
}
/* GPIO */



/* I2C Tasks */
#include "lib/ADS1015/src/ADS1015.h"

uint16_t adc_raw_joystick;
uint16_t adc_raw_joystick_2;
uint16_t adc_raw_battery_level = ADC_BATTERY_MIN;
uint16_t adc_raw_rssi;
#define JOYSTICK_OFF_CENTER 7
#define CENTER_JOYSTICK 127
uint8_t joystick_value_mapped = CENTER_JOYSTICK;
long map(long x, long in_min, long in_max, long out_min, long out_max);

static bool adc_is_first_read = true;

static void i2c_task(void *arg)
{
	mpu6050_rotation_t gyro;
	mpu6050_acceleration_t accel;
	float accel_bias[3] = {0, 0, 0};
	//float gyro_bias[3] = {0, 0, 0};

	int8_t range = mpu6050_get_full_scale_accel_range();
	float accel_res = mpu6050_get_accel_res(range);

	bool was_throttle_idle = false;
	TickType_t startTickThrottleIdle = xTaskGetTickCount();

	bool was_throttle_locked = false;

	ADS1015_init();
	while(1)
	{
		/* ADC */
		//TODO: osrr_state.adc.error = true;
		adc_raw_joystick = ADS1015_readADC_SingleEnded(0);
		// Update joystick value if throttle is not locked
		if (!is_throttle_locked && adc_raw_joystick != ADS1015_ERROR)
		{
			// Map throttle, checking for reversed user setting
			if (my_user_settings.throttle_reverse) joystick_value_mapped = 255 - map(adc_raw_joystick, 0, 1700, 0, 255);
			else joystick_value_mapped = map(adc_raw_joystick, 0, 1700, 0, 255);

			// On first read only
			if (adc_is_first_read)
			{
				adc_is_first_read = false;
				// Check for full stick input to enter setup mode
				if (joystick_value_mapped < 3 || joystick_value_mapped > 251) {
					ESP_LOGI(__FUNCTION__, "Remote entering setup mode (%d)", joystick_value_mapped);
					remote_in_setup_mode = true;
					display_blank_now = true;
				}
				// Check for center stick
				else if (joystick_value_mapped < CENTER_JOYSTICK - JOYSTICK_OFF_CENTER || joystick_value_mapped > CENTER_JOYSTICK + JOYSTICK_OFF_CENTER) {
					ESP_LOGW(__FUNCTION__, "Joystick not center (%d) on startup. Locking throttle", joystick_value_mapped);
					is_throttle_locked = true;
				}
			}

			// Check if throttle was locked so we can reset the idle throttle time
			if (was_throttle_locked) {
				was_throttle_locked = false;
				startTickThrottleIdle = xTaskGetTickCount(); // Reset idle throttle time
			}
		}
		else
		{
			// Throttle is locked
			joystick_value_mapped = CENTER_JOYSTICK; //NOTE: Zero input if is_throttle_locked
			was_throttle_locked = true;
		}

		// Check if joystick is center to determine if it's in use
		if (joystick_value_mapped > CENTER_JOYSTICK - JOYSTICK_OFF_CENTER && joystick_value_mapped < CENTER_JOYSTICK + JOYSTICK_OFF_CENTER)
		{
			if (!was_throttle_idle)
			{
				was_throttle_idle = true;
				startTickThrottleIdle = xTaskGetTickCount();
			}
			if (!gpio_usb_detect)
			{
				// Check if throttle has been idle for more than 10 minutes
				if ((xTaskGetTickCount() - startTickThrottleIdle)*portTICK_RATE_MS > 10 * 60 * 1000)
				{
					is_throttle_idle = true;
				}
			} else {
				// OSRR is charging
				startTickThrottleIdle = xTaskGetTickCount(); // Reset idle throttle time
			}
		} else {
			was_throttle_idle = false;
			is_throttle_idle = false;
		}

		adc_raw_battery_level = ADS1015_readADC_SingleEnded(2);
		adc_raw_joystick_2 = ADS1015_readADC_SingleEnded(1);
		adc_raw_rssi = ADS1015_readADC_SingleEnded(3);

		/* IMU */
		mpu6050_get_rotation(&gyro);
		mpu6050_get_acceleration(&accel);

		float accel_gx = (float) accel.accel_x * accel_res - accel_bias[0];
		float accel_gy = (float) accel.accel_y * accel_res - accel_bias[1];
		float accel_gz = (float) accel.accel_z * accel_res - accel_bias[2];

		accel_g_x_delta = accel_g_x - accel_gx;
		accel_g_y_delta = accel_g_y - accel_gy;
		accel_g_z_delta = accel_g_z - accel_gz;
		accel_g_x = (0.2 * accel_gx) + (0.8 * accel_g_x);
		accel_g_y = (0.2 * accel_gy) + (0.8 * accel_g_y);
		accel_g_z = (0.2 * accel_gz) + (0.8 * accel_g_z);

		gyro_x = gyro.gyro_x;
		gyro_y = gyro.gyro_y;
		gyro_z = gyro.gyro_z;
		//ESP_LOGI(__FUNCTION__, "ADC joy1:%d joy2:%d batt:%d rssi:%d IMU x:%f y:%f z:%f", adc_raw_joystick, adc_raw_joystick_2, adc_raw_battery_level, adc_raw_rssi, accel_g_x, accel_g_y, accel_g_z);
		//ESP_LOGI(__FUNCTION__, "IMU x:%d y:%d z:%d", gyro_x, gyro_y, gyro_z);

		if (accel_g_x == 0 && accel_g_y == 0 && accel_g_z == 0) {
			ESP_LOGE(__FUNCTION__, "IMU not responding");
			//TODO: osrr_state.imu.error = true;
		}

		/* Haptic */
		if (!my_user_settings.disable_buzzer) haptic_step();
	}
}
/* I2C Tasks */


/* VESC */
#include "packet.h"
#include "buffer.h"
#include "datatypes.h"

#define PACKET_VESC						0
TELEMETRY_DATA esc_telemetry;
TELEMETRY_DATA esc_telemetry_last_fault;

int uart_write_bytes();
static void uart_send_buffer(unsigned char *data, unsigned int len) {
	uart_write_bytes(1/*TODO: undefined: XBEE_UART_PORT_NUM*/, data, len);
}
void process_packet_vesc(unsigned char *data, unsigned int len) {
	esc_last_responded = xTaskGetTickCount();
	if (data[0] == COMM_GET_VALUES_SETUP)
	{
		int index = 1;
		esc_telemetry.temp_mos = buffer_get_float16(data, 10.0, &index);
		esc_telemetry.temp_motor = buffer_get_float16(data, 10.0, &index);
		esc_telemetry.current_motor = buffer_get_float32(data, 100.0, &index);
		esc_telemetry.current_in = buffer_get_float32(data, 100.0, &index);
		esc_telemetry.duty_now = buffer_get_float16(data, 1000.0, &index);
		esc_telemetry.rpm = buffer_get_float32(data, 1.0, &index);
		esc_telemetry.speed = buffer_get_float32(data, 1000.0, &index);
		esc_telemetry.v_in = buffer_get_float16(data, 10.0, &index);
		esc_telemetry.battery_level = buffer_get_float16(data, 1000.0, &index);
		esc_telemetry.amp_hours = buffer_get_float32(data, 10000.0, &index);
		esc_telemetry.amp_hours_charged = buffer_get_float32(data, 10000.0, &index);
		esc_telemetry.watt_hours = buffer_get_float32(data, 10000.0, &index);
		esc_telemetry.watt_hours_charged = buffer_get_float32(data, 10000.0, &index);
		esc_telemetry.tachometer = buffer_get_float32(data, 1000.0, &index);
		esc_telemetry.tachometer_abs = buffer_get_float32(data, 1000.0, &index);
		esc_telemetry.position = buffer_get_float32(data, 1e6, &index);
		esc_telemetry.fault_code = data[index++];
		esc_telemetry.vesc_id = data[index++];
		esc_telemetry.num_vescs = data[index++];
		esc_telemetry.battery_wh = buffer_get_float32(data, 1000.0, &index);

		if (esc_telemetry.fault_code != FAULT_CODE_NONE) {
			esc_telemetry_last_fault = esc_telemetry;
			if (!alert_visible)
			{
				alert_show = true;
				display_blank_now = true;
				display_second_screen = false;
			}
			if (!my_user_settings.disable_piezo) melody_play(MELODY_ESC_FAULT, false);
			haptic_play(MELODY_ESC_FAULT, false);
		}
		ESP_LOGI(__FUNCTION__, "Temp ESC %f Motor %f, Current In %f Out %f, Speed %f, Voltage %f Fault %d ESCs %d", esc_telemetry.temp_mos, esc_telemetry.temp_motor, esc_telemetry.current_in, esc_telemetry.current_motor, esc_telemetry.speed, esc_telemetry.v_in, esc_telemetry.fault_code, esc_telemetry.num_vescs);
	} else {
		ESP_LOGW(__FUNCTION__, "Unknown message type received from ESC (%d)", data[0]);
	}
}
/* VESC */

/* XBEE/UART */
#include "espnow.h"

#include "driver/uart.h"
#include "lib/vesc/buffer.h"
#include "lib/vesc/crc.h"

#define XBEE_TXD 16
#define XBEE_RXD 17
#define XBEE_BAUD 115200
#define XBEE_UART_PORT_NUM 1
#define XBEE_BUF_SIZE (1024)

char str_pairing_1[15] = {0};
char str_pairing_2[15] = {0};
char str_pairing_3[15] = {0};
char str_pairing_4[15] = {0};
bool remote_in_pairing_mode = false;
uint8_t remote_xbee_ch = 0x0B; // Valid range 0x0B - 0x1A (11-26)
uint16_t remote_xbee_id = 0x7FFF; // Valid range 0x0 - 0xFFFF

static void xbee_send_string(unsigned char *data) {
	unsigned int len = strlen((char*)data);
	uart_write_bytes(XBEE_UART_PORT_NUM, data, len);
}

#include "esp_wifi.h"
#include "nvs_flash.h"
void xbee_init(void)
{
	uint8_t mac[6] = {0};
	esp_wifi_get_mac(ESP_IF_WIFI_AP, &mac[0]); //24 0a c4 e0 df c9 aka 39628671344585
	printf("mac address %02x%02x%02x%02x%02x%02x\n", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
	
	uint64_t mac_long = ((uint64_t)mac[5] | ((uint64_t)mac[4] << 8) | ((uint64_t)mac[3] << 16) | ((uint64_t)mac[2] << 24)) | ((uint64_t)mac[1] << 32) | ((uint64_t)mac[0] << 40);
	printf("mac long %llu\n", mac_long);

	remote_xbee_id = mac_long % 0xFFFF;
	printf("XBEE ID 0x%04x\n", remote_xbee_id);

	remote_xbee_ch = mac_long % 0x1A;
	if (remote_xbee_ch < 0x0B) remote_xbee_ch += 0x0B;
	printf("XBEE CH 0x%02x\n", remote_xbee_ch);
}
/* WiFi should start before using ESPNOW */
static void example_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESP_IF_WIFI_AP));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void setNunchuckValues();
int packSendPayload(uint8_t * payload, int lenPay);

static bool xbee_wait_ok(uint8_t *data, bool is_fatal)
{
	TickType_t startTick = xTaskGetTickCount();
	TickType_t endTick, diffTick;
	bool receivedOK = false;
	bool receivedO = false;
	bool receivedK = false;
	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	while(!receivedOK && diffTick*portTICK_RATE_MS < 1500)
	{
		int length = uart_read_bytes(XBEE_UART_PORT_NUM, data, XBEE_BUF_SIZE, 20 / portTICK_RATE_MS);
		if (length) {
			for (int i = 0; i < length; i++) {
				if (data[i] == 0x4F) receivedO = true;
				if (data[i] == 0x4B) receivedK = true;
			}
		}
		if (receivedO && receivedK) receivedOK = true;

		endTick = xTaskGetTickCount();
		diffTick = endTick - startTick;
	}
	if (is_fatal && !receivedOK)
	{
		ESP_LOGE(__FUNCTION__,"XBEE did not OK! Haulting");
		while(1) {
			vTaskDelay(1000/portTICK_PERIOD_MS);
		}
	}
	return receivedOK;
}
static void xbee_task(void *arg)
{
	//TODO: Wait for ADC task to read Joystick and determine Setup Mode (don't send full throttle) or Throttle Lock
	vTaskDelay(250/portTICK_PERIOD_MS);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = XBEE_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(XBEE_UART_PORT_NUM, XBEE_BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(XBEE_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(XBEE_UART_PORT_NUM, XBEE_TXD, XBEE_RXD, 0, 0));

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(XBEE_BUF_SIZE);


	packet_init(uart_send_buffer, process_packet_vesc, PACKET_VESC);

	uint8_t limiter = 0;
    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(XBEE_UART_PORT_NUM, data, XBEE_BUF_SIZE, 20 / portTICK_RATE_MS);

		if (remote_in_setup_mode)
		{
			// Do not communicate with the receiver while in setup mode
			vTaskDelay(100/portTICK_PERIOD_MS);
			continue;
		}

		if (remote_in_pairing_mode)
		{
			//NOTE: Must execute after nvs_flash_init
			example_wifi_init(); // Enable WiFi for pairing mode
			xbee_init(); // Generate unique XBee ID & CH
			// Generate random XBee MY & DL
			uint16_t xbee_my_address = esp_random() % 0xFFFF;
			uint16_t xbee_rx_address = esp_random() % 0xFFFF;
			ESP_LOGI(__FUNCTION__,"Entering Pairing Mode");

			ESP_LOGI(__FUNCTION__,"Configuring radio");

			vTaskDelay(1000/portTICK_PERIOD_MS);
			xbee_send_string((unsigned char *)"+++");
			//TODO: Check for OK message from XBEE at all baud rates
			if (!xbee_wait_ok(data, false)) {
				sprintf(str_pairing_1, "Radio did not");
				sprintf(str_pairing_2, "respond");
				sprintf(str_pairing_3, "Power cycle to");
				sprintf(str_pairing_4, "try again");
				while(1) {
					vTaskDelay(100/portTICK_PERIOD_MS);
				}
			} else {
				sprintf(str_pairing_1, "Radio OK");
			}
			ESP_LOGI(__FUNCTION__,"XBEE READY");

			bool configuration_success = true;
			unsigned char write_data[10] = {0};
			sprintf((char*)write_data, "ATCH%02x\r", remote_xbee_ch); // Network Channel
			xbee_send_string(write_data);
			if (configuration_success) configuration_success = xbee_wait_ok(data, false);

			sprintf((char*)write_data, "ATID%04x\r", remote_xbee_id); // Network ID
			xbee_send_string(write_data);
			if (configuration_success) configuration_success = xbee_wait_ok(data, false);

			xbee_send_string((unsigned char*)"ATDH0\r"); // Destination High is 0
			if (configuration_success) configuration_success = xbee_wait_ok(data, false);

			sprintf((char*)write_data, "ATDL%04x\r", xbee_rx_address); // Destination Low is randomized
			xbee_send_string(write_data);
			if (configuration_success) configuration_success = xbee_wait_ok(data, false);

			sprintf((char*)write_data, "ATMY%04x\r", xbee_my_address); // My Address is randomized
			xbee_send_string(write_data);
			if (configuration_success) configuration_success = xbee_wait_ok(data, false);

			xbee_send_string((unsigned char*)"ATBD7\r"); // Baud 115200
			if (configuration_success) configuration_success = xbee_wait_ok(data, false);

			xbee_send_string((unsigned char*)"ATD70\r"); // Digital IO7 is Disabled
			if (configuration_success) configuration_success = xbee_wait_ok(data, false);

			xbee_send_string((unsigned char*)"ATWR\r"); // Write configuration
			if (configuration_success) configuration_success = xbee_wait_ok(data, false);

			xbee_send_string((unsigned char*)"ATCN\r"); // Exit Command mode
			if (configuration_success) configuration_success = xbee_wait_ok(data, false);

			if (configuration_success) {
				sprintf(str_pairing_2, "Config OK");
				sprintf(str_pairing_3, "Searching for");
				sprintf(str_pairing_4, "receiver");
			} else {
				sprintf(str_pairing_2, "Config Failed");
				while(1) {
					vTaskDelay(100/portTICK_PERIOD_MS);
				}
			}

			printf("Starting ESPNOW\n");
			// Pass XBEE CH, ID, MY, DL for transmission to receiver
			if (example_espnow_init(remote_xbee_ch, remote_xbee_id, xbee_my_address, xbee_rx_address, NULL) == ESP_OK)
			{
				display_second_screen = false; //NOTE: Pairing button on boot will set display_second_screen true
				display_blank_now = true;
				sprintf(str_pairing_3, "Pairing was");
				sprintf(str_pairing_4, "Successful");
				if (!my_user_settings.disable_piezo) melody_play(MELODY_BLE_SUCCESS, true);
				haptic_play(MELODY_BLE_SUCCESS, true);
				vTaskDelay(3000/portTICK_PERIOD_MS); // Wait a few seconds
			}
			else
			{
				display_blank_now = true;
				sprintf(str_pairing_3, "Pairing");
				sprintf(str_pairing_4, "FAILED");
				if (!my_user_settings.disable_piezo) melody_play(MELODY_BLE_FAIL, true);
				haptic_play(MELODY_BLE_FAIL, true);
				vTaskDelay(10000/portTICK_PERIOD_MS);
			}
			display_blank_now = true;

			printf("Exiting Pairing Mode\n");
			// Pairing complete
			remote_in_pairing_mode = false;

			// Turn off wifi to save power after pairing
			ESP_ERROR_CHECK(esp_wifi_stop());
		}
		else
		{
			if (len) {
				//printf("UART Received %d bytes\n", len);
				for (int i = 0;i < len;i++) {
					packet_process_byte(data[i], PACKET_VESC);
				}
			}

			setNunchuckValues();

			//Request telemetry
			if (++limiter > 10)
			{
				//printf("requesting telemetry\n");
				limiter = 0;
				uint8_t command[1] = { COMM_GET_VALUES_SETUP };
				packSendPayload(command, 1);
			}
		}
		
		vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

void setNunchuckValues() {
	int32_t ind = 0;
	uint8_t payload[11];

	payload[ind++] = COMM_SET_CHUCK_DATA;
	payload[ind++] = 127;//nunchuck.valueX;
	payload[ind++] = joystick_value_mapped;//nunchuck.valueY;
	buffer_append_bool(payload, 0/*nunchuck.lowerButton*/, &ind);
	buffer_append_bool(payload, 0/*nunchuck.upperButton*/, &ind);

	// Acceleration Data. Not used, Int16 (2 byte)
	payload[ind++] = 0;
	payload[ind++] = 0;
	payload[ind++] = 0;
	payload[ind++] = 0;
	payload[ind++] = 0;
	payload[ind++] = 0;

	packSendPayload(payload, 11);
}

int packSendPayload(uint8_t * payload, int lenPay) {

	uint16_t crcPayload = crc16(payload, lenPay);
	int count = 0;
	uint8_t messageSend[256];

	if (lenPay <= 256)
	{
		messageSend[count++] = 2;
		messageSend[count++] = lenPay;
	}
	else
	{
		messageSend[count++] = 3;
		messageSend[count++] = (uint8_t)(lenPay >> 8);
		messageSend[count++] = (uint8_t)(lenPay & 0xFF);
	}

	memcpy(&messageSend[count], payload, lenPay);

	count += lenPay;
	messageSend[count++] = (uint8_t)(crcPayload >> 8);
	messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
	messageSend[count++] = 3;
	messageSend[count] = '\0';


	// Sending package
	uart_write_bytes(XBEE_UART_PORT_NUM, messageSend, count);

	// Returns number of send bytes
	return count;
}
long map(long x, long in_min, long in_max, long out_min, long out_max) {
	if (x < in_min) x = in_min;
	if (x > in_max) x = in_max;
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/* UART */


/* DISPLAY */
static const char *TAG = "ST7789";

static void SPIFFS_Directory(char * path) {
	DIR* dir = opendir(path);
	assert(dir != NULL);
	while (true) {
		struct dirent*pe = readdir(dir);
		if (!pe) break;
		ESP_LOGI(__FUNCTION__,"d_name=%s d_ino=%d d_type=%x", pe->d_name,pe->d_ino, pe->d_type);
	}
	closedir(dir);
}

void ST7789_Task(void *pvParameters)
{
	// set font file
	FontxFile fx16G[2];
	FontxFile fx24G[2];
	FontxFile fx32G[2];
	FontxFile fx32L[2];
	InitFontx(fx16G,"/spiffs/ILGH16XB.FNT",""); // 8x16Dot Gothic
	InitFontx(fx24G,"/spiffs/ILGH24XB.FNT",""); // 12x24Dot Gothic
	InitFontx(fx32G,"/spiffs/ILGH32XB.FNT",""); // 16x32Dot Gothic
	InitFontx(fx32L,"/spiffs/LATIN32B.FNT",""); // 16x32Dot Latin

	FontxFile fx16M[2];
	FontxFile fx24M[2];
	FontxFile fx32M[2];
	InitFontx(fx16M,"/spiffs/ILMH16XB.FNT",""); // 8x16Dot Mincyo
	InitFontx(fx24M,"/spiffs/ILMH24XB.FNT",""); // 12x24Dot Mincyo
	InitFontx(fx32M,"/spiffs/ILMH32XB.FNT",""); // 16x32Dot Mincyo
	
	TFT_t dev;
	spi_master_init(&dev, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO, CONFIG_CS_GPIO, CONFIG_DC_GPIO, CONFIG_RESET_GPIO, CONFIG_BL_GPIO);
	lcdInit(&dev, CONFIG_WIDTH, CONFIG_HEIGHT, CONFIG_OFFSETX, CONFIG_OFFSETY);

#if CONFIG_INVERSION
	ESP_LOGI(TAG, "Enable Display Inversion");
	lcdInversionOn(&dev);
#endif

	// FreeSK8 Logo
	lcdFillScreen(&dev, BLACK);
	drawFirmwareVersion(&dev, (char *)version);
	if (remote_in_pairing_mode) {
		drawJPEG(&dev, (char*)"/spiffs/logo_badge_pairing.jpg", 206, 179, 17, 63);
	} else {
		drawJPEG(&dev, (char*)"/spiffs/logo_badge.jpg", 206, 114, 17, 63);
	}

	vTaskDelay(1000/portTICK_PERIOD_MS);
	lcdFillScreen(&dev, BLACK);

	bool is_display_visible = true;
	TickType_t remote_is_idle_tick = xTaskGetTickCount();
	TickType_t remote_is_visible_tick = xTaskGetTickCount();
	while(1) {
		// Check for setup mode
		if (remote_in_setup_mode)
		{
			// Move settings selection up and down
			if (joystick_value_mapped < 55 && user_settings_index < SETTING_MODEL) ++user_settings_index;
			if (joystick_value_mapped > 200 && user_settings_index > SETTING_PIEZO) --user_settings_index;

			lcdBacklightOn(&dev);
			if (display_blank_now)
			{
				display_blank_now = false;
				lcdFillScreen(&dev, BLACK);
			}
			drawSetupMenu(&dev, fx24G, &my_user_settings, user_settings_index);
			vTaskDelay(10/portTICK_PERIOD_MS);
			continue;
		}
		// Check for pairing mode
		if (remote_in_pairing_mode)
		{
			lcdBacklightOn(&dev);
			if (display_blank_now)
			{
				display_blank_now = false;
				lcdFillScreen(&dev, BLACK);
			}
			drawScreenPairing(&dev, fx24G, CONFIG_WIDTH, CONFIG_HEIGHT);
			vTaskDelay(10/portTICK_PERIOD_MS);
			continue;
		}

		// Check for functional IMU
		//TODO: if (osrr_state.imu.error == true){}
		if (accel_g_x == 0 && accel_g_y == 0 && accel_g_z == 0) {
			// IMU Not functional, leave display on
			is_display_visible = true;
			is_remote_idle = false;
		} else {
			// Check if remote is idle via IMU
			const int gyro_threshold = 1000; //TODO: define threshold
			if (abs(gyro_x) > gyro_threshold || abs(gyro_y) > gyro_threshold || abs(gyro_z) > gyro_threshold)
			{
				remote_is_idle_tick = xTaskGetTickCount();
				// Check if we were idle and reset the display values
				if (is_remote_idle) {
					printf("remote is moving %d, %d, %d\n", gyro_x, gyro_y, gyro_z);
					resetPreviousValues();
				}
				is_remote_idle = false;
			}
			// Check if Gyro has been idle for more than 10 seconds
			else if ((xTaskGetTickCount() - remote_is_idle_tick)*portTICK_RATE_MS > 10 * 1000)
			{
				is_remote_idle = true;
				//NOTE: helpful sometimes.. printf("remote is idle %d, %d, %d\n", gyro_x, gyro_y, gyro_z);
			}
			// Check if remote is in a visible orientation
			switch (my_user_settings.remote_model)
			{
				//TODO: left vs right handed values: my_user_settings.throttle_reverse ?
				case MODEL_ALBERT:
					if (accel_g_x > 0.7 || accel_g_z > 0.4) {
						if ((xTaskGetTickCount() - remote_is_visible_tick)*portTICK_RATE_MS > 500) {
							is_display_visible = false;
						}
					}
					else {
						remote_is_visible_tick = xTaskGetTickCount();
						is_display_visible = true;
					}
				break;
				case MODEL_BRUCE:
					if (accel_g_y > 0.5 || accel_g_z > 0.3) {
						if ((xTaskGetTickCount() - remote_is_visible_tick)*portTICK_RATE_MS > 500) {
							is_display_visible = false;
						}
					}
					else {
						remote_is_visible_tick = xTaskGetTickCount();
						is_display_visible = true;
					}
				break;
				case MODEL_CLINT:
					//TODO: estimated
					if (accel_g_y > 0.5 || accel_g_x > 0.6) {
						if ((xTaskGetTickCount() - remote_is_visible_tick)*portTICK_RATE_MS > 500) {
							is_display_visible = false;
						}
					}
					else {
						remote_is_visible_tick = xTaskGetTickCount();
						is_display_visible = true;
					}
				break;
			}
		}

		// Switch the backlight to save power
		if (is_remote_idle || !is_display_visible)
		{
			lcdBacklightOff(&dev);
		}
		else
		{
			lcdBacklightOn(&dev);
		}

		// Draw a blank screen when the remote is idle
		if (is_remote_idle)
		{
			lcdFillScreen(&dev, BLACK);
			alert_visible = false;
		}
		else
		// Draw the good stuff otherwise
		{
			if (display_blank_now)
			{
				display_blank_now = false;
				lcdFillScreen(&dev, BLACK);
				alert_visible = false;
				resetPreviousValues();
			}
			// Draw primary or secondary display
			if (display_second_screen && !alert_visible)
			{
				drawScreenSecondary(&dev, fx24G, CONFIG_WIDTH, CONFIG_HEIGHT, &my_user_settings);
			}
			else if (is_throttle_locked && !alert_visible)
			{
				// Display throttle locked alert over primary screen
				drawAlert(&dev, fx24G, RED, "Throttle", "locked", "", "Double click", "to unlock");
				drawScreenPrimary(&dev, fx24G, CONFIG_WIDTH, CONFIG_HEIGHT, &my_user_settings);
			}
			else
			{
				drawScreenPrimary(&dev, fx24G, CONFIG_WIDTH, CONFIG_HEIGHT, &my_user_settings);
			}
		}

		vTaskDelay(10/portTICK_PERIOD_MS);
	}
}

/* DISPLAY */

void app_main(void)
{
	gpio_init_remote();
	i2c_master_init();
	
#if 0
		//TODO: Calibrate does not work. what fails? idk
		mpu6050_acceleration_t accel;
		mpu6050_reset();
        mpu6050_calibrate(accel_bias, gyro_bias);
        ESP_LOGI(mpu6050_get_tag(), "Device being calibrated.");
        mpu6050_init();
        ESP_LOGI(mpu6050_get_tag(), "Device initialized.");
		while(true)
		{
			mpu6050_get_acceleration(&accel);
				printf("%d %d %d\n", accel.accel_x, accel.accel_y, accel.accel_y);
			vTaskDelay(10/portTICK_PERIOD_MS);
		}
#endif

	// Minimum mpu6050 init to get uncalibrated values
	mpu6050_init();
	mpu6050_set_full_scale_accel_range(MPU6050_ACCEL_FULL_SCALE_RANGE_16);

	ESP_LOGI(TAG, "Initializing SPIFFS");

	esp_vfs_spiffs_conf_t conf = {
		.base_path = "/spiffs",
		.partition_label = NULL,
		.max_files = 10,
		.format_if_mount_failed =true
	};

	// Use settings defined above toinitialize and mount SPIFFS filesystem.
	// Note: esp_vfs_spiffs_register is anall-in-one convenience function.
	esp_err_t ret = esp_vfs_spiffs_register(&conf);

	if (ret != ESP_OK) {
		if (ret == ESP_FAIL) {
			ESP_LOGE(TAG, "Failed to mount or format filesystem");
		} else if (ret == ESP_ERR_NOT_FOUND) {
			ESP_LOGE(TAG, "Failed to find SPIFFS partition");
		} else {
			ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)",esp_err_to_name(ret));
		}
		return;
	}

	size_t total = 0, used = 0;
	ret = esp_spiffs_info(NULL, &total,&used);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to get SPIFFS partition information (%s)",esp_err_to_name(ret));
	} else {
		ESP_LOGI(TAG,"Partition size: total: %d, used: %d", total, used);
	}

	SPIFFS_Directory("/spiffs/");

/* ESPNOW/Wifi */
// Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

/* ESPNOW/Wifi */

	// Grab the user settings on boot
	ret = load_user_settings(&my_user_settings);
	ESP_LOGI(__FUNCTION__, "Load user settings returned: %s", esp_err_to_name(ret));
	while (ret != ESP_OK)
	{
		my_user_settings.settings_version = SETTINGS_VERSION;
		my_user_settings.remote_model = MODEL_BRUCE;
		save_user_settings(&my_user_settings);
		ret = load_user_settings(&my_user_settings);
	}
	ESP_LOGI(__FUNCTION__, "Settings version=%d model=%d buzzer=%d piezo=%d fahrenheit=%d mph=%d", my_user_settings.settings_version, my_user_settings.remote_model, !my_user_settings.disable_buzzer, !my_user_settings.disable_piezo, my_user_settings.dispaly_fahrenheit, my_user_settings.display_mph);

	// Display task
	xTaskCreate(ST7789_Task, "display_task", 1024*4, NULL, 2, NULL);

	// I2C task (ADC/IMU)
	xTaskCreate(i2c_task, "i2c_task", 1024 * 2, NULL, 10, NULL);
	
	// Xbee task
	xTaskCreate(xbee_task, "xbee_task", 1024 * 4, NULL, 10, NULL);

	// Piezo task
	xTaskCreate(piezo_test, "piezo_test", 1024 * 2, NULL, 10, NULL);

	// Just chillin
	while(1)
	{
		vTaskDelay(100/portTICK_PERIOD_MS);
	}
}
