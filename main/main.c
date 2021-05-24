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

#include "lib/st7789/st7789.h"
#include "lib/st7789/fontx.h"
#include "lib/st7789/bmpfile.h"
#include "lib/st7789/pngle.h"
#include "lib/st7789/decode_image.h"

#include "esp-i2c.h"
#include "display.h"

void print_haptic_registers();
void test_haptic_now();

float accel_g_x, accel_g_x_delta;
float accel_g_y, accel_g_y_delta;
float accel_g_z, accel_g_z_delta;


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

    while (1) {
        if (accel_g_x > 1.75) {
			//melody_play(MELODY_LOG_START, false);
		}
		if (accel_g_y > 1.75) {
			//melody_play(MELODY_STARTUP, false);
		}
		if (accel_g_z > 1.75) {
			//melody_play(MELODY_BLE_SUCCESS, false);
		}
		//melody_step();
		
		
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
#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

int gpio_switch_1 = 0;
int gpio_switch_2 = 0;
int gpio_switch_3 = 0;
int gpio_switch_detect = 0;
int gpio_usb_detect = 0;
TickType_t startTick, endTick, diffTick;
static void GPIO_Task(void* arg)
{
	gpio_switch_3 = !gpio_get_level(GPIO_INPUT_IO_0);
	gpio_switch_1 = !gpio_get_level(GPIO_INPUT_IO_1);
	gpio_switch_2 = !gpio_get_level(GPIO_INPUT_IO_2);
	gpio_switch_detect = !gpio_get_level(GPIO_INPUT_IO_3);
	gpio_usb_detect = !gpio_get_level(GPIO_INPUT_IO_4);

    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
			switch(io_num)
			{
				case GPIO_INPUT_IO_1:
					gpio_switch_1 = !gpio_get_level(io_num);
					printf("SW1 %d\n", gpio_switch_1);
					if (gpio_switch_1 == 0)
					{
						//Turn LED ON
						gpio_set_level(GPIO_OUTPUT_IO_0, 1);
					} 
					else
					{
						//Turn LED OFF
						gpio_set_level(GPIO_OUTPUT_IO_0, 0);
					}
				break;
				case GPIO_INPUT_IO_2:
					gpio_switch_2 = !gpio_get_level(io_num);
					printf("SW2 %d\n", gpio_switch_2);
					if (gpio_switch_2 == 0)
					{
						//test_haptic_now();
					}
				break;
				case GPIO_INPUT_IO_0:
					gpio_switch_3 = !gpio_get_level(io_num);
					printf("SW3%d\n", gpio_switch_3);
				break;
				case GPIO_INPUT_IO_3: //switch detect
					printf("SWITCH DETECT %d\n", gpio_get_level(io_num));
					// Make sure user holds power for one second before power off
					if(gpio_get_level(io_num) == 1)
					{
						// Time down
						startTick = xTaskGetTickCount();
					}
					else
					{
						endTick = xTaskGetTickCount();
						diffTick = endTick - startTick;
						ESP_LOGI(__FUNCTION__, "Power button elapsed time[ms]:%d",diffTick*portTICK_RATE_MS);

						if (diffTick*portTICK_RATE_MS > 1000)
						{
							/// Turn battery power off
							gpio_set_level(GPIO_OUTPUT_IO_1, 0);
						}
					}
				break;
				case GPIO_INPUT_IO_4:
					gpio_usb_detect = !gpio_get_level(io_num);
					printf("USB DETECT %d\n", gpio_usb_detect);
				break;
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
	vTaskDelay(500/10);
	gpio_set_level(GPIO_OUTPUT_IO_0, 0);
	vTaskDelay(500/10);
	gpio_set_level(GPIO_OUTPUT_IO_0, 1);

	/// INPUTS
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 0;
	io_conf.pull_down_en = 1;
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PINS_UP;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
	io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    //change gpio intrrupt type
    //gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);
	//gpio_set_intr_type(GPIO_INPUT_IO_1, GPIO_INTR_ANYEDGE);
	//gpio_set_intr_type(GPIO_INPUT_IO_2, GPIO_INTR_ANYEDGE);
	//gpio_set_intr_type(GPIO_INPUT_IO_3, GPIO_INTR_ANYEDGE);
	//gpio_set_intr_type(GPIO_INPUT_IO_4, GPIO_INTR_ANYEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(GPIO_Task, "gpio_task", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
    gpio_isr_handler_add(GPIO_INPUT_IO_2, gpio_isr_handler, (void*) GPIO_INPUT_IO_2);
	gpio_isr_handler_add(GPIO_INPUT_IO_3, gpio_isr_handler, (void*) GPIO_INPUT_IO_3);
	gpio_isr_handler_add(GPIO_INPUT_IO_4, gpio_isr_handler, (void*) GPIO_INPUT_IO_4);

    //remove isr handler for gpio number.
    ////gpio_isr_handler_remove(GPIO_INPUT_IO_0);
}
/* GPIO */



/* I2C Tasks */
#include "lib/ADS1015/src/ADS1015.h"

uint16_t adc_raw_joystick;
uint16_t adc_raw_joystick_2;
uint16_t adc_raw_battery_level;
uint16_t adc_raw_rssi;
uint8_t joystick_value_mapped = 127; // Center stick
long map(long x, long in_min, long in_max, long out_min, long out_max);



static void i2c_task(void *arg)
{
	mpu6050_acceleration_t accel;
	float accel_bias[3] = {0, 0, 0};
	float gyro_bias[3] = {0, 0, 0};

	int8_t range = mpu6050_get_full_scale_accel_range();
	float accel_res = mpu6050_get_accel_res(range);

	ADS1015_init();
	while(1)
	{
		/* Program State */
		printf("SW1:%d SW2:%d SW3:%d USB:%d JOY1:%04d JOY2:%04d BATT:%04d RSSI:%04d ", gpio_switch_1, gpio_switch_2, gpio_switch_3, gpio_usb_detect, adc_raw_joystick, adc_raw_joystick_2, adc_raw_battery_level, adc_raw_rssi);
		/* ADC */
		adc_raw_joystick = ADS1015_readADC_SingleEnded(0);
		joystick_value_mapped = map(adc_raw_joystick, 2, 1632, 0, 255);
		printf("Throttle = %d ", joystick_value_mapped);

		adc_raw_battery_level = ADS1015_readADC_SingleEnded(1);
		adc_raw_joystick_2 = ADS1015_readADC_SingleEnded(2);
		adc_raw_rssi = ADS1015_readADC_SingleEnded(3);

		/* IMU */

		mpu6050_get_acceleration(&accel);
		//printf("accel raw x%d y%d z%d\n", accel.accel_x, accel.accel_y, accel.accel_y);

		float accel_gx = (float) accel.accel_x * accel_res - accel_bias[0];
		float accel_gy = (float) accel.accel_y * accel_res - accel_bias[1];
		float accel_gz = (float) accel.accel_z * accel_res - accel_bias[2];

		if (accel_gx < 0)
			accel_gx *= -1;
		if (accel_gy < 0)
			accel_gy *= -1;
		if (accel_gz < 0)
			accel_gz *= -1;

		accel_g_x_delta = accel_g_x - accel_gx;
		accel_g_y_delta = accel_g_y - accel_gy;
		accel_g_z_delta = accel_g_z - accel_gz;
		accel_g_x = accel_gx;
		accel_g_y = accel_gy;
		accel_g_z = accel_gz;

		printf("accel G x%02f y%02f z%02f\n", accel_g_x_delta, accel_g_y_delta, accel_g_z_delta);


	}
}
/* I2C Tasks */


/* VESC */
#include "packet.h"
#include "buffer.h"
#include "datatypes.h"

#define PACKET_VESC						0
TELEMETRY_DATA esc_telemetry;

int uart_write_bytes();
static void uart_send_buffer(unsigned char *data, unsigned int len) {
	for (int i = 0;i < len;i++) {
		uart_write_bytes(0/*XBEE_UART_PORT_NUM*/, data, len);
	}
}
void process_packet_vesc(unsigned char *data, unsigned int len) {

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

		printf("temp esc %f\n", esc_telemetry.temp_mos);
		printf("temp motor %f\n", esc_telemetry.temp_motor);
		printf("current motor %f\n", esc_telemetry.current_motor);
		printf("current in %f\n", esc_telemetry.current_in);
		printf("speed %f\n", esc_telemetry.speed);
		printf("batt lvl %f\n", esc_telemetry.battery_level);
		printf("voltage  %f\n", esc_telemetry.v_in);
		printf("fault %d\n", esc_telemetry.fault_code);
		printf("num vescs %d\n", esc_telemetry.num_vescs);
	}
}
/* VESC */

/* XBEE/UART */
bool remote_in_pairing_mode = false;
uint8_t remote_xbee_ch = 0x0B; // Valid range 0x0B - 0x1A
uint16_t remote_xbee_id = 0x7FFF; // Valid range 0x0 - 0xFFFF

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
}
/* WiFi should start before using ESPNOW */
static void example_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESP_IF_WIFI_AP) );
    ESP_ERROR_CHECK( esp_wifi_start());
}

#include "driver/uart.h"
#include "lib/vesc/buffer.h"
#include "lib/vesc/crc.h"

#define XBEE_TXD 16
#define XBEE_RXD 17
#define XBEE_BAUD 115200
#define XBEE_UART_PORT_NUM 1
#define XBEE_BUF_SIZE (1024)

void setNunchuckValues();
int packSendPayload(uint8_t * payload, int lenPay);

static void xbee_task(void *arg)
{
	//example_wifi_init();
	//xbee_init();

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

		if (remote_in_pairing_mode)
		{
			printf("TODO: pairing mode\n");
		}
		else
		{
			if (len) {
				printf("UART Received %d bytes\n", len);
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

	payload[ind++] = 35; //COMM_SET_CHUCK_DATA;
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

// You have to set these CONFIG value using menuconfig.
#if 0
#define CONFIG_WIDTH  240
#define CONFIG_HEIGHT 240
#define CONFIG_MOSI_GPIO 23
#define CONFIG_SCLK_GPIO 18
#define CONFIG_CS_GPIO -1
#define CONFIG_DC_GPIO 19
#define CONFIG_RESET_GPIO 15
#define CONFIG_BL_GPIO -1
#endif

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

	lcdFillScreen(&dev, BLACK);
#if 0
		char file[32];
		strcpy(file, "/spiffs/esp32.jpeg");
		JPEGTest(&dev, file, CONFIG_WIDTH, CONFIG_HEIGHT);
		vTaskDelay(1000/10);
#endif
	lcdFillScreen(&dev, BLACK);

	bool is_remote_idle = true;
	uint8_t idle_delay = 0;
	while(1) {

		//drawScreenDeveloper(&dev, fx24M, CONFIG_WIDTH, CONFIG_HEIGHT);
		const float delta_threshold = 0.025;
		if (accel_g_x_delta > delta_threshold || accel_g_y_delta > delta_threshold || accel_g_z_delta > delta_threshold)
		{
			idle_delay = 0;
			is_remote_idle = false;
		}
		else if (++idle_delay > 30)
		{
			idle_delay = 0;
			is_remote_idle = true;
		}

		if (!is_remote_idle)
		{
			lcdBacklightOn(&dev);
			drawScreenPrimary(&dev, fx24M, CONFIG_WIDTH, CONFIG_HEIGHT);
			//drawScreenDeveloper(&dev, fx24M, CONFIG_WIDTH, CONFIG_HEIGHT);
		}
		else
		{
			//lcdFillScreen(&dev, BLACK);
			lcdBacklightOff(&dev);
			vTaskDelay(100/10);
		}

		vTaskDelay(10/10);
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

	//TODO: haptic fails :(
	//haptic_test();

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
	example_wifi_init();
	xbee_init();

/* ESPNOW/Wifi */

	// Display task
	xTaskCreate(ST7789_Task, "display_task", 1024*4, NULL, 2, NULL);

	// I2C task (ADC/IMU)
	xTaskCreate(i2c_task, "i2c_task", 1024 * 2, NULL, 10, NULL);
	
	// Xbee task
	xTaskCreate(xbee_task, "xbee_task", 1024 * 2, NULL, 10, NULL);

	// Piezo task
	xTaskCreate(piezo_test, "piezo_test", 1024 * 1, NULL, 10, NULL);

	// Just chillin
	while(1)
	{
		vTaskDelay(100/portTICK_PERIOD_MS);
	}
}