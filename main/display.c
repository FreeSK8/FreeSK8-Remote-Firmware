#include "display.h"

TickType_t drawScreenDeveloper(TFT_t * dev, FontxFile *fx, int width, int height) {
	static int yscroll = -50;
	if (++yscroll > 25)
	{
		yscroll = -50;
		lcdFillScreen(dev, BLACK);
	}
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	// get font width & height
	uint8_t buffer[FontxGlyphBufSize];
	uint8_t fontWidth;
	uint8_t fontHeight;
	GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);
	//ESP_LOGI(__FUNCTION__,"fontWidth=%d fontHeight=%d",fontWidth,fontHeight);
	
	uint16_t xpos;
	uint16_t ypos;
	//int	stlen;
	uint8_t ascii[24];
	uint16_t color;

	//lcdFillScreen(dev, BLACK);
	lcdSetFontFill(dev, BLACK);

	//strcpy((char *)ascii, "ST7789");
	sprintf((char *)ascii, "JOY0:%04d", adc_raw_joystick);
	if (width < height) {
		xpos = ((width - fontHeight) / 2) - 25;
		ypos = (height - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdSetFontDirection(dev, DIRECTION90);
	} else {
		ypos = ((height - fontHeight) / 2) - 25;
		xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdSetFontDirection(dev, DIRECTION0);
	}
	color = WHITE;
	lcdDrawString(dev, fx, xpos, ypos +yscroll, ascii, color);

	sprintf((char *)ascii, "JOY1:%04d", adc_raw_joystick_2);
	if (width < height) {
		xpos = ((width - fontHeight) / 2) + 0;
		ypos = (height - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdSetFontDirection(dev, DIRECTION90);
	} else {
		ypos = ((height - fontHeight) / 2) + 0;
		xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdSetFontDirection(dev, DIRECTION0);
	}
	color = WHITE;
	lcdDrawString(dev, fx, xpos, ypos +yscroll, ascii, color);

	sprintf((char *)ascii, "BATT:%04d", adc_raw_battery_level);
	if (width < height) {
		xpos = ((width - fontHeight) / 2) + 25;
		ypos = (height - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdSetFontDirection(dev, DIRECTION90);
	} else {
		ypos = ((height - fontHeight) / 2) + 25;
		xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdSetFontDirection(dev, DIRECTION0);
	}
	color = WHITE;
	lcdDrawString(dev, fx, xpos, ypos +yscroll, ascii, color);

	sprintf((char *)ascii, "RSSI:%04d", adc_raw_rssi);
	if (width < height) {
		xpos = ((width - fontHeight) / 2) + 50;
		ypos = (height - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdSetFontDirection(dev, DIRECTION90);
	} else {
		ypos = ((height - fontHeight) / 2) + 50;
		xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdSetFontDirection(dev, DIRECTION0);
	}
	color = WHITE;
	lcdDrawString(dev, fx, xpos, ypos +yscroll, ascii, color);
	


	sprintf((char *)ascii, "S:%d,%d,%d U:%d", gpio_switch_1, gpio_switch_2, gpio_switch_3, gpio_usb_detect);
	if (width < height) {
		xpos = ((width - fontHeight) / 2) + 75;
		ypos = (height - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdSetFontDirection(dev, DIRECTION90);
	} else {
		ypos = ((height - fontHeight) / 2) + 75;
		xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdSetFontDirection(dev, DIRECTION0);
	}
	color = WHITE;
	lcdDrawString(dev, fx, xpos, ypos +yscroll, ascii, color);

	sprintf((char *)ascii, "V%02.1f S%02.1f F%d", esc_telemetry.v_in, esc_telemetry.speed, esc_telemetry.fault_code);
	if (width < height) {
		xpos = ((width - fontHeight) / 2) + 100;
		ypos = (height - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdSetFontDirection(dev, DIRECTION90);
	} else {
		ypos = ((height - fontHeight) / 2) + 100;
		xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdSetFontDirection(dev, DIRECTION0);
	}
	color = WHITE;
	lcdDrawString(dev, fx, xpos, ypos +yscroll, ascii, color);

	sprintf((char *)ascii, "%0.1f%% Wh%0.2f", esc_telemetry.battery_level * 100, esc_telemetry.watt_hours);
	if (width < height) {
		xpos = ((width - fontHeight) / 2) + 125;
		ypos = (height - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdSetFontDirection(dev, DIRECTION90);
	} else {
		ypos = ((height - fontHeight) / 2) + 125;
		xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdSetFontDirection(dev, DIRECTION0);
	}
	color = WHITE;
	lcdDrawString(dev, fx, xpos, ypos +yscroll, ascii, color);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	//ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_RATE_MS);
	return diffTick;
}

static uint16_t adc_raw_battery_level_previous;
static uint16_t adc_raw_rssi_previous;
static double battery_level_previous;
static int tachometer_abs_previous;
TickType_t drawScreenPrimary(TFT_t * dev, FontxFile *fx, int width, int height) {
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	// get font width & height
	uint8_t buffer[FontxGlyphBufSize];
	uint8_t fontWidth = 9;
	uint8_t fontHeight = 9;
	GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);
	//ESP_LOGI(__FUNCTION__,"fontWidth=%d fontHeight=%d",fontWidth,fontHeight);

	uint16_t xpos;
	uint16_t ypos;
	//int	stlen;
	uint8_t ascii[24];
	uint16_t color;

	lcdSetFontFill(dev, BLACK);


	//Battery
	if (adc_raw_battery_level != adc_raw_battery_level_previous)
	{
		adc_raw_battery_level_previous = adc_raw_battery_level;
		//TODO: 1052 max && ~825 min?
		sprintf((char *)ascii, "%04d", adc_raw_battery_level);
		{
			ypos = 40;
			xpos = 20;
			lcdSetFontDirection(dev, DIRECTION0);
		}
		color = WHITE;
		lcdDrawString(dev, fx, xpos, ypos, ascii, color);
		//lcdDrawRect(dev, 20, 40, 80, 10, RED);
		//lcdDrawFillRect(dev, 5/*left*/, 5/*down*/, 80 /*width*/, 40 /*height*/, RED);
	}


	//FAULT
	if (esc_telemetry.fault_code != 0)
	{
		JPEGTest(dev, (char*)"/spiffs/map_fault.jpg", 25, 25, 100, 0);
	}


	//RSSI
	if (adc_raw_rssi != adc_raw_rssi_previous)
	{
		adc_raw_rssi_previous = adc_raw_rssi;
		sprintf((char *)ascii, "%04d", adc_raw_rssi);
		{
			ypos = 40;
			xpos = 160;
			lcdSetFontDirection(dev, DIRECTION0);
		}
		color = WHITE;
		lcdDrawString(dev, fx, xpos, ypos, ascii, color);
	}


	//Board Battery
	if (esc_telemetry.battery_level != battery_level_previous)
	{
		battery_level_previous = esc_telemetry.battery_level;
		sprintf((char *)ascii, "%0.1f%%", esc_telemetry.battery_level * 100);
		{
			ypos = (height - fontHeight);
			xpos = 5;
			lcdSetFontDirection(dev, DIRECTION0);
		}
		color = WHITE;
		lcdDrawString(dev, fx, xpos, ypos, ascii, color);
	}


	// Odometer
	if (esc_telemetry.tachometer_abs != tachometer_abs_previous)
	{
		tachometer_abs_previous = esc_telemetry.tachometer_abs;
		sprintf((char *)ascii, "%02.2fkm", esc_telemetry.tachometer_abs / 1000.0);
		{
			ypos = (height - fontHeight);
			xpos = 160;
			lcdSetFontDirection(dev, DIRECTION0);
		}
		color = WHITE;
		lcdDrawString(dev, fx, xpos, ypos, ascii, color);
	}

	// Speed big numbers
	fontWidth = 9;
	fontHeight = 9;
	const float metersPerSecondToKph = 3.6;
	const int kphToMph = 0.621371;
	sprintf((char *)ascii, "%02d", (int)(esc_telemetry.speed * metersPerSecondToKph * kphToMph));
	{
		ypos = 42;
		xpos = (width - (strlen((char *)ascii) * 8 * fontWidth)) / 2;
		lcdSetFontDirection(dev, DIRECTION0);
	}
	color = YELLOW;
	lcdDrawString2(dev, fontHeight, fontWidth, xpos, ypos, ascii, color);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	//ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_RATE_MS);
	return diffTick;
}

TickType_t JPEGTest(TFT_t * dev, char * file, int width, int height, int offset_x, int offset_y) {
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	//lcdSetFontDirection(dev, 0);
	//lcdFillScreen(dev, BLACK);


	pixel_s **pixels;
	uint16_t imageWidth;
	uint16_t imageHeight;
	esp_err_t err = decode_image(&pixels, file, width, height, &imageWidth, &imageHeight);
	//ESP_LOGI(__FUNCTION__, "decode_image err=%d imageWidth=%d imageHeight=%d", err, imageWidth, imageHeight);
	if (err == ESP_OK) {

		uint16_t _width = width;
		uint16_t _cols = 0;
		if (width > imageWidth) {
			_width = imageWidth;
			_cols = (width - imageWidth) / 2;
		}
		//ESP_LOGD(__FUNCTION__, "_width=%d _cols=%d", _width, _cols);

		uint16_t _height = height;
		uint16_t _rows = 0;
		if (height > imageHeight) {
			_height = imageHeight;
			_rows = (height - imageHeight) / 2;
		}
		//ESP_LOGD(__FUNCTION__, "_height=%d _rows=%d", _height, _rows);
		uint16_t *colors = (uint16_t*)malloc(sizeof(uint16_t) * _width);

#if 0
		for(int y = 0; y < _height; y++){
			for(int x = 0;x < _width; x++){
				pixel_s pixel = pixels[y][x];
				uint16_t color = rgb565_conv(pixel.red, pixel.green, pixel.blue);
				lcdDrawPixel(dev, x+_cols, y+_rows, color);
			}
			vTaskDelay(1);
		}
#endif

		for(int y = 0; y < _height; y++){
			for(int x = 0;x < _width; x++){
				pixel_s pixel = pixels[y][x];
				colors[x] = rgb565_conv(pixel.red, pixel.green, pixel.blue);
			}
			lcdDrawMultiPixels(dev, _cols+offset_x, y+_rows+offset_y, _width, colors);
			vTaskDelay(1);
		}

		free(colors);
		release_image(&pixels, width, height);
		ESP_LOGD(__FUNCTION__, "Finish");
	}

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_RATE_MS);
	return diffTick;
}