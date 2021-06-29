#include "display.h"

extern long map(long x, long in_min, long in_max, long out_min, long out_max);

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

const int adc_raw_battery_minimum = 825;
const int adc_raw_battery_maximum = 1052;
const int adc_raw_rssi_maximum = 1639;
const int adc_raw_rssi_minimum = 850;
static uint8_t rssi_mapped_previous;
static uint8_t batt_pixel_previous;
static uint8_t gpio_usb_detect_previous;
static int tachometer_abs_previous;
static double esc_vin_previous;

void resetPreviousValues()
{
	tachometer_abs_previous = -1;
	rssi_mapped_previous = 0;
	batt_pixel_previous = 0;
	gpio_usb_detect_previous = 0;
	esc_vin_previous = 0;
}
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

	//Remote Battery
	const uint8_t batt_height = 20;
	uint8_t batt_pixel = map(adc_raw_battery_level, adc_raw_battery_minimum, adc_raw_battery_maximum, 10, 80); //Scale battery %
	if (batt_pixel != batt_pixel_previous)
	{
		lcdDrawRect(dev, 5, 5, 82, batt_height, BLUE); //Draw outline //TODO: Only draw once to reduce render time
		lcdDrawFillRect(dev, 82, batt_height -10, 86, batt_height -5, BLUE); //Draw nub //TODO: only once takes time blah blah
		color = GREEN;
		if (batt_pixel < 30) color = RED;
		else if (batt_pixel < 60) color = rgb565_conv(0xFF, 0xA5, 0x00);
		lcdDrawFillRect(dev, 6/*left*/, 6/*down*/, batt_pixel /*width*/, batt_height -2 /*height*/, color); //Draw battery %
		lcdDrawFillRect(dev, batt_pixel/*left*/, 6/*down*/, batt_pixel + (80-batt_pixel) /*width*/, batt_height -2 /*height*/, BLACK); //Clear empty space
	}
	// Remote is Charging
	if (gpio_usb_detect != gpio_usb_detect_previous)
	{
		gpio_usb_detect_previous = gpio_usb_detect;
		if (gpio_usb_detect) {
			//TODO: Draw charging icon
			//lcdDrawFillRect(dev, 90, 5, 100, 20, GREEN);
			lcdDrawString(dev, fx, 90, 24, (unsigned char*)"+", RED);
			lcdDrawString(dev, fx, 90, 25, (unsigned char*)"+", RED);
		}
		else
		{
			// Clear charging icon
			lcdDrawFillRect(dev, 90, 5, 100, 20, BLACK);
		}
	}


	//FAULT
	if (esc_telemetry.fault_code != 0)
	{
		JPEGTest(dev, (char*)"/spiffs/map_fault.jpg", 25, 25, 107, 0);
	}


	//RSSI
	//TODO: Detect 0 value && display NO SIGNAL
	uint8_t rssi_mapped = map(adc_raw_rssi, adc_raw_rssi_minimum, adc_raw_rssi_maximum, 1, 5);
	if (rssi_mapped != rssi_mapped_previous)
	{
		rssi_mapped_previous = rssi_mapped;
		sprintf((char *)ascii, "%04d", adc_raw_rssi);
		{
			ypos = 40;
			xpos = 160;
			lcdSetFontDirection(dev, DIRECTION0);
		}
		color = WHITE;
		//lcdDrawString(dev, fx, xpos, ypos, ascii, color);

		lcdDrawFillRect(dev, 180, 5, 235, 40, BLACK); //Blank drawing area
		uint8_t rssi_mapped = map(adc_raw_rssi, adc_raw_rssi_minimum, adc_raw_rssi_maximum, 1, 5);
		for (int i=0; i<rssi_mapped; ++i)
		{
			lcdDrawFillRect(dev, 181 + (i * 10)/*left*/, 6 + ((5-i) * 5)/*down*/, 181 + (i * 10) + 5 /*width*/, 38 /*height*/, GREEN); //Draw battery %
		}

		lcdDrawTriangle(dev, 170, 15, 10, 6, 0, GREEN);
		lcdDrawLine(dev, 170, 15, 170, 38, GREEN);
	}


	//Board Battery
	const uint8_t board_batt_width = 220;
	const uint8_t board_batt_x1 = 10;
	const uint8_t board_batt_y1 = 200;
	const uint8_t board_batt_y2 = 235;
	//esc_telemetry.battery_level -= 0.005;
	//if (esc_telemetry.battery_level < 0.0) esc_telemetry.battery_level = 1.0;
	uint8_t board_batt_pixel = map(esc_telemetry.battery_level * 100, 0, 100, board_batt_x1, board_batt_x1 + board_batt_width);
	// Draw battery and voltage
	if (esc_vin_previous != esc_telemetry.v_in)
	{
		esc_vin_previous = esc_telemetry.v_in;

		// Battery

		lcdDrawRect(dev, board_batt_x1, board_batt_y1, board_batt_x1 + board_batt_width+1, board_batt_y2, BLUE); //Draw outline //TODO: Only draw once to reduce render time
		lcdDrawFillRect(dev,
			board_batt_x1 + board_batt_width, //With plus x1
			board_batt_y1 + 5, // 5 below y1
			board_batt_x1 + board_batt_width + 5, // 5 wide
			board_batt_y2 - 5, // 5 above y2
			BLUE); //Draw nub //TODO: only once takes time blah blah
		color = GREEN;
		if (board_batt_pixel < (board_batt_width * 0.25) + board_batt_x1) color = RED;
		else if (board_batt_pixel < (board_batt_width * 0.45) + board_batt_x1) color = rgb565_conv(0xFF, 0xA5, 0x00);
		lcdDrawFillRect(dev, board_batt_x1 + 1/*left*/, board_batt_y1 + 1/*down*/, board_batt_pixel /*width*/, board_batt_y2 -2 /*height*/, color); //Draw battery %
		lcdDrawFillRect(dev, board_batt_pixel + 1/*left*/, board_batt_y1 + 1/*down*/, board_batt_pixel + (board_batt_width-board_batt_pixel+board_batt_x1) /*width*/, board_batt_y2 -2 /*height*/, BLACK); //Clear empty space

		// Voltage

		esc_vin_previous = esc_telemetry.v_in;
		sprintf((char *)ascii, "%3.1fV", esc_telemetry.v_in);
		{
			ypos = board_batt_y2 - 6;
			xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
			lcdSetFontDirection(dev, DIRECTION0);
		}
		color = WHITE;
		if (esc_telemetry.battery_level * 100 > 45) color = BLACK;

		lcdUnsetFontFill(dev);
		lcdDrawString(dev, fx, xpos, ypos, ascii, color);
	}

	// Alert dialog vs Speed and Odom
	if (alert_clear)
	{
		alert_show = false;
		alert_clear = false;
		alert_visible = false;
		lcdDrawFillRect(dev, 22, 42, 218, 198, BLACK);
	}
	if (alert_show && !alert_visible)
	{
		// Alert dialog
		alert_visible = true;
		lcdDrawFillRect(dev, 22, 42, 218, 198, BLACK);
		lcdDrawRoundRect(dev, 22, 42, 218, 198, 6, PURPLE);
		lcdDrawRoundRect(dev, 23, 43, 217, 197, 6, PURPLE);
		lcdDrawRoundRect(dev, 24, 44, 216, 196, 6, PURPLE);

		fontWidth = 2;
		fontHeight = 2;
		lcdSetFontDirection(dev, DIRECTION0);
		sprintf((char *)ascii, "Fault");
		ypos = 55;
		xpos = (width - (strlen((char *)ascii) * 8 * fontWidth)) / 2;
		color = YELLOW;
		lcdDrawString2(dev, fontHeight, fontWidth, xpos, ypos, ascii, color);

		GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);
		color = WHITE;
		sprintf((char *)ascii, "ESC ID %d", esc_telemetry_last_fault.vesc_id);
		ypos = 115;
		xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdDrawString(dev, fx, xpos, ypos, ascii, color);

		snprintf((char *)ascii, 15, "%s", mc_interface_fault_to_string(esc_telemetry_last_fault.fault_code));
		ypos = 135;
		xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdDrawString(dev, fx, xpos, ypos, ascii, color);

		color = GRAY;
		sprintf((char *)ascii, "%0.1fV %0.0fA>%0.0fA", esc_telemetry_last_fault.v_in, esc_telemetry_last_fault.current_in, esc_telemetry_last_fault.current_motor);
		ypos = 165;
		xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdDrawString(dev, fx, xpos, ypos, ascii, color);

		sprintf((char *)ascii, "%0.0fC / %0.0fC", esc_telemetry_last_fault.temp_mos, esc_telemetry_last_fault.temp_motor);
		ypos = 185;
		xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdDrawString(dev, fx, xpos, ypos, ascii, color);
	}
	else if (!alert_visible)
	{
		// Odometer
		if (esc_telemetry.tachometer_abs != tachometer_abs_previous)
		{
			tachometer_abs_previous = esc_telemetry.tachometer_abs;
			//TODO: Imperial selection: sprintf((char *)ascii, "%02.2fkm", esc_telemetry.tachometer_abs / 1000.0);
			sprintf((char *)ascii, "%02.2fmi", esc_telemetry.tachometer_abs / 1000.0 * KTOM);
			{
				ypos = board_batt_y1 - 1;
				xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
				lcdSetFontDirection(dev, DIRECTION0);
			}
			color = WHITE;
			lcdSetFontFill(dev, BLACK);
			lcdDrawString(dev, fx, xpos, ypos, ascii, color);
		}

		// Speed big numbers
		fontWidth = 9;
		fontHeight = 8;
		const float metersPerSecondToKph = 3.6;
		//TODO: Imperial selection: sprintf((char *)ascii, "%02d", (int)(esc_telemetry.speed * metersPerSecondToKph));
		sprintf((char *)ascii, "%02d", (int)(esc_telemetry.speed * metersPerSecondToKph * KTOM));
		{
			ypos = 42;
			xpos = (width - (strlen((char *)ascii) * 8 * fontWidth)) / 2;
			lcdSetFontDirection(dev, DIRECTION0);
		}
		color = YELLOW;
		lcdDrawString2(dev, fontHeight, fontWidth, xpos, ypos, ascii, color);
	}

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

TickType_t drawScreenPairing(TFT_t * dev, FontxFile *fx, int width, int height) {
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


	// Pairing dialog
	//lcdDrawFillRect(dev, 22, 42, 218, 198, BLACK);
	lcdDrawRoundRect(dev, 22, 42, 218, 198, 6, GREEN);
	lcdDrawRoundRect(dev, 23, 43, 217, 197, 6, GREEN);
	lcdDrawRoundRect(dev, 24, 44, 216, 196, 6, GREEN);

	fontWidth = 2;
	fontHeight = 2;
	lcdSetFontDirection(dev, DIRECTION0);
	sprintf((char *)ascii, "Pairing");
	ypos = 55;
	xpos = (width - (strlen((char *)ascii) * 8 * fontWidth)) / 2;
	color = YELLOW;
	lcdDrawString2(dev, fontHeight, fontWidth, xpos, ypos, ascii, color);

	GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);
	color = WHITE;
	sprintf((char *)ascii, "%s", str_pairing_1);
	ypos = 115;
	xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
	lcdDrawString(dev, fx, xpos, ypos, ascii, color);

	sprintf((char *)ascii, "%s", str_pairing_2);
	ypos = 135;
	xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
	lcdDrawString(dev, fx, xpos, ypos, ascii, color);

	//lcdDrawFillRect(dev, 25, 135, 215, 190, BLACK);

	color = GRAY;
	sprintf((char *)ascii, "%s", str_pairing_3);
	ypos = 165;
	xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
	lcdDrawString(dev, fx, xpos, ypos, ascii, color);

	sprintf((char *)ascii, "%s", str_pairing_4);
	ypos = 185;
	xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
	lcdDrawString(dev, fx, xpos, ypos, ascii, color);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	//ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_RATE_MS);
	return diffTick;
}

TickType_t drawAlert(TFT_t * dev, FontxFile *fx, uint16_t p_color, char * title, char * line1, char * line2, char * line3, char * line4) {
	alert_visible = true;

	const int width = 240;

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


	// Alert dialog border
	lcdDrawFillRect(dev, 22, 42, 218, 198, BLACK);
	lcdDrawRoundRect(dev, 22, 42, 218, 198, 6, p_color);
	lcdDrawRoundRect(dev, 23, 43, 217, 197, 6, p_color);
	lcdDrawRoundRect(dev, 24, 44, 216, 196, 6, p_color);

	fontWidth = 2;
	fontHeight = 2;
	lcdSetFontDirection(dev, DIRECTION0);
	snprintf((char *)ascii, 15, title);
	ypos = 55;
	xpos = (width - (strlen((char *)ascii) * 8 * fontWidth)) / 2;
	color = YELLOW;
	lcdDrawString2(dev, fontHeight, fontWidth, xpos, ypos, ascii, color);

	GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);
	color = WHITE;
	snprintf((char *)ascii, 15, "%s", line1);
	ypos = 115;
	xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
	lcdDrawString(dev, fx, xpos, ypos, ascii, color);

	snprintf((char *)ascii, 15, "%s", line2);
	ypos = 135;
	xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
	lcdDrawString(dev, fx, xpos, ypos, ascii, color);

	//lcdDrawFillRect(dev, 25, 135, 215, 190, BLACK);

	color = GRAY;
	snprintf((char *)ascii, 15, "%s", line3);
	ypos = 165;
	xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
	lcdDrawString(dev, fx, xpos, ypos, ascii, color);

	snprintf((char *)ascii, 15, "%s", line4);
	ypos = 185;
	xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
	lcdDrawString(dev, fx, xpos, ypos, ascii, color);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	//ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_RATE_MS);
	return diffTick;
}

TickType_t drawSetupMenu(TFT_t * dev, FontxFile *fx, user_settings_t *user_settings, uint8_t current_index) {
	static const int width = 240;
	static bool first_draw = true;
	static uint8_t previous_index = 255;
	static user_settings_t previous_settings;

	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

	uint8_t buffer[FontxGlyphBufSize];
	uint8_t fontWidth = 2;
	uint8_t fontHeight = 2;
	uint16_t xpos;
	uint16_t ypos;
	uint8_t ascii[20];
	uint16_t color;

	if (first_draw)
	{
		lcdSetFontFill(dev, BLACK);
		lcdSetFontDirection(dev, DIRECTION0);
		sprintf((char *)ascii, "OSRR Setup");
		ypos = 20;
		xpos = (width - (strlen((char *)ascii) * 8 * fontWidth)) / 2;
		color = YELLOW;
		lcdDrawString2(dev, fontHeight, fontWidth, xpos, ypos, ascii, color);
	}

	if (previous_index != current_index || memcmp(&previous_settings, user_settings, sizeof(user_settings_t)) != 0 )
	{
		// Get font size
		GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);

		if (current_index == 0 || previous_index == 0 || first_draw)
		{
			if (current_index == 0) color = WHITE;
			else color = GRAY;
			if (user_settings->disable_piezo) sprintf((char *)ascii, "Piezo: OFF");
			else sprintf((char *)ascii, " Piezo: ON ");
			ypos = 100;
			xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
			lcdDrawString(dev, fx, xpos, ypos, ascii, color);
		}

		if (current_index == 1 || previous_index == 1 || first_draw)
		{
			if (current_index == 1) color = WHITE;
			else color = GRAY;
			if (user_settings->disable_buzzer) sprintf((char *)ascii, "Buzzer: OFF");
			else sprintf((char *)ascii, " Buzzer: ON ");
			ypos = 125;
			xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
			lcdDrawString(dev, fx, xpos, ypos, ascii, color);
		}

		if (current_index == 2 || previous_index == 2 || first_draw)
		{
			if (current_index == 2) color = WHITE;
			else color = GRAY;
			if (user_settings->display_mph) sprintf((char *)ascii, " Speed: MPH ");
			else sprintf((char *)ascii, " Speed: KPH ");
			ypos = 150;
			xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
			lcdDrawString(dev, fx, xpos, ypos, ascii, color);
		}

		if (current_index == 3 || previous_index == 3 || first_draw)
		{
			if (current_index == 3) color = WHITE;
			else color = GRAY;
			if (user_settings->dispaly_fahrenheit) sprintf((char *)ascii, "Temp: Fahrenheit");
			else sprintf((char *)ascii, "  Temp: Celsius  ");
			ypos = 175;
			xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
			lcdDrawString(dev, fx, xpos, ypos, ascii, color);
		}

		if (current_index == 4 || previous_index == 4 || first_draw)
		{
			if (current_index == 4) color = WHITE;
			else color = GRAY;
			if (user_settings->throttle_reverse) sprintf((char *)ascii, " Throttle: Reverse ");
			else sprintf((char *)ascii, " Throttle: Forward ");
			ypos = 200;
			xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
			lcdDrawString(dev, fx, xpos, ypos, ascii, color);
		}

		if (current_index == 5 || previous_index == 5 || first_draw)
		{
			if (current_index == 5) color = WHITE;
			else color = GRAY;
			switch (user_settings->remote_model) {
				case MODEL_UNDEFINED:
					sprintf((char *)ascii, "Model: NotSet");
					break;
				case MODEL_ALBERT:
					sprintf((char *)ascii, "Model: Albert");
					break;
				case MODEL_BRUCE:
					sprintf((char *)ascii, " Model: Bruce ");
					break;
				case MODEL_CLINT:
					sprintf((char *)ascii, " Model: Clint ");
					break;
			}
			ypos = 225;
			xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
			lcdDrawString(dev, fx, xpos, ypos, ascii, color);
		}

		// Update previous values for next iteration
		previous_index = current_index;
		previous_settings = (*user_settings);
	}

	// First draw only happens once
	first_draw = false;

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	//ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_RATE_MS);
	return diffTick;
}
