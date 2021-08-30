#include <math.h>
#include "display.h"

extern long map(long x, long in_min, long in_max, long out_min, long out_max);

static bool fault_indicator_displayed = false;
static bool is_speed_visible = true;

static const int8_t x_offset = -4; //TODO: offset used on Albert. the rest are unknown

/**
 * Asymmetric sigmoidal approximation
 * https://www.desmos.com/calculator/oyhpsu8jnw
 *
 * c - c / [1 + (k*x/v)^4.5]^3
 */
static inline uint8_t asigmoidal(uint16_t voltage, uint16_t minVoltage, uint16_t maxVoltage) {
	uint8_t result = 101 - (101 / pow(1 + pow(1.33 * (voltage - minVoltage)/(maxVoltage - minVoltage) ,4.5), 3));
	return result >= 100 ? 100 : result;
}

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
	


	sprintf((char *)ascii, "T:%d U:%d", xTaskGetTickCount() * portTICK_PERIOD_MS / 1000, gpio_usb_detect);
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
//batt 696 = 4.22
//batt 691 = 4.19
//batt 658 = 3.985V
//batt 622 = 3.775
//batt 538 = 3.308
const int adc_raw_battery_minimum = 525; //TODO: estimated
const int adc_raw_battery_maximum = 720; // Battery at full charge
const int adc_raw_rssi_maximum = 1092; // Maximum observed RSSI
const int adc_raw_rssi_minimum = 519; // Lowest oberseved RSSI
static uint16_t adc_raw_rssi_avg = 0; // Average displayed RSSI value
static uint8_t rssi_mapped_previous;
static uint8_t batt_pixel_previous;
static uint8_t gpio_usb_detect_previous;
static int tachometer_abs_previous = 0;
static double esc_vin_previous = 0;
static double esc_battery_previous = 0;
static int speed_now_previous = -1;
static uint8_t remote_battery_previous = 0;
static uint8_t joystick_value_mapped_previous = 0;
void resetPreviousValues()
{
	tachometer_abs_previous = -1;
	rssi_mapped_previous = 0;
	batt_pixel_previous = 0;
	gpio_usb_detect_previous = 0;
	esc_vin_previous = 0;
	esc_battery_previous = 0;
	fault_indicator_displayed = false;
	speed_now_previous = -1;
	joystick_value_mapped_previous = 0;
	is_speed_visible = true;
	remote_battery_previous = 0;
}
TickType_t drawScreenPrimary(TFT_t * dev, FontxFile *fx, int width, int height, user_settings_t *user_settings) {
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
		drawJPEG(dev, (char*)"/spiffs/map_fault.jpg", 25, 25, 107, 0);
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

		if (user_settings->dispaly_fahrenheit) sprintf((char *)ascii, "%0.0fF / %0.0fF", CTOF(esc_telemetry_last_fault.temp_mos), CTOF(esc_telemetry_last_fault.temp_motor));
		else sprintf((char *)ascii, "%0.0fC / %0.0fC", esc_telemetry_last_fault.temp_mos, esc_telemetry_last_fault.temp_motor);
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
			if (user_settings->display_mph) sprintf((char *)ascii, "%02.2fmi", esc_telemetry.tachometer_abs / 1000.0 * KTOM);
			else sprintf((char *)ascii, "%02.2fkm", esc_telemetry.tachometer_abs / 1000.0);
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
		if (user_settings->display_mph) sprintf((char *)ascii, "%02d", (int)(esc_telemetry.speed * metersPerSecondToKph * KTOM));
		else sprintf((char *)ascii, "%02d", (int)(esc_telemetry.speed * metersPerSecondToKph));
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

void drawCircularGauge(TFT_t * dev, uint8_t x, uint8_t y, uint8_t radius, uint8_t width, uint16_t start_angle, uint16_t end_angle, uint8_t percentage, uint16_t color_on, uint16_t color_off)
{
	// Don't draw more than 100%
	if (percentage > 100) percentage = 100;

	// How many degrees does the gauge have to work with? Watch out for crossing 360
	int angle_range = end_angle - start_angle;
	if (angle_range < 0) angle_range += 360;
	// How much of the angle does the value require?
	uint16_t value_angle = angle_range * (percentage/100.0);
	// Arcs are 3 degrees per segment
	uint8_t segments_illuminated = value_angle / 3.0;
	uint8_t segments_dimmed = (angle_range / 3.0) - segments_illuminated;
	//printf("input %d, ar %d, va %d, si %d, sd %d\n", percentage, angle_range, value_angle, segments_illuminated, segments_dimmed);
	lcdFillArc(dev, x, y, start_angle, segments_illuminated, radius, radius, width, color_on);

	// Clear unused arc section
	lcdFillArc(dev, x, y, start_angle + value_angle - (value_angle%6), segments_dimmed, radius, radius, width, color_off);
}

TickType_t drawScreenRound(TFT_t * dev, FontxFile *fx, int width, int height, user_settings_t *user_settings) {
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

	// Testing/debugging without connected vehicle
	{
		//adc_raw_battery_level += 10;
		//if (adc_raw_battery_level > adc_raw_battery_maximum) adc_raw_battery_level = adc_raw_battery_minimum;

		//adc_raw_rssi += 20;
		//if (adc_raw_rssi > adc_raw_rssi_maximum) adc_raw_rssi = adc_raw_rssi_minimum;

		//esc_telemetry.v_in = 42.19;
		//esc_telemetry.battery_level -= 0.005;
		//if (esc_telemetry.battery_level < 0.0) esc_telemetry.battery_level = 1.0;

		//joystick_value_mapped += 2;
	}

	//Remote Battery
	//uint8_t remote_battery = map(adc_raw_battery_level, adc_raw_battery_minimum, adc_raw_battery_maximum, 1, 10);
	uint8_t remote_battery = asigmoidal(adc_raw_battery_level, adc_raw_battery_minimum, adc_raw_battery_maximum);
	remote_battery /= 10;
	if (remote_battery != remote_battery_previous)
	{
		remote_battery_previous = remote_battery;
		drawCircularGauge(dev, x_offset + 120, 110, 100, 5, 290, 350, remote_battery * 10, BLUE, RED);
	}


	// Remote is Charging
	if (gpio_usb_detect != gpio_usb_detect_previous)
	{
		gpio_usb_detect_previous = gpio_usb_detect;
		if (gpio_usb_detect) {
			//TODO: Draw charging icon
			lcdDrawString(dev, fx, x_offset + 120 - 6, 24, (unsigned char*)"+", RED);
			lcdDrawString(dev, fx, x_offset + 120 - 6, 25, (unsigned char*)"+", RED);
		}
		else
		{
			// Clear charging icon
			lcdDrawFillRect(dev, x_offset + 120 - 6, 5, x_offset + 120 + 6, 20, BLACK);
		}
	}


	//FAULT
	if (esc_telemetry.fault_code != 0 && !fault_indicator_displayed)
	{
		drawJPEG(dev, (char*)"/spiffs/map_fault.jpg", 25, 25, x_offset + 107, 25);
		fault_indicator_displayed = true;
	} else if (esc_telemetry.fault_code == 0 && fault_indicator_displayed) {
		lcdDrawFillRect(dev, 97, 25, x_offset+107+25, 25+25, BLACK); // Clear fault indicator
		fault_indicator_displayed = false;
	}


	//RSSI
	adc_raw_rssi_avg = (uint16_t)(0.1 * adc_raw_rssi) + (uint16_t)(0.9 * adc_raw_rssi_avg);

	uint8_t rssi_mapped = map(adc_raw_rssi_avg, adc_raw_rssi_minimum, adc_raw_rssi_maximum, 1, 10);
	if (rssi_mapped != rssi_mapped_previous)
	{
		drawCircularGauge(dev, x_offset+120, 110, 100, 5, 10, 70, 100-/*invert*/(rssi_mapped*10), BLACK, GREEN);
	}

	// Alert dialog vs Speed, Odom and Voltage
	if (alert_clear)
	{
		alert_show = false;
		alert_clear = false;
		alert_visible = false;
		lcdDrawFillRect(dev, x_offset + 32, 50, x_offset + 210, 185, BLACK);
		//TODO: lol this is slow af: lcdDrawFillCircle(dev, x_offset + 120, 120, 100, BLACK);
		resetPreviousValues(); // Force all round gauges to redraw after alert
	}
	if (alert_show && !alert_visible)
	{
		alert_visible = true;
		lcdDrawFillRect(dev, 22, 50, 200, 185, BLACK);
		//TODO: lol this is slow af: lcdDrawFillCircle(dev, x_offset + 120, 120, 100, GREEN);
		resetPreviousValues(); // Force all round gauges to redraw after alert

		fontWidth = 2;
		fontHeight = 2;
		lcdSetFontDirection(dev, DIRECTION0);
		sprintf((char *)ascii, "Fault");
		ypos = 55;
		xpos = x_offset + (width - (strlen((char *)ascii) * 8 * fontWidth)) / 2;
		color = YELLOW;
		lcdDrawString2(dev, fontHeight, fontWidth, xpos, ypos, ascii, color);

		GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);
		color = WHITE;
		sprintf((char *)ascii, "ESC ID %d", esc_telemetry_last_fault.vesc_id);
		ypos = 115;
		xpos = x_offset + (width - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdDrawString(dev, fx, xpos, ypos, ascii, color);

		snprintf((char *)ascii, 15, "%s", mc_interface_fault_to_string(esc_telemetry_last_fault.fault_code));
		ypos = 135;
		xpos = x_offset + (width - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdDrawString(dev, fx, xpos, ypos, ascii, color);

		color = GRAY;
		sprintf((char *)ascii, "%0.1fV %0.0fA>%0.0fA", esc_telemetry_last_fault.v_in, esc_telemetry_last_fault.current_in, esc_telemetry_last_fault.current_motor);
		ypos = 160;
		xpos = x_offset + (width - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdDrawString(dev, fx, xpos, ypos, ascii, color);

		if (user_settings->dispaly_fahrenheit) sprintf((char *)ascii, "%0.0fF / %0.0fF", CTOF(esc_telemetry_last_fault.temp_mos), CTOF(esc_telemetry_last_fault.temp_motor));
		else sprintf((char *)ascii, "%0.0fC / %0.0fC", esc_telemetry_last_fault.temp_mos, esc_telemetry_last_fault.temp_motor);
		ypos = 180;
		xpos = x_offset + (width - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdDrawString(dev, fx, xpos, ypos, ascii, color);
	}
	else if (!alert_visible)
	{
		// Throttle
		double joystick_position = 0;
		if (joystick_value_mapped > 129)
		{
			joystick_position = (joystick_value_mapped - 128) / 128.0;
		} else if (joystick_value_mapped < 125)
		{
			joystick_position = 1.0 - (joystick_value_mapped / 128.0);
		} else {
			joystick_position = 0;
		}
		if (abs(joystick_value_mapped_previous - joystick_value_mapped) > 12)
		{
			joystick_value_mapped_previous = joystick_value_mapped;
			printf("joy %d, %f, %d\n", joystick_value_mapped, joystick_position, (int)(100-(joystick_position * 100)));
			drawCircularGauge(dev, x_offset + 120, 110, 85, 5, 0, 90, 100-(joystick_position * 100), BLACK, PURPLE);
		}

		// Odometer
		// Do not update if less than 0.01km distance
		if (fabs(esc_telemetry.tachometer_abs / 1000.0 - tachometer_abs_previous / 1000.0) > 0.01)
		{
			tachometer_abs_previous = esc_telemetry.tachometer_abs;
			if (user_settings->display_mph) sprintf((char *)ascii, "%04.2fmi", esc_telemetry.tachometer_abs / 1000.0 * KTOM);
			else sprintf((char *)ascii, "%04.2fkm", esc_telemetry.tachometer_abs / 1000.0);
			/*
			{
				ypos = 165;
				xpos = x_offset + (width - (strlen((char *)ascii) * fontWidth)) / 2;
				lcdSetFontDirection(dev, DIRECTION0);
			}
			color = WHITE;
			lcdDrawString(dev, fx, xpos, ypos, ascii, color);
			*/
			fontWidth = 2;
			fontHeight = 2;
			{
				ypos = 145;
				xpos = x_offset + (width - (strlen((char *)ascii) * 6 /*font1 multiplier*/ * fontWidth)) / 2;
				lcdSetFontDirection(dev, DIRECTION0);
			}
			color = WHITE;
			lcdDrawString3(dev, fontHeight, fontWidth, xpos, ypos, ascii, color);
		}

		// Voltage
		// Only update if we've changed more than 0.1V (displayed)
		if (fabs(esc_vin_previous - esc_telemetry.v_in) > 0.1)
		{
			esc_vin_previous = esc_telemetry.v_in;
			/*
			sprintf((char *)ascii, " %3.1fV ", esc_telemetry.v_in);
			{
				ypos = 190;
				xpos = x_offset + (width - (strlen((char *)ascii) * fontWidth)) / 2;
				lcdSetFontDirection(dev, DIRECTION0);
			}
			color = WHITE;
			lcdSetFontFill(dev, BLACK);
			lcdDrawString(dev, fx, xpos, ypos, ascii, color);
			*/
			fontWidth = 2;
			fontHeight = 2;
			sprintf((char *)ascii, " %3.1fV ", esc_telemetry.v_in);
			{
				ypos = 170;
				xpos = x_offset + (width - (strlen((char *)ascii) * 6 /*font1 multiplier*/ * fontWidth)) / 2;
				lcdSetFontDirection(dev, DIRECTION0);
			}
			color = WHITE;
			lcdDrawString3(dev, fontHeight, fontWidth, xpos, ypos, ascii, color);
		}

		// Speed big numbers
		// Only update if ESC is responding
		if ((xTaskGetTickCount() - esc_last_responded)*portTICK_RATE_MS < 10 * 1000)
		{
			is_speed_visible = true;
			const float metersPerSecondToKph = 3.6;
			int speed_now = 0;
			if (user_settings->display_mph) speed_now = (int)(esc_telemetry.speed * metersPerSecondToKph * KTOM);
			else speed_now = (int)(esc_telemetry.speed * metersPerSecondToKph);
			if (speed_now != speed_now_previous)
			{
				fontWidth = 6;
				fontHeight = 5;
				speed_now_previous = speed_now;
				sprintf((char *)ascii, "%02d", abs(speed_now));
				{
					ypos = 60;
					xpos = x_offset + (width - (strlen((char *)ascii) * 8 * fontWidth)) / 2;
					lcdSetFontDirection(dev, DIRECTION0);
				}
				color = YELLOW;
				lcdDrawString2(dev, fontHeight, fontWidth, xpos, ypos, ascii, color);
			}
		}
		else
		{
			// Clear out vehicle speed
			if (is_speed_visible)
			{
				is_speed_visible = false;
				lcdDrawFillRect(dev, x_offset + 64, 60, x_offset + 184, 142, BLACK);
			}
		}

	}

	// Vehicle Battery
	// Draw vehicle battery if it's changed more than 5%
	if (fabs(esc_battery_previous - esc_telemetry.battery_level) > 0.05)
	{
		esc_battery_previous = esc_telemetry.battery_level;

		// Battery
		drawCircularGauge(dev, x_offset + 120, 110, 100, 5, 90, 270, esc_telemetry.battery_level * 100, BLUE, RED);
	}

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	//ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_RATE_MS);
	return diffTick;
}

void drawFirmwareVersion(TFT_t * dev, char * version)
{
	// Draw firmware version
	uint8_t fontWidth = 2;
	uint8_t fontHeight = 2;
	uint8_t ypos = 200;
	uint8_t xpos = x_offset + (240 /*display width*/ - (strlen(version) * 6 /*font1 multiplier*/ * fontWidth)) / 2;
	lcdDrawString3(dev, fontHeight, fontWidth, xpos, ypos, (uint8_t *)version, WHITE);
}

TickType_t drawJPEG(TFT_t * dev, char * file, int width, int height, int offset_x, int offset_y) {
	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();

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
		//ESP_LOGD(__FUNCTION__, "Finish");
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
	//TODO: border for new round displays
	//lcdDrawFillRect(dev, 22, 42, 218, 198, BLACK);
	//lcdDrawRoundRect(dev, 22, 42, 218, 198, 6, p_color);
	//lcdDrawRoundRect(dev, 23, 43, 217, 197, 6, p_color);
	//lcdDrawRoundRect(dev, 24, 44, 216, 196, 6, p_color);

	fontWidth = 2;
	fontHeight = 2;
	lcdSetFontDirection(dev, DIRECTION0);
	snprintf((char *)ascii, 15, title);
	ypos = 55;
	xpos = x_offset + (width - (strlen((char *)ascii) * 8 * fontWidth)) / 2;
	color = YELLOW;
	lcdDrawString2(dev, fontHeight, fontWidth, xpos, ypos, ascii, color);

	GetFontx(fx, 0, buffer, &fontWidth, &fontHeight);
	color = WHITE;
	snprintf((char *)ascii, 15, "%s", line1);
	ypos = 115;
	xpos = x_offset + (width - (strlen((char *)ascii) * fontWidth)) / 2;
	lcdDrawString(dev, fx, xpos, ypos, ascii, color);

	snprintf((char *)ascii, 15, "%s", line2);
	ypos = 135;
	xpos = x_offset + (width - (strlen((char *)ascii) * fontWidth)) / 2;
	lcdDrawString(dev, fx, xpos, ypos, ascii, color);

	//lcdDrawFillRect(dev, 25, 135, 215, 190, BLACK);

	color = GRAY;
	snprintf((char *)ascii, 15, "%s", line3);
	ypos = 165;
	xpos = x_offset + (width - (strlen((char *)ascii) * fontWidth)) / 2;
	lcdDrawString(dev, fx, xpos, ypos, ascii, color);

	snprintf((char *)ascii, 15, "%s", line4);
	ypos = 185;
	xpos = x_offset + (width - (strlen((char *)ascii) * fontWidth)) / 2;
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

		if (current_index == SETTING_PIEZO || previous_index == SETTING_PIEZO || first_draw)
		{
			if (current_index == SETTING_PIEZO) color = WHITE;
			else color = GRAY;
			if (user_settings->disable_piezo) sprintf((char *)ascii, "Piezo: OFF");
			else sprintf((char *)ascii, " Piezo: ON ");
			ypos = 100;
			xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
			lcdDrawString(dev, fx, xpos, ypos, ascii, color);
		}

		if (current_index == SETTING_BUZZER || previous_index == SETTING_BUZZER || first_draw)
		{
			if (current_index == SETTING_BUZZER) color = WHITE;
			else color = GRAY;
			if (user_settings->disable_buzzer) sprintf((char *)ascii, "Buzzer: OFF");
			else sprintf((char *)ascii, " Buzzer: ON ");
			ypos = 125;
			xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
			lcdDrawString(dev, fx, xpos, ypos, ascii, color);
		}

		if (current_index == SETTING_SPEED || previous_index == SETTING_SPEED || first_draw)
		{
			if (current_index == SETTING_SPEED) color = WHITE;
			else color = GRAY;
			if (user_settings->display_mph) sprintf((char *)ascii, " Speed: MPH ");
			else sprintf((char *)ascii, " Speed: KPH ");
			ypos = 150;
			xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
			lcdDrawString(dev, fx, xpos, ypos, ascii, color);
		}

		if (current_index == SETTING_TEMP || previous_index == SETTING_TEMP || first_draw)
		{
			if (current_index == SETTING_TEMP) color = WHITE;
			else color = GRAY;
			if (user_settings->dispaly_fahrenheit) sprintf((char *)ascii, "Temp: Fahrenheit");
			else sprintf((char *)ascii, "  Temp: Celsius  ");
			ypos = 175;
			xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
			lcdDrawString(dev, fx, xpos, ypos, ascii, color);
		}

		if (current_index == SETTING_THROTTLE || previous_index == SETTING_THROTTLE || first_draw)
		{
			if (current_index == SETTING_THROTTLE) color = WHITE;
			else color = GRAY;
			if (user_settings->throttle_reverse) sprintf((char *)ascii, " Throttle: Reverse ");
			else sprintf((char *)ascii, " Throttle: Forward ");
			ypos = 200;
			xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
			lcdDrawString(dev, fx, xpos, ypos, ascii, color);
		}

		if (current_index == SETTING_MODEL || previous_index == SETTING_MODEL || first_draw)
		{
			if (current_index == SETTING_MODEL) color = WHITE;
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
