#include <math.h>
#include "display.h"

extern long map(long x, long in_min, long in_max, long out_min, long out_max);

static bool fault_indicator_displayed = false;

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

//TODO: Simplify. drawCircularGaugeSmall only changes 1 parameter and utilizes lcdFillArc3 (can also be simplified)
void drawCircularGaugeSmall(TFT_t * dev, uint8_t x, uint8_t y, uint8_t radius, uint8_t width, uint16_t start_angle, uint16_t end_angle, uint8_t percentage, uint16_t color_on, uint16_t color_off)
{
	uint8_t segment_degrees = 9;

	// Don't draw more than 100%
	if (percentage > 100) percentage = 100;

	// How many degrees does the gauge have to work with? Watch out for crossing 360
	int angle_range = end_angle - start_angle;
	if (angle_range < 0) angle_range += 360;
	// How much of the angle does the value require?
	uint16_t value_angle = angle_range * (percentage/100.0);
	// Arcs are 3 degrees per segment
	uint8_t segments_illuminated = value_angle / (double)segment_degrees;
	uint8_t segments_dimmed = (angle_range / (double)segment_degrees) - segments_illuminated;
	//printf("input %d, ar %d, va %d, si %d, sd %d\n", percentage, angle_range, value_angle, segments_illuminated, segments_dimmed);
	lcdFillArc3(dev, x, y, start_angle, segments_illuminated, radius, radius, width, color_on, segment_degrees);

	// Clear unused arc section
	lcdFillArc3(dev, x, y, start_angle + value_angle - (value_angle%(segment_degrees*2)), segments_dimmed, radius, radius, width, color_off, segment_degrees);
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

	// Alert dialog vs Speed, Odom and Voltage
	if (alert_clear)
	{
		alert_show = false;
		alert_clear = false;
		alert_visible = false;
		lcdDrawFillRect(dev, 32, 50, 210, 185, BLACK);
		//TODO: lol this is slow af: lcdDrawFillCircle(dev, 120, 120, 100, BLACK);
	}
	if (alert_show && !alert_visible)
	{
		alert_visible = true;
		lcdDrawFillRect(dev, 22, 50, 200, 185, BLACK);
		//TODO: lol this is slow af: lcdDrawFillCircle(dev, 120, 120, 100, GREEN);

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
		ypos = 160;
		xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
		lcdDrawString(dev, fx, xpos, ypos, ascii, color);

		if (user_settings->dispaly_fahrenheit) sprintf((char *)ascii, "%0.0fF / %0.0fF", CTOF(esc_telemetry_last_fault.temp_mos), CTOF(esc_telemetry_last_fault.temp_motor));
		else sprintf((char *)ascii, "%0.0fC / %0.0fC", esc_telemetry_last_fault.temp_mos, esc_telemetry_last_fault.temp_motor);
		ypos = 180;
		xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
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
		drawCircularGauge(dev, 120, 110, 85, 5, 0, 90, 100-(joystick_position * 100), BLACK, PURPLE);

		// Odometer
		{
			if (user_settings->display_mph) sprintf((char *)ascii, "%03.1fmi", esc_telemetry.tachometer_abs / 1000.0 * KTOM);
			else sprintf((char *)ascii, "%04.1fkm", esc_telemetry.tachometer_abs / 1000.0);
			fontWidth = 3;
			fontHeight = 3;
			{
				ypos = 135;
				xpos = (width - (strlen((char *)ascii) * 6 /*font1 multiplier*/ * fontWidth)) / 2;
				lcdSetFontDirection(dev, DIRECTION0);
			}
			color = WHITE;
			lcdDrawString3(dev, fontHeight, fontWidth, xpos, ypos, ascii, color);
		}

		// Voltage
		{
			fontWidth = 3;
			fontHeight = 3;
			sprintf((char *)ascii, " %3.1fV ", esc_telemetry.v_in);
			{
				ypos = 165;
				xpos = (width - (strlen((char *)ascii) * 6 /*font1 multiplier*/ * fontWidth)) / 2;
				lcdSetFontDirection(dev, DIRECTION0);
			}
			color = WHITE;
			lcdDrawString3(dev, fontHeight, fontWidth, xpos, ypos, ascii, color);
		}

		// Speed big numbers
		{
			// Compute Speed
			const float metersPerSecondToKph = 3.6;
			int speed_now = 0;
			if (user_settings->display_mph) speed_now = (int)(esc_telemetry.speed * metersPerSecondToKph * KTOM);
			else speed_now = (int)(esc_telemetry.speed * metersPerSecondToKph);
			// Ensure all speeds are positive
			speed_now = abs(speed_now);
			// Wrap Speed if triple digits
			if (speed_now > 99) speed_now -= 100;
			// Check if ESC is responding
			bool is_esc_responding = (xTaskGetTickCount() - esc_last_responded)*portTICK_RATE_MS < 1000;

			{
				fontWidth = 6;
				fontHeight = 5;
				sprintf((char *)ascii, "%02d", speed_now);
				{
					ypos = 50;
					xpos = (width - (strlen((char *)ascii) * 8 * fontWidth)) / 2;
					lcdSetFontDirection(dev, DIRECTION0);
				}
				// If ESC is not responding display speed in GRAY
				if (is_esc_responding) color = YELLOW;
				else color = GRAY;
				lcdDrawString2(dev, fontHeight, fontWidth, xpos, ypos, ascii, color);
			}
		}
	}

	//Remote Battery
	//uint8_t remote_battery = map(adc_raw_battery_level, adc_raw_battery_minimum, adc_raw_battery_maximum, 1, 10);
	uint8_t remote_battery = asigmoidal(adc_raw_battery_level, adc_raw_battery_minimum, adc_raw_battery_maximum);
	remote_battery /= 10;
	drawCircularGauge(dev, 120, 110, 100, 5, 290, 350, remote_battery * 10, BLUE, RED);


	// Remote is Charging
	{
		if (gpio_usb_detect) {
			//TODO: Draw charging icon
			lcdDrawString(dev, fx, 120 - 6, 24, (unsigned char*)"+", RED);
			lcdDrawString(dev, fx, 120 - 6, 25, (unsigned char*)"+", RED);
		}
		else
		{
			// Clear charging icon
			lcdDrawFillRect(dev, 120 - 6, 5, 120 + 6, 20, BLACK);
		}
	}


	//FAULT
	if (esc_telemetry.fault_code != 0 && !fault_indicator_displayed)
	{
		drawJPEG(dev, (char*)"/spiffs/map_fault.jpg", 25, 25, 107, 25);
		fault_indicator_displayed = true;
	} else if (esc_telemetry.fault_code == 0 && fault_indicator_displayed) {
		lcdDrawFillRect(dev, 97, 25, 107+25, 25+25, BLACK); // Clear fault indicator
		fault_indicator_displayed = false;
	}


	//RSSI
	adc_raw_rssi_avg = (uint16_t)(0.1 * adc_raw_rssi) + (uint16_t)(0.9 * adc_raw_rssi_avg);

	uint8_t rssi_mapped = map(adc_raw_rssi_avg, adc_raw_rssi_minimum, adc_raw_rssi_maximum, 10, 100);
	double rssi_log = log10(rssi_mapped) - 1.0; // Results in 0.0 to 1.0
	{
		if (rssi_log < 0.1) rssi_log = 0.1; // rssi_log is minimum 10% to show 1 dot on the LCD
		drawCircularGauge(dev, 120, 110, 100, 5, 10, 70, 100-/*invert*/(rssi_log*100), BLACK, GREEN);
	}


	// Vehicle Battery
	// Draw vehicle battery if it's changed more than 5%
	drawCircularGauge(dev, 120, 110, 100, 5, 90, 270, esc_telemetry.battery_level * 100, BLUE, RED);

	lcdUpdate(dev);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	//ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_RATE_MS);
	return diffTick;
}

TickType_t drawScreenSecondary(TFT_t * dev, FontxFile *fx, int width, int height, user_settings_t *user_settings) {
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
	//uint8_t remote_battery = map(adc_raw_battery_level, adc_raw_battery_minimum, adc_raw_battery_maximum, 1, 10);
	uint8_t remote_battery = asigmoidal(adc_raw_battery_level, adc_raw_battery_minimum, adc_raw_battery_maximum);
	remote_battery /= 10;
	drawCircularGauge(dev, 120, 110, 100, 5, 290, 350, remote_battery * 10, BLUE, RED);


	// Remote is Charging
	{
		if (gpio_usb_detect) {
			//TODO: Draw charging icon
			lcdDrawString(dev, fx, 120 - 6, 24, (unsigned char*)"+", RED);
			lcdDrawString(dev, fx, 120 - 6, 25, (unsigned char*)"+", RED);
		}
		else
		{
			// Clear charging icon
			lcdDrawFillRect(dev, 120 - 6, 5, 120 + 6, 20, BLACK);
		}
	}


	//RSSI
	adc_raw_rssi_avg = (uint16_t)(0.1 * adc_raw_rssi) + (uint16_t)(0.9 * adc_raw_rssi_avg);

	uint8_t rssi_mapped = map(adc_raw_rssi_avg, adc_raw_rssi_minimum, adc_raw_rssi_maximum, 10, 100);
	double rssi_log = log10(rssi_mapped) - 1.0; // Results in 0.0 to 1.0
	{
		if (rssi_log < 0.1) rssi_log = 0.1; // rssi_log is minimum 10% to show 1 dot on the LCD
		drawCircularGauge(dev, 120, 110, 100, 5, 10, 70, 100-/*invert*/(rssi_log*100), BLACK, GREEN);
	}

	// Temps
	static uint8_t esc_temp_mapped;
	static uint8_t motor_temp_mapped;
	static double display_temperature;

	// ESC Temp
	{
		if (user_settings->dispaly_fahrenheit) {
			esc_temp_mapped = map(CTOF(esc_telemetry.temp_mos), CTOF(0), CTOF(100), 1, 20);
			display_temperature = CTOF(esc_telemetry.temp_mos);
		} else {
			esc_temp_mapped = map(esc_telemetry.temp_mos, 0, 100, 1, 20);
			display_temperature = esc_telemetry.temp_mos;
		}
		if (display_temperature < 0) display_temperature = 0;

		ypos = 115;
		drawCircularGaugeSmall(dev, 80, ypos, 30, 4, 200, 160, /*100-invert*/(esc_temp_mapped*5), RED, GREEN);
		fontWidth = 2;
		fontHeight = 2;
		sprintf((char *)ascii, "%03.0f", display_temperature);
		{
			ypos -= 8;
			xpos = -40 + (width - (strlen((char *)ascii) * 6 /*font1 multiplier*/ * fontWidth)) / 2;
			lcdSetFontDirection(dev, DIRECTION0);
		}
		color = WHITE;
		lcdDrawString3(dev, fontHeight, fontWidth, xpos, ypos, ascii, color);
		lcdDrawString3(dev, 1, 1, 80, ypos + 30, (uint8_t *)"E", color);
	}

	// Motor Temp
	{
		if (user_settings->dispaly_fahrenheit) {
			motor_temp_mapped = map(CTOF(esc_telemetry.temp_motor), CTOF(0), CTOF(100), 1, 20);
			display_temperature = CTOF(esc_telemetry.temp_motor);
		} else {
			motor_temp_mapped = map(esc_telemetry.temp_motor, 0, 100, 1, 20);
			display_temperature = esc_telemetry.temp_motor;
		}
		if (display_temperature < 0) display_temperature = 0;

		ypos = 115;
		drawCircularGaugeSmall(dev, 160, ypos, 30, 4, 200, 160, /*100-invert*/(motor_temp_mapped*5), RED, GREEN);
		fontWidth = 2;
		fontHeight = 2;
		sprintf((char *)ascii, "%03.0f", display_temperature);
		{
			ypos -= 8;
			xpos = 40 + (width - (strlen((char *)ascii) * 6 /*font1 multiplier*/ * fontWidth)) / 2;
			lcdSetFontDirection(dev, DIRECTION0);
		}
		color = WHITE;
		lcdDrawString3(dev, fontHeight, fontWidth, xpos, ypos, ascii, color);
		lcdDrawString3(dev, 1, 1, 160, ypos + 30, (uint8_t *)"M", color);
	}

	// Uptime
	//sprintf((char *)ascii, "%d", xTaskGetTickCount() * portTICK_PERIOD_MS / 1000);
	//xpos = (width - (strlen((char *)ascii) * 6 /*font1 multiplier*/ * 1/*fontWidth*/)) / 2;
	//lcdDrawString3(dev, 1, 1, xpos, 110, ascii, color);

	// Efficiency and Range
	static double efficiency = 0;
	static double range = 0;
	{
		// Efficiency
		{
			if (user_settings->display_mph) efficiency = (esc_telemetry.watt_hours - esc_telemetry.watt_hours_charged) / (esc_telemetry.tachometer_abs / 1000.0 * KTOM);
			else efficiency = (esc_telemetry.watt_hours - esc_telemetry.watt_hours_charged) / (esc_telemetry.tachometer_abs / 1000.0);
			if (isnan(efficiency)) efficiency = 0;
			//TODO: Do we need a minimum distance? esc_telemetry.tachometer_abs / 1000.0 < 0.01

			fontWidth = 2;
			fontHeight = 2;
			sprintf((char *)ascii, " %03.1f ", efficiency);
			{
				ypos = 35;
				xpos = (width - (strlen((char*)ascii) * 8 * fontWidth)) / 2;
			}
			lcdDrawString2(dev, fontHeight, fontWidth, xpos, ypos, ascii, YELLOW);

			//TODO: draw once
			if (user_settings->display_mph) sprintf((char *)ascii, "wh/mi");
			else sprintf((char *)ascii, "wh/km");
			fontWidth = 1;
			fontHeight = 1;
			{
				ypos += 30;
				xpos = (width - (strlen((char*)ascii) * 8 * fontWidth)) / 2;
			}
			lcdDrawString2(dev, fontWidth, fontHeight, xpos, ypos, ascii, WHITE);
		}

		// Range
		{
			range = (esc_telemetry.battery_wh * esc_telemetry.battery_level) / efficiency;
			if (isnan(range)) range = 0;

			fontWidth = 2;
			fontHeight = 2;
			sprintf((char *)ascii, " %03.1f ", range);
			{
				ypos = 150;
				xpos = (width - (strlen((char*)ascii) * 8 * fontWidth)) / 2;
			}
			lcdDrawString2(dev, fontHeight, fontWidth, xpos, ypos, ascii, YELLOW);

			//TODO: draw once
			fontWidth = 1;
			fontHeight = 1;
			if (user_settings->display_mph) sprintf((char *)ascii, "mi");
			else sprintf((char *)ascii, "km");
			{
				ypos += 30;
				xpos = (width - (strlen((char*)ascii) * 8 * fontWidth)) / 2;
			}
			lcdDrawString2(dev, 1, 1, xpos, ypos, ascii, WHITE);
		}
	}

	// Vehicle Battery
	// Draw vehicle battery if it's changed more than 5%
	drawCircularGauge(dev, 120, 110, 100, 5, 90, 270, esc_telemetry.battery_level * 100, BLUE, RED);

	lcdUpdate(dev);

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
	uint8_t xpos = (240 /*display width*/ - (strlen(version) * 6 /*font1 multiplier*/ * fontWidth)) / 2;
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
	//lcdDrawRoundRect(dev, 22, 42, 218, 198, 6, GREEN);
	//lcdDrawRoundRect(dev, 23, 43, 217, 197, 6, GREEN);
	//lcdDrawRoundRect(dev, 24, 44, 216, 196, 6, GREEN);

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

	lcdUpdate(dev);

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
	static const uint8_t y_line_height = 25;

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
		/* TODO: No room to draw "OSRR Setup"
		sprintf((char *)ascii, "Setup");
		ypos = 15;
		xpos = (width - (strlen((char *)ascii) * 8 * fontWidth)) / 2;
		color = YELLOW;
		lcdDrawString2(dev, fontHeight, fontWidth, xpos, ypos, ascii, color);
		*/
	}

	ypos = 42;
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
			xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
			lcdDrawString(dev, fx, xpos, ypos, ascii, color);
		}

		ypos += y_line_height;
		if (current_index == SETTING_BUZZER || previous_index == SETTING_BUZZER || first_draw)
		{
			if (current_index == SETTING_BUZZER) color = WHITE;
			else color = GRAY;
			if (user_settings->disable_buzzer) sprintf((char *)ascii, "Haptic: OFF");
			else sprintf((char *)ascii, " Haptic: ON ");
			xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
			lcdDrawString(dev, fx, xpos, ypos, ascii, color);
		}

		ypos += y_line_height;
		if (current_index == SETTING_SPEED || previous_index == SETTING_SPEED || first_draw)
		{
			if (current_index == SETTING_SPEED) color = WHITE;
			else color = GRAY;
			if (user_settings->display_mph) sprintf((char *)ascii, " Speed: MPH ");
			else sprintf((char *)ascii, " Speed: KPH ");
			xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
			lcdDrawString(dev, fx, xpos, ypos, ascii, color);
		}

		ypos += y_line_height;
		if (current_index == SETTING_TEMP || previous_index == SETTING_TEMP || first_draw)
		{
			if (current_index == SETTING_TEMP) color = WHITE;
			else color = GRAY;
			if (user_settings->dispaly_fahrenheit) sprintf((char *)ascii, "Temp: Fahrenheit");
			else sprintf((char *)ascii, "  Temp: Celsius  ");
			xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
			lcdDrawString(dev, fx, xpos, ypos, ascii, color);
		}

		ypos += y_line_height;
		if (current_index == SETTING_THROTTLE || previous_index == SETTING_THROTTLE || first_draw)
		{
			if (current_index == SETTING_THROTTLE) color = WHITE;
			else color = GRAY;
			if (user_settings->throttle_reverse) sprintf((char *)ascii, " Throttle: Reverse ");
			else sprintf((char *)ascii, " Throttle: Forward ");
			xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
			lcdDrawString(dev, fx, xpos, ypos, ascii, color);
		}

		ypos += y_line_height;
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
				case MODEL_CUSTOM:
					sprintf((char *)ascii, "Model: Custom");
					break;
			}
			xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
			lcdDrawString(dev, fx, xpos, ypos, ascii, color);
		}

		ypos += y_line_height;
		if (current_index == SETTING_LEFTY || previous_index == SETTING_LEFTY || first_draw)
		{
			if (current_index == SETTING_LEFTY) color = WHITE;
			else color = GRAY;
			if (user_settings->left_handed) sprintf((char *)ascii, " Lefty: True ");
			else sprintf((char *)ascii, " Lefty: False ");
			xpos = (width - (strlen((char *)ascii) * fontWidth)) / 2;
			lcdDrawString(dev, fx, xpos, ypos, ascii, color);
		}

		// Update previous values for next iteration
		previous_index = current_index;
		previous_settings = (*user_settings);
	}

	// First draw only happens once
	first_draw = false;

	lcdUpdate(dev);

	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
	//ESP_LOGI(__FUNCTION__, "elapsed time[ms]:%d",diffTick*portTICK_RATE_MS);
	return diffTick;
}
