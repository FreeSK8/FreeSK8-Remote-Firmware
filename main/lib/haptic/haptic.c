#include "haptic.h"

#include "../../esp-i2c.h"

#include "esp_log.h"

int haptic_notes=0;
int haptic_wholenote = 0;
int haptic_divider = 0;
int haptic_note_duration = 0;
int haptic_this_note = 0;
bool is_haptic_playing_pause = false;
uint32_t haptic_next_note = 0;
int *haptic;

static long map(long x, long in_min, long in_max, long out_min, long out_max) {
	if (x < in_min) x = in_min;
	if (x > in_max) x = in_max;
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

esp_err_t haptic_spin(int8_t speed)
{
    esp_err_t result;
	uint8_t addr = 0x60; // A0 & A1 are pulled low = 0xC0
	uint8_t regValue = 0x80;			// Before we do anything, we'll want to
									//  clear the fault status. To do that
									//  write 0x80 to register 0x01 on the
									//  DRV8830.
	result = i2c_master_write_slave_reg(I2C_PORT_NUM, addr, 0x01, &regValue, 1); // Clear the fault status.

	regValue = abs(speed);      // Find the byte-ish abs value of the input
	if (regValue > 63) regValue = 63; // Cap the value at 63.
	regValue = regValue<<2;           // Left shift to make room for bits 1:0
	if (speed < 0) regValue |= 0x01;  // Set bits 1:0 based on sign of input.
	else           regValue |= 0x02;

	result = i2c_master_write_slave_reg(I2C_PORT_NUM, addr, 0x00, &regValue, 1);

	return result;
}

esp_err_t haptic_stop()
{
    esp_err_t result;
	uint8_t addr = 0x60; // A0 & A1 are pulled low = 0xC0
	uint8_t regValue = 0x0;
	result = i2c_master_write_slave_reg(I2C_PORT_NUM, addr, 0x00, &regValue, 1); // Stop
	return result;
}

static uint64_t millis(void)
{
    ///printf("millis %lld\n", esp_timer_get_time());
	return esp_timer_get_time() / 1e3;
}

void haptic_play(int index, bool interrupt_melody)
{
	if ((is_haptic_playing && !interrupt_melody))
	{
		return;
	}
	switch (index)
	{
		case MELODY_GOTCHI_FAULT:
			haptic = (int*)&melody_gotchi_fault;
			haptic_notes = sizeof(melody_gotchi_fault)/sizeof(melody_gotchi_fault[0])/2;
			haptic_wholenote = (60000 * 4) / tempo_gotchi_fault;
		break;
		case MELODY_ESC_FAULT:
			haptic = (int*)&melody_esc_fault;
			haptic_notes = sizeof(melody_esc_fault)/sizeof(melody_esc_fault[0])/2;
			haptic_wholenote = (60000 * 4) / tempo_esc_fault;
		break;
		case MELODY_BLE_FAIL:
			haptic = (int*)&melody_ble_fail;
			haptic_notes = sizeof(melody_ble_fail)/sizeof(melody_ble_fail[0])/2;
			haptic_wholenote = (60000 * 4) / tempo_ble_fail;
		break;
		case MELODY_BLE_SUCCESS:
			haptic = (int*)&melody_ble_success;
			haptic_notes = sizeof(melody_ble_success)/sizeof(melody_ble_success[0])/2;
			haptic_wholenote = (60000 * 4) / tempo_ble_success;
		break;
		case MELODY_STORAGE_LIMIT:
			haptic = (int*)&melody_storage_limit;
			haptic_notes = sizeof(melody_storage_limit)/sizeof(melody_storage_limit[0])/2;
			haptic_wholenote = (60000 * 4) / tempo_storage_limit;
		break;
		case MELODY_ESC_TEMP:
			haptic = (int*)&melody_esc_temp;
			haptic_notes = sizeof(melody_esc_temp)/sizeof(melody_esc_temp[0])/2;
			haptic_wholenote = (60000 * 4) / tempo_esc_temp;
		break;
		case MELODY_MOTOR_TEMP:
			haptic = (int*)&melody_motor_temp;
			haptic_notes = sizeof(melody_motor_temp)/sizeof(melody_motor_temp[0])/2;
			haptic_wholenote = (60000 * 4) / tempo_motor_temp;
		break;
		case MELODY_VOLTAGE_LOW:
			haptic = (int*)&melody_voltage_low;
			haptic_notes = sizeof(melody_voltage_low)/sizeof(melody_voltage_low[0])/2;
			haptic_wholenote = (60000 * 4) / tempo_voltage_low;
		break;
		case MELODY_LOG_START:
			haptic = (int*)&melody_ascending;
			haptic_notes = sizeof(melody_ascending)/sizeof(melody_ascending[0])/2;
			haptic_wholenote = (60000 * 4) / tempo_ascending;
		break;
		case MELODY_LOG_STOP:
			haptic = (int*)&melody_descending;
			haptic_notes = sizeof(melody_descending)/sizeof(melody_descending[0])/2;
			haptic_wholenote = (60000 * 4) / tempo_descending;
		break;
		case MELODY_STARTUP:
			haptic = (int*)&melody_startup;
			haptic_notes = sizeof(melody_startup)/sizeof(melody_startup[0])/2;
			haptic_wholenote = (60000 * 4) / tempo_startup;
		break;
		case MELODY_GPS_LOCK:
			haptic = (int*)&melody_gps_locked;
			haptic_notes = sizeof(melody_gps_locked)/sizeof(melody_gps_locked[0])/2;
			haptic_wholenote = (60000 * 4) / tempo_gps_locked;
		break;
		case MELODY_GPS_LOST:
			haptic = (int*)&melody_gps_lost;
			haptic_notes = sizeof(melody_gps_lost)/sizeof(melody_gps_lost[0])/2;
			haptic_wholenote = (60000 * 4) / tempo_gps_lost;
		break;
		case MELODY_CHRIPS:
			haptic = (int*)&melody_chirps;
			haptic_notes=sizeof(melody_chirps)/sizeof(melody_chirps[0])/2;
			haptic_wholenote = (60000 * 4) / tempo_chirps;
		break;
		default:
			ESP_LOGW(__FUNCTION__,"Invalid index %d", index);
		break;
	}

	haptic_divider = 0;
	haptic_note_duration = 0;
	haptic_this_note = 0; // Play from the beginning
	is_haptic_playing = true;
	is_haptic_playing_pause = false;
	haptic_next_note = millis();
}

void haptic_step(void)
{
    ///printf("haptic step\n");
	if (is_haptic_playing)
	{
		// Check if we've reached the end
		if ((haptic_this_note >= haptic_notes *2))
		{
			haptic_this_note = 0;
			is_haptic_playing = false;
			haptic_stop();
			return;
		}

		// Check if it's time to play the next note
		uint32_t now = millis();
		if (now >= haptic_next_note)
		{
			// calculates the duration of each note
			haptic_divider = haptic[haptic_this_note + 1];
			if (haptic_divider > 0) {
				// regular note, just proceed
				haptic_note_duration = (haptic_wholenote) / haptic_divider;
			} else if (haptic_divider < 0) {
				// dotted notes are represented with negative durations!!
				haptic_note_duration = (haptic_wholenote) / abs(haptic_divider);
				haptic_note_duration *= 1.5; // increases the duration in half for dotted notes
			}

			if (is_haptic_playing_pause)
			{
				// stop the waveform generation before the next note.
				haptic_stop();
				haptic_next_note = now + (haptic_note_duration * 0.5);
				is_haptic_playing_pause = false; // Set to false so we play a note on the next step
				haptic_this_note += 2; // Increment current note by 1 (note + duration)
			}
			else
			{
				// we only play the note for 50% of the duration, leaving 50% as a pause
				if (haptic[haptic_this_note] != REST)
				{
					haptic_spin(map((uint32_t)(haptic[haptic_this_note]), NOTE_B0, NOTE_DS8, 25, 63));
				}

				haptic_next_note = now + (haptic_note_duration * 0.5);
				is_haptic_playing_pause = true; // Set to true so we pause on the next step
			}
		}
	}
	else
	{
		haptic_stop();
	}
}