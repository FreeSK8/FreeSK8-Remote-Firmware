#include "melody.h"
#include "esp_log.h"
#include "driver/ledc.h"

int melody_notes=0;
int melody_wholenote = 0;
int melody_divider = 0;
int melody_note_duration = 0;
int melody_this_note = 0;
bool is_melody_playing_pause = false;
uint32_t melody_next_note = 0;
int *melody;
int melody_last_alert_index = MELODY_NONE;
uint16_t melody_snooze_seconds = 0;

#define LEDC_CHANNEL 0
esp_err_t set_frequency_and_duty_cycle(uint32_t frequency, uint32_t duty_cycle_percent)
{
    esp_err_t result;
    if (frequency != 0)
    {
        result = ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL, frequency);
        ///printf("frequency result: %d\n", result);
    } else {
        result = ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL, 0);
		return result;
    }
    
    result = ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL, duty_cycle_percent);
    ///printf("duty result: %d\n", result);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL);
	return result;
}

uint64_t millis(void)
{
    ///printf("millis %lld\n", esp_timer_get_time());
	return esp_timer_get_time() / 1e3;
}

void melody_play(int index, bool interrupt_melody)
{
	if ((is_melody_playing && !interrupt_melody) || melody_snooze_seconds > 0)
	{
		return;
	}
	switch (index)
	{
		case MELODY_GOTCHI_FAULT:
			melody = (int*)&melody_gotchi_fault;
			// sizeof gives the number of bytes, each int value is composed of two bytes (16 bits)
			// there are two values per note (pitch and duration), so for each note there are four bytes
			melody_notes=sizeof(melody_gotchi_fault)/sizeof(melody_gotchi_fault[0])/2;
			// this calculates the duration of a whole note in ms (60s/tempo)*4 beats
			melody_wholenote = (60000 * 4) / tempo_gotchi_fault;
			melody_last_alert_index = MELODY_GOTCHI_FAULT;
		break;
		case MELODY_ESC_FAULT:
			melody = (int*)&melody_esc_fault;
			melody_notes=sizeof(melody_esc_fault)/sizeof(melody_esc_fault[0])/2;
			melody_wholenote = (60000 * 4) / tempo_esc_fault;
			melody_last_alert_index = MELODY_ESC_FAULT;
		break;
		case MELODY_BLE_FAIL:
			melody = (int*)&melody_ble_fail;
			melody_notes=sizeof(melody_ble_fail)/sizeof(melody_ble_fail[0])/2;
			melody_wholenote = (60000 * 4) / tempo_ble_fail;
		break;
		case MELODY_BLE_SUCCESS:
			melody = (int*)&melody_ble_success;
			melody_notes=sizeof(melody_ble_success)/sizeof(melody_ble_success[0])/2;
			melody_wholenote = (60000 * 4) / tempo_ble_success;
		break;
		case MELODY_STORAGE_LIMIT:
			melody = (int*)&melody_storage_limit;
			melody_notes=sizeof(melody_storage_limit)/sizeof(melody_storage_limit[0])/2;
			melody_wholenote = (60000 * 4) / tempo_storage_limit;
			melody_last_alert_index = MELODY_STORAGE_LIMIT;
		break;
		case MELODY_ESC_TEMP:
			melody = (int*)&melody_esc_temp;
			melody_notes=sizeof(melody_esc_temp)/sizeof(melody_esc_temp[0])/2;
			melody_wholenote = (60000 * 4) / tempo_esc_temp;
			melody_last_alert_index = MELODY_ESC_TEMP;
		break;
		case MELODY_MOTOR_TEMP:
			melody = (int*)&melody_motor_temp;
			melody_notes=sizeof(melody_motor_temp)/sizeof(melody_motor_temp[0])/2;
			melody_wholenote = (60000 * 4) / tempo_motor_temp;
			melody_last_alert_index = MELODY_MOTOR_TEMP;
		break;
		case MELODY_VOLTAGE_LOW:
			melody = (int*)&melody_voltage_low;
			melody_notes=sizeof(melody_voltage_low)/sizeof(melody_voltage_low[0])/2;
			melody_wholenote = (60000 * 4) / tempo_voltage_low;
			melody_last_alert_index = MELODY_VOLTAGE_LOW;
		break;
		case MELODY_LOG_START:
			melody = (int*)&melody_ascending;
			melody_notes=sizeof(melody_ascending)/sizeof(melody_ascending[0])/2;
			melody_wholenote = (60000 * 4) / tempo_ascending;
		break;
		case MELODY_LOG_STOP:
			melody = (int*)&melody_descending;
			melody_notes=sizeof(melody_descending)/sizeof(melody_descending[0])/2;
			melody_wholenote = (60000 * 4) / tempo_descending;
		break;
		case MELODY_STARTUP:
			melody = (int*)&melody_startup;
			melody_notes=sizeof(melody_startup)/sizeof(melody_startup[0])/2;
			melody_wholenote = (60000 * 4) / tempo_startup;
		break;
		case MELODY_GPS_LOCK:
			melody = (int*)&melody_gps_locked;
			melody_notes=sizeof(melody_gps_locked)/sizeof(melody_gps_locked[0])/2;
			melody_wholenote = (60000 * 4) / tempo_gps_locked;
		break;
		case MELODY_GPS_LOST:
			melody = (int*)&melody_gps_lost;
			melody_notes=sizeof(melody_gps_lost)/sizeof(melody_gps_lost[0])/2;
			melody_wholenote = (60000 * 4) / tempo_gps_lost;
		break;
		case MELODY_CHRIPS:
			melody = (int*)&melody_chirps;
			melody_notes=sizeof(melody_chirps)/sizeof(melody_chirps[0])/2;
			melody_wholenote = (60000 * 4) / tempo_chirps;
		break;
		default:
			ESP_LOGW(__FUNCTION__,"Invalid index %d", index);
		break;
	}

	melody_divider = 0;
	melody_note_duration = 0;
	melody_this_note = 0; // Play from the beginning
	is_melody_playing = true;
	is_melody_playing_pause = false;
	melody_next_note = millis();
	//nrf_pwm_set_enabled(true);
}

void melody_step(void)
{
    ///printf("melody step\n");
	if (is_melody_playing)
	{
		// Check if we've reached the end
		if ((melody_this_note >= melody_notes *2))
		{
			melody_this_note = 0;
			is_melody_playing = false;
			ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL, 0);
			return;
		}

		// Check if it's time to play the next note
		uint32_t now = millis();
		if (now >= melody_next_note)
		{
			// calculates the duration of each note
			melody_divider = melody[melody_this_note + 1];
			if (melody_divider > 0) {
				// regular note, just proceed
				melody_note_duration = (melody_wholenote) / melody_divider;
			} else if (melody_divider < 0) {
				// dotted notes are represented with negative durations!!
				melody_note_duration = (melody_wholenote) / abs(melody_divider);
				melody_note_duration *= 1.5; // increases the duration in half for dotted notes
			}

			if (is_melody_playing_pause)
			{
                ///printf("pause\n");
				// stop the waveform generation before the next note.
				set_frequency_and_duty_cycle((uint32_t)(melody[melody_this_note]), 0);
				melody_next_note = now + (melody_note_duration * 0.1);
				is_melody_playing_pause = false; // Set to false so we play a note on the next step
				melody_this_note += 2; // Increment current note by 1 (note + duration)
			}
			else
			{
                ///printf("note %d\n", (uint32_t)(melody[melody_this_note]));
				// we only play the note for 90% of the duration, leaving 10% as a pause
				set_frequency_and_duty_cycle((uint32_t)(melody[melody_this_note]), 45);
				melody_next_note = now + (melody_note_duration * 0.9);
				is_melody_playing_pause = true; // Set to true so we pause on the next step
			}
		}
	}
	else
	{
		ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL, 0);
	}
}