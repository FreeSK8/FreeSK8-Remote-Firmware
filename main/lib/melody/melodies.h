#ifndef MELODIES_H_
#define MELODIES_H_

#include "melody_notes.h"

const int melody_gotchi_fault[] = {
    // Never Gonna Give You Up - Rick Astley
    // Score available at https://musescore.com/chlorondria_5/never-gonna-give-you-up_alto-sax
    // Arranged by Chlorondria
    NOTE_A4,16, NOTE_B4,16, NOTE_D5,16, NOTE_B4,16,

    NOTE_FS5,-8, NOTE_FS5,-8, NOTE_E5,-4, NOTE_A4,16, NOTE_B4,16, NOTE_D5,16, NOTE_B4,16,
    NOTE_E5,-8, NOTE_E5,-8, NOTE_D5,-8, NOTE_CS5,16, NOTE_B4,8, NOTE_A4,16, NOTE_B4,16, NOTE_D5,16, NOTE_B4,16,
    NOTE_D5,4, NOTE_E5,8, NOTE_CS5,-8, NOTE_B4,16, NOTE_A4,4, NOTE_A4,8,

    NOTE_E5,4, NOTE_D5,2, NOTE_A4,16, NOTE_B4,16, NOTE_D5,16, NOTE_B4,16, //40
    NOTE_FS5,-8, NOTE_FS5,-8, NOTE_E5,-4, NOTE_A4,16, NOTE_B4,16, NOTE_D5,16, NOTE_B4,16,
    NOTE_A5,4, NOTE_CS5,8, NOTE_D5,-8, NOTE_CS5,16, NOTE_B4,8, NOTE_A4,16, NOTE_B4,16, NOTE_D5,16, NOTE_B4,16,
    NOTE_D5,4, NOTE_E5,8, NOTE_CS5,-8, NOTE_B4,16, NOTE_A4,4, NOTE_A4,8, 
    NOTE_E5,4, NOTE_D5,2, NOTE_A4,16, NOTE_B4,16, NOTE_D5,16, NOTE_B4,16,

    NOTE_FS5,-8, NOTE_FS5,-8, NOTE_E5,-4, NOTE_A4,16, NOTE_B4,16, NOTE_D5,16, NOTE_B4,16, //45
    NOTE_A5,4, NOTE_CS5,8, NOTE_D5,-8, NOTE_CS5,16, NOTE_B4,8, NOTE_A4,16, NOTE_B4,16, NOTE_D5,16, NOTE_B4,16,
    NOTE_D5,4, NOTE_E5,8, NOTE_CS5,-8, NOTE_B4,16, NOTE_A4,4, NOTE_A4,8, 
    NOTE_E5,4, NOTE_D5,2, NOTE_A4,16, NOTE_B4,16, NOTE_D5,16, NOTE_B4,16,
    NOTE_FS5,-8, NOTE_FS5,-8, NOTE_E5,-4, NOTE_A4,16, NOTE_B4,16, NOTE_D5,16, NOTE_B4,16, //45

    NOTE_A5,4, NOTE_CS5,8, NOTE_D5,-8, NOTE_CS5,16, NOTE_B4,8, NOTE_A4,16, NOTE_B4,16, NOTE_D5,16, NOTE_B4,16,
    NOTE_D5,4, NOTE_E5,8, NOTE_CS5,-8, NOTE_B4,16, NOTE_A4,4, NOTE_A4,8,

    NOTE_E5,4, NOTE_D5,2, REST,4
};

const int tempo_gotchi_fault=114;

const int melody_esc_fault[] = {
   NOTE_C5,-4, NOTE_C4,-4, REST,4, REST,4, REST,4,
};
const int tempo_esc_fault = 100;

const int melody_ble_fail[] = {
   NOTE_C2,4, NOTE_C2,4, NOTE_B2,4
};
const int tempo_ble_fail = 360;

const int melody_ble_success[] = {
    NOTE_D4,4, NOTE_D4,4, NOTE_G4,4
};
const int tempo_ble_success = 360;

const int melody_storage_limit[] = {
    NOTE_B2,-4, NOTE_C4,4, NOTE_C4,4, NOTE_C5,5, NOTE_C5,5, NOTE_C6,4,
};
const int tempo_storage_limit = 200;

const int melody_esc_temp[] = {
    NOTE_B2,-4, NOTE_C4,4, NOTE_C4,4, NOTE_C5,5,
};
const int tempo_esc_temp = 200;

const int melody_motor_temp[] = {
    NOTE_B2,-4, NOTE_C4,4, NOTE_C4,4, NOTE_C5,5, NOTE_C5,5,
};
const int tempo_motor_temp = 200;

const int melody_voltage_low[] = {
   NOTE_B2,-4, NOTE_C4,4, NOTE_C5,5,
};
const int tempo_voltage_low = 200;

const int melody_ascending[] = {
    // Ascending
    NOTE_D4,8, NOTE_E4,8, NOTE_F4,8, NOTE_G4,8,
};
const int tempo_ascending = 242;

const int melody_descending[] = {
    // Descending
    NOTE_G4,8, NOTE_F4,8, NOTE_E4,8, NOTE_D4,8,
};
const int tempo_descending = 242;

const int melody_startup[] = {
    // Super totally not Mario
    NOTE_E5,8, NOTE_E5,8, REST,8, NOTE_E5,8, REST,8, NOTE_C5,8, NOTE_E5,8,
    NOTE_G5,4, REST,4, NOTE_G4,8, REST,4,
};
const int tempo_startup = 200;

const int melody_gps_locked[] = {
    // GPS Signal Acquisition
    NOTE_G4,8, NOTE_G5,8, NOTE_G4,8, NOTE_G5,8, NOTE_G4,8, NOTE_G5,8,
};
const int tempo_gps_locked = 420;

const int melody_gps_lost[] = {
    // GPS Signal Lost
    NOTE_D4,8, NOTE_E4,8, NOTE_D4,8, NOTE_E4,8, NOTE_D4,8, NOTE_E4,8,
};
const int tempo_gps_lost = 420;

#endif