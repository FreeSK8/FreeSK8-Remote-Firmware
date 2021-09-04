#ifndef HAPTIC_H_
#define HAPTIC_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "../melody/melody_notes.h"

bool is_haptic_playing;

void haptic_play(int index, bool interrupt_haptic);
void haptic_step(void);

#endif