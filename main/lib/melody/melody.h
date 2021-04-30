#ifndef MELODY_H_
#define MELODY_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "melody_notes.h"

bool is_melody_playing;

void melody_play(int index, bool interrupt_melody);
void melody_step(void);

#endif