#pragma once 
#include "config.h"
#include <Arduino.h>


void buzzerTone(int hertz) {
    if (BUZZER_ENABLE) { tone(BUZZER, hertz); }
}

void noBuzzerTone() {
    if (BUZZER_ENABLE) { noTone(BUZZER); } 
}
