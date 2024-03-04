#pragma once 
#include "config.h"
#include <Arduino.h>

struct ColorRGB {
  uint8_t r, g, b;
}; 
ColorRGB White {255,255,255};
ColorRGB Black {0,0,0};
ColorRGB Red {255,0,0};
ColorRGB Green {0,255,0}; 
ColorRGB Blue {0,0,255};
ColorRGB Fuchsia {255,0,255};
ColorRGB Yellow {255,255,0};
ColorRGB Aqua {0,255,255};

void setColor(ColorRGB color) {
  analogWrite(LED_RED, 255 - color.r);
  analogWrite(LED_GRN, 255 - color.g);
  analogWrite(LED_BLU, 255 - color.b);
}

