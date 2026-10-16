// TCA9554 8-bit I2C IO expander (LCD reset/CS, touch reset, buzzer, ...).
#pragma once
#include <stdint.h>

namespace expander {
bool begin();
// pin: 1..8 (EXIO numbering from the schematic)
void write(uint8_t pin, bool level);
}  // namespace expander
