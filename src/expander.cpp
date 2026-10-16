#include "expander.h"

#include <Wire.h>

#include "board_pins.h"

namespace expander {

static uint8_t out_ = 0;

static bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(TCA9554_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool begin() {
    // EXIO1-4 and EXIO8 outputs; EXIO5-7 (IMU/RTC interrupts) inputs.
    out_ = (1 << (EXIO_LCD_RST - 1)) | (1 << (EXIO_LCD_CS - 1)) | (1 << (EXIO_TP_RST - 1)) | (1 << (EXIO_SD_CS - 1));
    if (!writeReg(0x01, out_)) return false;
    return writeReg(0x03, 0x70);
}

void write(uint8_t pin, bool level) {
    uint8_t bit = 1 << (pin - 1);
    out_ = level ? (out_ | bit) : (out_ & ~bit);
    writeReg(0x01, out_);
}

}  // namespace expander
