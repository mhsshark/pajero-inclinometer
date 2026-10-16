#include "imu.h"

#include <Arduino.h>
#include <Wire.h>

#include "board_pins.h"

namespace imu {

static uint8_t addr_ = 0;

enum : uint8_t {
    REG_WHO_AM_I = 0x00,  // reads 0x05
    REG_CTRL1 = 0x02,
    REG_CTRL2 = 0x03,  // accel full scale / ODR
    REG_CTRL3 = 0x04,  // gyro full scale / ODR
    REG_CTRL5 = 0x06,  // low-pass filters
    REG_CTRL7 = 0x08,  // sensor enable
    REG_AX_L = 0x35,
    REG_RESET = 0x60,
};

static constexpr float ACC_LSB_PER_G = 8192.0f;  // +-4 g
static constexpr float GYR_LSB_PER_DPS = 64.0f;  // +-512 dps

static bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static bool readRegs(uint8_t reg, uint8_t *buf, size_t n) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(addr_, (uint8_t)n) != n) return false;
    for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
    return true;
}

static bool probe(uint8_t a) {
    addr_ = a;
    uint8_t id = 0;
    return readRegs(REG_WHO_AM_I, &id, 1) && id == 0x05;
}

bool begin() {
    if (!probe(QMI8658_ADDR_A) && !probe(QMI8658_ADDR_B)) {
        addr_ = 0;
        return false;
    }
    writeReg(REG_RESET, 0xB0);
    delay(20);
    bool ok = writeReg(REG_CTRL1, 0x40)     // register auto-increment, little endian
              && writeReg(REG_CTRL2, 0x15)  // accel +-4 g, 250 Hz
              && writeReg(REG_CTRL3, 0x55)  // gyro +-512 dps, 250 Hz
              && writeReg(REG_CTRL5, 0x11)  // accel + gyro LPF enabled
              && writeReg(REG_CTRL7, 0x03); // enable accel + gyro
    delay(30);
    return ok;
}

bool read(Sample &s) {
    uint8_t b[12];
    if (!addr_ || !readRegs(REG_AX_L, b, sizeof b)) return false;
    auto i16 = [&](int i) { return (int16_t)(b[i] | (b[i + 1] << 8)); };
    s.ax = i16(0) / ACC_LSB_PER_G;
    s.ay = i16(2) / ACC_LSB_PER_G;
    s.az = i16(4) / ACC_LSB_PER_G;
    const float k = (float)M_PI / 180.0f / GYR_LSB_PER_DPS;
    s.gx = i16(6) * k;
    s.gy = i16(8) * k;
    s.gz = i16(10) * k;
    return true;
}

uint8_t address() { return addr_; }

}  // namespace imu
