// QMI8658 6-axis IMU, polled over I2C.
#pragma once
#include <stdint.h>

namespace imu {
struct Sample {
    float ax, ay, az;  // g, chip axes
    float gx, gy, gz;  // rad/s, chip axes
};

bool begin();
bool read(Sample &s);
uint8_t address();
}  // namespace imu
