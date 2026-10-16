// User tunables.
#pragma once

// ---- Mounting / axes ------------------------------------------------------
// Device frame used by the firmware: X = screen right, Y = screen up, Z = out of the screen
// (towards the driver). Map the QMI8658 chip axes onto it here. Calibration (long press)
// absorbs any rotation of the chip within the screen plane and small mounting tilts, but not
// a mirrored/flipped axis: if pitch or roll move the wrong way, flip the sign below.
// ESP32-S3-Touch-LCD-2.8C: chip X points up the screen, chip Z points into the screen.
#define IMU_MAP_X(ax, ay, az) (ay)
#define IMU_MAP_Y(ax, ay, az) (ax)
#define IMU_MAP_Z(ax, ay, az) (-(az))
#define ROLL_SIGN 1.0f   // +1: right side down rotates the horizon counter-clockwise
#define PITCH_SIGN 1.0f  // +1: nose up moves the needle toward the black numbers

// ---- Behaviour ------------------------------------------------------------
#define LONG_PRESS_MS 2000        // hold time to set level
#define CAL_SAMPLE_MS 1000        // averaging window while setting level
#define FILTER_TAU_S 0.3f         // complementary filter: lower = trusts accelerometer more
#define NEEDLE_OMEGA 32.0f        // display spring stiffness (rad/s); lower = lazier, more fluid-like ball
#define BACKLIGHT_PERCENT 90
#define BUZZER_ENABLED 1          // short beep when level is set
#define SERIAL_DEBUG 0            // 1: start with the 5 Hz angle/fps log on (toggle any time with 'd')

// ---- Display timing (Waveshare demo values) -------------------------------
#define LCD_PCLK_HZ 16000000      // 16 MHz is reliable; demo allows up to 30 MHz
#define LCD_BOUNCE_LINES 10
