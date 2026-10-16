// Waveshare ESP32-S3-Touch-LCD-2.8C pin map (from the Waveshare wiki / demo).
#pragma once

// Shared I2C bus: TCA9554 expander, QMI8658 IMU, PCF85063 RTC, GT911 touch
#define PIN_I2C_SDA 15
#define PIN_I2C_SCL 7
#define I2C_FREQ_HZ 400000

// TCA9554 IO expander
#define TCA9554_ADDR 0x20
#define EXIO_LCD_RST 1
#define EXIO_TP_RST 2
#define EXIO_LCD_CS 3
#define EXIO_SD_CS 4
#define EXIO_BUZZER 8

// ST7701 3-wire SPI (init only)
#define PIN_LCD_SDA 1
#define PIN_LCD_SCL 2

// RGB565 parallel bus
#define PIN_LCD_HSYNC 38
#define PIN_LCD_VSYNC 39
#define PIN_LCD_DE 40
#define PIN_LCD_PCLK 41
// data lines D0..D15 = B1..B5, G0..G5, R1..R5
#define PIN_LCD_DATA {5, 45, 48, 47, 21, 14, 13, 12, 11, 10, 9, 46, 3, 8, 18, 17}

#define PIN_LCD_BL 6

// QMI8658 6-axis IMU (address depends on SA0 strap; both are probed)
#define QMI8658_ADDR_A 0x6B
#define QMI8658_ADDR_B 0x6A

// "Set level" button: on-board BOOT button, also IO0 on the 12-pin header.
// Wire an external momentary switch between IO0 and GND (active low).
#define PIN_BUTTON 0
