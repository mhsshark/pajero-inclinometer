#include "display.h"

#include <Arduino.h>
#include <driver/spi_master.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_rgb.h>
#include <freertos/semphr.h>
#include <string.h>

#include "board_pins.h"
#include "config.h"
#include "expander.h"

namespace display {

static esp_lcd_panel_handle_t panel_ = nullptr;
// Triple buffering: one on screen, at most one queued for the next vsync, one being drawn.
static uint16_t *fb_[3] = {nullptr, nullptr, nullptr};
static int back_ = 1;
static volatile int shown_ = 0, queued_ = 0;
static volatile uint32_t vsyncs_ = 0, flips_ = 0;
static SemaphoreHandle_t vsync_sem_ = nullptr;
static spi_device_handle_t spi_ = nullptr;

// ---- ST7701 init over 9-bit 3-wire SPI (D/C bit sent as the 1-bit "command" phase) ----
static void spiSend(bool data, uint8_t v) {
    spi_transaction_t t = {};
    t.cmd = data ? 1 : 0;
    t.addr = v;
    spi_device_polling_transmit(spi_, &t);
}

// Waveshare ESP32-S3-Touch-LCD-2.8C init sequence: {cmd, n, data...}, 0xFF n=0xFE -> delay
static const uint8_t kInit[] = {
    0xFF, 5, 0x77, 0x01, 0x00, 0x00, 0x13,
    0xEF, 1, 0x08,
    0xFF, 5, 0x77, 0x01, 0x00, 0x00, 0x10,
    0xC0, 2, 0x3B, 0x00,
    0xC1, 2, 0x10, 0x0C,
    0xC2, 2, 0x07, 0x0A,
    0xC7, 1, 0x00,
    0xCC, 1, 0x10,
    0xCD, 1, 0x08,
    0xB0, 16, 0x05, 0x12, 0x98, 0x0E, 0x0F, 0x07, 0x07, 0x09, 0x09, 0x23, 0x05, 0x52, 0x0F, 0x67, 0x2C, 0x11,
    0xB1, 16, 0x0B, 0x11, 0x97, 0x0C, 0x12, 0x06, 0x06, 0x08, 0x08, 0x22, 0x03, 0x51, 0x11, 0x66, 0x2B, 0x0F,
    0xFF, 5, 0x77, 0x01, 0x00, 0x00, 0x11,
    0xB0, 1, 0x5D,
    0xB1, 1, 0x3E,
    0xB2, 1, 0x81,
    0xB3, 1, 0x80,
    0xB5, 1, 0x4E,
    0xB7, 1, 0x85,
    0xB8, 1, 0x20,
    0xC1, 1, 0x78,
    0xC2, 1, 0x78,
    0xD0, 1, 0x88,
    0xE0, 3, 0x00, 0x00, 0x02,
    0xE1, 11, 0x06, 0x30, 0x08, 0x30, 0x05, 0x30, 0x07, 0x30, 0x00, 0x33, 0x33,
    0xE2, 12, 0x11, 0x11, 0x33, 0x33, 0xF4, 0x00, 0x00, 0x00, 0xF4, 0x00, 0x00, 0x00,
    0xE3, 4, 0x00, 0x00, 0x11, 0x11,
    0xE4, 2, 0x44, 0x44,
    0xE5, 16, 0x0D, 0xF5, 0x30, 0xF0, 0x0F, 0xF7, 0x30, 0xF0, 0x09, 0xF1, 0x30, 0xF0, 0x0B, 0xF3, 0x30, 0xF0,
    0xE6, 4, 0x00, 0x00, 0x11, 0x11,
    0xE7, 2, 0x44, 0x44,
    0xE8, 16, 0x0C, 0xF4, 0x30, 0xF0, 0x0E, 0xF6, 0x30, 0xF0, 0x08, 0xF0, 0x30, 0xF0, 0x0A, 0xF2, 0x30, 0xF0,
    0xE9, 2, 0x36, 0x01,
    0xEB, 7, 0x00, 0x01, 0xE4, 0xE4, 0x44, 0x88, 0x40,
    0xED, 16, 0xFF, 0x10, 0xAF, 0x76, 0x54, 0x2B, 0xCF, 0xFF, 0xFF, 0xFC, 0xB2, 0x45, 0x67, 0xFA, 0x01, 0xFF,
    0xEF, 6, 0x08, 0x08, 0x08, 0x45, 0x3F, 0x54,
    0xFF, 5, 0x77, 0x01, 0x00, 0x00, 0x00,
    0x11, 0,          // sleep out
    0xFF, 0xFE, 120,  // delay 120 ms
    0x3A, 1, 0x66,
    0x36, 1, 0x00,
    0x35, 1, 0x00,
    0x29, 0,          // display on
};

static bool initController() {
    spi_bus_config_t bus = {};
    bus.mosi_io_num = PIN_LCD_SDA;
    bus.miso_io_num = -1;
    bus.sclk_io_num = PIN_LCD_SCL;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = 64;
    if (spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO) != ESP_OK) return false;

    spi_device_interface_config_t dev = {};
    dev.command_bits = 1;
    dev.address_bits = 8;
    dev.mode = 0;
    dev.clock_speed_hz = 10 * 1000 * 1000;
    dev.spics_io_num = -1;  // CS is on the IO expander
    dev.queue_size = 1;
    if (spi_bus_add_device(SPI2_HOST, &dev, &spi_) != ESP_OK) return false;

    expander::write(EXIO_LCD_RST, false);
    delay(10);
    expander::write(EXIO_LCD_RST, true);
    delay(50);

    expander::write(EXIO_LCD_CS, false);
    for (size_t i = 0; i < sizeof kInit;) {
        uint8_t cmd = kInit[i++], n = kInit[i++];
        if (cmd == 0xFF && n == 0xFE) {
            delay(kInit[i++]);
            continue;
        }
        spiSend(false, cmd);
        while (n--) spiSend(true, kInit[i++]);
    }
    expander::write(EXIO_LCD_CS, true);

    // Pins are shared with the SD card; release the bus.
    spi_bus_remove_device(spi_);
    spi_bus_free(SPI2_HOST);
    return true;
}

static bool IRAM_ATTR onVsync(esp_lcd_panel_handle_t, const esp_lcd_rgb_panel_event_data_t *, void *) {
    BaseType_t woken = pdFALSE;
    vsyncs_ = vsyncs_ + 1;
    if (shown_ != queued_) flips_ = flips_ + 1;
    shown_ = queued_;
    xSemaphoreGiveFromISR(vsync_sem_, &woken);
    return woken == pdTRUE;
}

bool begin() {
    ledcAttach(PIN_LCD_BL, 20000, 10);
    setBacklight(0);

    if (!initController()) return false;

    esp_lcd_rgb_panel_config_t cfg = {};
    cfg.clk_src = LCD_CLK_SRC_DEFAULT;
    cfg.timings.pclk_hz = LCD_PCLK_HZ;
    cfg.timings.h_res = W;
    cfg.timings.v_res = H;
    cfg.timings.hsync_pulse_width = 8;
    cfg.timings.hsync_back_porch = 10;
    cfg.timings.hsync_front_porch = 50;
    cfg.timings.vsync_pulse_width = 2;
    cfg.timings.vsync_back_porch = 18;
    cfg.timings.vsync_front_porch = 8;
    cfg.data_width = 16;
    cfg.bits_per_pixel = 16;
    cfg.num_fbs = 3;
    cfg.bounce_buffer_size_px = LCD_BOUNCE_LINES * W;
    cfg.dma_burst_size = 64;
    cfg.hsync_gpio_num = PIN_LCD_HSYNC;
    cfg.vsync_gpio_num = PIN_LCD_VSYNC;
    cfg.de_gpio_num = PIN_LCD_DE;
    cfg.pclk_gpio_num = PIN_LCD_PCLK;
    cfg.disp_gpio_num = -1;
    const int data[16] = PIN_LCD_DATA;
    for (int i = 0; i < 16; i++) cfg.data_gpio_nums[i] = data[i];
    cfg.flags.fb_in_psram = 1;
    // Drop scanned-out lines from the data cache so the LCD stream doesn't evict the lines
    // being rendered (safe: we never write the buffer that is being scanned out).
    cfg.flags.bb_invalidate_cache = 1;

    // The panel's interrupt (which also refills the bounce buffers from PSRAM, ~26 MB/s) is
    // bound to the core that creates it. Create it on core 0 so core 1 is free for rendering.
    vsync_sem_ = xSemaphoreCreateBinary();
    struct Ctx {
        esp_lcd_rgb_panel_config_t *cfg;
        SemaphoreHandle_t done;
        bool ok;
    } ctx{&cfg, xSemaphoreCreateBinary(), false};
    xTaskCreatePinnedToCore(
        [](void *arg) {
            Ctx *c = (Ctx *)arg;
            if (esp_lcd_new_rgb_panel(c->cfg, &panel_) == ESP_OK) {
                esp_lcd_rgb_panel_event_callbacks_t cbs = {};
                cbs.on_vsync = onVsync;
                esp_lcd_rgb_panel_register_event_callbacks(panel_, &cbs, nullptr);
                esp_lcd_panel_reset(panel_);
                esp_lcd_panel_init(panel_);
                c->ok = true;
            }
            xSemaphoreGive(c->done);
            vTaskDelete(nullptr);
        },
        "lcd_init", 4096, &ctx, 10, nullptr, 0);
    xSemaphoreTake(ctx.done, portMAX_DELAY);
    vSemaphoreDelete(ctx.done);
    if (!ctx.ok) return false;
    esp_lcd_rgb_panel_get_frame_buffer(panel_, 3, (void **)&fb_[0], (void **)&fb_[1], (void **)&fb_[2]);
    for (auto *fb : fb_) memset(fb, 0, W * H * 2);
    shown_ = 0;
    queued_ = 0;
    back_ = 1;
    return true;
}

uint16_t *backBuffer() { return fb_[back_]; }

void counters(uint32_t &vsyncs, uint32_t &flips) {
    vsyncs = vsyncs_;
    flips = flips_;
}

void present() {
    // Keep at most one frame queued: wait until the previously queued one is on screen.
    while (queued_ != shown_)
        if (xSemaphoreTake(vsync_sem_, pdMS_TO_TICKS(100)) != pdTRUE) break;
    // Passing one of the driver's own framebuffers just switches scan-out to it at the next frame.
    esp_lcd_panel_draw_bitmap(panel_, 0, 0, W, H, fb_[back_]);
    queued_ = back_;
    back_ = 3 - shown_ - queued_;  // the buffer that is neither on screen nor queued
}

void fillAll(const uint16_t *img) {
    for (auto *fb : fb_) memcpy(fb, img, W * H * 2);
}

void setBacklight(uint8_t percent) {
    if (percent > 100) percent = 100;
    ledcWrite(PIN_LCD_BL, (uint32_t)percent * 1023 / 100);
}

}  // namespace display
