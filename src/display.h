// ST7701 480x480 RGB panel with three PSRAM framebuffers, switched on vsync (no tearing,
// and drawing never waits for the panel unless a finished frame is still queued).
#pragma once
#include <stdint.h>

namespace display {
constexpr int W = 480, H = 480;

bool begin();
// Buffer not currently scanned out; draw here, then present().
uint16_t *backBuffer();
// Queue the back buffer for the next vsync and move on to a free buffer.
void present();
// Copy a full-screen image into all buffers (static parts of the dial).
void fillAll(const uint16_t *img);
void setBacklight(uint8_t percent);
// Panel refreshes and refreshes that showed a new frame, since boot.
void counters(uint32_t &vsyncs, uint32_t &flips);
}  // namespace display
