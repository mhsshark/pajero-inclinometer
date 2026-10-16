// Minimal anti-aliased software rasterizer for RGB565 buffers.
// Only used at boot to pre-render the dial, so it favours clarity over speed.
// Portable: no Arduino dependencies (also built by tools/sim on the host).
#pragma once
#include <stdint.h>

struct FontGlyph {
    uint32_t offset;   // into bitmap
    uint8_t w, h;
    int8_t xoff;       // from pen x
    int8_t yoff;       // from baseline (negative = above)
    uint8_t adv;
};

struct Font {
    const uint8_t *bitmap;
    const FontGlyph *glyphs;
    const char *chars;
    int cap_height;
};

enum class Align { Left, Center, Right };

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

struct Canvas {
    uint16_t *px;
    int w, h;

    void clear(uint16_t c);
    void blend(int x, int y, uint16_t c, float a);
    void fillRect(int x, int y, int w, int h, uint16_t c);

    // Disc / annulus with soft edges. Optional half-plane clip: keep pixels with y < clip_y (upper)
    // or y >= clip_y (lower) when clip != 0.
    void fillAnnulus(float cx, float cy, float r0, float r1, uint16_t c, int clip = 0, float clip_y = 0);
    void fillCircle(float cx, float cy, float r, uint16_t c) { fillAnnulus(cx, cy, -1, r, c); }

    // Thick line with round caps.
    void line(float x0, float y0, float x1, float y1, float width, uint16_t c);
    // Thick line with flat (butt) caps.
    void bar(float x0, float y0, float x1, float y1, float width, uint16_t c);
    void triangle(float x0, float y0, float x1, float y1, float x2, float y2, uint16_t c);
    void roundRect(float x, float y, float w, float h, float r, uint16_t c, float alpha = 1.0f);

    int textWidth(const Font &f, const char *s) const;
    // y is the vertical centre of the digits' cap height.
    void text(const Font &f, const char *s, float x, float y, uint16_t c, Align a = Align::Center);
};
