// Pajero Gen 1 style inclinometer face (blue/orange variant), 480x480 RGB565.
// Portable: no Arduino dependencies (also built by tools/sim on the host).
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "canvas.h"

enum class GaugeMessage { None, HoldButton, HoldStill, LevelSet, TryAgain, ImuError, Count };

class Gauge {
public:
    static constexpr int W = 480, H = 480;

    // alloc_big: large, rarely-read buffers (PSRAM on the ESP32).
    // alloc_fast: small tables read every pixel (internal RAM on the ESP32). Returns false on OOM.
    bool begin(void *(*alloc_big)(size_t), void *(*alloc_fast)(size_t));

    // Static ring/bezel; copy into every framebuffer once.
    const uint16_t *background() const { return bg_.px; }

    // Redraw the ball window into fb: the face (sky/ground, horizon and vertical pitch scale)
    // rotates with roll, and the orange needle slides along the pitch scale with pitch.
    // roll_deg > 0: vehicle's right side down. pitch_deg > 0: nose up.
    void drawBall(uint16_t *fb, float roll_deg, float pitch_deg, GaugeMessage msg) const;

    // The same in pieces, so the rows can be split across CPU cores:
    //   f = prepare(roll, pitch); drawRows(fb, f, k, n) on each core k of n; drawOverlays(fb, msg).
    struct Frame {
        int32_t du, dv;      // ball (u, v) step per screen pixel, 16.16
        float cs, sn, p;
        int32_t needle_v;    // needle texture v offset, 16.16
    };
    Frame prepare(float roll_deg, float pitch_deg) const;
    void drawRows(uint16_t *fb, const Frame &f, int first, int step) const;
    void drawOverlays(uint16_t *fb, GaugeMessage msg) const;

private:
    struct OverlayPx {
        uint16_t x, y;
        uint16_t premul;  // colour premultiplied by alpha (rendered over black)
        uint8_t a;        // 0..255
    };
    struct Overlay {
        OverlayPx *px = nullptr;
        int n = 0;
    };
    // Small RGB565 image with alpha, sampled rotated (bilinear) in ball coordinates.
    struct Layer {
        uint16_t *col = nullptr;  // premultiplied by alpha
        uint8_t *a = nullptr;
        int w = 0, h = 0;
    };

    void renderBackground();
    void renderFace();
    Layer buildLayer(int w, int h, void (*draw)(Canvas &, bool mask));
    void renderShading();
    Overlay buildOverlay(int x, int y, int w, int h, void (*draw)(Canvas &, int ox, int oy, bool mask, const void *ctx),
                         const void *ctx);
    static void applyOverlay(uint16_t *fb, const Overlay &o);

    void *(*alloc_)(size_t) = nullptr;
    void *(*alloc_fast_)(size_t) = nullptr;
    Canvas bg_{};
    uint32_t *face_ = nullptr;      // colour by signed distance from the horizon, 1/8 px steps (spread)
    Layer scale_;                   // vertical pitch scale, part of the rotating face
    uint8_t *scale_occ_ = nullptr;  // 4x4-texel blocks of scale_ with any coverage (dilated by 1)
    Layer needle_;                  // pitch needle, slides along the scale
    // Radial tables indexed by squared distance from the centre (see radIndex in gauge.cpp)
    uint8_t *edge_ = nullptr;    // ball coverage (AA rim), 0..255
    uint16_t *shade_ = nullptr;  // darkening toward the rim, 0..256 (dithered down to 0..32)
    // Specular highlight, separable: hilite = hx_[x] * hy_[y] >> 8
    uint8_t *hx_ = nullptr, *hy_ = nullptr;
    int16_t *span_ = nullptr;   // per bbox row: first x with any coverage (rows are symmetric)
    // The ball shaded all-sky / all-ground (dither, highlight and AA rim baked in), BOX x BOX.
    // Per frame each row is copied from one or the other on either side of the horizon.
    uint16_t *sky_img_ = nullptr, *gnd_img_ = nullptr;
    Overlay msgs_[(int)GaugeMessage::Count];
};
