#include "gauge.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "font_data.h"

// Keep the per-frame hot path in internal instruction RAM on the ESP32 (flash cache misses
// are expensive while the LCD is streaming from PSRAM).
#ifdef ESP_PLATFORM
#include <esp_attr.h>
#define GAUGE_HOT IRAM_ATTR
#define DRAM_ATTR_COMPAT DRAM_ATTR
#else
#define GAUGE_HOT
#define DRAM_ATTR_COMPAT
#endif

// ---- Geometry -------------------------------------------------------------
static constexpr float CX = 240.0f, CY = 240.0f;
static constexpr int BALL_R = 168;                 // visible ball window radius
static constexpr float RIM_R = 173.0f;             // silver rim outer radius
static constexpr float RING_R = 239.5f;            // outer scale ring
static constexpr float PX_PER_DEG = 3.4f;          // pitch scale: needle travel per degree
static constexpr float MAX_DEG = 45.0f;
static constexpr int BOX = 2 * BALL_R + 2;         // ball bounding box size
static constexpr int BOX_X = (int)CX - BOX / 2, BOX_Y = (int)CY - BOX / 2;
static constexpr int FACE_SUB = 8;                 // face strip samples per pixel
static constexpr int FACE_HALF = (BOX / 2 + 2) * FACE_SUB;
static constexpr int FACE_LEN = 2 * FACE_HALF;
static constexpr int NEEDLE_W = 56, NEEDLE_H = 28;
static constexpr int SCALE_W = 136, SCALE_H = 348;      // tight box around the pitch scale
static constexpr int OCC_W = (SCALE_W + 3) / 4, OCC_H = (SCALE_H + 3) / 4;
// Radial tables: index = (X^2 + Y^2) >> 4 with X, Y = 2 * (pixel centre offset), i.e. 4 d^2 / 16.
static constexpr int RAD_LEN = (4 * (BALL_R + 2) * (BALL_R + 2) >> 4) + 1;
static constexpr float NEEDLE_CX = 28, NEEDLE_CY = 18;  // reading point inside the needle image

// ---- Palette (matched to images.jpeg) -------------------------------------
static const uint16_t C_BLACK = rgb565(0, 0, 0);
static const uint16_t C_RING_TOP = rgb565(24, 24, 26);
static const uint16_t C_RING_BOT = rgb565(178, 182, 188);
static const uint16_t C_BEZEL = rgb565(70, 72, 76);
static const uint16_t C_RIM = rgb565(225, 228, 232);
static const uint16_t C_RIM_DARK = rgb565(90, 92, 96);
static const uint16_t C_WHITE = rgb565(245, 245, 245);
static const uint16_t C_INK = rgb565(20, 20, 22);
static const uint16_t C_ORANGE = rgb565(242, 118, 28);
static const uint16_t C_ORANGE_DARK = rgb565(200, 80, 10);
static const uint16_t C_RED_ORANGE = rgb565(240, 70, 20);
static const uint16_t C_SKY = rgb565(118, 186, 216);
static const uint16_t C_GROUND = rgb565(196, 199, 203);
static const uint16_t C_TAPE = rgb565(28, 28, 30);

static void drawScale(Canvas &c, bool mask);
static void drawNeedle(Canvas &c, bool mask);

bool Gauge::begin(void *(*alloc_big)(size_t), void *(*alloc_fast)(size_t)) {
    alloc_ = alloc_big;
    alloc_fast_ = alloc_fast;
    bg_ = {(uint16_t *)alloc_big(W * H * 2), W, H};
    face_ = (uint32_t *)alloc_fast(FACE_LEN * 4);
    edge_ = (uint8_t *)alloc_fast(RAD_LEN);
    shade_ = (uint16_t *)alloc_fast(RAD_LEN * 2);
    hx_ = (uint8_t *)alloc_fast(BOX);
    hy_ = (uint8_t *)alloc_fast(BOX);
    span_ = (int16_t *)alloc_fast(BOX * sizeof(int16_t));
    sky_img_ = (uint16_t *)alloc_big(BOX * BOX * 2);
    gnd_img_ = (uint16_t *)alloc_big(BOX * BOX * 2);
    if (!bg_.px || !face_ || !edge_ || !shade_ || !hx_ || !hy_ || !span_ || !sky_img_ || !gnd_img_) return false;

    renderBackground();
    renderFace();
    scale_ = buildLayer(SCALE_W, SCALE_H, drawScale);
    needle_ = buildLayer(NEEDLE_W, NEEDLE_H, drawNeedle);
    if (!scale_.col || !needle_.col) return false;
    // Occupancy of the scale in 4x4 blocks; a bilinear tap at texel (iu, iv) reads up to (iu+1, iv+1),
    // so mark a block if any texel within one of it is covered.
    scale_occ_ = (uint8_t *)alloc_fast(OCC_W * OCC_H);
    if (!scale_occ_) return false;
    for (int by = 0; by < OCC_H; by++)
        for (int bx = 0; bx < OCC_W; bx++) {
            uint8_t any = 0;
            for (int y = by * 4; y <= by * 4 + 4 && y < SCALE_H; y++)
                for (int x = bx * 4; x <= bx * 4 + 4 && x < SCALE_W; x++) any |= scale_.a[y * SCALE_W + x];
            scale_occ_[by * OCC_W + bx] = any != 0;
        }
    renderShading();


    static const char *const texts[] = {nullptr, "KEEP HOLDING", "HOLD STILL", "LEVEL SET", "TRY AGAIN", "IMU ERROR"};
    static_assert(sizeof texts / sizeof *texts == (int)GaugeMessage::Count, "message table");
    for (int i = 1; i < (int)GaugeMessage::Count; i++) {
        msgs_[i] = buildOverlay(
            (int)CX - 90, (int)CY + 58, 180, 44,
            [](Canvas &c, int ox, int oy, bool m, const void *ctx) {
                const char *s = (const char *)ctx;
                c.roundRect(CX - 88 - ox, CY + 60 - oy, 176, 40, 12, m ? C_WHITE : C_BLACK, m ? 0.8f : 1.0f);
                c.text(font_msg, s, CX - ox, CY + 80 - oy, C_WHITE);
            },
            texts[i]);
    }
    return true;
}

// Point on the outer ring at `deg` above (+) / below (-) the horizontal, on the left (side=-1) or right (+1).
static void ringPoint(float r, float deg, int side, float &x, float &y) {
    float a = deg * (float)M_PI / 180.0f;
    x = CX + side * r * cosf(a);
    y = CY - r * sinf(a);
}

void Gauge::renderBackground() {
    Canvas &c = bg_;
    c.clear(C_BLACK);
    c.fillAnnulus(CX, CY, RIM_R - 1, RING_R, C_RING_TOP, -1, CY);
    c.fillAnnulus(CX, CY, RIM_R - 1, RING_R, C_RING_BOT, +1, CY);
    c.fillAnnulus(CX, CY, RING_R - 5, RING_R, C_BEZEL);

    // Orange zero line across the ring on both sides
    c.bar(CX - RING_R + 4, CY, CX - RIM_R + 1, CY, 6, C_ORANGE);
    c.bar(CX + RIM_R - 1, CY, CX + RING_R - 4, CY, 6, C_ORANGE);

    // Upper half: 10..40, white on black. Lower half: 5..45, black on grey.
    // Both ends of the tilted horizon are read: e.g. 15 deg shows up on the lower scale at one end.
    char buf[4];
    for (int side = -1; side <= 1; side += 2) {
        for (int deg = 10; deg <= 40; deg += 10) {
            float x0, y0, x1, y1, lx, ly;
            ringPoint(RIM_R + 1, deg, side, x0, y0);
            ringPoint(RIM_R + 16, deg, side, x1, y1);
            ringPoint(RIM_R + 36, deg, side, lx, ly);
            c.bar(x0, y0, x1, y1, 3.5f, C_WHITE);
            snprintf(buf, sizeof buf, "%d", deg);
            c.text(font_label, buf, lx, ly, C_WHITE);
        }
        for (int deg = 5; deg <= 45; deg += 10) {
            float x0, y0, x1, y1, lx, ly;
            ringPoint(RIM_R + 1, -deg, side, x0, y0);
            ringPoint(RIM_R + 16, -deg, side, x1, y1);
            ringPoint(RIM_R + 36, -deg, side, lx, ly);
            c.bar(x0, y0, x1, y1, 3.5f, C_INK);
            snprintf(buf, sizeof buf, "%d", deg);
            c.text(font_label, buf, lx, ly, C_INK);
        }
    }

    // Silver rim around the ball window
    c.fillAnnulus(CX, CY, BALL_R - 2, RIM_R, C_RIM);
    c.fillAnnulus(CX, CY, RIM_R - 1.2f, RIM_R + 0.3f, C_RIM_DARK);
}

static inline float cover(float v, float lo, float hi) {  // 1-px box filter over [lo, hi]
    float c = fminf(v + 0.5f, hi) - fmaxf(v - 0.5f, lo);
    return c < 0 ? 0 : (c > 1 ? 1 : c);
}

static inline uint32_t spread(uint16_t c);
static inline uint16_t unspread(uint32_t s);
static inline uint16_t lerp565(uint16_t a, uint16_t b, uint32_t w5);
static inline uint32_t shadePx(uint32_t sc, uint32_t sh, uint32_t h);
static inline uint32_t shadeAtImpl(const uint16_t *shade, int x, int y, int ri);

void Gauge::renderFace() {
    // 1-D cross-section of the rotating face, perpendicular to the horizon (v < 0: sky side).
    // Stored pre-spread (see spread()) for the per-pixel shading multiply.
    Canvas strip{(uint16_t *)alloc_(FACE_LEN * 2), FACE_LEN, 1};
    if (!strip.px) return;
    for (int i = 0; i < FACE_LEN; i++) {
        float v = (i + 0.5f - FACE_HALF) / FACE_SUB;
        strip.px[i] = v < 0 ? C_SKY : C_GROUND;
        strip.blend(i, 0, C_ORANGE, cover(v, -3.5f, 2.5f));
        strip.blend(i, 0, C_ORANGE_DARK, cover(v, 2.5f, 4.5f));
        face_[i] = spread(strip.px[i]);
    }
}

// Vertical pitch scale, centred in its image. mask: draw everything white (alpha pass).
static void drawScale(Canvas &c, bool m) {
    const float ox = c.w / 2.0f, oy = c.h / 2.0f;
    const float tape_half = 9, tape_end = MAX_DEG * PX_PER_DEG + 14;
    c.bar(ox, oy - tape_end, ox, oy + tape_end, tape_half * 2, m ? C_WHITE : C_TAPE);

    char buf[4];
    for (int deg = 5; deg <= (int)MAX_DEG; deg += 5) {
        bool major = deg % 10 == 0;
        float len = major ? 40 : 22;
        float up = oy - deg * PX_PER_DEG, dn = oy + deg * PX_PER_DEG;
        // graduations inside the tape
        c.bar(ox - 5, up, ox + 5, up, 1.6f, C_WHITE);
        c.bar(ox - 5, dn, ox + 5, dn, 1.6f, C_WHITE);
        // upper half: black ladder on the left, black numbers on the right
        c.bar(ox - tape_half - 3, up, ox - tape_half - 3 - len, up, major ? 3.5f : 2.5f, m ? C_WHITE : C_INK);
        // lower half: orange ladder on the right, orange numbers on the left
        c.bar(ox + tape_half + 3, dn, ox + tape_half + 3 + len, dn, major ? 3.5f : 2.5f, m ? C_WHITE : C_ORANGE);
        if (major && deg <= 40) {
            snprintf(buf, sizeof buf, "%d", deg);
            c.text(font_tape, buf, ox + tape_half + 21, up, m ? C_WHITE : C_INK, Align::Left);
            c.text(font_tape, buf, ox - tape_half - 21, dn, m ? C_WHITE : C_ORANGE_DARK, Align::Right);
        }
    }
}

// Pitch needle: flat base across the tape with a rounded dome on top (see the cropped reference photo).
static void drawNeedle(Canvas &c, bool m) {
    const float x = NEEDLE_CX, y = NEEDLE_CY;  // y = reading line
    uint16_t edge = m ? C_WHITE : C_INK, fill = m ? C_WHITE : C_RED_ORANGE;
    uint16_t lite = m ? C_WHITE : rgb565(255, 150, 110);
    c.roundRect(x - 24.5f, y - 4.5f, 49, 11, 4.5f, edge);
    c.fillAnnulus(x, y - 2, -1, 13.5f, edge, -1, y - 2);
    c.roundRect(x - 23, y - 3, 46, 8, 3.5f, fill);
    c.fillAnnulus(x, y - 1.5f, -1, 12, fill, -1, y - 1.5f);
    c.roundRect(x - 6, y - 11, 9, 4, 2, lite, m ? 1.0f : 0.7f);
}

Gauge::Layer Gauge::buildLayer(int w, int h, void (*draw)(Canvas &, bool)) {
    // Drawn twice: colours over black (= premultiplied) and all-white over black (= alpha).
    Layer l;
    // Scratch canvases in big memory; the result goes to fast memory (it's sampled every pixel).
    Canvas col{(uint16_t *)alloc_(w * h * 2), w, h}, msk{(uint16_t *)alloc_(w * h * 2), w, h};
    l.a = (uint8_t *)alloc_fast_(w * h);
    l.col = (uint16_t *)alloc_fast_(w * h * 2);
    if (!col.px || !msk.px || !l.a || !l.col) {
        l.col = nullptr;
        return l;
    }
    col.clear(C_BLACK);
    msk.clear(C_BLACK);
    draw(col, false);
    draw(msk, true);
    for (int i = 0; i < w * h; i++) {
        l.a[i] = (uint8_t)(((msk.px[i] >> 5) & 63) * 255 / 63);
        l.col[i] = col.px[i];
    }
    l.w = w;
    l.h = h;
    // scratch canvases are leaked deliberately: built once at boot
    return l;
}

void Gauge::renderShading() {
    // Radial tables: AA rim coverage and glass-ball darkening toward the rim.
    for (int i = 0; i < RAD_LEN; i++) {
        float d = sqrtf((i * 16 + 8) / 4.0f);
        float cov = BALL_R - d + 0.5f;
        edge_[i] = (uint8_t)((cov < 0 ? 0 : (cov > 1 ? 1 : cov)) * 255 + 0.5f);
        float r = d / BALL_R;
        shade_[i] = (uint16_t)((1.0f - 0.38f * r * r * r * r) * 256);
    }
    // Row spans: first column with any coverage (rows are symmetric about the centre).
    for (int y = 0; y < BOX; y++) {
        float dy = y + 0.5f - BOX / 2.0f;
        float half = sqrtf(fmaxf(0, (BALL_R + 0.5f) * (BALL_R + 0.5f) - dy * dy));
        int x0 = (int)floorf(BOX / 2.0f - half);
        span_[y] = (int16_t)(x0 < 0 ? 0 : (half > 0 ? x0 : BOX / 2));
    }
    // Soft specular highlight, upper-left: exp(-(hx^2 + hy^2)) is separable.
    for (int i = 0; i < BOX; i++) {
        float d = i + 0.5f - BOX / 2.0f;
        float hx = (d + 60) / 95, hy = (d + 85) / 55;
        hx_[i] = (uint8_t)(expf(-hx * hx) * 255);
        hy_[i] = (uint8_t)(expf(-hy * hy) * 46);
    }
    // Pre-shaded sky and ground balls.
    for (int y = 0; y < BOX; y++) {
        const int Y = 2 * y + 1 - BOX;
        for (int x = 0; x < BOX; x++) {
            const int X = 2 * x + 1 - BOX;
            int ri = (X * X + Y * Y) >> 4;
            if (ri >= RAD_LEN) ri = RAD_LEN - 1;
            const uint16_t bg = bg_.px[(BOX_Y + y) * W + BOX_X + x];
            const uint32_t sh = shadeAtImpl(shade_, x, y, ri), h = (hx_[x] * hy_[y]) >> 8;
            uint16_t sky = unspread(shadePx(spread(C_SKY), sh, h)), gnd = unspread(shadePx(spread(C_GROUND), sh, h));
            const uint8_t m = edge_[ri];
            if (m != 255) {
                sky = lerp565(bg, sky, (m + 4) >> 3);
                gnd = lerp565(bg, gnd, (m + 4) >> 3);
            }
            sky_img_[y * BOX + x] = sky;
            gnd_img_[y * BOX + x] = gnd;
        }
    }
}

Gauge::Overlay Gauge::buildOverlay(int x, int y, int w, int h,
                                   void (*draw)(Canvas &, int, int, bool, const void *), const void *ctx) {
    Overlay o;
    Canvas col{(uint16_t *)alloc_(w * h * 2), w, h}, msk{(uint16_t *)alloc_(w * h * 2), w, h};
    if (!col.px || !msk.px) return o;
    col.clear(C_BLACK);
    msk.clear(C_BLACK);
    draw(col, x, y, false, ctx);
    draw(msk, x, y, true, ctx);
    int n = 0;
    for (int i = 0; i < w * h; i++) n += msk.px[i] != 0;
    o.px = (OverlayPx *)alloc_(n * sizeof(OverlayPx) + 1);
    if (!o.px) return o;
    for (int j = 0; j < h; j++)
        for (int i = 0; i < w; i++) {
            uint16_t m = msk.px[j * w + i];
            if (!m) continue;
            o.px[o.n++] = {(uint16_t)(x + i), (uint16_t)(y + j), col.px[j * w + i], (uint8_t)(((m >> 5) & 63) * 255 / 63)};
        }
    // scratch canvases are leaked deliberately: built once at boot, a few KB in PSRAM
    return o;
}

// RGB565 <-> "spread" 32-bit form (0x07E0F81F) so all channels can be scaled with one multiply.
static inline __attribute__((always_inline)) uint32_t spread(uint16_t c) { return (c | ((uint32_t)c << 16)) & 0x07E0F81Fu; }
static inline __attribute__((always_inline)) uint16_t unspread(uint32_t s) { return (uint16_t)((s & 0xF81Fu) | ((s >> 16) & 0x07E0u)); }
static inline __attribute__((always_inline)) uint16_t lerp565(uint16_t a, uint16_t b, uint32_t w5) {  // w5: 0..32
    uint32_t sa = spread(a), sb = spread(b);
    return unspread(((sa * (32 - w5) + sb * w5) >> 5) & 0x07E0F81Fu);
}

// d*(1-a) + premul, per channel with saturation
static inline __attribute__((always_inline)) uint16_t over(uint16_t d, uint16_t premul, uint8_t a) {
    uint32_t inv = (255 - a) >> 3;  // 0..31
    uint32_t r = (((spread(d) * inv) >> 5) & 0x07E0F81Fu) + spread(premul);
    if (r & 0x08000000u) r |= 0x07E00000u;
    if (r & 0x00010000u) r |= 0x0000F800u;
    if (r & 0x00000020u) r |= 0x0000001Fu;
    return unspread(r & 0x07E0F81Fu);
}

void Gauge::applyOverlay(uint16_t *fb, const Overlay &o) {
    for (int i = 0; i < o.n; i++) {
        const OverlayPx &p = o.px[i];
        uint16_t &d = fb[p.y * W + p.x];
        d = over(d, p.premul, p.a);
    }
}

// Bilinear sample of a layer at 16.16 texel coordinates; false where fully transparent.
static inline __attribute__((always_inline)) bool sampleLayer(const uint16_t *col, const uint8_t *alpha, int w, int h, int32_t u, int32_t v,
                               uint16_t &premul, uint8_t &a) {
    int iu = u >> 16, iv = v >> 16;
    if (iu < 0 || iv < 0 || iu >= w - 1 || iv >= h - 1) return false;
    const int i = iv * w + iu;
    const uint8_t *ap = alpha + i;
    uint32_t a00 = ap[0], a01 = ap[1], a10 = ap[w], a11 = ap[w + 1];
    if (!(a00 | a01 | a10 | a11)) return false;
    const uint16_t *cp0 = col + i;
    if ((a00 & a01 & a10 & a11) == 255 && cp0[0] == cp0[1] && cp0[0] == cp0[w] && cp0[0] == cp0[w + 1]) {
        premul = cp0[0];  // solid interior: no filtering needed
        a = 255;
        return true;
    }
    uint32_t fu = (u >> 11) & 31, fv = (v >> 11) & 31;
    uint32_t top = a00 * (32 - fu) + a01 * fu, bot = a10 * (32 - fu) + a11 * fu;
    a = (uint8_t)((top * (32 - fv) + bot * fv) >> 10);
    if (!a) return false;
    const uint16_t *cp = col + i;
    premul = lerp565(lerp565(cp[0], cp[1], fu), lerp565(cp[w], cp[w + 1], fu), fv);
    return true;
}

DRAM_ATTR_COMPAT static const uint8_t bayer8[4][4] = {{0, 4, 1, 5}, {6, 2, 7, 3}, {1, 5, 0, 4}, {7, 3, 6, 2}};

Gauge::Frame Gauge::prepare(float roll_deg, float pitch_deg) const {
    if (!isfinite(roll_deg)) roll_deg = 0;
    if (!isfinite(pitch_deg)) pitch_deg = 0;
    if (roll_deg > 90) roll_deg = 90;
    if (roll_deg < -90) roll_deg = -90;
    if (pitch_deg > MAX_DEG) pitch_deg = MAX_DEG;
    if (pitch_deg < -MAX_DEG) pitch_deg = -MAX_DEG;
    // Screen (dx, dy) -> ball (u, v): undo the visual CCW roll. Pitch only moves the needle,
    // which sits on the tape at v = -pitch (nose up -> towards the black numbers).
    Frame f;
    const float a = roll_deg * (float)M_PI / 180.0f;
    f.cs = cosf(a);
    f.sn = sinf(a);
    f.p = pitch_deg * PX_PER_DEG;
    f.du = (int32_t)(f.cs * 65536);
    f.dv = (int32_t)(f.sn * 65536);
    f.needle_v = (int32_t)((NEEDLE_CY - 0.5f + f.p) * 65536);
    return f;
}

// Shading for one pixel: ball darkening (dithered) + specular highlight, on a spread colour.
static inline __attribute__((always_inline)) uint32_t shadePx(uint32_t sc, uint32_t sh, uint32_t h) {
    uint32_t s = ((sc * sh) >> 5) & 0x07E0F81Fu;
    if (h) {
        s += (h >> 3) << 11 | (h >> 2) << 21 | (h >> 3);  // spread(grey h)
        if (s & 0x08000000u) s |= 0x07E00000u;
        if (s & 0x00010000u) s |= 0x0000F800u;
        if (s & 0x00000020u) s |= 0x0000001Fu;
        s &= 0x07E0F81Fu;
    }
    return s;
}

// Dithered ball darkening at bbox pixel (x, y), 0..32.
static inline __attribute__((always_inline)) uint32_t shadeAtImpl(const uint16_t *shade, int x, int y, int ri) {
    uint32_t sh = (shade[ri] + bayer8[y & 3][x & 3]) >> 3;
    return sh > 32 ? 32 : sh;
}

// Scale a spread colour by w5/32.
static inline __attribute__((always_inline)) uint32_t scale5(uint32_t s, uint32_t w5) {
    return ((s * w5) >> 5) & 0x07E0F81Fu;
}

GAUGE_HOT void Gauge::drawRows(uint16_t *fb, const Frame &f, int first, int step) const {
    const int32_t FP = 1 << 16;
    const int32_t du = f.du, dv = f.dv;
    const int32_t face_off = (int32_t)FACE_HALF << 13;  // FACE_SUB = 8 -> >> 13
    const int32_t scale_u = (int32_t)((SCALE_W / 2.0f - 0.5f) * FP), scale_v = (int32_t)((SCALE_H / 2.0f - 0.5f) * FP);
    const int32_t needle_u = (int32_t)((NEEDLE_CX - 0.5f) * FP), needle_v = f.needle_v;
    // Horizon band (orange line + underline + AA) in ball v; outside it the face is plain sky/ground.
    const float band_lo = -5.0f, band_hi = 6.0f;
    // Ball-space half extents of the scale image (the needle always lies inside it).
    const float ah = SCALE_W / 2.0f + 1, bh = SCALE_H / 2.0f + 1;

    for (int by = first; by < BOX; by += step) {
        const int x0 = span_[by], x1 = BOX - x0;
        if (x0 >= x1) continue;
        const float dy = by + 0.5f - BOX / 2.0f;
        const float dx0 = x0 + 0.5f - BOX / 2.0f;
        const float u0 = dx0 * f.cs - dy * f.sn, v0 = dx0 * f.sn + dy * f.cs;
        const int32_t u0f = (int32_t)(u0 * FP), v0f = (int32_t)(v0 * FP);
        uint16_t *out = fb + (BOX_Y + by) * W + BOX_X;
        const uint16_t *bgrow = bg_.px + (BOX_Y + by) * W + BOX_X;
        const uint16_t *sky = sky_img_ + by * BOX, *gnd = gnd_img_ + by * BOX;
        const int Y = 2 * by + 1 - BOX;
        const uint32_t hyr = hy_[by];

        // Pixel range [a, b) where c0 + k * (x - x0) lies within [lo, hi].
        auto slab = [&](float c0, float k, float lo, float hi, int &a, int &b) {
            float t0 = 0, t1 = (float)(x1 - x0);
            if (fabsf(k) < 1e-6f) {
                if (c0 < lo || c0 > hi) t1 = -1;
            } else {
                float ta = (lo - c0) / k, tb = (hi - c0) / k;
                if (ta > tb) { float t = ta; ta = tb; tb = t; }
                if (ta > t0) t0 = ta;
                if (tb < t1) t1 = tb;
            }
            a = b = x1;
            if (t1 >= t0) {
                a = x0 + (int)floorf(t0);
                b = x0 + (int)ceilf(t1) + 1;
                if (a < x0) a = x0;
                if (b > x1) b = x1;
            }
        };

        // 1) Face: copy pre-shaded sky/ground runs, compute the horizon band per pixel.
        int ba, bb;
        slab(v0, f.sn, band_lo, band_hi, ba, bb);
        if (ba >= bb) {
            const float vm = v0 + f.sn * (x1 - x0) * 0.5f;
            memcpy(out + x0, (vm < 0 ? sky : gnd) + x0, (x1 - x0) * 2);
        } else {
            // v increases with x when sn > 0: sky on the left of the band, ground on the right.
            const bool sky_left = f.sn > 0 || (fabsf(f.sn) < 1e-6f && v0 < 0);
            if (ba > x0) memcpy(out + x0, (sky_left ? sky : gnd) + x0, (ba - x0) * 2);
            if (x1 > bb) memcpy(out + bb, (sky_left ? gnd : sky) + bb, (x1 - bb) * 2);
            int32_t v = v0f + dv * (ba - x0) + face_off;
            for (int x = ba; x < bb; x++, v += dv) {
                int fi = v >> 13;
                fi = fi < 0 ? 0 : (fi >= FACE_LEN ? FACE_LEN - 1 : fi);
                const int X = 2 * x + 1 - BOX;
                int ri = (X * X + Y * Y) >> 4;
                if (ri >= RAD_LEN) ri = RAD_LEN - 1;
                uint16_t c = unspread(shadePx(face_[fi], shadeAtImpl(shade_, x, by, ri), (hx_[x] * hyr) >> 8));
                const uint8_t m = edge_[ri];
                if (m != 255) c = lerp565(bgrow[x], c, (m + 4) >> 3);
                out[x] = c;
            }
        }

        // 2) Pitch scale and needle, composited (shaded) only where they are opaque.
        auto composite = [&](int x, uint16_t lc, uint8_t a) {
            const int X = 2 * x + 1 - BOX;
            int ri = (X * X + Y * Y) >> 4;
            if (ri >= RAD_LEN) ri = RAD_LEN - 1;
            // shade the layer like the ball surface; fade with rim coverage
            uint32_t s = shadePx(spread(lc), shadeAtImpl(shade_, x, by, ri), (((hx_[x] * hyr) >> 8) * a) >> 8);
            const uint8_t m = edge_[ri];
            if (m != 255) {
                s = scale5(s, (m + 4) >> 3);
                a = (uint8_t)((a * m) >> 8);
            }
            out[x] = over(out[x], unspread(s), a);
        };
        int la, lb, va, vb;
        slab(u0, f.cs, -ah, ah, la, lb);
        slab(v0, f.sn, -bh, bh, va, vb);
        if (va > la) la = va;
        if (vb < lb) lb = vb;
        {
            const uint8_t *occ = scale_occ_;
            int32_t u = u0f + du * (la - x0) + scale_u, v = v0f + dv * (la - x0) + scale_v;
            for (int x = la; x < lb; x++, u += du, v += dv) {
                const uint32_t iu = (uint32_t)(u >> 16), iv = (uint32_t)(v >> 16);
                if (iu >= SCALE_W - 1 || iv >= SCALE_H - 1 || !occ[(iv >> 2) * OCC_W + (iu >> 2)]) continue;
                uint16_t lc;
                uint8_t a;
                if (sampleLayer(scale_.col, scale_.a, SCALE_W, SCALE_H, u, v, lc, a)) composite(x, lc, a);
            }
        }
        // needle: its own (small) interval
        const float nv0 = v0 + f.p;
        slab(u0, f.cs, -NEEDLE_W / 2.0f - 1, NEEDLE_W / 2.0f + 1, la, lb);
        slab(nv0, f.sn, -NEEDLE_CY - 1, NEEDLE_H - NEEDLE_CY + 1, va, vb);
        if (va > la) la = va;
        if (vb < lb) lb = vb;
        {
            int32_t u = u0f + du * (la - x0) + needle_u, v = v0f + dv * (la - x0) + needle_v;
            for (int x = la; x < lb; x++, u += du, v += dv) {
                uint16_t lc;
                uint8_t a;
                if (sampleLayer(needle_.col, needle_.a, NEEDLE_W, NEEDLE_H, u, v, lc, a)) composite(x, lc, a);
            }
        }
    }
}

void Gauge::drawOverlays(uint16_t *fb, GaugeMessage msg) const {
    if (msg != GaugeMessage::None) applyOverlay(fb, msgs_[(int)msg]);
}

void Gauge::drawBall(uint16_t *fb, float roll_deg, float pitch_deg, GaugeMessage msg) const {
    Frame f = prepare(roll_deg, pitch_deg);
    drawRows(fb, f, 0, 1);
    drawOverlays(fb, msg);
}
