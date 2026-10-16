#include "canvas.h"

#include <math.h>
#include <string.h>

static inline float clamp01(float v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }

void Canvas::clear(uint16_t c) {
    for (int i = 0; i < w * h; i++) px[i] = c;
}

void Canvas::blend(int x, int y, uint16_t c, float a) {
    if (x < 0 || y < 0 || x >= w || y >= h || a <= 0) return;
    uint16_t &d = px[y * w + x];
    if (a >= 1) { d = c; return; }
    float r = ((c >> 11) & 31) * a + ((d >> 11) & 31) * (1 - a);
    float g = ((c >> 5) & 63) * a + ((d >> 5) & 63) * (1 - a);
    float b = (c & 31) * a + (d & 31) * (1 - a);
    d = (uint16_t)(((int)(r + 0.5f) << 11) | ((int)(g + 0.5f) << 5) | (int)(b + 0.5f));
}

void Canvas::fillRect(int x, int y, int rw, int rh, uint16_t c) {
    for (int j = y; j < y + rh; j++)
        for (int i = x; i < x + rw; i++)
            if (i >= 0 && j >= 0 && i < w && j < h) px[j * w + i] = c;
}

void Canvas::fillAnnulus(float cx, float cy, float r0, float r1, uint16_t c, int clip, float clip_y) {
    int y0 = (int)floorf(cy - r1 - 1), y1 = (int)ceilf(cy + r1 + 1);
    int x0 = (int)floorf(cx - r1 - 1), x1 = (int)ceilf(cx + r1 + 1);
    if (clip < 0 && y1 > clip_y) y1 = (int)ceilf(clip_y) - 1;
    if (clip > 0 && y0 < clip_y) y0 = (int)floorf(clip_y);
    for (int y = y0; y <= y1; y++) {
        float py = y + 0.5f;
        if (clip < 0 && py > clip_y) continue;
        if (clip > 0 && py < clip_y) continue;
        for (int x = x0; x <= x1; x++) {
            float dx = x + 0.5f - cx, dy = py - cy;
            float d = sqrtf(dx * dx + dy * dy);
            float a = clamp01(r1 - d + 0.5f);
            if (r0 >= 0) a = fminf(a, clamp01(d - r0 + 0.5f));
            blend(x, y, c, a);
        }
    }
}

static void segment(Canvas &cv, float x0, float y0, float x1, float y1, float width, uint16_t c, bool round_caps) {
    float hw = width * 0.5f;
    int bx0 = (int)floorf(fminf(x0, x1) - hw - 1), bx1 = (int)ceilf(fmaxf(x0, x1) + hw + 1);
    int by0 = (int)floorf(fminf(y0, y1) - hw - 1), by1 = (int)ceilf(fmaxf(y0, y1) + hw + 1);
    float vx = x1 - x0, vy = y1 - y0;
    float len = sqrtf(vx * vx + vy * vy);
    if (len < 1e-4f) len = 1e-4f;
    float ux = vx / len, uy = vy / len;
    for (int y = by0; y <= by1; y++) {
        for (int x = bx0; x <= bx1; x++) {
            float px = x + 0.5f - x0, py = y + 0.5f - y0;
            float t = px * ux + py * uy;        // along
            float n = fabsf(-px * uy + py * ux); // across
            float a;
            if (round_caps) {
                float tc = t < 0 ? 0 : (t > len ? len : t);
                float dx = px - ux * tc, dy = py - uy * tc;
                a = clamp01(hw - sqrtf(dx * dx + dy * dy) + 0.5f);
            } else {
                a = fminf(clamp01(hw - n + 0.5f), fminf(clamp01(t + 0.5f), clamp01(len - t + 0.5f)));
            }
            cv.blend(x, y, c, a);
        }
    }
}

void Canvas::line(float x0, float y0, float x1, float y1, float width, uint16_t c) {
    segment(*this, x0, y0, x1, y1, width, c, true);
}

void Canvas::bar(float x0, float y0, float x1, float y1, float width, uint16_t c) {
    segment(*this, x0, y0, x1, y1, width, c, false);
}

void Canvas::triangle(float x0, float y0, float x1, float y1, float x2, float y2, uint16_t c) {
    // make counter-clockwise in y-down space so all edge distances are positive inside
    float area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
    if (area < 0) { float t = x1; x1 = x2; x2 = t; t = y1; y1 = y2; y2 = t; }
    const float xs[3] = {x0, x1, x2}, ys[3] = {y0, y1, y2};
    int bx0 = (int)floorf(fminf(x0, fminf(x1, x2))) - 1, bx1 = (int)ceilf(fmaxf(x0, fmaxf(x1, x2))) + 1;
    int by0 = (int)floorf(fminf(y0, fminf(y1, y2))) - 1, by1 = (int)ceilf(fmaxf(y0, fmaxf(y1, y2))) + 1;
    for (int y = by0; y <= by1; y++) {
        for (int x = bx0; x <= bx1; x++) {
            float a = 1;
            for (int e = 0; e < 3; e++) {
                float ax = xs[e], ay = ys[e], bx = xs[(e + 1) % 3], by = ys[(e + 1) % 3];
                float ex = bx - ax, ey = by - ay, l = sqrtf(ex * ex + ey * ey);
                float d = ((x + 0.5f - ax) * ey - (y + 0.5f - ay) * ex) / l;
                a = fminf(a, clamp01(-d + 0.5f));
            }
            blend(x, y, c, a);
        }
    }
}

void Canvas::roundRect(float x, float y, float rw, float rh, float r, uint16_t c, float alpha) {
    for (int j = (int)floorf(y); j <= (int)ceilf(y + rh); j++) {
        for (int i = (int)floorf(x); i <= (int)ceilf(x + rw); i++) {
            float px = i + 0.5f, py = j + 0.5f;
            float qx = fmaxf(fabsf(px - (x + rw / 2)) - (rw / 2 - r), 0);
            float qy = fmaxf(fabsf(py - (y + rh / 2)) - (rh / 2 - r), 0);
            float d = sqrtf(qx * qx + qy * qy) - r;
            blend(i, j, c, clamp01(0.5f - d) * alpha);
        }
    }
}

static const FontGlyph *glyph(const Font &f, char ch) {
    const char *p = strchr(f.chars, ch);
    return p ? &f.glyphs[p - f.chars] : nullptr;
}

int Canvas::textWidth(const Font &f, const char *s) const {
    int wsum = 0;
    for (; *s; s++)
        if (const FontGlyph *g = glyph(f, *s)) wsum += g->adv;
    return wsum;
}

void Canvas::text(const Font &f, const char *s, float x, float y, uint16_t c, Align a) {
    int tw = textWidth(f, s);
    int pen = (int)lroundf(a == Align::Left ? x : (a == Align::Center ? x - tw / 2.0f : x - tw));
    int base = (int)lroundf(y + f.cap_height / 2.0f);
    for (; *s; s++) {
        const FontGlyph *g = glyph(f, *s);
        if (!g) continue;
        for (int j = 0; j < g->h; j++)
            for (int i = 0; i < g->w; i++)
                blend(pen + g->xoff + i, base + g->yoff + j, c, f.bitmap[g->offset + j * g->w + i] / 255.0f);
        pen += g->adv;
    }
}
