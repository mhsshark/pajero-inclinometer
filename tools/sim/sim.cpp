// Host preview of the gauge renderer: writes PPM frames for a few poses.
// Build/run: tools/sim/run.sh
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <chrono>

#include "../../src/gauge.h"

static void writePPM(const char *path, const uint16_t *fb) {
    FILE *f = fopen(path, "wb");
    fprintf(f, "P6\n%d %d\n255\n", Gauge::W, Gauge::H);
    for (int i = 0; i < Gauge::W * Gauge::H; i++) {
        uint16_t c = fb[i];
        uint8_t rgb[3] = {(uint8_t)(((c >> 11) & 31) * 255 / 31), (uint8_t)(((c >> 5) & 63) * 255 / 63),
                          (uint8_t)((c & 31) * 255 / 31)};
        fwrite(rgb, 1, 3, f);
    }
    fclose(f);
}

int main(int argc, char **argv) {
    const char *out = argc > 1 ? argv[1] : ".";
    static Gauge g;
    if (!g.begin(malloc, malloc)) return 1;
    static uint16_t fb[Gauge::W * Gauge::H];
    struct Pose { const char *name; float roll, pitch; GaugeMessage msg; } poses[] = {
        {"level", 0, 0, GaugeMessage::None},
        {"right15", 15, 0, GaugeMessage::None},          // 0120.jpg
        {"noseup20", 0, 20, GaugeMessage::None},          // 0121.jpg
        {"nosedown15_left10", -10, -15, GaugeMessage::None}, // 0122.jpg
        {"extreme", 35, 30, GaugeMessage::None},
        {"levelset", 0, 0, GaugeMessage::LevelSet},
        {"steep_left", -70, -40, GaugeMessage::None},
        {"vertical", 90, 10, GaugeMessage::None},
        {"vertical_neg", -90, 0, GaugeMessage::None},
    };
    for (auto &p : poses) {
        memcpy(fb, g.background(), sizeof fb);
        g.drawBall(fb, p.roll, p.pitch, p.msg);
        char path[256];
        snprintf(path, sizeof path, "%s/%s.ppm", out, p.name);
        writePPM(path, fb);
    }
    auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < 200; i++) g.drawBall(fb, i * 0.3f, i * 0.2f - 20, GaugeMessage::None);
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();
    printf("host drawBall: %.2f ms/frame\n", us / 200.0 / 1000.0);
    return 0;
}
