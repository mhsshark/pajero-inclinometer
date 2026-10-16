// Pajero Gen 1 style inclinometer for the Waveshare ESP32-S3-Touch-LCD-2.8C.
//
// Core 0: IMU at 200 Hz -> complementary filter -> vehicle roll/pitch.
// Core 1 (loop): button, damping, rendering at panel refresh rate.
#include <Arduino.h>
#include <Wire.h>
#include <esp_heap_caps.h>

#include "attitude.h"
#include "board_pins.h"
#include "button.h"
#include "config.h"
#include "display.h"
#include "expander.h"
#include "gauge.h"
#include "imu.h"

static Gauge gauge;
static Attitude attitude;
static Button button(PIN_BUTTON);

// ---- shared between the IMU task and the render loop ----
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static float s_roll = 0, s_pitch = 0;
static Vec3 s_acc{0, 0, 0};
static volatile bool s_imu_ok = false;
enum CalState { CAL_IDLE, CAL_REQUESTED, CAL_RUNNING, CAL_OK, CAL_FAILED, CAL_CLEAR };
static volatile CalState s_cal = CAL_IDLE;

static void *psramAlloc(size_t n) { return heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT); }
static void *fastAlloc(size_t n) {  // internal RAM, PSRAM as a fallback
    void *p = heap_caps_malloc(n, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    return p ? p : psramAlloc(n);
}

static void imuTask(void *) {
    imu::Sample raw;
    uint32_t last_us = micros(), cal_until = 0;
    TickType_t wake = xTaskGetTickCount();

    // Boot-time gyro bias (only used if the vehicle is still while powering up).
    attitude.calStart();
    for (int i = 0; i < 100; i++) {
        if (imu::read(raw)) {
            attitude.calAdd({IMU_MAP_X(raw.ax, raw.ay, raw.az), IMU_MAP_Y(raw.ax, raw.ay, raw.az), IMU_MAP_Z(raw.ax, raw.ay, raw.az)},
                            {IMU_MAP_X(raw.gx, raw.gy, raw.gz), IMU_MAP_Y(raw.gx, raw.gy, raw.gz), IMU_MAP_Z(raw.gx, raw.gy, raw.gz)});
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(5));
    }
    attitude.calGyroBiasOnly();

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(5));
        if (!imu::read(raw)) {
            s_imu_ok = false;
            continue;
        }
        s_imu_ok = true;
        uint32_t now = micros();
        float dt = (now - last_us) * 1e-6f;
        last_us = now;
        if (dt > 0.05f) dt = 0.05f;

        Vec3 a{IMU_MAP_X(raw.ax, raw.ay, raw.az), IMU_MAP_Y(raw.ax, raw.ay, raw.az), IMU_MAP_Z(raw.ax, raw.ay, raw.az)};
        Vec3 g{IMU_MAP_X(raw.gx, raw.gy, raw.gz), IMU_MAP_Y(raw.gx, raw.gy, raw.gz), IMU_MAP_Z(raw.gx, raw.gy, raw.gz)};

        if (s_cal == CAL_CLEAR) {
            attitude.calClear();
            s_cal = CAL_IDLE;
        }
        if (s_cal == CAL_REQUESTED) {
            attitude.calStart();
            cal_until = millis() + CAL_SAMPLE_MS;
            s_cal = CAL_RUNNING;
        }
        if (s_cal == CAL_RUNNING) {
            attitude.calAdd(a, g);
            if ((int32_t)(millis() - cal_until) >= 0) s_cal = attitude.calCommit() ? CAL_OK : CAL_FAILED;
        }

        attitude.update(a, g, dt);
        portENTER_CRITICAL(&mux);
        s_roll = attitude.roll();
        s_pitch = attitude.pitch();
        s_acc = a;
        portEXIT_CRITICAL(&mux);
    }
}

static uint32_t beep_until = 0;
static void beep(uint32_t ms) {
#if BUZZER_ENABLED
    expander::write(EXIO_BUZZER, true);
    beep_until = millis() + ms;
#endif
}

// ---- second render core: draws every other row on core 0 ----
static TaskHandle_t render_worker = nullptr, render_main = nullptr;
static uint16_t *job_fb = nullptr;
static Gauge::Frame job_frame;

static void renderWorker(void *) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        gauge.drawRows(job_fb, job_frame, 0, 2);
        xTaskNotifyGive(render_main);
    }
}

static void renderFrame(uint16_t *fb, float roll, float pitch, GaugeMessage msg) {
    job_fb = fb;
    job_frame = gauge.prepare(roll, pitch);
    xTaskNotifyGive(render_worker);
    gauge.drawRows(fb, job_frame, 1, 2);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    gauge.drawOverlays(fb, msg);
}

void setup() {
    Serial.begin(115200);
    // Never let logging stall the gauge: with no host reading the USB port, drop output instead of blocking.
    Serial.setTxTimeoutMs(0);
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ);
    button.begin();

    if (!expander::begin()) Serial.println("TCA9554 expander not found");
    if (!display::begin()) Serial.println("display init failed");
    if (!gauge.begin(psramAlloc, fastAlloc)) Serial.println("out of memory for gauge");
    Serial.printf("free internal RAM: %u bytes\n", (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

    display::fillAll(gauge.background());
    gauge.drawBall(display::backBuffer(), 0, 0, GaugeMessage::None);
    display::present();
    gauge.drawBall(display::backBuffer(), 0, 0, GaugeMessage::None);
    display::present();
    gauge.drawBall(display::backBuffer(), 0, 0, GaugeMessage::None);
    display::present();
    for (int b = 0; b <= BACKLIGHT_PERCENT; b += 3) {
        display::setBacklight(b);
        delay(8);
    }

    render_main = xTaskGetCurrentTaskHandle();
    xTaskCreatePinnedToCore(renderWorker, "render", 4096, nullptr, 4, &render_worker, 0);

    attitude.begin();
    if (imu::begin()) {
        Serial.printf("QMI8658 at 0x%02X\n", imu::address());
        xTaskCreatePinnedToCore(imuTask, "imu", 4096, nullptr, 5, nullptr, 0);
    } else {
        Serial.println("QMI8658 not found");
    }
}

void loop() {
    static Spring roll_s, pitch_s;
    static uint32_t last_ms = millis(), msg_until = 0, frames = 0, fps_t0 = millis(), dbg_t = 0;
    static uint32_t draw_us = 0;
    static GaugeMessage timed_msg = GaugeMessage::None;
    static float drawn[3][2] = {{1e9f, 1e9f}, {1e9f, 1e9f}, {1e9f, 1e9f}};
    static GaugeMessage drawn_msg[3] = {GaugeMessage::Count, GaugeMessage::Count, GaugeMessage::Count};
    static int slot = 0;

    uint32_t now = millis();
    float dt = (now - last_ms) * 1e-3f;
    last_ms = now;
    if (dt > 0.1f) dt = 0.1f;

    if (beep_until && (int32_t)(now - beep_until) >= 0) {
        expander::write(EXIO_BUZZER, false);
        beep_until = 0;
    }

    // ---- set-level button ----
    button.update(now);
    if (button.longPress(now, LONG_PRESS_MS) && s_imu_ok && s_cal != CAL_RUNNING) {
        s_cal = CAL_REQUESTED;
        beep(40);
    }
    // ---- serial commands: z = set level (zero), c = clear level reference, d = toggle log ----
    static bool debug_log = SERIAL_DEBUG;
    while (Serial.available()) {
        int ch = Serial.read();
        if ((ch == 'z' || ch == 'Z') && s_imu_ok && s_cal == CAL_IDLE) {
            Serial.println("setting level: keep still for 1 s");
            s_cal = CAL_REQUESTED;
            beep(40);
        } else if (ch == 'c' || ch == 'C') {
            Serial.println("level reference cleared");
            s_cal = CAL_CLEAR;
        } else if (ch == 'd' || ch == 'D') {
            debug_log = !debug_log;
            Serial.println(debug_log ? "log on" : "log off");
        } else if (ch == '?' || ch == 'h') {
            Serial.println("commands: z = set level (zero), c = clear level reference, d = toggle log");
        }
    }
    if (s_cal == CAL_OK || s_cal == CAL_FAILED) {
        bool ok = s_cal == CAL_OK;
        timed_msg = ok ? GaugeMessage::LevelSet : GaugeMessage::TryAgain;
        msg_until = now + 1500;
        s_cal = CAL_IDLE;
        beep(ok ? 150 : 600);
        Serial.println(ok ? "level set" : "level not set: keep the vehicle still");
    }

    GaugeMessage msg = GaugeMessage::None;
    if (imu::address() == 0 || !s_imu_ok) msg = GaugeMessage::ImuError;
    else if (s_cal == CAL_REQUESTED || s_cal == CAL_RUNNING) msg = GaugeMessage::HoldStill;
    else if (button.held() && button.heldFor(now) > 300 && button.heldFor(now) < LONG_PRESS_MS) msg = GaugeMessage::HoldButton;
    else if (msg_until && (int32_t)(now - msg_until) < 0) msg = timed_msg;

    // ---- angles ----
    float roll, pitch;
    Vec3 acc;
    portENTER_CRITICAL(&mux);
    roll = s_roll;
    pitch = s_pitch;
    acc = s_acc;
    portEXIT_CRITICAL(&mux);
    roll_s.step(roll, NEEDLE_OMEGA, dt);
    pitch_s.step(pitch, NEEDLE_OMEGA, dt);

    // ---- render (skip when all three buffers already hold this state) ----
    float r = roundf(roll_s.pos * 20) / 20, p = roundf(pitch_s.pos * 20) / 20;
    bool same = true;
    for (int i = 0; i < 3; i++) same &= drawn[i][0] == r && drawn[i][1] == p && drawn_msg[i] == msg;
    if (!same) {
        uint32_t t0 = micros();
        renderFrame(display::backBuffer(), r, p, msg);
        uint32_t t1 = micros();
        display::present();
        draw_us = t1 - t0;
        drawn[slot][0] = r;
        drawn[slot][1] = p;
        drawn_msg[slot] = msg;
        slot = (slot + 1) % 3;
        frames++;
    } else {
        delay(5);
    }

    if (debug_log && now - dbg_t >= 200) {
        dbg_t = now;
        float fps = frames * 1000.0f / (now - fps_t0 + 1);
        frames = 0;
        fps_t0 = now;
        static uint32_t last_vs = 0, last_fl = 0;
        uint32_t vs, fl;
        display::counters(vs, fl);
        Serial.printf("roll %+6.1f pitch %+6.1f | shown %+7.1f %+7.1f | render %4.1f fps (draw %lu us) | panel %u Hz, new %u/s\n",
                      roll, pitch, roll_s.pos, pitch_s.pos, fps, (unsigned long)draw_us, (unsigned)((vs - last_vs) * 5),
                      (unsigned)((fl - last_fl) * 5));
        last_vs = vs;
        last_fl = fl;
    }
}
