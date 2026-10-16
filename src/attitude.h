// Gravity estimate (complementary filter), mounting calibration, damped display angles.
#pragma once
#include <math.h>

struct Vec3 {
    float x, y, z;
};

struct Quat {
    float w, x, y, z;
};

class Attitude {
public:
    // Load calibration from flash.
    void begin();

    // One IMU sample in the device frame (see config.h). dt in seconds.
    void update(const Vec3 &acc_g, const Vec3 &gyro_rad, float dt);

    // Level reference: average accel/gyro over a still period, then commit().
    void calStart();
    void calAdd(const Vec3 &acc_g, const Vec3 &gyro_rad);
    bool calCommit();  // false if the board moved too much
    // Boot-time gyro bias only (not saved); false if the board was moving.
    bool calGyroBiasOnly();
    // Forget the stored level reference (back to the raw mounting).
    void calClear();

    // Vehicle angles in degrees (roll > 0: right side down, pitch > 0: nose up).
    float roll() const { return roll_; }
    float pitch() const { return pitch_; }

private:
    Quat mount_{1, 0, 0, 0};  // rotates device frame -> level frame
    Vec3 gyro_bias_{0, 0, 0};
    Vec3 up_{0, 1, 0};        // filtered "up" in the level frame
    bool primed_ = false;
    float roll_ = 0, pitch_ = 0;

    Vec3 cal_acc_{}, cal_gyro_{};
    float cal_acc_sq_ = 0;
    int cal_n_ = 0;
};

// Critically damped spring: gives the needle/ball a fluid-damped feel.
// Uses the exact solution, so it is stable for any time step (a stalled frame can't blow it up).
struct Spring {
    float pos = 0, vel = 0;
    void step(float target, float omega, float dt) {
        if (!isfinite(pos) || !isfinite(vel)) {
            pos = target;
            vel = 0;
            return;
        }
        // x(t) = (x0 + (v0 + w x0) t) e^-wt,  v(t) = (v0 - w (v0 + w x0) t) e^-wt
        const float x0 = pos - target, k = vel + omega * x0, e = expf(-omega * dt);
        pos = target + (x0 + k * dt) * e;
        vel = (vel - omega * k * dt) * e;
    }
};
