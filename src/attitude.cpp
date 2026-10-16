#include "attitude.h"

#include <Preferences.h>
#include <math.h>

#include "config.h"

static Vec3 rotate(const Quat &q, const Vec3 &v) {
    // v' = v + 2w(q x v) + 2 q x (q x v)
    Vec3 u{q.x, q.y, q.z};
    Vec3 t{2 * (u.y * v.z - u.z * v.y), 2 * (u.z * v.x - u.x * v.z), 2 * (u.x * v.y - u.y * v.x)};
    return {v.x + q.w * t.x + (u.y * t.z - u.z * t.y), v.y + q.w * t.y + (u.z * t.x - u.x * t.z),
            v.z + q.w * t.z + (u.x * t.y - u.y * t.x)};
}

static float norm(const Vec3 &v) { return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z); }

static Vec3 scale(const Vec3 &v, float k) { return {v.x * k, v.y * k, v.z * k}; }

void Attitude::begin() {
    Preferences p;
    if (p.begin("incl", true)) {
        if (p.getBytesLength("mount") == sizeof mount_) p.getBytes("mount", &mount_, sizeof mount_);
        if (p.getBytesLength("gbias") == sizeof gyro_bias_) p.getBytes("gbias", &gyro_bias_, sizeof gyro_bias_);
        p.end();
    }
}

void Attitude::update(const Vec3 &acc_dev, const Vec3 &gyro_dev, float dt) {
    Vec3 a = rotate(mount_, acc_dev);
    Vec3 w = rotate(mount_, {gyro_dev.x - gyro_bias_.x, gyro_dev.y - gyro_bias_.y, gyro_dev.z - gyro_bias_.z});
    float an = norm(a);
    if (an < 0.2f) return;  // free fall / bad sample
    a = scale(a, 1 / an);

    if (!primed_) {
        up_ = a;
        primed_ = true;
    } else {
        // A world-fixed vector seen from a body rotating at w: d(up)/dt = -w x up
        Vec3 d{-(w.y * up_.z - w.z * up_.y), -(w.z * up_.x - w.x * up_.z), -(w.x * up_.y - w.y * up_.x)};
        Vec3 g{up_.x + d.x * dt, up_.y + d.y * dt, up_.z + d.z * dt};
        // Trust the accelerometer less while it reads far from 1 g (braking, bumps).
        float k = dt / (FILTER_TAU_S + dt);
        if (fabsf(an - 1) > 0.15f) k *= 0.1f;
        up_ = {g.x + (a.x - g.x) * k, g.y + (a.y - g.y) * k, g.z + (a.z - g.z) * k};
        up_ = scale(up_, 1 / norm(up_));
    }

    // Screen faces the driver: X = right, Y = up, Z = towards the driver (rearwards).
    roll_ = ROLL_SIGN * atan2f(-up_.x, up_.y) * 180.0f / (float)M_PI;
    pitch_ = PITCH_SIGN * atan2f(up_.z, sqrtf(up_.x * up_.x + up_.y * up_.y)) * 180.0f / (float)M_PI;
}

void Attitude::calStart() {
    cal_acc_ = cal_gyro_ = {0, 0, 0};
    cal_acc_sq_ = 0;
    cal_n_ = 0;
}

void Attitude::calAdd(const Vec3 &a, const Vec3 &g) {
    cal_acc_ = {cal_acc_.x + a.x, cal_acc_.y + a.y, cal_acc_.z + a.z};
    cal_gyro_ = {cal_gyro_.x + g.x, cal_gyro_.y + g.y, cal_gyro_.z + g.z};
    cal_acc_sq_ += a.x * a.x + a.y * a.y + a.z * a.z;
    cal_n_++;
}

static bool still(const Vec3 &sum, float sq, int n) {
    if (n < 20) return false;
    Vec3 mean = scale(sum, 1.0f / n);
    float var = sq / n - (mean.x * mean.x + mean.y * mean.y + mean.z * mean.z);
    return var < 0.0025f;  // < ~0.05 g RMS
}

bool Attitude::calGyroBiasOnly() {
    if (!still(cal_acc_, cal_acc_sq_, cal_n_)) return false;
    gyro_bias_ = scale(cal_gyro_, 1.0f / cal_n_);
    return true;
}

void Attitude::calClear() {
    mount_ = {1, 0, 0, 0};
    primed_ = false;
    Preferences p;
    if (p.begin("incl", false)) {
        p.remove("mount");
        p.end();
    }
}

bool Attitude::calCommit() {
    if (!still(cal_acc_, cal_acc_sq_, cal_n_)) return false;
    Vec3 mean = scale(cal_acc_, 1.0f / cal_n_);

    // Shortest rotation taking the measured "up" onto the device's +Y (screen up).
    Vec3 u = scale(mean, 1 / norm(mean));
    Vec3 t{0, 1, 0};
    float dot = u.x * t.x + u.y * t.y + u.z * t.z;
    Vec3 c{u.y * t.z - u.z * t.y, u.z * t.x - u.x * t.z, u.x * t.y - u.y * t.x};
    Quat q{1 + dot, c.x, c.y, c.z};
    float qn = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (qn < 1e-3f) q = {0, 0, 0, 1};  // upside down: 180 deg about Z
    else q = {q.w / qn, q.x / qn, q.y / qn, q.z / qn};

    mount_ = q;
    gyro_bias_ = scale(cal_gyro_, 1.0f / cal_n_);
    primed_ = false;  // snap the filter to the new reference

    Preferences p;
    if (p.begin("incl", false)) {
        p.putBytes("mount", &mount_, sizeof mount_);
        p.putBytes("gbias", &gyro_bias_, sizeof gyro_bias_);
        p.end();
    }
    return true;
}
