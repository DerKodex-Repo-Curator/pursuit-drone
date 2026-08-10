#include "madgwick.h"

#include <cmath>

namespace {

constexpr float DEG2RAD = 0.017453292519943295f;
constexpr float RAD2DEG = 57.29577951308232f;

/* How long after a reset the filter runs at high gain before settling to the
 * flight value. Long enough to converge from an arbitrary attitude, short
 * enough that it is over before anyone can arm. */
constexpr float SETTLE_TIME_S = 2.0f;
constexpr float SETTLE_BETA = 2.0f;

float inv_sqrt(float x)
{
    /* The famous bit-twiddling version is a false economy on an ESP32-S3,
     * which has a single-precision FPU: sqrtf is a hardware instruction and
     * the approximation costs accuracy for nothing. */
    return x > 0.0f ? 1.0f / sqrtf(x) : 0.0f;
}

}  /* namespace */

Madgwick::Madgwick(float beta)
    : q0_(1.0f), q1_(0.0f), q2_(0.0f), q3_(0.0f),
      beta_(SETTLE_BETA), flight_beta_(beta),
      settle_s_(0.0f), converged_(false)
{
}

void Madgwick::reset()
{
    q0_ = 1.0f; q1_ = q2_ = q3_ = 0.0f;
    beta_ = SETTLE_BETA;
    settle_s_ = 0.0f;
    converged_ = false;
}

void Madgwick::quaternion(float q[4]) const
{
    q[0] = q0_; q[1] = q1_; q[2] = q2_; q[3] = q3_;
}

void Madgwick::align_to_accel(float ax, float ay, float az)
{
    const float n = sqrtf(ax * ax + ay * ay + az * az);
    if (n < 1e-6f) return;
    ax /= n; ay /= n; az /= n;

    /* Convention, stated explicitly because getting it backwards produces an
     * estimate that is exactly wrong rather than obviously broken:
     *
     * An accelerometer measures SPECIFIC FORCE, not gravity. Sitting level
     * and upright it reads +1g on +Z, because the table pushes up. Rotating
     * the body by roll phi puts +sin(phi) on Y; by pitch theta puts
     * -sin(theta) on X.
     *
     * Roll and pitch are observable from that vector. Yaw is not — rotating
     * about the gravity vector does not change it — so yaw starts at zero and
     * is only ever meaningful as a rate. */
    const float roll = atan2f(ay, az);
    const float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    const float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);

    q0_ = cr * cp;
    q1_ = sr * cp;
    q2_ = cr * sp;
    q3_ = -sr * sp;

    const float inv = inv_sqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
    q0_ *= inv; q1_ *= inv; q2_ *= inv; q3_ *= inv;
}

void Madgwick::update(float gx, float gy, float gz,
                      float ax, float ay, float az, float dt_s)
{
    if (!(dt_s > 0.0f) || dt_s > 0.5f) return;   /* also rejects NaN */
    if (!std::isfinite(gx) || !std::isfinite(gy) || !std::isfinite(gz)) return;

    gx *= DEG2RAD; gy *= DEG2RAD; gz *= DEG2RAD;

    /* Quaternion derivative from the gyro alone. */
    float qDot0 = 0.5f * (-q1_ * gx - q2_ * gy - q3_ * gz);
    float qDot1 = 0.5f * ( q0_ * gx + q2_ * gz - q3_ * gy);
    float qDot2 = 0.5f * ( q0_ * gy - q1_ * gz + q3_ * gx);
    float qDot3 = 0.5f * ( q0_ * gz + q1_ * gy - q2_ * gx);

    const float anorm = sqrtf(ax * ax + ay * ay + az * az);
    /* Only correct when the accelerometer is plausibly measuring gravity.
     * During a hard manoeuvre it measures thrust as well, and feeding that in
     * tips the estimated horizon toward the acceleration — the classic
     * "leans into every turn" failure. Skipping the correction lets the gyro
     * carry those moments, which is exactly what it is good at. */
    if (anorm > 1e-6f && std::isfinite(anorm)) {
        const float inv = 1.0f / anorm;
        ax *= inv; ay *= inv; az *= inv;

        /* Gradient of the objective f(q) = (estimated gravity) - (measured).
         * Written out flat because the compiler cannot factor this as well as
         * Madgwick's own algebra does. */
        const float _2q0 = 2.0f * q0_, _2q1 = 2.0f * q1_;
        const float _2q2 = 2.0f * q2_, _2q3 = 2.0f * q3_;
        const float _4q0 = 4.0f * q0_, _4q1 = 4.0f * q1_, _4q2 = 4.0f * q2_;
        const float _8q1 = 8.0f * q1_, _8q2 = 8.0f * q2_;
        const float q0q0 = q0_ * q0_, q1q1 = q1_ * q1_;
        const float q2q2 = q2_ * q2_, q3q3 = q3_ * q3_;

        float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1_ - _2q0 * ay
                 - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        float s2 = 4.0f * q0q0 * q2_ + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay
                 - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        float s3 = 4.0f * q1q1 * q3_ - _2q1 * ax + 4.0f * q2q2 * q3_
                 - _2q2 * ay;

        const float sn = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        if (sn > 0.0f) {
            s0 *= sn; s1 *= sn; s2 *= sn; s3 *= sn;
            qDot0 -= beta_ * s0;
            qDot1 -= beta_ * s1;
            qDot2 -= beta_ * s2;
            qDot3 -= beta_ * s3;
        }
    }

    q0_ += qDot0 * dt_s;
    q1_ += qDot1 * dt_s;
    q2_ += qDot2 * dt_s;
    q3_ += qDot3 * dt_s;

    const float qn = inv_sqrt(q0_*q0_ + q1_*q1_ + q2_*q2_ + q3_*q3_);
    if (qn > 0.0f) {
        q0_ *= qn; q1_ *= qn; q2_ *= qn; q3_ *= qn;
    } else {
        /* Degenerate quaternion — should be unreachable, but a zero-norm
         * attitude would produce NaN Euler angles and drive the mixer with
         * them. Recover rather than propagate. */
        q0_ = 1.0f; q1_ = q2_ = q3_ = 0.0f;
    }

    /* Decay from the high startup gain to the flight value. */
    if (!converged_) {
        settle_s_ += dt_s;
        if (settle_s_ >= SETTLE_TIME_S) {
            beta_ = flight_beta_;
            converged_ = true;
        } else {
            const float t = settle_s_ / SETTLE_TIME_S;
            beta_ = SETTLE_BETA + (flight_beta_ - SETTLE_BETA) * t;
        }
    }
}

EulerAngles Madgwick::euler() const
{
    EulerAngles e;
    e.roll = atan2f(2.0f * (q0_ * q1_ + q2_ * q3_),
                    1.0f - 2.0f * (q1_ * q1_ + q2_ * q2_)) * RAD2DEG;

    /* asin domain-clamped: accumulated rounding can push the argument a hair
     * past 1.0 at 90 degrees of pitch, and asinf(1.0000001) is NaN — which
     * would then reach the mixer. */
    float sp = 2.0f * (q0_ * q2_ - q3_ * q1_);
    sp = sp > 1.0f ? 1.0f : (sp < -1.0f ? -1.0f : sp);
    e.pitch = asinf(sp) * RAD2DEG;

    e.yaw = atan2f(2.0f * (q0_ * q3_ + q1_ * q2_),
                   1.0f - 2.0f * (q2_ * q2_ + q3_ * q3_)) * RAD2DEG;
    return e;
}
