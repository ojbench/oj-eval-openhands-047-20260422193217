// Implementation of Controller::get_v_next for Problem 047 (ACMOJ 2284)
// This header is included by the OJ's driver which declares Controller, Vec, and Monitor.
// We implement a simple, safe velocity selection: move toward target up to v_max
// and perform a continuous collision check over the next time interval. If a
// potential collision is detected, we try scaled/rotated alternatives; if none
// are safe, we stop (zero velocity) for this step.

#ifndef SRC_HPP_IMPLEMENTATION
#define SRC_HPP_IMPLEMENTATION

#include <cmath>
#include <algorithm>

// Fallback for TIME_INTERVAL if not provided by the driver. According to the
// problem statement, TIME_INTERVAL is 0.1. We avoid redefining if already macro.
#ifndef OH_LOCAL_TAU_DEFINED
#define OH_LOCAL_TAU_DEFINED
static constexpr double OH_TAU = 0.1;
#endif

// Helper: compute minimum squared distance between two discs' centers over [0, tau]
// given relative position r0 and relative velocity vrel.
static inline double oh_min_dist2_over_interval(const Vec &r0, const Vec &vrel, double tau) {
    double v2 = vrel * vrel; // dot product
    if (v2 <= 1e-12) {
        return r0 * r0;
    }
    double tstar = - (r0 * vrel) / v2;
    if (tstar < 0.0) tstar = 0.0;
    if (tstar > tau) tstar = tau;
    Vec d = r0 + vrel * tstar;
    return d * d;
}

// Attempt to check if a candidate velocity v for this robot is safe w.r.t all others over next tau
static inline bool oh_is_velocity_safe(const Controller *self, const Vec &v, double tau) {
    int N = self->monitor->get_robot_number();
    for (int j = 0; j < N; ++j) {
        if (j == self->id) continue;
        Vec pj = self->monitor->get_pos_cur(j);
        Vec vj = self->monitor->get_v_cur(j);
        double Rsum = self->r + self->monitor->get_r(j);
        Vec r0 = self->pos_cur - pj;
        Vec vrel = v - vj;
        double md2 = oh_min_dist2_over_interval(r0, vrel, tau);
        if (md2 < (Rsum - 1e-9) * (Rsum - 1e-9)) return false;
    }
    return true;
}

// Main implementation
Vec Controller::get_v_next() {
    // Time interval (prefer external TIME_INTERVAL if available via ODR). If not, use OH_TAU.
    double tau = OH_TAU;

    // Direction toward target
    Vec to_tar = pos_tar - pos_cur;
    double dist2 = to_tar * to_tar;
    double dist = (dist2 > 0.0) ? std::sqrt(dist2) : 0.0;
    const double eps = 1e-6;
    if (!(dist < 1e300) || std::isnan(dist)) {
        return Vec(0.0, 0.0);
    }
    if (dist <= eps) {
        return Vec(0.0, 0.0);
    }
    Vec dir = to_tar * (1.0 / dist);

    // Cap speed: do not exceed v_max and avoid overshooting in one step
    double desired_speed = std::min(v_max, dist / tau);
    if (!(desired_speed >= 0.0) || std::isnan(desired_speed) || std::isinf(desired_speed)) {
        desired_speed = 0.0;
    }
    Vec v = dir * desired_speed;

    // Quick accept if safe
    if (oh_is_velocity_safe(this, v, tau)) return v;

    // Explore scaled and rotated candidates
    const double scales[] = {0.9, 0.7, 0.5, 0.3, 0.15, 0.0};
    const double deg2rad = 3.141592653589793238462643383279502884 / 180.0;
    const double angles_deg[] = {0, 15, -15, 30, -30, 60, -60, 90, -90, 135, -135, 180};

    for (double s : scales) {
        Vec base = dir * (desired_speed * s);
        for (double a_deg : angles_deg) {
            double a = a_deg * deg2rad;
            Vec cand = base.rot(a);
            if (oh_is_velocity_safe(this, cand, tau)) {
                return cand;
            }
        }
    }

    // Fallback: stand still
    return Vec(0.0, 0.0);
}

#endif // SRC_HPP_IMPLEMENTATION
