#include <stdlib.h>
#include <math.h>

// compute_acceleration_c: mirrors Python compute_acceleration behavior roughly.
// void compute_acceleration_c(const double* radar_readings, const double* directions, int N, const double* target_vec, const double* vel, double* out_axay)
// - radar_readings: array length N
// - directions: array length N*2 (unit vectors x,y)
// - out_axay: array length 2, written with ax, ay

__declspec(dllexport) void compute_acceleration_c(const double* radar_readings, const double* directions, int N, const double* target_vec, const double* vel, double* out_axay) {
    double k_goal = 0.6;
    double avoid_gain = 1.8;
    double max_acc = 5.0;
    double max_range = 1.0;

    if (N > 0) {
        for (int i = 0; i < N; ++i) {
            if (radar_readings[i] > max_range) max_range = radar_readings[i];
        }
    }

    double tx = target_vec[0];
    double ty = target_vec[1];
    double tnorm = sqrt(tx*tx + ty*ty);
    double tdirx = 0.0, tdiry = 0.0;
    if (tnorm > 1e-6) { tdirx = tx/tnorm; tdiry = ty/tnorm; }

    double ax = k_goal * tdirx;
    double ay = k_goal * tdiry;

    double eps = 1e-3;
    for (int i = 0; i < N; ++i) {
        double d = radar_readings[i];
        if (d <= 0) continue;
        if (d <= max_range) {
            double dirx = directions[i*2 + 0];
            double diry = directions[i*2 + 1];
            double strength = avoid_gain * (1.0 / ((d + eps) * (d + eps)));
            ax += -dirx * strength;
            ay += -diry * strength;
        }
    }

    ax += -0.2 * vel[0];
    ay += -0.2 * vel[1];

    double m = sqrt(ax*ax + ay*ay);
    if (m > max_acc && m > 0.0) {
        double s = max_acc / m;
        ax *= s; ay *= s;
    }

    out_axay[0] = ax;
    out_axay[1] = ay;
}
