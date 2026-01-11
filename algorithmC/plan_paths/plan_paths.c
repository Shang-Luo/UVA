#include <stdlib.h>
#include <stdio.h>

// Simple C implementation that returns straight-line interpolated paths.
// Interface (exported):
// double* plan_paths_c(const double* verts_flat, const int* poly_counts, int n_polys, int total_vertices, const double* starts, const double* goals, int n_agents, int steps)
// - verts_flat, poly_counts: accepted but ignored by this simple implementation
// - starts: array length n_agents*2 (x,y)
// - goals: array length n_agents*2 (x,y)
// - returns pointer to double array of length n_agents * steps * 2
// Caller must free with free_buffer(double*)

__declspec(dllexport) double* plan_paths_c(const double* verts_flat, const int* poly_counts, int n_polys, int total_vertices, const double* starts, const double* goals, int n_agents, int steps) {
    if (n_agents <= 0 || steps <= 0) return NULL;
    int out_len = n_agents * steps * 2;
    double* out = (double*)malloc(sizeof(double) * out_len);
    if (!out) return NULL;
    for (int i = 0; i < n_agents; ++i) {
        double sx = starts[i*2 + 0];
        double sy = starts[i*2 + 1];
        double gx = goals[i*2 + 0];
        double gy = goals[i*2 + 1];
        for (int j = 0; j < steps; ++j) {
            double t = (steps > 1) ? ((double)j / (double)(steps-1)) : 0.0;
            double x = sx + (gx - sx) * t;
            double y = sy + (gy - sy) * t;
            int idx = (i*steps + j) * 2;
            out[idx + 0] = x;
            out[idx + 1] = y;
        }
    }
    return out;
}

__declspec(dllexport) void free_buffer(double* ptr) {
    if (ptr) free(ptr);
}
