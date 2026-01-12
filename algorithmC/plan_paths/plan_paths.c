#include <stdlib.h>
#include <stdio.h>

// Simple C implementation that returns straight-line interpolated paths.
// New interface format (exported):
// double* plan_paths_c(const double* verts_flat, const int* poly_counts, int n_polys, int total_vertices, const double* starts, const double* goals, int n_agents, int steps)
// The returned buffer layout (all doubles):
// [ n_agents, counts[0], counts[1], ..., counts[n_agents-1], coords... ]
// where counts[i] is number of (x,y) points for agent i, and coords are
// concatenated x,y pairs for each agent in order. Caller must call free_buffer.

__declspec(dllexport) double* plan_paths_c(const double* verts_flat, const int* poly_counts, int n_polys, int total_vertices, const double* starts, const double* goals, int n_agents, int steps) {
    if (n_agents <= 0 || steps <= 0) return NULL;
    // determine counts per agent: if start==goal, return single point, else return 'steps' points
    int* counts = (int*)malloc(sizeof(int) * n_agents);
    if (!counts) return NULL;
    int total_points = 0;
    for (int i = 0; i < n_agents; ++i) {
        double sx = starts[i*2 + 0];
        double sy = starts[i*2 + 1];
        double gx = goals[i*2 + 0];
        double gy = goals[i*2 + 1];
        if (sx == gx && sy == gy) {
            counts[i] = 1;
        } else {
            counts[i] = steps;
        }
        total_points += counts[i];
    }

    // buffer: 1 (n_agents) + n_agents (counts) + total_points*2 (coords)
    int header = 1 + n_agents;
    int out_len = header + total_points * 2;
    double* out = (double*)malloc(sizeof(double) * out_len);
    if (!out) {
        free(counts);
        return NULL;
    }

    out[0] = (double)n_agents;
    for (int i = 0; i < n_agents; ++i) {
        out[1 + i] = (double)counts[i];
    }

    int coord_idx = header;
    for (int i = 0; i < n_agents; ++i) {
        double sx = starts[i*2 + 0];
        double sy = starts[i*2 + 1];
        double gx = goals[i*2 + 0];
        double gy = goals[i*2 + 1];
        int cnt = counts[i];
        for (int j = 0; j < cnt; ++j) {
            double t = (cnt > 1) ? ((double)j / (double)(cnt-1)) : 0.0;
            double x = sx + (gx - sx) * t;
            double y = sy + (gy - sy) * t;
            out[coord_idx++] = x;
            out[coord_idx++] = y;
        }
    }
    free(counts);
    return out;
}

__declspec(dllexport) void free_buffer(double* ptr) {
    if (ptr) free(ptr);
}
