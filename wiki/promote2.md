任务：生成一个 C 源文件（可编译为 Windows DLL），实现 Python ctypes 所需的 C 接口：`plan_paths_c`、`compute_acceleration_c`，并提供 `free_buffer`。代码应清晰、可读、易于编译（MSVC 与 MinGW），不依赖第三方库，仅使用标准 C 库。请严格遵循下面的函数签名、内存所有权与数据格式要求，并包含注释、错误检查与示例说明。

背景与目标接口（必须 EXACT）：
plan_paths_c
- 签名（C）：
  __declspec(dllexport) double* plan_paths_c(const double* verts_flat, const int* poly_counts, int n_polys, int total_vertices, const double* starts, const double* goals, int n_agents, int steps)
- 语义与参数说明：
  - `verts_flat`: 所有多边形顶点按序展平为 [x0,y0,x1,y1,...]，当没有多边形时可以为 NULL。
  - `poly_counts`: 每个多边形顶点个数的数组，长度为 `n_polys`；若没有多边形可为 NULL。
  - `n_polys`: 多边形数量（可为 0）。
  - `total_vertices`: `verts_flat` 中顶点总数（即 sum(poly_counts)）。
  - `starts`: 长度为 `n_agents*2` 的数组：[sx0,sy0,sx1,sy1,...]
  - `goals`: 同上
  - `n_agents`: 无人机数量（>=0）
  - `steps`: 每条路径采样点的最大个数（>0）
- 返回值格式（所有元素均以 double 存放）：
  - 返回分配的 `double*` 指针（调用方必须用库提供的 `free_buffer` 释放），内容按序：
    [ n_agents, counts[0], counts[1], ..., counts[n_agents-1], coords... ]
    - 第0个 double：n_agents 的值（以 double 存放）
    - 接下来 n_agents 个 double：每个 agent 返回的点数 counts[i]（整数值，以 double 存放）
    - 随后是所有 agent 的坐标点按顺序拼接的 (x,y) 双精度数组，长度为 sum(counts)*2
  - 若发生错误（例如内存分配失败、输入不合法），返回 NULL
- 行为约定/实现说明要求：
  - 若 start==goal（完全相同坐标），counts[i] = 1 且返回该单点。
  - 在本提示中，允许实现“简化但正确”的路径生成：如果没有障碍（n_polys==0 或 poly_counts==NULL），或为简化实现，则返回 start->goal 的线性插值，点数为 `steps`（若 start==goal 则为1）。为保兼容性，建议按以下策略实现：
    - 计算每 agent 的 counts[i]：若 start==goal -> 1，否则 = steps（但不得超过 steps）
    - 对每条路径做等距参数插值，包含两端点（当 counts>1 时，t 从 0 到 1，分为 counts-1 段）
  - 函数要正确计算输出缓冲区长度并用 `malloc` 分配，失败时释放已分配资源并返回 NULL。
  - 提供并导出 `free_buffer(double* ptr)`，用来释放 `malloc` 分配的内存。
  - 要求：代码在 Windows 下可生成 DLL（使用 `__declspec(dllexport)`），但也应能在非 Windows 下编译（可以用宏封装导出定义）。
  - 风格：清晰注释、严谨的边界检查、避免未定义行为。

其他通用要求：
- 添加 `#include <stdlib.h> <stdio.h> <math.h>` 等必要头文件。
- 使用 `malloc`/`free` 管理内存；返回给 Python 的缓冲区必须由 `free_buffer` 释放。
- 使用明确的类型转换并避免未对齐访问。
- 导出符号使用 `__declspec(dllexport)`，并提供宏封装以便跨平台（例如：EXPORT宏）。
- 提供函数注释，说明返回格式、内存所有权与示例。
- 包含简单单元测试代码片段（可在 C 文件或 README）或至少在注释中给出如何用 Python ctypes 调用的示例。

质量与边界条件（请严格遵守）：
- 返回缓冲区布局必须与上文精确一致（first double = n_agents，随后 n_agents doubles 为 counts）。
- 函数不得泄漏内存（在错误路径上也要释放已分配资源）。
- 所有指针参数应先做 NULL 检查。
- 注释中须包含 Python 侧解析返回 buffer 的示例代码（可简短但完整）。
- 文件顶部写明“此文件由 LLM 生成以符合特定 ctypes 接口要求”，并列出实现日期与作者（LLM 名称可写为“自动生成”）。
- 代码中禁止包含任何未授权外部依赖。
- 代码风格请接近现有示例（例如 plan_paths.c），并保持一致的返回格式与 `free_buffer` 语义。
附件：
```plan_paths.c 
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
```

以下是用户的函数具体实现的方法：
