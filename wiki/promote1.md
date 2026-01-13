任务：生成一个个 C 源文件（可编译为 Windows DLL），实现 Python ctypes 所需的 C 接口：`compute_acceleration_c`，并提供 `free_buffer`。代码应清晰、可读、易于编译（MSVC 与 MinGW），不依赖第三方库，仅使用标准 C 库。请严格遵循下面的函数签名、内存所有权与数据格式要求，并包含注释、错误检查与示例说明。

背景与目标接口（必须 EXACT）：
compute_acceleration_c
- 签名（C）：
  __declspec(dllexport) void compute_acceleration_c(const double* radar_readings, const double* directions, int N, const double* target_vec, const double* vel, double* out_axay)
- 参数说明：
  - `radar_readings`: 长度 N 的距离数组（双精度），距离单位可任意但内部一致。
  - `directions`: 长度 2*N 的数组，每个方向为 (dx,dy) 连续排列（单位向量或近似单位向量）。
  - `N`: 方向数量（>=1）。
  - `target_vec`: 长度 2 的数组，表示从当前位置到目标点的向量 (tx,ty)。
  - `vel`: 长度 2 的数组，当前速度 (vx,vy)。
  - `out_axay`: 长度 2 的输出数组，调用方提供由 C 函数写入 (ax, ay)。
- 行为约定/实现目标：
  - 实现一个简单的反应式控制器：
    - 目标吸引力：沿 `target_vec` 方向产生与其大小或单位方向成比例的吸引加速度（系数 k_goal）。
    - 障碍避让力：对每个 `i`，若 radar_readings[i] < max_range（或 >0），则在 `-directions[i]` 方向上施加与距离的反比（或反二次）成比例的排斥力，近处障碍影响更强（系数 avoid_gain）。
    - 速度阻尼：根据当前速度产生一个阻尼项（如 -k_damp * vel）以抑制震荡。
    - 合成最终加速度并限制在 `max_acc` 以内（即若长度超过 max_acc，则按比例缩放）。
  - 参数可写死为常数或通过 `params` 传入；但此 C 接口不带 params 参数，因此在实现中定义几个合理默认常量：`k_goal=1.0`, `avoid_gain=0.5`, `max_acc=2.0`, `max_range=10.0`, `k_damp=0.1`。
  - 函数必须对输入进行合理检查（N>0，指针非 NULL 等），若发现非法输入则写入 `out_axay[0]=0` 与 `out_axay[1]=0` 并返回。
  - 实现中要求数值稳定、无 NaN/inf 的未定义行为。

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
```

**以下是用户的函数具体实现的方法：**
