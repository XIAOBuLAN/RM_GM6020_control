#ifndef PLOT_H
#define PLOT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 波形输出: 通过 USART6 (3-pin 串口口, 外壳丝印为 "UART1") 以 VOFA+ 的 JustFloat
// 协议周期发送 3 路数据: 目标转速 / 实际转速 / 电流指令。
// 用法: 在 1kHz 控制循环里每拍调用一次, 内部按 5:1 分频(200Hz)并用 DMA 非阻塞发送,
//       不会拖慢控制周期。
// VOFA+ 设置: 数据格式 JustFloat, 波特率 115200, 3 通道。
void PlotUpdate(float ref_rpm, float fdb_rpm, float cmd);

#ifdef __cplusplus
}
#endif

// 调试观察变量: 文件作用域普通全局变量, 调试器在任意断点位置都能直接 watch
extern float   g_dbg_ref_rpm;   // 本拍目标转速 (rpm)
extern float   g_dbg_fdb_rpm;   // 本拍实际转速 (rpm)
extern int16_t g_dbg_cmd;       // 本拍输出电流指令 (counts, 16384 ≈ 3A)
extern float   g_dbg_ref_rad;   // 本拍目标角度 (rad) —— 仅位置模式有意义
extern float   g_dbg_fdb_rad;   // 本拍实际角度 (rad)

#endif // PLOT_H
