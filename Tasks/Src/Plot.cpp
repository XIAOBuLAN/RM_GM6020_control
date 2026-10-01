#include "Plot.h"

#include <string.h>

#include "usart.h"   // huart6

// ---- 调试观察变量 (定义, 声明在 Plot.h) ----
float   g_dbg_ref_rpm = 0.0f;
float   g_dbg_fdb_rpm = 0.0f;
int16_t g_dbg_cmd     = 0;
float   g_dbg_ref_rad = 0.0f;
float   g_dbg_fdb_rad = 0.0f;

namespace {
// 发送分频: 控制周期 1kHz, 每 kPlotDecim 拍发一帧 -> 200Hz
// 一帧 16 字节在 115200 波特率下约占 1.4ms, 200Hz 时串口占用约 28%
constexpr uint32_t kPlotDecim = 5;

// 一帧 = 3 个 float32 (小端, 12 字节) + JustFloat 帧尾 (4 字节)
constexpr uint16_t kPlotBytes = 16;

// VOFA+ JustFloat 帧尾: 小端 float 0x7F800000 (+inf)
constexpr uint8_t kFrameTail[4] = {0x00, 0x00, 0x80, 0x7F};

uint32_t divider_;
uint8_t  frame_[kPlotBytes];   // DMA 传输期间该缓冲区必须保持有效 -> 放静态区
}  // namespace

void PlotUpdate(float ref_rpm, float fdb_rpm, float cmd) {
    // 记录下来供调试器观察(任意断点位置都能看到)
    g_dbg_ref_rpm = ref_rpm;
    g_dbg_fdb_rpm = fdb_rpm;
    g_dbg_cmd     = static_cast<int16_t>(cmd);

    if (++divider_ < kPlotDecim) return;
    divider_ = 0;

    // 上一帧还没发完就跳过本帧: 非阻塞, 不拖慢 1kHz 控制循环
    if (huart6.gState != HAL_UART_STATE_READY) return;

    const float channels[3] = {ref_rpm, fdb_rpm, cmd};
    memcpy(&frame_[0],  channels, sizeof(channels));     // 小端 float32 x3
    memcpy(&frame_[12], kFrameTail, sizeof(kFrameTail)); // JustFloat 帧尾

    (void)HAL_UART_Transmit_DMA(&huart6, frame_, kPlotBytes);
}
