#include "ControlTask.h"

#include <cmath>

#include "Gm6020.h"
#include "Pid.h"
#include "can.h"
#include "iwdg.h"
#include "tim.h"

namespace {
constexpr float kTwoPi = 6.28318530717958647692f;

// ======================= 可调配置 =========================
// 电机拨码开关设置的 ID (1~7); 电机 CAN 线接在主控 CAN1
constexpr uint8_t kMotorId = 1;

// ---- 跟随模式二选一 ----
//   kSpeedSine -> 速度正弦跟随 (单速度环)
//   kAngleSine -> 位置正弦跟随 (角度环 + 速度环 串级)
enum class Mode { kSpeedSine, kAngleSine };
constexpr Mode kMode = Mode::kSpeedSine;

// ---- 定义正弦曲线 ----
// 速度曲线: v_ref(t) = kSpeedAmp * sin(2*pi*kSpeedFreq*t)   [rad/s]
constexpr float kSpeedAmp  = 10.0f;   // 幅值 10 rad/s ≈ 95.5 rpm
constexpr float kSpeedFreq = 0.5f;    // 频率 0.5 Hz
// 位置曲线: th_ref(t) = kAngleAmp * sin(2*pi*kAngleFreq*t)  [rad]
constexpr float kAngleAmp  = 1.0f;    // 幅值 1.0 rad ≈ 57.3°
constexpr float kAngleFreq = 0.25f;   // 频率 0.25 Hz

// ---- PID 参数 ----
// 速度环: rad/s 误差 -> 电压指令, 输出限幅 ±25000 (≈±24V)
constexpr float kSpeedKp = 60.0f, kSpeedKi = 1200.0f, kSpeedKd = 0.0f;
// 角度环: rad 误差 -> 目标角速度 [rad/s], 输出限幅即电机允许的最大角速度
constexpr float kAngleKp = 15.0f, kAngleKi = 0.0f, kAngleKd = 0.6f;
constexpr float kAnglePidOutLimit = 8.0f;

// 断联保护: 超过该时间没收到电机反馈帧 -> 输出清零、PID 复位
constexpr uint32_t kFeedbackTimeoutMs = 100;

// =================== 运行状态 =======================
Gm6020 motor_(kMotorId);

Pid speed_pid_(kSpeedKp, kSpeedKi, kSpeedKd, 25000.0f, -25000.0f);
Pid angle_pid_(kAngleKp, kAngleKi, kAngleKd, kAnglePidOutLimit, -kAnglePidOutLimit);

uint32_t tick_;          // 1kHz 计数 = 上电以来的毫秒数, 正弦曲线的时间基准
uint32_t last_rx_tick_;  // 最近一次收到目标电机反馈帧的时刻

// 把 FIFO 里积压的反馈帧全部取空并交给协议解析
void CanRxProcess(CAN_HandleTypeDef *hcan, uint32_t fifo) {
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    while (HAL_CAN_GetRxFifoFillLevel(hcan, fifo) > 0U) {
        if (HAL_CAN_GetRxMessage(hcan, fifo, &header, data) != HAL_OK) return;
        CanFeedbackCallback(header.StdId, data);
    }
}
}  // namespace

// ---- ControlTask.h 声明的三个接口 ----

void ControlTaskInit(void) {

    CAN_FilterTypeDef filter = {};
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;

    filter.FilterBank           = 0;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &filter);

    filter.FilterBank           = 14;
    filter.FilterFIFOAssignment = CAN_RX_FIFO1;
    HAL_CAN_ConfigFilter(&hcan2, &filter);

    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

    HAL_TIM_Base_Start_IT(&htim6);  // 启动 1kHz 控制节拍
}

void MainTask(void) {
    tick_++;
    const float t = static_cast<float>(tick_) * 0.001f;

    float voltage = 0.0f;
    if (tick_ - last_rx_tick_ <= kFeedbackTimeoutMs) {
        if (kMode == Mode::kSpeedSine) {
            const float v_ref = kSpeedAmp * sinf(kTwoPi * kSpeedFreq * t);
            voltage = speed_pid_.calc(v_ref, motor_.vel());
        } else {  // kAngleSine: 外环角度环算目标角速度, 内环速度环算电压
            const float th_ref   = kAngleAmp * sinf(kTwoPi * kAngleFreq * t);
            const float v_target = angle_pid_.calc(th_ref, motor_.angle());
            voltage = speed_pid_.calc(v_target, motor_.vel());
        }
    } else {
        speed_pid_.reset();
        angle_pid_.reset();
    }
    motor_.setVoltage(static_cast<int16_t>(voltage));

    // 发送电压控制帧; 三个邮箱都被占用时丢弃本拍, 下拍重发
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U) {
        CAN_TxHeaderTypeDef header = {};
        header.StdId = motor_.txId();
        header.IDE   = CAN_ID_STD;
        header.RTR   = CAN_RTR_DATA;
        header.DLC   = 8;
        uint8_t data[8] = {};
        motor_.encode(data);
        uint32_t mailbox = 0;
        (void)HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox);
    }

    // 喂独立看门狗, 防止死循环卡死
    HAL_IWDG_Refresh(&hiwdg);
}

void CanFeedbackCallback(uint32_t std_id, const uint8_t *data) {
    if (std_id == motor_.rxId()) {  // 只解析本电机
        motor_.decode(data);
        last_rx_tick_ = tick_;
    }
}

// 回调函数

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        MainTask();
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CanRxProcess(hcan, CAN_RX_FIFO0);
}

extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CanRxProcess(hcan, CAN_RX_FIFO1);
}
