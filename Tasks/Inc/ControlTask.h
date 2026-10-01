/*
 * @Author: Xiaobu 279435887+XIAOBuLAN@users.noreply.github.com
 * @Date: 2026-09-27 11:19:05
 * @LastEditors: Xiaobu 279435887+XIAOBuLAN@users.noreply.github.com
 * @LastEditTime: 2026-09-27 11:31:54
 * @FilePath: \gm6020_sin_task\Tasks\ControlTask.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef CONTROLTASK_H
#define CONTROLTASK_H

#include <stdint.h>

// 供 C 中断处理(stm32f4xx_it.c 及 HAL 弱回调)调用的接口
#ifdef __cplusplus
extern "C" {
#endif

// 上电初始化: 配置 CAN 过滤器、启动 CAN 与 1kHz 控制定时器
// 在 main.c 中 MX_XXX_Init() 全部执行完后调用一次
void ControlTaskInit(void);

// 1kHz 控制入口, 在 TIM6 周期中断(HAL_TIM_PeriodElapsedCallback)中调用
void MainTask(void);

// CAN 接收回调, 在 HAL_CAN_RxFifo0MsgPendingCallback 中调用
void CanFeedbackCallback(uint32_t std_id, const uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif // CONTROLTASK_H
