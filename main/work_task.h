#ifndef __WORK_TASK_H
#define __WORK_TASK_H

#include <stdint.h>
#include <stdbool.h>

/*
 * 后台工作任务
 *
 * 功能:
 *   1. 分段式 PID 温度闭环控制
 *   2. 输入电压检测 (PA2)
 *   3. 手柄连接检测 (NTC, PA6)
 *   4. 运动检测 (震动开关, PA8)
 *   5. 休眠/自动关机计时
 *   6. 过温保护
 *   7. 温度到达检测
 *   8. 活动状态跟踪
 */

/* 工作状态 */
typedef enum {
    WORK_STATE_ACTIVE,      /* 正常工作 */
    WORK_STATE_SLEEP,       /* 休眠 (降低温度到 sleep_temp) */
    WORK_STATE_SHUTDOWN,    /* 关机 (关闭加热) */
} work_state_t;

/* 初始化后台任务 */
void WorkTask_Init(void);

/* 在 main loop 中周期性调用 */
void WorkTask_Process(void);

/* 标记用户活动 (重置休眠/关机计时器, 唤醒) */
void WorkTask_MarkActivity(void);

/* 获取当前工作状态 */
work_state_t WorkTask_GetState(void);

/* 温度是否已达到目标温度附近 */
bool WorkTask_IsTempReached(void);

/* 大焊点自动加温(Boost)是否正在生效(含回落过程) */
bool WorkTask_IsBoostActive(void);

/* 运行时目标温度 (RAM, 不落 flash):
 * HOME 页临时调解的温度保存在此, 整个运行期间持续生效,
 * 休眠唤醒时恢复到该值(而非默认值). 掉电丢失. */
void    WorkTask_SetRuntimeTemp(int16_t temp);
int16_t WorkTask_GetRuntimeTemp(void);

/* 手柄型号切换后的安全闭环: 重载 PID + 目标重钳位 + 清 Boost 复位.
 * 调用方需已更新 g_config.handle_type 并 Config_Save() 后调用. */
void WorkTask_OnHandleChanged(void);

/* 获取最新实测温度 */
int16_t WorkTask_GetMeasuredTemp(void);

/* 获取当前加热功率 */
int16_t WorkTask_GetPower(void);

/* 获取加热百分比 (0~100) */
uint8_t WorkTask_GetPowerPercent(void);

/* 加热器故障检测 */
bool WorkTask_IsHeaterFault(void);
void WorkTask_ClearHeaterFault(void);

/* ========== 新增接口 (Cai-Zi 功能) ========== */

/* 获取输入电压 (*10, 如 240 = 24.0V) */
uint16_t WorkTask_GetVinX10(void);

/* 获取 NTC 手柄温度 (°C) */
int16_t WorkTask_GetNtcTemp(void);

/* 手柄是否已连接 */
bool WorkTask_IsHandleConnected(void);

/* 是否检测到运动 (震动开关) */
bool WorkTask_IsMotionDetected(void);

#endif