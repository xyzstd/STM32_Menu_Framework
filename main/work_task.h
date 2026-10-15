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
 *   3. 运动检测 (震动开关, PC13)
 *   4. 休眠/自动关机计时
 *   5. 过温保护
 *   6. 温度到达检测
 *   7. 活动状态跟踪
 *   8. 热电偶短路/开路检测 (锁存断热)
 *   9. 上电硬件链路自检 (热电偶/发热芯/电流/电压, 失败锁存故障码)
 */

/* 冷态判定门限 (°C): 烙铁头高于此温度时发热芯阻值已随温度偏移,
 * 冷态阻值匹配/标定不可靠. Cur Cal 电流标定、发热芯型号匹配、
 * 手柄切换确认共用此门限, 需统一修改此处. */
#define TYPE_CUR_COOL_MAX   50

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

/* 长流程(温度校准等)期间抑制休眠/关机计时, 防止中途被入睡打断 */
void WorkTask_SetSleepSuppress(bool en);

/* 校准页(Cur Cal)加热抑制: 置位断热不控热(热态烙铁头自然降温后冷态标定),
 * 清除恢复加热使能重新起控. 页面 enter/exit 配对调用. */
void WorkTask_SetCalSuppress(bool en);

/* 获取当前工作状态 */
work_state_t WorkTask_GetState(void);

/* 温度是否已达到目标温度附近 */
bool WorkTask_IsTempReached(void);

/* 运行时目标温度 (RAM, 不落 flash):
 * HOME 页临时调解的温度保存在此, 整个运行期间持续生效,
 * 休眠唤醒时恢复到该值(而非默认值). 掉电丢失. */
void    WorkTask_SetRuntimeTemp(int16_t temp);
int16_t WorkTask_GetRuntimeTemp(void);

/* 手柄型号切换后的安全闭环: 重载 PID + 目标重钳位.
 * 调用方需已更新 g_config.handle_type 并 Config_Save() 后调用. */
void WorkTask_OnHandleChanged(void);

/* 获取最新实测温度 */
int16_t WorkTask_GetMeasuredTemp(void);

/* 获取当前加热功率 */
int16_t WorkTask_GetPower(void);

/* 最近一次自整定结果是否已成功写入 Flash.
 * 仅当 PID_TuneLastSucceeded() 为 true 时有意义. */
bool WorkTask_TuneSaveSucceeded(void);

/* 获取加热百分比 (0~100) */
uint8_t WorkTask_GetPowerPercent(void);

/* 加热器故障检测 */
bool WorkTask_IsHeaterFault(void);

/* 任一锁存保护故障(过温/热电偶失效/加热器故障)是否处于锁定态 */
bool WorkTask_HasFault(void);

/* 清除全部锁存故障, 允许重新加热.
 * 注: 上电自检故障(selftest_fault)不在此清除 —— 需经 WorkTask_Recover 重跑
 * 自检链路复核, 而非直接解除锁存. */
void WorkTask_ClearFaults(void);

/* 按故障类型恢复 (烙铁插好后长按确认, 见 HOME 页):
 * 仅过温(链路完好) → 只清过温锁存, 不重跑自检链路;
 * 链路类故障(热电偶/加热器/自检) → 重跑自检链路, 全链路通过才恢复. */
void WorkTask_Recover(void);

/* ========== 非阻塞发热芯测量脉冲 (换头链检 / Cur Cal 共用) ==========
 * 原实现 busy-wait 约 200ms 等 PWM 稳定, 期间主循环停摆(输入事件积压、UI 冻结).
 * 现改为状态机: Start 施脉冲 → 下一个 PID 周期读电流 → Poll 取结果.
 * 调用方(页面)负责轮询, 并让 display 返回一个短周期以驱动轮询. */
typedef enum {
    WORK_PULSE_IDLE = 0,   /* 未开始 / 已收尾 */
    WORK_PULSE_BUSY,       /* 脉冲进行中 */
    WORK_PULSE_READY,      /* 结果就绪(本次已给出, 状态随即回 IDLE) */
} work_pulse_t;

/* 换头链路检测(非阻塞): 启动时做瞬时检查(准入/热电偶)并发起测量脉冲.
 * 返回 false = 未受理(已有测量在跑 / 锁存故障 / 自检未完成 / 热电偶异常),
 * 调用方应立即拒绝本次操作. 模拟模式恒受理. */
bool WorkTask_CheckChainStart(uint8_t idx);

/* 轮询链检结论: READY 时 *ok_out 给出最终结论(电流区间 / 型号匹配 / 供电).
 * 未发起过链检时返回 IDLE. */
work_pulse_t WorkTask_CheckChainPoll(bool *ok_out);

/* 发起冷态发热芯测量脉冲(Cur Cal 标定用, 非阻塞).
 * require_cold=true 时非冷态直接拒绝(阻值已偏移, 标定不可靠).
 * 返回 false = 未受理. 模拟模式恒受理并立即就绪. */
bool WorkTask_PulseStart(bool require_cold);

/* 轮询冷态测量结果: READY 时 *cur_out 为发热芯导通电流 ADC. */
work_pulse_t WorkTask_PulsePoll(uint16_t *cur_out);

/* 中止进行中的测量(页面退出等): 清测量占空比并断热, 结果作废.
 * 无测量时为空操作, 不影响正常加热使能. */
void WorkTask_PulseAbort(void);

/* 发热芯型号匹配 (两级保护) 公开包装: Cur Cal 页保存参考值前校验,
 * 拒绝拿 T12 头填 T245/T210 参考值等误操作 (判定规则见 type_cur_match). */
bool WorkTask_TypeCurMatch(uint16_t cur, uint8_t idx);

/* ========== 新增接口 (Cai-Zi 功能) ========== */

/* 获取输入电压 (*10, 如 240 = 24.0V) */
uint16_t WorkTask_GetVinX10(void);

/* 是否检测到运动 (震动开关) */
bool WorkTask_IsMotionDetected(void);

/* 上电硬件链路自检故障码 (0=通过, 非 0=对应链路失败, 仅重启可复测) */
uint8_t WorkTask_GetSelftestCode(void);

/* 统一锁存故障码 (0=无故障, 供 HOME 状态行显示 ErrN):
 * 自检锁存时返回自检码(1~4); 否则按运行中锁存故障映射
 * 1=热电偶失效 2=加热器故障 3=过温. 优先级: 自检 > 热电偶 > 加热器 > 过温. */
uint8_t WorkTask_GetFaultCode(void);

/* 首次冷态电流标定引导待处理 (当前型号无冷态电流参考, 自检通过后置位;
 * 含插头不匹配引导: T12 配置已标定但插入 450°C 量级头):
 * 由 UI 层监视并自动进入 Cur Cal 页, 引导用户先冷态标定再正常加热.
 * 用户标定完成或明确跳过(退出引导页)后由 WorkTask_BootCalDone 清除. */
bool WorkTask_IsBootCalPending(void);
void WorkTask_BootCalDone(void);

/* 插头不匹配引导: 自检实测脉冲电流 (供 Cur Cal 页预选插入的型号; 未触发时 0) */
uint16_t WorkTask_GetTipCur(void);

/* 按实测脉冲电流匹配插入的发热芯型号 (Cur Cal 插头不匹配引导默认选中用):
 * 与各已标定参考取差值最小者; 候选均未标定 → 默认 450°C 量级型号(T245). */
uint8_t WorkTask_MatchTipType(uint16_t cur);

#endif
