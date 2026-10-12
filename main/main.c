#include "stm32f10x.h"
#include "timer.h"
#include "oled.h"
#include "menu.h"
#include "input.h"
#include "ui_task.h"
#include "work_task.h"
#include "buzzer.h"
#include "serial.h"
#include "ota_boot.h"      /* OTA_SerialFeedRx: 注册为串口接收回调 */
#include "ota_config.h"
#include "config.h"
#include "heater.h"
#include "pid_controller.h"
#include "adc.h"

/* 独立看门狗(IWDG): ~2s 超时. 一旦跑飞/卡死/进异常死循环, 主循环无法喂狗,
 * IWDG 超时强制复位 MCU. 复位后回到 Boot, 加热 PWM 归 0, 绝不无限加热烧烙铁.
 * 本项目标准库未含 IWDG 驱动, 直接寄存器操作 (stm32f10x.h 已定义 IWDG 结构体).
 *   LSI ≈ 40kHz, 64 分频 → 625Hz; RLR=1250 → 1250/625 = 2.0s. */
#define WDG_PRESCALER_PR   0x04UL          /* IWDG_PR: 64 分频 */
#define WDG_RELOAD          1250UL          /* ~2s 超时 */

static void Watchdog_Init(void);
static void Watchdog_Kick(void);


static void Watchdog_Init(void)
{
    /* 写访问使能保护寄存器 PR/RLR */
    IWDG->KR = 0x5555UL;
    IWDG->PR = WDG_PRESCALER_PR;            /* 64 分频 */
    IWDG->RLR = WDG_RELOAD;                 /* 重载值 */
    IWDG->KR = 0xAAAAUL;                    /* 先装载, 保证启动即完整超时窗口 */
    IWDG->KR = 0xCCCCUL;                    /* 启动 IWDG */
}

static void Watchdog_Kick(void)
{
    IWDG->KR = 0xAAAAUL;                    /* 喂狗: 重载计数器 */
}

int main(void)
{
    /* 1. 重定位向量表到 App 区（必须放在最前面，任何中断触发之前）*/
    SCB->VTOR = OTA_APP_FLASH_BASE;

    /* 2. 初始化外设 */
    /* 中断优先级分组必须先于任何 NVIC_Init 生效: NVIC_Init 依据配置当时的
     * AIRCR PRIGROUP 计算优先级编码, 分组晚设会让先配置的中断(如 USART1)
     * 编码出的优先级与注释意图不符. 故在此最先设置, 供 Serial/Timer/ADC 共用. */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Watchdog_Init();    /* 先启动 IWDG, 之后主循环周期性喂狗; 跑飞/死锁时超时复位防无限加热 */
    Serial_Init();
    /* 把 OTA 请求序列识别挂到串口接收上: 由应用层注册, 而不是让 bsp/serial.c
     * 反过来 include ota_boot.h —— 后者会让 bsp/ 依赖 ota/. */
    Serial_SetRxCallback(OTA_SerialFeedRx);
    Timer_Init();
    AD_Init();
    Buzzer_Init();
    OLED_Init();

    /* 3. 加载配置 + 初始化 PID 和加热
     *    必须在 Menu_Init 之前完成: Menu_Init 上电即激活常驻页(HOME)进入 handler,
     *    home_enter 会访问 WorkTask 运行时温度并调用 PID_Controller_*,
     *    若这些子系统未就绪则读到未初始化数据/复位无效 PID 状态. */
    Config_Init();
    /* 应用配置的亮度与蜂鸣器开关 (必须在 Config_Init 之后) */
    OLED_SetContrast(g_config.brightness);
    Buzzer_SetEnabled(g_config.buzzer_enable != 0);
    Heater_Init();
    PID_Controller_Init();
    PID_Controller_SetSetpoint((int16_t)g_config.default_temp);
    WorkTask_Init();

    /* 4. 菜单 + 输入 + UI 渲染调度 (依赖上述运行时数据已就绪) */
    Menu_Init();
    Input_Init();
    UI_Task_Init();

    /* 5. 开全局中断 —— 必须在所有外设 Init 之后（Bootloader 跳转时已关中断，
     *    硬件复位不会清零 PRIMASK；此处才开，确保中断向量表已就绪、
     *    NVIC 配置已生效，不会进入未初始化 handler）*/
    __enable_irq();

    while(1)
    {
        Input_Process();    /* 消费输入事件(主循环上下文, 不阻塞 1ms 输入节拍) */
        Buzzer_Process();
        WorkTask_Process();
        UI_Task_Process();
        OLED_FlushStep();   /* 分帧刷屏: 每轮推一页(≤1.5ms), 避免整屏推送阻塞输入 */
        Watchdog_Kick();    /* 喂狗: 主循环正常运行时周期性刷新 */
    }
}
