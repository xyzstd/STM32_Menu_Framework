#include "menu.h"
#include "oled.h"
#include "oled_data.h"
#include "work_task.h"
#include "pid_controller.h"
#include "config.h"
#include "temp.h"
#include "pid.h"
#include "heater.h"
#include "buzzer.h"
#include <string.h>

static menu_result_t home_handler(const menu_cmd_t *cmd, void *user_data);
static menu_period_t home_display(void *user_data);
static void home_enter(void *user_data);

static menu_result_t home_handler(const menu_cmd_t *cmd, void *user_data)
{
    (void)user_data;
    int16_t current_set = WorkTask_GetRuntimeTemp();

    switch(cmd->event)
    {
        case MENU_EVENT_UP:
            current_set += (int16_t)(5 * cmd->step);
            if(current_set > (int16_t)Config_GetTempMax())
                current_set = (int16_t)Config_GetTempMax();
            WorkTask_SetRuntimeTemp(current_set);
            PID_Controller_SetSetpoint(current_set);
            break;
        case MENU_EVENT_DOWN:
            current_set -= (int16_t)(5 * cmd->step);
            if(current_set < TEMP_MIN)
                current_set = TEMP_MIN;
            WorkTask_SetRuntimeTemp(current_set);
            PID_Controller_SetSetpoint(current_set);
            break;
        case MENU_EVENT_CLICK:
            return MENU_RET_DONE;   /* 结束常驻页; 因 child_num>0 由框架自动下钻子列表 */
        case MENU_EVENT_LPRESS:
            /* 故障锁存时: 长按恢复(见 WorkTask_Recover)——
             * 过温只清锁存不重检链路; 链路类故障重跑自检, 通过才恢复 */
            if (WorkTask_HasFault())
            {
                WorkTask_Recover();
                Buzzer_Beep(BUZZER_EVENT_TEMP_REACHED);
            }
            break;
        default:
            break;
    }
    return MENU_RET_OK;
}

static menu_period_t home_display(void *user_data)
{
    (void)user_data;
    int16_t set_temp;
    int16_t meas_temp;
    uint8_t percent;
    bool handle_ok;
    work_state_t state;

    set_temp   = WorkTask_GetRuntimeTemp();
    meas_temp  = WorkTask_GetMeasuredTemp();
    percent    = WorkTask_GetPowerPercent();
    handle_ok  = !WorkTask_HasFault();   /* 传感器/加热故障(短路/开路/过温等)视为不可加热 */
    state      = WorkTask_GetState();

    OLED_Clear();

    /*========== 第1行 (y=0-15): 设定温度 + 电压 + 功率条 ==========*/

    OLED_ShowString(2, 0, "SET:", OLED_8X16);
    OLED_ShowNum(34, 0, set_temp, 3);
    OLED_ShowDegree(58, 0);
    OLED_ShowChar(66, 0, 'C', OLED_8X16);

    /* 当前设置手柄类型显示 (如 T245, 右对齐到原电压位置 x=116), 与 "°C" 拉开间距 */
    {
        const char *name = Config_GetHandleName();
        OLED_ShowString(116 - (uint8_t)(8 * strlen(name)), 0, name, OLED_8X16);
    }

    /*========== 第2行 (y=16-47, 大字号温度 + °C 完整框于框内) ==========*/

    /* 大字号温度: 按实际字符数(1~3 位, 负数含符号位)动态居中, 修复升温/降温
     * 到 1~2 位数时大字偏左; 外框圆角与其它页统一为 6 */
    {
        int32_t mag    = (meas_temp < 0) ? -(int32_t)meas_temp : (int32_t)meas_temp;
        uint8_t digits = (mag >= 100) ? 3 : (mag >= 10) ? 2 : 1;
        uint8_t span   = (uint8_t)(digits + ((meas_temp < 0) ? 1 : 0));   /* 含负号占位 */
        int16_t num_x  = (int16_t)(4 + (120 - (int16_t)(span * 16 + 16)) / 2);   /* +16: 尾部 °C 宽 */

        OLED_DrawRoundRect(4, 16, 120, 32, 6, OLED_COLOR_SET);
        OLED_ShowLargeNum(num_x, 16, meas_temp);

        /* 大字号 °C 紧随数字右侧 (页对齐 y=16, 不越出外框) */
        {
            uint8_t page, col;
            for(page = 0; page < 4; page++)
            {
                for(col = 0; col < 16; col++)
                {
                    uint8_t data = OLED_Rev8(OLED_F16x32[10][page * 16 + col]);
                    if(data)
                        OLED_DisplayBuf[2 + page][(uint8_t)(num_x + span * 16) + col] |= data;
                }
            }
        }
    }

    /*========== 第3行 (y=48-63): 状态行 ==========*/

    /* 分隔线 y=47: 与大温度框底边(亦在 y=47)连成通栏横线 */
    OLED_DrawLine(0, 47, 127, 47, OLED_COLOR_SET);

    /* 主状态行 (8x16, 单行互斥显示, 避免覆盖残留像素):
     * 优先级阶梯即索引顺序: 自检故障码 > 传感器故障 > 关机 > 休眠 >
     * 大焊点加温(Boost) > 温度到达 > 加热. 查表取文本, 分支只裁决索引. */
    {
        static const char *const state_text[] = {"NoHnd", "OFF", "Sleep", "Boost", "OK", "Heat"};
        uint8_t state_idx = 0;

        uint8_t st_code = WorkTask_GetFaultCode();
        if (st_code)
        {
            /* 锁存故障统一显示 ErrN: 自检码 1~4; 运行中 1=热电偶 2=加热器
             * 3=过温. 修复: 原仅查自检码, 运行中故障会错误显示 "NoHnd". */
            OLED_ShowString(2, 48, "Err", OLED_8X16);
            OLED_ShowNum(26, 48, st_code, 2);
        }
        else
        {
            if (!handle_ok)
                state_idx = 0;
            else if (state == WORK_STATE_SHUTDOWN)
                state_idx = 1;
            else if (state == WORK_STATE_SLEEP)
                state_idx = 2;
            else if (WorkTask_IsBoostActive())
                state_idx = 3;
            else if (WorkTask_IsTempReached())
                state_idx = 4;
            else
                state_idx = 5;

            OLED_ShowString(2, 48, state_text[state_idx], OLED_8X16);
        }
    }

    /* 加热功率进度条 (x=43~94, 左侧状态文本最长到 x=42, 右侧 100% 数字从 x=96 起) */
    OLED_DrawProgressBar(43, 51, 52, 10, percent);

    /* 功率百分比 (右对齐, 不补前导 0) */
    {
        uint8_t p = percent;
        uint8_t plen = (p >= 100) ? 3 : (p >= 10) ? 2 : 1;
        OLED_ShowNum(120 - (int16_t)(plen * 8), 48, p, plen);
        OLED_ShowChar(120, 48, '%', OLED_8X16);
    }

    OLED_UpdateDiff();      /* 页级差量推送: 仅更新内容变化页, 减少软 I2C 占用 */
    return 100;  /* 100ms 页面刷新 */
}

static void home_enter(void *user_data)
{
    (void)user_data;
    /* 进入 HOME: 同步 PID 设定值到运行时目标温度(不重置为默认值),
     * 保持用户临时调解的温度在整个运行期间持续生效 */
    PID_Controller_SetSetpoint(WorkTask_GetRuntimeTemp());
    PID_Controller_Reset();
}

const menu_ops_t home_ops = {
    .enter   = home_enter,
    .handler = home_handler,
    .display = home_display,
    .exit    = NULL,
};