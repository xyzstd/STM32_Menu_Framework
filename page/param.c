#include "param.h"
#include "config.h"
#include "buzzer.h"
#include "toast.h"

void param_enter(param_t *it)
{
    it->dirty = false;
    if (it->load) it->load(it);

    /* 装载即钳位: 配置字段可能落在 [min,max] 之外, 而 param_handler 只在
     * 首次 UP/DOWN 时才钳位 —— 不钳的话进入页面会显示超范围值, 且用户看到的
     * 初值是错的. 可达场景: default_temp 全局共享而 temp_max[] 按型号 ——
     * 若某型号上限被调低, work_task 只钳了 runtime_temp, default_temp 仍可能越界. */
    if (it->val < it->min) it->val = it->min;
    if (it->val > it->max) it->val = it->max;

    it->base = it->val;   /* 记住进入初值(钳位后), 供 exit 判定是否真改动 */
}

menu_result_t param_handler(const menu_cmd_t *cmd, param_t *it)
{
    switch(cmd->event)
    {
    case MENU_EVENT_UP:
    case MENU_EVENT_DOWN:
    {
        int16_t old = it->val;

        if (it->toggle)
            it->val = (it->val == it->min) ? it->max : it->min;
        else
        {
            int16_t delta = (int16_t)(it->step * cmd->step);
            it->val += (cmd->event == MENU_EVENT_UP) ? delta : -delta;
            if (it->val < it->min) it->val = it->min;
            if (it->val > it->max) it->val = it->max;
        }
        it->dirty = true;

        /* 仅值真的变化才回调: 已到上/下限后继续转不会再改值, 此时回调"实时生效"
         * 型页面(如亮度)只是把同一个值反复写一遍 —— 白费一次 I2C 事务. */
        if (it->on_change && it->val != old)
            it->on_change(it);
        break;
    }

    case MENU_EVENT_CLICK:
    case MENU_EVENT_LPRESS:
        return MENU_RET_DONE;   /* 直接退出(退出回调 store 落盘) */

    default:
        break;
    }
    return MENU_RET_OK;
}

param_exit_result_t param_exit(param_t *it)
{
    param_exit_result_t r = PARAM_EXIT_UNCHANGED;

    if (it->dirty && it->store && it->val != it->base)
    {
        it->store(it);

        if (Config_Save())
            r = PARAM_EXIT_SAVED;
        else
        {
            /* 落盘失败: 回滚 RAM 副本, 让 UI/运行时状态与 Flash 保持一致.
             * 不回滚的话, 重进页面 load() 读到的是新值且 val==base, 于是表现成
             * "已经保存了"而 Flash 仍是旧值, 重启才回到旧值 —— 正是最误导人的
             * 静默数据丢失. 回滚后用户立刻能看到设置并未生效.
             * on_change 需一并重放: "实时生效"型效果(如亮度)由它施加, 不重放会
             * 让实际效果停留在未保存的新值上, 与回滚后的 RAM 值不一致. */
            it->val = it->base;
            it->store(it);
            if (it->on_change)
                it->on_change(it);
            r = PARAM_EXIT_SAVE_FAILED;
        }
    }
    it->dirty = false;
    return r;
}

void param_exit_notify(param_t *it)
{
    param_exit_result_t r = param_exit(it);

    if (r == PARAM_EXIT_SAVED)
        Toast_Saved();
    else if (r == PARAM_EXIT_SAVE_FAILED)
        Buzzer_Beep(BUZZER_EVENT_ALARM);
    /* PARAM_EXIT_UNCHANGED: 未改动, 静默 */
}
