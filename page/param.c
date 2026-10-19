#include "param.h"
#include "config.h"
#include "buzzer.h"
#include "toast.h"

/* 本模块的 RAM 优化: param_t 只保留可变状态(val/base/dirty, 12 B), 不可变的
 * 范围/步进/回调移进 param_cfg_t 由各页以 const 实例定义(存 Flash).
 * 原实现把两者放同一结构, 6 个实例各 28 B 且全落 .data, 共 172 B; 现为 12 B/个.
 * 代价是每次访问范围/回调多一次指针解引用(下面统一先取 c = it->cfg). */

void param_enter(param_t *it)
{
    const param_cfg_t *c = it->cfg;

    it->dirty = false;
    if (c->load) c->load(it);

    /* 装载即钳位: 配置字段可能落在 [min,max] 之外, 而 param_handler 只在
     * 首次 UP/DOWN 时才钳位 —— 不钳的话进入页面会显示超范围值, 且用户看到的
     * 初值是错的. 可达场景: default_temp 全局共享而 temp_max[] 按型号 ——
     * 若某型号上限被调低, work_task 只钳了 runtime_temp, default_temp 仍可能越界. */
    if (it->val < c->min) it->val = c->min;
    if (it->val > c->max) it->val = c->max;

    it->base = it->val;   /* 记住进入初值(钳位后), 供 exit 判定是否真改动 */
}

menu_result_t param_handler(const menu_cmd_t *cmd, param_t *it)
{
    const param_cfg_t *c = it->cfg;

    switch(cmd->event)
    {
    case MENU_EVENT_UP:
    case MENU_EVENT_DOWN:
    {
        int16_t old = it->val;

        if (c->toggle)
            it->val = (it->val == c->min) ? c->max : c->min;
        else
        {
            int16_t delta = (int16_t)(c->step * cmd->step);
            it->val += (cmd->event == MENU_EVENT_UP) ? delta : -delta;
            if (it->val < c->min) it->val = c->min;
            if (it->val > c->max) it->val = c->max;
        }
        it->dirty = true;

        /* 仅值真的变化才回调: 已到上/下限后继续转不会再改值, 此时回调"实时生效"
         * 型页面(如亮度)只是把同一个值反复写一遍 —— 白费一次 I2C 事务. */
        if (c->on_change && it->val != old)
            c->on_change(it);
        break;
    }

    case MENU_EVENT_CLICK:
    case MENU_EVENT_LPRESS:
        return MENU_RET_DONE;   /* 直接退出(退出回调 store 落盘) */

    default:
        break;
    }
    return MENU_RET_STAY;
}

param_exit_result_t param_exit(param_t *it)
{
    const param_cfg_t *c = it->cfg;
    param_exit_result_t r = PARAM_EXIT_UNCHANGED;

    if (it->dirty && c->store && it->val != it->base)
    {
        c->store(it);

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
            c->store(it);
            if (c->on_change)
                c->on_change(it);
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
