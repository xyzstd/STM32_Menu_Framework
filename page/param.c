#include "param.h"
#include "config.h"

void param_enter(param_t *it)
{
    it->dirty = false;
    if (it->load) it->load(it);

    /* 装载即钳位: 配置字段可能落在 [min,max] 之外, 而 param_handler 只在
     * 首次 UP/DOWN 时才钳位 —— 不钳的话进入页面会显示超范围值, 且用户看到的
     * 初值是错的. 可达场景: default_temp 全局共享而 temp_max[] 按型号,
     * 在 T245(上限450) 下设 450 后切到 T12(上限400), work_task 只钳了
     * runtime_temp, default_temp 仍是 450. */
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
        if (it->on_change) it->on_change(it);
        break;

    case MENU_EVENT_CLICK:
    case MENU_EVENT_LPRESS:
        return MENU_RET_DONE;   /* 直接退出(退出回调 store 落盘) */

    default:
        break;
    }
    return MENU_RET_OK;
}

bool param_exit(param_t *it)
{
    bool saved = false;

    if (it->dirty && it->store && it->val != it->base)
    {
        it->store(it);
        /* 返回"确实落盘成功"而非"用户改过值": 落盘失败时必须让调用方
         * 拿到 false, 否则页面弹出"已保存"却什么都没写进 Flash —— 正是
         * 最误导人的静默数据丢失. RAM 副本已更新, 用户退出重进再改一次
         * 即可重试落盘. */
        saved = Config_Save();
    }
    it->dirty = false;
    return saved;
}
