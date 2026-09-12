#include "param.h"
#include "config.h"

void param_enter(param_t *it)
{
    it->dirty = false;
    if (it->load) it->load(it);
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

void param_exit(param_t *it)
{
    if (it->dirty && it->store && it->val != it->base)
    {
        it->store(it);
        Config_Save();
    }
    it->dirty = false;
}