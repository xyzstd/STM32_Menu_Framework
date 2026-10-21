#include "menu.h"
#include "home.h"
#include "temp.h"
#include "pid.h"
#include "calibrate.h"
#include "settings.h"
#include "ota_wait.h"
#include "handle.h"
#include "autotune.h"
#include "cur_cal.h"

const menu_item_t menu_table[] =
{
    [0]={
        .parent      = MENU_NO_PARENT,
        .first_child = 1,
        .child_num   = 1,
        .ops         = NULL,
    },

    [1]={
        .parent      = 0,
        .first_child = 2,
        .child_num   = 6,
        .flags       = MENU_FLAG_DIRECT_HANDLER,
        .ops         = &home_ops,
        .name        = "首页",
    },

    [2]={
        .parent      = 1,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        .ops         = &temp_ops,
        .name        = "温度设置",
    },

    [3]={
        .parent      = 1,
        .first_child = 12,
        .child_num   = 3,
        .ops         = NULL,
        .name        = "PID设置",
    },

    [4]={
        .parent      = 1,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        .ops         = &calibrate_ops,
        .name        = "温度校准",
    },

    [5]={
        .parent      = 1,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        .ops         = &handle_ops,
        .name        = "手柄型号",
    },

    [6]={
        .parent      = 1,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,

        .ops         = &ota_wait_ops,
        .name        = "固件升级",
    },

    [7]={
        .parent      = 1,
        .first_child = 8,
        .child_num   = 4,
        .ops         = NULL,
        .name        = "系统设置",
    },

    [8]={
        .parent      = 7,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,

        .ops         = &sleep_set_ops,
        .name        = "休眠设置",
    },

    [9]={
        .parent      = 7,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,

        .ops         = &func_set_ops,
        .name        = "功能设置",
    },

    [10]={
        .parent      = 7,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        .ops         = &reset_ops,
        .name        = "恢复默认",
    },

    [11]={
        .parent      = 7,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,

        .ops         = &version_ops,
        .name        = "版本信息",
    },

    [12]={
        .parent      = 3,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        .ops         = &pid_ops,
        .name        = "PID参数",
    },

    [13]={
        .parent      = 3,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        .ops         = &autotune_ops,
        .name        = "自动整定",
    },

    [14]={
        .parent      = 3,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        .ops         = &cur_cal_ops,
        .name        = "电流标定",
    },
};

const uint8_t menu_max_num = sizeof(menu_table) / sizeof(menu_table[0]);

#define MENU_TABLE_COUNT (sizeof(menu_table) / sizeof(menu_table[0]))

typedef char menu_table_count_guard[(MENU_TABLE_COUNT <= 255u) ? 1 : -1];
typedef char menu_table_home_guard[(MENU_TABLE_COUNT > MENU_HOME_ID) ? 1 : -1];
