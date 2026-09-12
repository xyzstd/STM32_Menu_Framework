#include "menu.h"
#include "home.h"
#include "temp.h"
#include "pid.h"
#include "calibrate.h"
#include "settings.h"
#include "ota_wait.h"
#include "handle.h"

/*
 * 菜单表: 索引(菜单数据表)
 *
 * ID  0 = 虚拟根  (占位容器, MENU_NO_PARENT; 唯一子项即 HOME: first_child=1, child_num=1)
 * ID  1 = HOME    (常驻页, parent=0; 顶层列表: first_child=2, child_num=6)
 * ID  2 = 温度设置
 * ID  3 = PID参数 (子菜单容器, first_child=14, child_num=2)
 * ID  4 = 温度校准
 * ID  5 = 手柄型号
 * ID  6 = 固件升级
 * ID  7 = 系统设置 (子菜单容器, first_child=8, child_num=6)
 * ID  8-13 = 系统设置子菜单
 * ID 14-15 = PID子菜单
 *
 * 编码: 本文件为 UTF-8, 中文名与 bsp/oled_data.c 字库 Index(UTF-8) 逐字节匹配.
 */
const menu_item_t menu_table[] =
{
    /* ========== 占位虚拟根: 给常驻页(HOME)提供真实父级, 消除对"无父"的特判 ========== */
    [0]={
        .parent      = MENU_NO_PARENT,
        .first_child = 1,           /* 唯一子项: HOME */
        .child_num   = 1,
        .ops         = NULL,        /* 占位容器, 永不显示/激活 */
    },

    [1]={
        .parent      = 0,           /* 上层为虚拟根 */
        .first_child = 2,           /* 顶层列表: 温度设置为首项 */
        .child_num   = 6,
        .flags       = MENU_FLAG_DIRECT_HANDLER,   /* 常驻页: 上电即激活, 列表由 handler 内 CLICK 进入 */
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
        .first_child = 14,
        .child_num   = 2,
        .ops         = NULL,
        .name        = "PID参数",
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
        .flags       = MENU_FLAG_PASSIVE_VIEW,   /* 只读页: CLICK/LPRESS 一律退出, 无需 handler */
        .ops         = &ota_wait_ops,
        .name        = "固件升级",
    },

    [7]={
        .parent      = 1,
        .first_child = 8,
        .child_num   = 6,
        .ops         = NULL,
        .name        = "系统设置",
    },

    /* ========== 系统设置子菜单 ========== */

    [8]={
        .parent      = 7,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        .ops         = &sleep_time_ops,
        .name        = "休眠时间",
    },

    [9]={
        .parent      = 7,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        .ops         = &shutdown_ops,
        .name        = "自动关机",
    },

    [10]={
        .parent      = 7,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        .ops         = &buzzer_ops,
        .name        = "蜂鸣器",
    },

    [11]={
        .parent      = 7,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        .ops         = &brightness_ops,
        .name        = "屏幕亮度",
    },

    [12]={
        .parent      = 7,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        .ops         = &reset_ops,
        .name        = "恢复默认",
    },

    [13]={
        .parent      = 7,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        .flags       = MENU_FLAG_PASSIVE_VIEW,   /* 只读信息页: 任意事件即退 */
        .ops         = &version_ops,
        .name        = "版本信息",
    },

    /* ========== PID参数子菜单 ========== */

    [14]={
        .parent      = 3,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        .ops         = &pid_agg_ops,       /* 激进参数页: enter/exit 薄闭包绑定静态实例 */
        .name        = "Fast PID",
    },

    [15]={
        .parent      = 3,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        .ops         = &pid_cons_ops,      /* 保守参数页 */
        .name        = "Slow PID",
    },
};

const uint8_t menu_max_num = sizeof(menu_table) / sizeof(menu_table[0]);