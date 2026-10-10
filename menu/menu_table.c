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

/*
 * 菜单表: 索引(菜单数据表)
 *
 * ID  0 = 虚拟根  (占位容器, MENU_NO_PARENT; 唯一子项即 HOME: first_child=1, child_num=1)
 * ID  1 = HOME    (常驻页, parent=0; 顶层列表: first_child=2, child_num=6)
 * ID  2 = 温度设置
 * ID  3 = PID参数 (子菜单容器, first_child=15, child_num=4)
 * ID  4 = 温度校准
 * ID  5 = 手柄型号
 * ID  6 = 固件升级
 * ID  7 = 系统设置 (子菜单容器, first_child=8, child_num=7)
 * ID  8-14 = 系统设置子菜单
 * ID 15-18 = PID子菜单
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
        .first_child = 15,
        .child_num   = 4,
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
        /* 本页要"停留驻留"等待上位机(display 里每 250ms 轮询 OTA_GetAppRequest()
         * 并播等待动画): 若内核在任意 CLICK/LPRESS 上立即退出, 用户就无法停在
         * 本页等握手 → 升级页形同虚设. 故退出手势由页面 handler 自行裁决
         * (只有长按才退出, 见 ota_wait.c). */
        .ops         = &ota_wait_ops,
        .name        = "固件升级",
    },

    [7]={
        .parent      = 1,
        .first_child = 8,
        .child_num   = 7,
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
        .ops         = &sleep_temp_ops,
        .name        = "休眠温度",
    },

    [10]={
        .parent      = 7,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        .ops         = &shutdown_ops,
        .name        = "自动关机",
    },

    [11]={
        .parent      = 7,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        .ops         = &buzzer_ops,
        .name        = "蜂鸣器",
    },

    [12]={
        .parent      = 7,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        .ops         = &brightness_ops,
        .name        = "屏幕亮度",
    },

    [13]={
        .parent      = 7,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        .ops         = &reset_ops,
        .name        = "恢复默认",
    },

    [14]={
        .parent      = 7,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        /* 只读信息页: 退出手势由 version_handler 裁决(任意 CLICK/LPRESS 即退) */
        .ops         = &version_ops,
        .name        = "版本信息",
    },

    /* ========== PID参数子菜单 ========== */

    [15]={
        .parent      = 3,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        .ops         = &pid_agg_ops,       /* 激进参数页(多参: P/I/D): 归可下探类, 空心圆+三角 */
        .name        = "快速PID",
    },

    [16]={
        .parent      = 3,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        .ops         = &pid_cons_ops,      /* 保守参数页(多参): 可下探类 */
        .name        = "慢速PID",
    },

    [17]={
        .parent      = 3,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        .ops         = &autotune_ops,      /* 继电自整定: 自动写入 Slow PID 组 */
        .name        = "自动整定",
    },

    [18]={
        .parent      = 3,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        .ops         = &cur_cal_ops,       /* 电流标定: 实测各型号冷态脉冲电流参考值 */
        .name        = "电流标定",
    },
};

const uint8_t menu_max_num = sizeof(menu_table) / sizeof(menu_table[0]);
