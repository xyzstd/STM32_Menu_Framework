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
 * ID  3 = PID参数 (子菜单容器, first_child=12, child_num=4)
 * ID  4 = 温度校准
 * ID  5 = 手柄型号
 * ID  6 = 固件升级
 * ID  7 = 系统设置 (子菜单容器, first_child=8, child_num=4)
 * ID  8 = 休眠设置 (多参页: 休眠时间/休眠温度/自动关机 合并在同一页)
 * ID  9 = 功能设置 (多参页: 蜂鸣器/屏幕亮度 合并在同一页)
 * ID 10-11 = 系统设置其余子项
 * ID 12-15 = PID子菜单
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
        .first_child = 12,
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
        .child_num   = 4,
        .ops         = NULL,
        .name        = "系统设置",
    },

    /* ========== 系统设置子菜单 ========== */

    [8]={
        .parent      = 7,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        /* 三参一页(休眠时间/休眠温度/自动关机): 三者存在隐藏耦合(关机时间必须
         * 晚于休眠时间, 否则休眠永不触发), 分页时用户无法察觉, 故合并同屏并
         * 在冲突时给出诊断. 带 ops 的叶子项在列表里显示右侧三角 ▶(与 PID 多参页
         * 一致); 左侧靶心圆点所有行都有, 并非本项特有. */
        .ops         = &sleep_set_ops,
        .name        = "休眠设置",
    },

    [9]={
        .parent      = 7,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        /* 两参一页(蜂鸣器/屏幕亮度): 原为两页, 2026-10 合并为多参调节页,
         * 与"休眠设置"同一形态(导航态选字段 → CLICK 进调参 → LPRESS 退出).
         * 亮度在调参时即时预览, 页面底部有指示条. */
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
        /* 只读信息页: 退出手势由 version_handler 裁决(任意 CLICK/LPRESS 即退) */
        .ops         = &version_ops,
        .name        = "版本信息",
    },

    /* ========== PID参数子菜单 ========== */

    [12]={
        .parent      = 3,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        .ops         = &pid_agg_ops,       /* 激进参数页(多参: P/I/D): 归可下探类, 空心圆+三角 */
        .name        = "快速PID",
    },

    [13]={
        .parent      = 3,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        .ops         = &pid_cons_ops,      /* 保守参数页(多参): 可下探类 */
        .name        = "慢速PID",
    },

    [14]={
        .parent      = 3,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        .ops         = &autotune_ops,      /* 继电自整定: 自动写入 Slow PID 组 */
        .name        = "自动整定",
    },

    [15]={
        .parent      = 3,
        .first_child = MENU_NO_CHILD,
        .child_num   = MENU_LEAF,
        .ops         = &cur_cal_ops,       /* 电流标定: 实测各型号冷态脉冲电流参考值 */
        .name        = "电流标定",
    },
};

const uint8_t menu_max_num = sizeof(menu_table) / sizeof(menu_table[0]);

/* ==================== 编译期守卫(能力范围内的那部分) ====================
 * 索引类型是 uint8_t, 项数超过 255 会被静默截断 —— 这类"由 sizeof 得出的量"在
 * C99 里是整数常量表达式, 可以用负数组长度手法断言(AC5 不认 C11 _Static_assert).
 *
 * 表内 parent/child 关系**无法在此断言**: C99 下 const 数组成员不是整数常量表达式,
 * menu_table[3].first_child + menu_table[3].child_num 不能用于数组长度或 case 标签.
 * 那部分由构建期脚本 build_tools/check_menu_table.py 静态解析本文件覆盖(改表即报,
 * 不必上机); 内核的 menu_validate_table 则作为现场运行期兜底保留. */
#define MENU_TABLE_COUNT (sizeof(menu_table) / sizeof(menu_table[0]))

typedef char menu_table_count_guard[(MENU_TABLE_COUNT <= 255u) ? 1 : -1];
typedef char menu_table_home_guard[(MENU_TABLE_COUNT > MENU_HOME_ID) ? 1 : -1];
