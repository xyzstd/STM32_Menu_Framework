#ifndef __PARAM_H
#define __PARAM_H

#include "menu.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== 通用单参数页模板 ====================
 * 供"单值直接可调"页面(休眠/关机/蜂鸣/亮度/温度)复用.
 *
 * 每页维护一份静态 param_t 实例 (范围/步进/读写回调/on_change + 编辑缓冲),
 * 页面以薄闭包把实例绑进 ops 的 enter/handler/exit:
 *     static void      xxx_enter(void){ param_enter(&xxx); }
 *     static menu_result_t xxx_handler(const menu_cmd_t *cmd){ return param_handler(cmd, &xxx); }
 *     static void      xxx_exit(void){ param_exit(&xxx); }
 * display 为每页私有布局, 直接读 xxx.val.
 *
 * 编辑缓冲为实例的 val 字段, 回调经参数取实例, 无运行期指针校验:
 *   load(enter) 写缓冲 → 修改缓冲 → store(exit) 落盘. */
typedef struct param_s param_t;

struct param_s {
    int16_t  min, max, step;   /* 范围/步进 (数值型) */
    uint8_t  toggle;           /* 1=开关型(UP/DOWN 在 min/max 间翻转), 0=数值型 */
    void   (*load)(param_t *it);      /* enter: 配置字段 → 编辑缓冲 */
    void   (*store)(param_t *it);     /* 落盘: 编辑缓冲 → 配置字段 */
    void   (*on_change)(param_t *it); /* 值变化实时生效回调(可选) */

    int16_t base;              /* enter 时的编辑缓冲初值, exit 与终值比对决定是否落盘 */
    int16_t val;               /* 当前编辑缓冲 */
    bool    dirty;             /* 编辑期间是否被修改 */
};

void          param_enter(param_t *it);
menu_result_t param_handler(const menu_cmd_t *cmd, param_t *it);
void          param_exit(param_t *it);

#endif