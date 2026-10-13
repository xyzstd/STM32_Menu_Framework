#ifndef __PARAM_H
#define __PARAM_H

#include "menu.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== 通用单参数页模板 ====================
 * 供"单值直接可调"页面(休眠/关机/蜂鸣/亮度/温度)复用.
 *
 * 每页维护一份静态 param_t 实例 (范围/步进/读写回调/on_change + 编辑缓冲),
 * 页面以薄闭包把实例绑进 ops 的 enter/handler/exit —— 三个回调都带
 * void *user_data, 与 menu_ops_t 的签名一致(照抄时别漏掉该参数):
 *     static void          xxx_enter(void *ud){ (void)ud; param_enter(&xxx); }
 *     static menu_result_t xxx_handler(const menu_cmd_t *cmd, void *ud){ (void)ud; return param_handler(cmd, &xxx); }
 *     static void          xxx_exit(void *ud){ (void)ud; param_exit_notify(&xxx); }
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

/* param_exit 的结果. 必须区分"未改动"与"改动但落盘失败":
 * 前者应静默, 后者必须给用户反馈 —— 否则就是最误导人的静默数据丢失. */
typedef enum {
    PARAM_EXIT_UNCHANGED = 0,   /* 未改动(或改回了初值), 无需落盘 */
    PARAM_EXIT_SAVED,           /* 有改动且已成功落盘 */
    PARAM_EXIT_SAVE_FAILED,     /* 有改动但落盘失败(RAM 副本已回滚) */
} param_exit_result_t;

void                param_enter(param_t *it);
menu_result_t       param_handler(const menu_cmd_t *cmd, param_t *it);
param_exit_result_t param_exit(param_t *it);

/* 退出并给出统一反馈: 已保存→弹"已保存"框; 落盘失败→蜂鸣; 未改动→静默.
 * 各页统一调用本函数, 避免出现"成功弹框、失败却毫无提示"的不一致. */
void                param_exit_notify(param_t *it);

#endif
