#ifndef __PARAM_H
#define __PARAM_H

#include "menu.h"
#include <stdint.h>
#include <stdbool.h>

/* ==================== 通用单参数页模板 ====================
 * 供"单值直接可调"页面(蜂鸣/亮度/温度)复用.
 *
 * **结构拆成两半, 目的是省 RAM**:
 *   - param_cfg_t  不可变配置(范围/步进/回调) —— 各页以 const 实例定义, 落在
 *                  Flash(RO), 不占 RAM;
 *   - param_t      可变编辑状态(val/base/dirty) —— 唯一占 RAM 的部分, 12 B/个.
 * 原实现把两者塞在同一个结构里, 6 个实例各 28 B 且全落 .data, 共 172 B;
 * 拆分后 RAM 部分只剩 12 B/个(详见 param.c 顶部说明).
 * (休眠时间/休眠温度/自动关机三项曾用本模板, 2026-10 合并为"休眠设置"
 *  三参一页后改为自建状态机, 现仅蜂鸣/亮度/温度三处使用.)
 *
 * 每页维护"一份 const cfg + 一份 RAM 状态", 页面以薄闭包绑进 ops 的
 * enter/handler/exit —— 三个回调都带 void *user_data, 与 menu_ops_t 的签名一致
 * (照抄时别漏掉该参数):
 *     static const param_cfg_t xxx_cfg = {
 *         .min = 0, .max = 60, .step = 1, .load = xxx_load, .store = xxx_store,
 *     };
 *     static param_t xxx = { .cfg = &xxx_cfg };
 *
 *     static void          xxx_enter(void *ud){ (void)ud; param_enter(&xxx); }
 *     static menu_result_t xxx_handler(const menu_cmd_t *cmd, void *ud){ (void)ud; return param_handler(cmd, &xxx); }
 *     static void          xxx_exit(void *ud){ (void)ud; param_exit_notify(&xxx); }
 * display 为每页私有布局, 直接读 xxx.val.
 *
 * 编辑缓冲为 param_t 的 val 字段, 回调经参数取实例, 无运行期指针校验:
 *   load(enter) 写缓冲 → 修改缓冲 → store(exit) 落盘. */
typedef struct param_s param_t;

/* 不可变配置: 范围/步进/回调. 无运行时变化时一律用 const 实例(存 Flash). */
typedef struct {
    int16_t  min, max, step;   /* 范围/步进 (数值型) */
    uint8_t  toggle;           /* 1=开关型(UP/DOWN 在 min/max 间翻转), 0=数值型 */
    void   (*load)(param_t *it);      /* enter: 配置字段 → 编辑缓冲 */
    void   (*store)(param_t *it);     /* 落盘: 编辑缓冲 → 配置字段 */
    void   (*on_change)(param_t *it); /* 值变化实时生效回调(可选) */
} param_cfg_t;

/* 可变编辑状态: 唯一占 RAM 的部分(12 B). */
struct param_s {
    const param_cfg_t *cfg;    /* 指向不可变配置(通常为 Flash 中的 const 实例) */
    int16_t base;              /* enter 时的编辑缓冲初值, exit 与终值比对决定是否落盘 */
    int16_t val;               /* 当前编辑缓冲 */
    bool    dirty;             /* 编辑期间是否被修改 */
};

/* param_exit 的结果. 必须区分"未改动"与"改动但落盘失败":
 * 前者应静默, 后者必须给用户反馈 —— 否则就是最误导人的静默数据丢失.
 * ⚠️ 切勿当布尔用: SAVED=1 与 SAVE_FAILED=2 **都为真**, 写成 if (param_exit(&x))
 *    会把"落盘失败"当成成功(弹"已保存"却什么都没写). 请显式比较, 或用
 *    param_exit_notify(). */
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
