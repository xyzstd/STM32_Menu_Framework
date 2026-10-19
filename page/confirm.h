#ifndef __CONFIRM_H
#define __CONFIRM_H

#include "menu.h"
#include <stdbool.h>

/* ==================== 通用 YES/NO 确认页 ====================
 * 内嵌于宿主页面的子状态机, 供"破坏性/重要操作"复用.
 * 布局: 标题 + NO/YES 双选项, 选中项反显.
 *
 * 典型用法 (宿主 handler):
 *   if (cf.active) {
 *       bool ok = false;
 *       menu_result_t r = confirm_handle(cmd, &cf, &ok);
 *       if (ok) { ...执行写操作...; return MENU_RET_DONE; }   // YES 确认
 *       if (r == MENU_RET_DONE) { ...取消处理...; return ...; } // NO / 长按取消
 *       return MENU_RET_STAY;                                    // 仍在确认态
 *   }
 * 显示: confirm_display(&cf, title) 只画内容(不含 Clear/Update, 由宿主统一管理). */
typedef struct {
    bool   active;   /* true=正处于确认态 */
    uint8_t choice;  /* 0=NO, 1=YES */
} confirm_t;

void         confirm_open(confirm_t *c);   /* 进入确认态 (默认停在 NO) */
menu_result_t confirm_handle(const menu_cmd_t *cmd, confirm_t *c, bool *confirmed);
void         confirm_display(const confirm_t *c, const char *title);

#endif
