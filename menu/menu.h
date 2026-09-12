#ifndef __MENU_H
#define __MENU_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* 结构字段的"空值"哨兵 —— 统一用 0 表示"无"。
 * 0 兼作占位虚拟根(索引0)的自身索引: 虚拟根永不成为当前活动节点, 故无歧义. */
#define MENU_NO_PARENT 0U    /* parent: 无父(仅占位虚拟根, parent 也指向自身 0) */
#define MENU_NO_CHILD  0U    /* first_child: 叶子项无子项 */
#define MENU_INVALID   0U    /* 内部状态哨兵: 无有效序号(渲染层 last_first/prev_index) */
#define MENU_HOME_ID 1U      /* HOME 位于索引 0 的占位虚拟根之下 */
#define MENU_LEAF    0U       /* child_num: 叶子项无子项个数 */

/* 节点多功能标志(替代原 jump_id 的"目标ID"语义, 其余位预留) */
#define MENU_FLAG_DIRECT_HANDLER  0x01U   /* 常驻页: 进入/返回时直接激活为 handler(如 HOME) */
#define MENU_FLAG_PASSIVE_VIEW    0x08U   /* 只读信息页: 任意事件(CLICK/LPRESS)一律退出, 无需 handler 样板 */

typedef enum{
    MENU_EVENT_UP=0,
    MENU_EVENT_DOWN,
    MENU_EVENT_CLICK,
    MENU_EVENT_LPRESS,
}menu_event_t;

typedef struct{
	menu_event_t event;
	int16_t step;
}menu_cmd_t;

typedef enum{
    MENU_STATE_NORMAL=0,
    MENU_STATE_HANDLER,
}menu_state_t;

typedef enum{
    MENU_RET_OK=0,          /* 继续处理, 留在本页 */
    MENU_RET_DONE,          /* 结束本页; 离场方向由节点结构(child_num)推导: 有子项→下钻, 否则→回父列表 */
}menu_result_t;

typedef uint16_t menu_period_t;

/* 页面回调: 内核经 Menu_GetUserData(item) 注入业务上下文(user_data).
 * 表内显式填了 .user_data 则用之, 否则默认指向节点自身. 回调可忽略. */
typedef struct{
    void (*enter)(void *user_data);
    menu_result_t (*handler)(const menu_cmd_t *cmd, void *user_data);
    menu_period_t (*display)(void *user_data);
    void (*exit)(void *user_data);
}menu_ops_t;


typedef struct{
    uint8_t parent;
    uint8_t first_child;
    uint8_t child_num;
	uint8_t flags;
    const char *name;
	void *user_data;
    const menu_ops_t *ops;
}menu_item_t;


extern const menu_item_t menu_table[];
extern const uint8_t menu_max_num;

/* 取节点回调的业务上下文: 显式填 .user_data 用之, 否则默认指向节点自身 */
void *Menu_GetUserData(const menu_item_t *item);


void Menu_Init(void);
void Menu_HandleEvent(const menu_cmd_t *cmd);


uint8_t Menu_GetCurrentId(void);
menu_state_t Menu_GetState(void);

const menu_item_t *Menu_GetItem(uint8_t id);
const menu_item_t *Menu_GetCurrent(void);

bool Menu_GetSiblingRange(uint8_t id,uint8_t *first,uint8_t *last);


#endif