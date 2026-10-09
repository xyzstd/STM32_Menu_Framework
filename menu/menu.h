#ifndef __MENU_H
#define __MENU_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define MENU_NO_PARENT 0U    /* parent: 无父(仅占位虚拟根, parent 也指向自身 0) */
#define MENU_NO_CHILD  0U    /* first_child: 叶子项无子项 */
#define MENU_INVALID   0U    /* 内部状态哨兵: 无有效序号(渲染层 last_first/prev_index) */
#define MENU_HOME_ID 1U      /* HOME 位于索引 0 的占位虚拟根之下 */
#define MENU_LEAF    0U       /* child_num: 叶子项无子项个数 */

#define MENU_FLAG_DIRECT_HANDLER  0x01U   /* 常驻页: 进入/返回时直接激活为 handler(如 HOME) */

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

void *Menu_GetUserData(const menu_item_t *item);


void Menu_Init(void);
void Menu_EventDispatch(const menu_cmd_t *cmd);


uint8_t Menu_GetCurrentId(void);
menu_state_t Menu_GetState(void);

/* 强制跳转到指定节点(设置当前 id):
 * 离开当前页面(若有 HANDLER 态则调 exit 回调) → 定位到目标节点;
 * 目标为可激活页面(常驻页 DIRECT_HANDLER, 或带 display+handler 的叶子页如
 * Cur Cal 引导页)则直接激活进 handler, 否则落在其父列表。
 * 用于异常处理: 任意页面发生锁存故障时回到常驻页 HOME 统一处理;
 * 亦用于首次冷态标定引导自动进入 Cur Cal 页。 */
void Menu_SetCurrentId(uint8_t id);

const menu_item_t *Menu_GetItem(uint8_t id);

bool Menu_GetSiblingRange(uint8_t id,uint8_t *first,uint8_t *last);

#endif
