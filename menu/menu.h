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

/* ==================== 表配置错误的发现手段 ====================
 * 菜单表配错(索引 0 不是占位虚拟根 / HOME 不在索引 1 / parent-child 不一致等)时,
 * menu_validate_table 会失败, 内核**静默降级**(保持虚拟根+列表态, 所有导航入口
 * 停用) —— 现象只有"上电后按键完全无反应", 没有日志也没有提示. 三层手段按"越早
 * 发现越好"排列:
 *   1. 构建期: build_tools/check_menu_table.py 解析本工程的 menu_table.c 做同样
 *      的校验并直接报错 —— 首选, 改表即知, 不必上机.
 *   2. 编译期: menu_table.c 里的 MENU_TABLE_SIZE_GUARD(仅能查"项数不超索引宽度"
 *      这类由 sizeof 得出的量; C99 下 const 数组成员不是整数常量表达式, 表内
 *      parent/child 关系无法静态断言).
 *   3. 运行期: 内核仍保留 menu_validate_table 作为现场兜底(表在 Flash, 理论上不会
 *      变, 但这是"不因配错而崩"的最后一道), 并可用下面的 MENU_DEBUG_ASSERT 在
 *      调试期立刻停住.
 * 内核不关心应用是否启用了看门狗等外部设施 —— 陷阱动作默认只是自旋. */

/* 调试断言开关: 定义 MENU_DEBUG_ASSERT=1 时, Menu_Init 内校验失败即执行
 * MENU_ASSERT_TRAP() 停住(便于挂调试器定位); 默认 0 完全编译掉. */
#ifndef MENU_DEBUG_ASSERT
#define MENU_DEBUG_ASSERT 0
#endif

/* 断言失败时的陷阱动作: 默认纯自旋 —— 停在此处等调试器中断, 不引用任何外部符号,
 * 也不关心应用是否有看门狗之类的周期性维护需求. 若你的环境需要(例如必须喂狗才
 * 能停住), 可在包含本头文件前重定义本宏注入自己的实现; 内核保持零外部依赖. */
#ifndef MENU_ASSERT_TRAP
#define MENU_ASSERT_TRAP() \
    do { volatile uint32_t menu_trap_spin_ = 0; while (!menu_trap_spin_) { } } while (0)
#endif

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

/* 菜单表是否通过校验(等价于 menu_validate_table 的结果).
 * 为 false 时内核处于"虚拟根+列表态": 所有导航入口停用, 事件被丢弃.
 * 应用可在启动后查询本函数并显示自检错误, 避免用户面对"按键完全无反应"
 * 却无从判断原因. 与 MENU_DEBUG_ASSERT 是互补手段: 前者给现场/量产,
 * 后者给调试期立刻停住. */
bool Menu_IsValid(void);

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
