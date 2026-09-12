#include "menu.h"

typedef struct{
    uint8_t current;
    menu_state_t state;
}menu_ctrl_t;

static menu_ctrl_t menu_ctrl;

static const menu_item_t *menu_current(void);

static void menu_activate(void);
static void menu_exit_handler(void);

static void menu_state_normal(const menu_cmd_t *cmd);
static void menu_state_handler(const menu_cmd_t *cmd);

static void menu_move_up(void);
static void menu_move_down(void);
static void menu_enter(void);
static void menu_back(void);


void Menu_Init(void)
{
    menu_ctrl.current=MENU_HOME_ID;
    menu_ctrl.state=MENU_STATE_NORMAL;
    menu_activate();   /* 上电即激活常驻页(HOME)进入 handler */
}


void *Menu_GetUserData(const menu_item_t *item)
{
    /* 表内显式填了 user_data 用之, 否则默认指向节点自身,
     * 使回调无需在表中填写也能获得非空且可用的上下文 */
    return (item->user_data) ? item->user_data : (void *)item;
}


void Menu_HandleEvent(const menu_cmd_t *cmd)
{
    if(!cmd)
        return;

    if(menu_ctrl.state==MENU_STATE_HANDLER)
        menu_state_handler(cmd);
    else
        menu_state_normal(cmd);
}


uint8_t Menu_GetCurrentId(void)
{
    return menu_ctrl.current;
}


menu_state_t Menu_GetState(void)
{
    return menu_ctrl.state;
}


const menu_item_t *Menu_GetItem(uint8_t id)
{
    if(id>=menu_max_num)
        return NULL;

    return &menu_table[id];
}


const menu_item_t *Menu_GetCurrent(void)
{
    return menu_current();
}


static const menu_item_t *menu_current(void)
{
    if(menu_ctrl.current>=menu_max_num)
        return NULL;

    return &menu_table[menu_ctrl.current];
}


/* 激活当前节点为处理页: 切 HANDLER + 调 enter。
 * 不修改 current, 不做事件路由 —— 事件统一由 menu_state_handler 分发。 */
static void menu_activate(void)
{
    const menu_item_t *item=menu_current();

    if(!item||!item->ops)
        return;

    if(!item->ops->display)
    {
        menu_ctrl.state=MENU_STATE_NORMAL;
        return;
    }

    menu_ctrl.state=MENU_STATE_HANDLER;

    if(item->ops->enter)
        item->ops->enter(Menu_GetUserData(item));
}


static void menu_exit_handler(void)
{
    const menu_item_t *item=menu_current();

    if(item&&item->ops&&item->ops->exit)
        item->ops->exit(Menu_GetUserData(item));

    menu_ctrl.state=MENU_STATE_NORMAL;
}


static void menu_state_handler(const menu_cmd_t *cmd)
{
    const menu_item_t *item=menu_current();
    menu_result_t ret=MENU_RET_OK;
    bool has_children;   /* 是否下钻子列表: 先取, 避免依赖"退出 handler 后 current 不变"的隐式约定 */

    if(!item||!item->ops)
    {
        menu_ctrl.state=MENU_STATE_NORMAL;
        return;
    }

    /* 只读信息页: 内核统一裁决 —— CLICK/LPRESS 退出, UP/DOWN 忽略(页面无内容可滚)。
     * 契约: 该标志仅用于叶子项(child_num==0), 无子列表下钻语义, 故直接退出回父列表. */
    if(item->flags & MENU_FLAG_PASSIVE_VIEW)
    {
        if(cmd->event==MENU_EVENT_CLICK || cmd->event==MENU_EVENT_LPRESS)
            menu_exit_handler();
        return;
    }

    switch(cmd->event)
    {
        case MENU_EVENT_UP:
        case MENU_EVENT_DOWN:
        case MENU_EVENT_CLICK:
            if(item->ops->handler)
                ret=item->ops->handler(cmd, Menu_GetUserData(item));
            break;

        case MENU_EVENT_LPRESS:
            if(item->ops->handler)
                ret=item->ops->handler(cmd, Menu_GetUserData(item));
            else
                menu_exit_handler();
            break;

        default:
            break;
    }

    has_children = (item->child_num > 0);

    if(ret==MENU_RET_DONE)
    {
        menu_exit_handler();

        if(has_children)
            menu_enter();   /* 常驻页(如 HOME): 下钻到子列表首项; 叶子则已回父列表 */
    }
}


static void menu_state_normal(const menu_cmd_t *cmd)
{
    switch(cmd->event)
    {
        case MENU_EVENT_UP:
            menu_move_up();
            break;

        case MENU_EVENT_DOWN:
            menu_move_down();
            break;

        case MENU_EVENT_CLICK:
            menu_enter();
            break;

        case MENU_EVENT_LPRESS:
            menu_back();   
            break;

        default:
            break;
    }
}


static void menu_enter(void)
{
    const menu_item_t *item=menu_current();

    if(!item)
        return;

    if(item->child_num)
    {
        /* 进入子菜单: 落在首项; 防御菜单表配置, 校验半开区间 [first_child, end) 不越界,
         * 与 Menu_GetSiblingRange 的判据保持一致, 防误配 child_num 把 current 设为表外节点 */
        uint16_t end = (uint16_t)item->first_child + item->child_num;
        if(item->first_child >= menu_max_num || end > menu_max_num)
            return;
        /* 不变式: 容器下钻后必为列表态。当前两个调用路径(列表 CLICK / 常驻页 DONE)本就
         * 处于 NORMAL, 此处显式落地, 防未来在 HANDLER 态误触容器下钻导致列表被当页面渲染 */
        menu_ctrl.state = MENU_STATE_NORMAL;
        menu_ctrl.current = item->first_child;
    }
    else if(item->ops)
    {
        /* 叶子项: 激活为 handler */
        menu_activate();
    }
}

static void menu_back(void)
{
    const menu_item_t *item=menu_current();
    const menu_item_t *target;

    if(!item)
        return;

    menu_ctrl.state=MENU_STATE_NORMAL;

    /* 防御菜单表 parent 越界配置: 越界则保持原节点(显示层 GetSiblingRange 兜底), 不写入非法 current */
    if(item->parent >= menu_max_num)
        return;

    /* 不变式: NORMAL 态 current 必为真实列表项, 其父级恒为真实节点(非占位虚拟根)。
     * DIRECT_HANDLER 节点(如 HOME)永远处于 handler 态、不会滞留 NORMAL, 故 current 的 parent 不可能为 0。 */
    menu_ctrl.current=item->parent;
    

    /* 回到常驻页(如 HOME): 直接激活为 handler, 保持常驻显示 */
    target=menu_current();
    if(target && (target->flags & MENU_FLAG_DIRECT_HANDLER))
        menu_activate();
}


static void menu_move_up(void)
{
    uint8_t first,last;

    if(!Menu_GetSiblingRange(menu_ctrl.current,&first,&last))
        return;

    if(menu_ctrl.current<=first)
        menu_ctrl.current=last;
    else
        menu_ctrl.current--;
}


static void menu_move_down(void)
{
    uint8_t first,last;

    if(!Menu_GetSiblingRange(menu_ctrl.current,&first,&last))
        return;

    if(menu_ctrl.current>=last)
        menu_ctrl.current=first;
    else
        menu_ctrl.current++;
}


bool Menu_GetSiblingRange(uint8_t id,uint8_t *first,uint8_t *last)
{
    const menu_item_t *item;
    const menu_item_t *parent;
    uint16_t end;

    if(!first||!last)
        return false;

    item=Menu_GetItem(id);

    if(!item)
        return false;

    /* 常驻页(HOME)通过占位虚拟根拥有真实父级(idx 0), 故此处无需对"无父"特判:
     * 唯一的无父节点是虚拟根本身, 其 parent=0xFF 经 Menu_GetItem 必返回 NULL,
     * 由下方 !parent 统一拦截. */
    parent=Menu_GetItem(item->parent);

    /* 父节点必须确实拥有子列表 */
    if(!parent || parent->child_num==0 || parent->first_child==MENU_NO_CHILD)
        return false;

    /* 子列表半开区间 [first, end) */
    end=(uint16_t)parent->first_child + parent->child_num;

    if(end>menu_max_num)
        return false;

    /* 防御: id 必须是该列表成员, 防陈旧 current 产生越界移动 */
    if(id<parent->first_child || id>=end)
        return false;

    *first=parent->first_child;
    *last=(uint8_t)(end-1);

    return true;
}

