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


void Menu_EventDispatch(const menu_cmd_t *cmd)
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


/* 强制跳转(见 menu.h 注释): 仅做"离开当前页 → 定位 → 常驻页激活"三步,
 * 不改动列表滚动/差量渲染等既有逻辑; 渲染层经 GetCurrentId/GetState 自动适配. */
void Menu_SetCurrentId(uint8_t id)
{
    if(id >= menu_max_num)
        return;

    /* 真无操作才提前返回: 仅当"已在目标节点 且 该节点已是活动 handler 态".
     * 旧实现只比 id, 于是调用方若已parked在目标索引(如 CurCal_BootEnter 在
     * current 恰为 Cur Cal 时), 调用被静默丢弃, 而调用方已经把副作用状态
     * (cal_boot/cal_idx 等)改好了 —— 结果既没激活页面, 标志又留在半途,
     * 直到某次无关的 exit 才被当成"跳过引导"误处理. */
    if(id == menu_ctrl.current && menu_ctrl.state == MENU_STATE_HANDLER)
        return;

    if(menu_ctrl.state == MENU_STATE_HANDLER)
        menu_exit_handler();   /* 离开当前页面: 调 exit 回调, 回落 NORMAL */

    menu_ctrl.current = id;

    /* 目标是否为可激活页面(常驻页 DIRECT_HANDLER, 或带 display+handler 的叶子页
     * 如 Cur Cal 引导页)统一交给 menu_activate 判定, 此处不再重复条件 ——
     * 不可激活(无 ops / 缺 display / 缺 handler)时 menu_activate 保持列表态. */
    menu_activate();
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
 * 不修改 current, 不做事件路由 —— 事件统一由 menu_state_handler 分发。
 *
 * 可激活契约(单点把关): 页面须同时提供 display(可渲染) 与 handler(可响应事件).
 * 只有 display 而无 handler 的页面在 HANDLER 态无法响应任何事件(既不能长按退出,
 * 也处理不了 CLICK/UP/DOWN), 属无效配置 —— 一律不激活, 回落列表态.
 * menu_state_handler 与 Menu_SetCurrentId 都经本函数进入 HANDLER, 故契约在此唯一判定. */
static void menu_activate(void)
{
    const menu_item_t *item=menu_current();

    if(!item||!item->ops)
        return;

    if(!item->ops->display || !item->ops->handler)
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
    uint8_t acted_id;    /* 派发时所在节点 id, 供下面判断 handler 是否已经自行跳页 */

    if(!item||!item->ops)
    {
        menu_ctrl.state=MENU_STATE_NORMAL;
        return;
    }

    acted_id = menu_ctrl.current;

    switch(cmd->event)
    {
        case MENU_EVENT_UP:
        case MENU_EVENT_DOWN:
        case MENU_EVENT_CLICK:
        case MENU_EVENT_LPRESS:
            /* 契约(见 menu_activate): 能进 HANDLER 态的页面必有 handler,
             * 故不再为"无 handler"留兜底分支(原 LPRESS 的 else 退出已移除). */
            if(item->ops->handler)
                ret=item->ops->handler(cmd, Menu_GetUserData(item));
            break;

        default:
            break;
    }

    /* handler 有权调 Menu_SetCurrentId 自行跳页(如手柄页跳 Cur Cal、故障跳 HOME).
     * 那种情况下当前节点与状态已被它接管, 此处若再按旧 item 收尾, 就会对新页面
     * 做一次"没有配对 enter 的 exit"(不平衡), 并按旧节点的 child_num 误下钻.
     * 现状两个跳转点都返回 MENU_RET_OK 才没暴露; 显式加护栏, 不再依赖调用方自觉. */
    if(menu_ctrl.current != acted_id || menu_ctrl.state != MENU_STATE_HANDLER)
        return;

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
     * 唯一的无父节点是虚拟根本身, 其 parent = MENU_NO_PARENT = 0 指向自身(idx 0),
     * Menu_GetItem(0) 是合法节点, 不会被 !parent 拦下; 真正兜底的是下方
     * "id 必须是该列表成员"的范围校验 —— 虚拟根 first_child=1/child_num=1,
     * 而它自身 id=0 落在 [1,2) 之外, 故返回 false. */
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

