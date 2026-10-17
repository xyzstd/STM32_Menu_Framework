#include "menu.h"

typedef struct{
    uint8_t current;
    menu_state_t state;
}menu_ctrl_t;

static menu_ctrl_t menu_ctrl;


static void menu_state_normal(const menu_cmd_t *cmd);
static void menu_state_handler(const menu_cmd_t *cmd);

static void menu_enter_handler(void);
static void menu_exit_handler(void);

static void menu_move_up(void);
static void menu_move_down(void);
static void menu_enter(void);
static void menu_back(void);


void Menu_Init(void)
{
    menu_ctrl.state   = MENU_STATE_NORMAL;
    menu_ctrl.current = MENU_HOME_ID;
    menu_enter_handler();
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


void *Menu_GetUserData(const menu_item_t *item)
{
    return (item->user_data) ? item->user_data : (void *)item;
}


void Menu_SetCurrentId(uint8_t id)
{

    if(id == MENU_NO_PARENT || id >= menu_max_num)
        return;

    if(id == menu_ctrl.current && menu_ctrl.state == MENU_STATE_HANDLER)
        return;

    if(menu_ctrl.state == MENU_STATE_HANDLER)
    {
        menu_exit_handler();

        if(menu_ctrl.state == MENU_STATE_HANDLER)
            return;
    }

    menu_ctrl.current = id;

    menu_enter_handler();
}


static inline bool menu_ops_activatable(const menu_ops_t *ops)
{
    return ops != NULL && ops->display != NULL && ops->handler != NULL;
}


static inline uint16_t menu_child_end(const menu_item_t *item)
{
    uint16_t end;

    if(item == NULL || item->child_num == MENU_LEAF ||
        item->first_child == MENU_NO_CHILD)
        return 0u;

    end = (uint16_t)item->first_child + item->child_num;
    if(item->first_child >= menu_max_num || end > menu_max_num)
        return 0u;

    return end;
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


static void menu_state_handler(const menu_cmd_t *cmd)
{
    const menu_item_t *item=Menu_GetItem(menu_ctrl.current);
    menu_result_t ret=MENU_RET_OK;
    uint8_t acted_id;

    if(item==NULL || !menu_ops_activatable(item->ops))
    {
        menu_ctrl.state=MENU_STATE_NORMAL;
        return;
    }

    acted_id = menu_ctrl.current;

    if(cmd->event <= MENU_EVENT_LPRESS)
        ret = item->ops->handler(cmd, Menu_GetUserData(item));

    if(menu_ctrl.current != acted_id || menu_ctrl.state != MENU_STATE_HANDLER)
        return;

    if(ret==MENU_RET_DONE)
    {
        menu_exit_handler();

        if(menu_ctrl.current == acted_id && item->child_num > 0)
            menu_enter();
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


static void menu_enter_handler(void)
{
    const menu_item_t *item=Menu_GetItem(menu_ctrl.current);

    if(item==NULL || !menu_ops_activatable(item->ops))
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
    const menu_item_t *item=Menu_GetItem(menu_ctrl.current);

    menu_ctrl.state=MENU_STATE_NORMAL;

    if(item&&item->ops&&item->ops->exit)
        item->ops->exit(Menu_GetUserData(item));
}


static void menu_enter(void)
{
    const menu_item_t *item=Menu_GetItem(menu_ctrl.current);

    if(!item)
        return;

    if(item->child_num)
    {
        if(menu_child_end(item) == 0u)
            return;

        menu_ctrl.state = MENU_STATE_NORMAL;
        menu_ctrl.current = item->first_child;
    }
    else if(item->ops)
    {
        menu_enter_handler();
    }
}

static void menu_back(void)
{
    const menu_item_t *item=Menu_GetItem(menu_ctrl.current);
    const menu_item_t *target;

    if(!item)
        return;

    if(item->parent >= menu_max_num)
        return;

    menu_ctrl.current=item->parent;


    target=Menu_GetItem(menu_ctrl.current);
    if(target->flags & MENU_FLAG_DIRECT_HANDLER)
        menu_enter_handler();
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

    parent=Menu_GetItem(item->parent);
    if(!parent)
        return false;

    end = menu_child_end(parent);
    if(end == 0u)
        return false;

    if(id<parent->first_child || id>=end)
        return false;

    *first=parent->first_child;
    *last=(uint8_t)(end-1);

    return true;
}
