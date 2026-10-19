#ifndef __MENU_H
#define __MENU_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define MENU_NO_PARENT 0U    /* parent: none; only the virtual root, which points to itself */
#define MENU_NO_CHILD  0U    /* first_child: leaf has no children */
#define MENU_HOME_ID 1U      /* HOME is the only child of the virtual root at index 0 */
#define MENU_LEAF    0U      /* child_num: leaf has no children */

#define MENU_FLAG_DIRECT_HANDLER  0x01U   /* resident page: enter/back activates it as handler (e.g. HOME) */

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
    MENU_RET_STAY=0,
    MENU_RET_DONE,
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


void Menu_Init(void);
void Menu_EventDispatch(const menu_cmd_t *cmd);

uint8_t Menu_GetCurrentId(void);
menu_state_t Menu_GetState(void);
void *Menu_GetUserData(const menu_item_t *item);

void Menu_SetCurrentId(uint8_t id);

const menu_item_t *Menu_GetItem(uint8_t id);

bool Menu_GetSiblingRange(uint8_t id,uint8_t *first,uint8_t *last);

#endif
