#include "menu.h"
#include "oled.h"
#include <stdio.h>

#define TIME_DAY_MAX  31
#define TIME_HOUR_MAX 23
#define TIME_MIN_MAX  59

//ҵ����
static void time_save(void);
static void time_focus_up(void);
static void time_focus_down(void);
static void time_inc(void);
static void time_dec(void);
static menu_result_t time_handler(const menu_cmd_t *cmd, void *user_data);

//��ʾ��
static void time_draw_focus(void);
static void time_draw_edit(void);
static menu_period_t time_display(void *user_data);

//��ʼ��
static void time_enter(void *user_data);
//�˳�����
static void time_exit(void *user_data);
	

typedef enum{TIME_FOCUS_DAY=0,TIME_FOCUS_HOUR,TIME_FOCUS_MIN}time_focus_t;
typedef enum{TIME_MODE_SELECT=0,TIME_MODE_EDIT}time_mode_t;

typedef struct
{
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    time_focus_t focus;
    time_mode_t mode;
}time_edit_t;


static time_edit_t time_edit;

static void time_save(void)
{
    /* ����RTC */
}


static void time_focus_up(void)
{
    time_edit.focus=(time_edit.focus==TIME_FOCUS_DAY)?TIME_FOCUS_MIN:(time_focus_t)((uint8_t)(time_edit.focus-1));
}


static void time_focus_down(void)
{
    time_edit.focus=(time_edit.focus==TIME_FOCUS_MIN)?TIME_FOCUS_DAY:(time_focus_t)((uint8_t)(time_edit.focus+1));
}


static void time_inc(void)
{
    switch(time_edit.focus)
    {
    case TIME_FOCUS_DAY:
        if(++time_edit.day>TIME_DAY_MAX)time_edit.day=1;
        break;

    case TIME_FOCUS_HOUR:
        if(++time_edit.hour>TIME_HOUR_MAX)time_edit.hour=0;
        break;

    case TIME_FOCUS_MIN:
        if(++time_edit.min>TIME_MIN_MAX)time_edit.min=0;
        break;

    default:
        break;
    }
}


static void time_dec(void)
{
    switch(time_edit.focus)
    {
    case TIME_FOCUS_DAY:
        time_edit.day=time_edit.day<=1?TIME_DAY_MAX:time_edit.day-1;
        break;

    case TIME_FOCUS_HOUR:
        time_edit.hour=time_edit.hour==0?TIME_HOUR_MAX:time_edit.hour-1;
        break;

    case TIME_FOCUS_MIN:
        time_edit.min=time_edit.min==0?TIME_MIN_MAX:time_edit.min-1;
        break;

    default:
        break;
    }
}


static menu_result_t time_handler(const menu_cmd_t *cmd, void *user_data)
{
    switch(time_edit.mode)
    {
    case TIME_MODE_SELECT:

        if(cmd->event==MENU_EVENT_UP)
            time_focus_up();
        else if(cmd->event==MENU_EVENT_DOWN)
            time_focus_down();
        else if(cmd->event==MENU_EVENT_CLICK)
            time_edit.mode=TIME_MODE_EDIT;
        else if(cmd->event==MENU_EVENT_LPRESS)
            return MENU_RET_DONE;   /* 无论是否在选中态, 长按直接返回上一级 */

        break;


    case TIME_MODE_EDIT:

        if(cmd->event==MENU_EVENT_UP)
            time_inc();
        else if(cmd->event==MENU_EVENT_DOWN)
            time_dec();
        else if(cmd->event==MENU_EVENT_CLICK)
            time_edit.mode=TIME_MODE_SELECT;
        else if(cmd->event==MENU_EVENT_LPRESS)
            return MENU_RET_DONE;   /* 无论是否在调参态, 长按直接返回上一级 */

        break;


    default:
        break;
    }

    return MENU_RET_OK;
}


static void time_draw_focus(void)
{
    switch(time_edit.focus)
    {
    case TIME_FOCUS_DAY:
        OLED_DrawRoundRect(44,0,24,16,6,OLED_COLOR_SET);
        break;

    case TIME_FOCUS_HOUR:
        OLED_DrawRoundRect(44,16,24,16,6,OLED_COLOR_SET);
        break;

    case TIME_FOCUS_MIN:
        OLED_DrawRoundRect(44,32,24,16,6,OLED_COLOR_SET);
        break;

    default:
        break;
    }
}


static void time_draw_edit(void)
{
    switch(time_edit.focus)
    {
    case TIME_FOCUS_DAY:
        OLED_ReverseRoundRect(44,0,24,16,6);
        break;

    case TIME_FOCUS_HOUR:
        OLED_ReverseRoundRect(44,16,24,16,6);
        break;

    case TIME_FOCUS_MIN:
        OLED_ReverseRoundRect(44,32,24,16,6);
        break;

    default:
        break;
    }
}


static menu_period_t time_display(void *user_data)
{
    char buf[8];

    OLED_Clear();

    OLED_ShowString(0,0,"Day",OLED_8X16);
    OLED_ShowString(0,16,"Hour",OLED_8X16);
    OLED_ShowString(0,32,"Min",OLED_8X16);

    sprintf(buf,"%02d",time_edit.day);
    OLED_ShowString(48,0,buf,OLED_8X16);

    sprintf(buf,"%02d",time_edit.hour);
    OLED_ShowString(48,16,buf,OLED_8X16);

    sprintf(buf,"%02d",time_edit.min);
    OLED_ShowString(48,32,buf,OLED_8X16);

    if(time_edit.mode==TIME_MODE_SELECT)
        time_draw_focus();
    else
        time_draw_edit();

    OLED_Update();

    return 0;
}


static void time_enter(void *user_data)
{
	time_edit.day=1;
    time_edit.hour=0;
    time_edit.min=0;
    time_edit.mode=TIME_MODE_SELECT;
    time_edit.focus=TIME_FOCUS_DAY;
}


static void time_exit(void *user_data)
{
    time_save();
}


const menu_ops_t time_ops =
{
    .enter=time_enter,
    .handler=time_handler,
    .display=time_display,
    .exit=time_exit,
};
