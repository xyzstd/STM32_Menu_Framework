#include "toast.h"
#include "oled.h"
#include "oled_data.h"
#include "timer.h"

/* ==================== 保存提示框 ====================
 * 叠加在页面渲染结果之上的卡片弹窗(白底圆角 + 内嵌细边框 + 对勾图标 + 黑字),
 * 约 0.6s 后自动消失. 绘制后单独差量推送(仅框所在页刷新), 到期再由渲染调度方
 * 重绘页面把框擦除, 全程不阻塞主循环。
 *
 * 原先实现在 main/ui_task.c: 页面为了一句"已保存"反馈必须 include ui_task.h,
 * 造成 page -> main 反向依赖. 提示框本身只依赖 OLED 与 tick, 归页面层更合适. */

#define TOAST_HOLD_MS   600
#define TOAST_BOX_X     24   /* 框 80x30 居中: x 24..103, y 17..46 */
#define TOAST_BOX_Y     17
#define TOAST_BOX_W     80
#define TOAST_BOX_H     30
#define TOAST_BOX_R     8    /* 外框圆角 (内嵌细边框取 R-3) */

static uint32_t toast_until = 0;   /* 显示截止 tick (0=不显示) */

static void toast_check(int16_t x, int16_t y);

void Toast_Saved(void)
{
    toast_until = Timer_GetTick() + TOAST_HOLD_MS;
}

/* 弹窗内的小对勾图标 (2px 粗, 黑): 起笔 → 转折 → 收笔, 占 12x10 */
static void toast_check(int16_t x, int16_t y)
{
    uint8_t i;

    for (i = 0; i < 2; i++)
    {
        OLED_DrawLine(x,     (int16_t)(y + 4 + i), (int16_t)(x + 4),  (int16_t)(y + 8 + i), OLED_COLOR_CLEAR);
        OLED_DrawLine((int16_t)(x + 4), (int16_t)(y + 8 + i), (int16_t)(x + 11), (int16_t)(y + i),     OLED_COLOR_CLEAR);
    }
}

bool Toast_Pending(void)
{
    /* 到期判定用有符号差: tick 回绕时仍正确 */
    if (toast_until != 0 && (int32_t)(Timer_GetTick() - toast_until) >= 0)
        toast_until = 0;

    return (toast_until != 0);
}

bool Toast_Overlay(void)
{
    if (!Toast_Pending())
        return false;

    /* 先清底层(黑): 连圆角外的方形四角一起清, 保证弹窗四周干净不残留页面像素 */
    OLED_FillRect(TOAST_BOX_X, TOAST_BOX_Y, TOAST_BOX_W, TOAST_BOX_H, OLED_COLOR_CLEAR);

    /* 白底圆角卡片 */
    OLED_FillRoundRect(TOAST_BOX_X, TOAST_BOX_Y, TOAST_BOX_W, TOAST_BOX_H,
                       TOAST_BOX_R, OLED_COLOR_SET);

    /* 内嵌 1px 细边框: 卡片式双线, 比纯反显色块更精致 */
    OLED_DrawRoundRect((int16_t)(TOAST_BOX_X + 3), (int16_t)(TOAST_BOX_Y + 3),
                       (uint8_t)(TOAST_BOX_W - 6), (uint8_t)(TOAST_BOX_H - 6),
                       (uint8_t)(TOAST_BOX_R - 3), OLED_COLOR_CLEAR);

    toast_check((int16_t)(TOAST_BOX_X + 7), (int16_t)(TOAST_BOX_Y + 10));
    OLED_ShowChinese((int16_t)(TOAST_BOX_X + 25), (int16_t)(TOAST_BOX_Y + 7), "已保存");

    OLED_UpdateDiff();
    return true;
}
