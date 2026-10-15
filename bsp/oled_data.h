#ifndef __OLED_DATA_H
#define __OLED_DATA_H

#include <stdint.h>

/* Character encoding —— 字库 Index 与 oled.c OLED_ShowChinese 均按 UTF-8 存储/解析 */
#define OLED_CHARSET_UTF8
//#define OLED_CHARSET_GB2312

/* Font cell structure */
typedef struct 
{
#ifdef OLED_CHARSET_UTF8
	char Index[4];		/* UTF-8 码点(最多 3 字节)+终止; 查找按实际字节数比较 */
#endif
	
#ifdef OLED_CHARSET_GB2312
	char Index[4];		/* previously-GB2312, kept only for reference */
#endif
	
	uint8_t Data[32];	/* Font bitmap */
} ChineseCell_t;

/* ASCII font declarations */
/* 8x16 / 6x8 为紧凑表(每行 = 1 字符 + 字形字节), 按字符线性查找, 不用 ch-' ' 索引;
 * 只编入运行期可达字符(G8 宏开关, 见 oled_data.c 裁剪说明);
 * 行数随开关变化, 由 oled_data.c 定义处 sizeof 导出(用法同 OLED_CF16x16_COUNT) */
extern const uint8_t OLED_F8x16[][17];
extern const uint8_t OLED_F6x8[][7];
extern const uint8_t OLED_F8x16_COUNT;
extern const uint8_t OLED_F6x8_COUNT;
extern const uint8_t OLED_F16x32[][64];

/* Chinese font declarations */
extern const ChineseCell_t OLED_CF16x16[];
extern const uint16_t OLED_CF16x16_COUNT;

/* 原 extern const uint8_t Diode[]; 已删除:
 * 该图标的定义与唯一使用者都不存在(全工程 0 引用), 声明悬空 —— 留着只会让
 * 后续有人引用时得到莫名其妙的链接错误. 需要时再加定义与声明. */

#endif
