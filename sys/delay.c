#include "stm32f10x.h"

/* SysTick 时钟 = HCLK = 72MHz */
#define DELAY_SYS_CLK_MHZ   72U
/* SysTick->CTRL 位掩码 */
#define SYSTICK_ENABLE_CLKSRC  0x00000005U   /* bit0 ENABLE + bit2 CLKSOURCE(HCLK) */
#define SYSTICK_COUNTFLAG      0x00010000U   /* bit16 COUNTFLAG: 计数到 0 置位 */
#define SYSTICK_DISABLE        0x00000004U   /* 仅 CLKSOURCE，关闭使能 */

/**
  * @brief  微秒级延时
  * @param  xus 延时时长，范围：0~233015
  * @retval 无
  */
void Delay_Us(uint32_t xus)
{
	SysTick->LOAD = DELAY_SYS_CLK_MHZ * xus;	//设置定时器重装值
	SysTick->VAL = 0x00;					//清空当前计数值
	SysTick->CTRL = SYSTICK_ENABLE_CLKSRC;	//设置时钟源为HCLK，启动定时器
	while(!(SysTick->CTRL & SYSTICK_COUNTFLAG));	//等待计数到0
	SysTick->CTRL = SYSTICK_DISABLE;			//关闭定时器
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_Ms(uint32_t xms)
{
	while(xms--)
	{
		Delay_Us(1000);
	}
}
 
/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_S(uint32_t xs)
{
	while(xs--)
	{
		Delay_Ms(1000);
	}
} 
