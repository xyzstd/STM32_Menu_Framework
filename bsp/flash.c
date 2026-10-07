#include "stm32f10x.h"   
#include "flash.h"
#include <string.h>

/* 落盘失败与"粘滞错误标志"的成因:
 * 标准库每一次擦/写都先 FLASH_WaitForLastOperation(), 而它读的是 FLASH->SR 的
 * EOP/PGERR/WRPRTERR. 一旦某次操作留下 PGERR/WRPRTERR 未被清除, FLASH_GetBank1Status
 * 就恒返回 FLASH_ERROR_PG/WRP, 使 `if (status == FLASH_COMPLETE)` 永不成立 ——
 * 此后所有擦除与编程都"直接返回失败", 芯片上却毫无变化. 若调用方再不检查返回值,
 * 就表现为"UI 提示已保存, 重启后数据却是旧的"这种最难查的静默数据丢失.
 * 故本模块统一: 进入操作前清干净错误标志, 操作后检查状态并把结果如实返回.
 * (对比 ota/app_side/ota_boot.c 的 FLASH_ClearFlag 用法, 二者口径一致.) */

static void flash_clear_sticky_flags(void);

/* 操作前清除粘滞错误标志 + 结束标志, 避免上一次的残留让本次操作直接失败 */
static void flash_clear_sticky_flags(void)
{
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
}

void Flash_Read(uint32_t addr,void *buf,uint16_t len)
{
    memcpy(buf,(const void *)addr,len);
}

bool Flash_ErasePage(uint32_t addr)
{
    FLASH_Status st;

    FLASH_Unlock();
    flash_clear_sticky_flags();
    /* 命名避开 FLASH_ErasePage: 标准库函数名在部分版本里是宏, 同名局部变量会被展开 */
    st = FLASH_ErasePage(addr);
    FLASH_Lock();

    return (st == FLASH_COMPLETE);
}

/* 写 Flash。契约: 目标区必须先擦除(Flash 只能 1→0, 未擦除区重写会静默损坏),
 * 由 Config_Save 先 Flash_ErasePage 后写入保证.
 * 返回 true 仅当每一个半字都编程成功. */
bool Flash_Write(uint32_t addr,const void *buf,uint16_t len)
{
    uint16_t i;
    const uint8_t *src=buf;
    FLASH_Status status = FLASH_COMPLETE;

    /* 半字编程要求 addr 2 字节对齐(页基址天然满足, i 恒为偶数故 addr+i 恒偶).
     * 奇数长度时末字节补 0xFF(已擦除态)凑满半字: 不越界读 src, 也不丢该字节数据. */
    FLASH_Unlock();
    flash_clear_sticky_flags();

    for(i=0;i<len;i+=2)
    {
        uint16_t data=src[i];

        if(i+1<len)
            data|=(uint16_t)src[i+1]<<8;
        else
            data|=0xFF00;

        status = FLASH_ProgramHalfWord(addr+i,data);
        if (status != FLASH_COMPLETE)
            break;   /* 首个失败即停: 后续半字必然同样失败, 且错误标志已污染状态 */
    }

    FLASH_Lock();

    return (status == FLASH_COMPLETE);
}
