#include "config.h"
#include "flash.h"
#include <string.h>

config_t g_config;

/* 默认配置 */
static const config_t default_config =
{
    .magic         = CONFIG_MAGIC,
    .version       = CONFIG_VERSION,
    .handle_type   = HANDLE_T12,
    .default_temp  = 300,
    .sleep_temp    = 200,
    /* PID 参数 (每型号一套; T245/T210 先用 T12 参考值占位, 可在型号页自行微调) */
    .pid_agg_kp    = {1100, 1100, 1100},   /* 激进 11.00 */
    .pid_agg_ki    = {50,   50,   50  },   /* 激进 0.50 */
    .pid_agg_kd    = {100,  100,  100 },   /* 激进 1.00 */
    .pid_cons_kp   = {50,   50,   50  },   /* 保守 0.50 */
    .pid_cons_ki   = {60,   60,   60  },   /* 保守 0.60 */
    .pid_cons_kd   = {600,  600,  600 },   /* 保守 6.00 */
    /* 温度上限: T12 常规 400°C(留余量给 Boost 爬升至 450), T245/T210 常规 450°C */
    .temp_max      = {400,  450,  450 },
    .boost_max     = {450,  450,  450 },
    .cal_off_100   = {0, 0, 0},
    .cal_off_200   = {0, 0, 0},
    .cal_off_300   = {0, 0, 0},
    .cal_off_350   = {0, 0, 0},
    .cal_off_400   = {0, 0, 0},
    .cal_off_450   = {0, 0, 0},
    .sleep_time    = 5,      /* 5分钟 */
    .shutdown_time = 30,     /* 30分钟 */
    .buzzer_enable = 1,      /* 开 */
    .brightness    = 180,
    /* ADC 校准点 (每型号一套, 参考 Cai-Zi; T245/T210 先同 T12 占位) */
    .cal_adc_100   = {790,  790,  790 },   /* 100°C ADC 值 */
    .cal_adc_200   = {1670, 1670, 1670},   /* 200°C ADC 值 */
    .cal_adc_300   = {2900, 2900, 2900},   /* 300°C ADC 值 */
    .cal_adc_350   = {3396, 3396, 3396},   /* 350°C ADC 值 */
    .cal_adc_400   = {3892, 3892, 3892},   /* 400°C ADC 值 */
    .cal_adc_450   = {4095, 4095, 4095},   /* 450°C ADC 值 */
    .crc           = 0,
};

/* 型号索引越界保护: 非法 handle_type 一律回落到 T12 */
uint8_t Config_GetHandleIdx(void)
{
    if (g_config.handle_type >= HANDLE_NUM)
        return HANDLE_T12;
    return g_config.handle_type;
}

uint16_t Config_GetTempMax(void)
{
    return g_config.temp_max[Config_GetHandleIdx()];
}

uint16_t Config_GetBoostMax(void)
{
    return g_config.boost_max[Config_GetHandleIdx()];
}

/* 手柄型号显示名 (与 HANDLE_* 索引一一对应) */
static const char *const handle_names[HANDLE_NUM] = {"T12", "T245", "T210"};

const char *Config_GetHandleName(void)
{
    return handle_names[Config_GetHandleIdx()];
}

/* 型号校准有效性: 6 个 ADC 校准点须严格单调递增且首点非 0(对应 adc_to_temp
 * 的防退化判定). 非法时测温会退化线性映射, 读数偏差大, 需重新校准. */
bool Config_IsHandleCalValid(uint8_t h)
{
    if (h >= HANDLE_NUM) return false;
    return (g_config.cal_adc_100[h] != 0 &&
            g_config.cal_adc_200[h] > g_config.cal_adc_100[h] &&
            g_config.cal_adc_300[h] > g_config.cal_adc_200[h] &&
            g_config.cal_adc_350[h] > g_config.cal_adc_300[h] &&
            g_config.cal_adc_400[h] > g_config.cal_adc_350[h] &&
            g_config.cal_adc_450[h] > g_config.cal_adc_400[h]);
}

/* 简单 CRC16 校验 */
static uint16_t config_calc_crc(const config_t *cfg)
{
    uint16_t crc = 0xFFFF;
    const uint8_t *p = (const uint8_t *)cfg;
    uint16_t len = sizeof(config_t) - sizeof(cfg->crc);
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        crc ^= p[i];
        uint8_t j;
        for (j = 0; j < 8; j++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

void Config_Init(void)
{
    /* 先清零, 使结构体内对齐填充字节(handle_type 后)在任何存储类/加载路径下都确定为零,
     * 保证 config_calc_crc 逐字节计算不依赖"全局/const 零初始化"的隐式约定 */
    memset(&g_config, 0, sizeof(g_config));

    Config_Load();

    if (g_config.magic != CONFIG_MAGIC || g_config.version != CONFIG_VERSION)
        Config_Reset();
    else
    {
        uint16_t calc = config_calc_crc(&g_config);
        if (calc != g_config.crc)
            Config_Reset();
    }
}

void Config_Load(void)
{
    Flash_Read(CONFIG_FLASH_ADDR, &g_config, sizeof(g_config));
}

void Config_Save(void)
{
    g_config.crc = config_calc_crc(&g_config);
    Flash_ErasePage(CONFIG_FLASH_ADDR);
    Flash_Write(CONFIG_FLASH_ADDR, &g_config, sizeof(g_config));
}

void Config_Reset(void)
{
    memcpy(&g_config, &default_config, sizeof(g_config));
    Config_Save();
}