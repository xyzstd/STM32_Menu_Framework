#include "config.h"
#include "flash.h"
#include <string.h>
#include <stddef.h>   /* offsetof: CRC 覆盖范围与布局守卫 */

config_t g_config;
cal_t    g_cal;
type_cal_t g_typecal;

/* 默认配置 (仅用户设置; 校准数据独立在 default_cal) */
static const config_t default_config =
{
    .magic         = CONFIG_MAGIC,
    .version       = CONFIG_VERSION,
    .size          = sizeof(config_t),
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
    /* 温度上限: T12/T245/T210 统一 450°C */
    .temp_max      = {450,  450,  450 },
    .sleep_time    = 5,      /* 5分钟 */
    .shutdown_time = 30,     /* 30分钟 */
    .buzzer_enable = 1,      /* 开 */
    .brightness    = 180,
    .crc           = 0,
};

/* 迁移结构必须与 v12 当时的 config_t 完全同布局(大小 72 / crc 偏移 68):
 * 布局若被改动, 读取旧数据会字段错位 —— 那比"回落默认"更糟(会静默写入错误配置).
 * 编译期守卫: 一旦有人增删本结构字段, 构建立即失败. 数值经 gcc 独立核对. */
typedef char config_v12_layout_must_match[
    (sizeof(config_v12_t) == 72 && offsetof(config_v12_t, crc) == 68) ? 1 : -1];

/* 默认校准 (每型号一套, 参考 Cai-Zi; T245/T210 先同 T12 占位, 可在校准页重新标定) */
static const cal_t default_cal =
{
    .magic         = CAL_MAGIC,
    .version       = CAL_VERSION,
    .cal_off_100   = {0, 0, 0},
    .cal_off_200   = {0, 0, 0},
    .cal_off_300   = {0, 0, 0},
    .cal_off_350   = {0, 0, 0},
    .cal_off_400   = {0, 0, 0},
    .cal_off_450   = {0, 0, 0},
    .cal_adc_100   = {790,  790,  790 },   /* 100°C ADC 值 */
    .cal_adc_200   = {1670, 1670, 1670},   /* 200°C ADC 值 */
    .cal_adc_300   = {2900, 2900, 2900},   /* 300°C ADC 值 */
    .cal_adc_350   = {3396, 3396, 3396},   /* 350°C ADC 值 */
    .cal_adc_400   = {3892, 3892, 3892},   /* 400°C ADC 值 */
    .cal_adc_450   = {3920, 3920, 3920},   /* 450°C ADC 值: 出厂占位须显著低于开路阈值
                                              (THERMO_ADC_OPEN=4000), 否则未校准状态焊 450°C
                                              会被热电偶开路检测误锁存; 真机校准时 450°C 点
                                              也应满足 ＜4000 且留裕量 */
    .crc           = 0,
};

/* 默认电流标定: 出厂全 0 = 各型号均"未标定", 型号匹配检查默认跳过.
 * 真机用菜单 "Cur Cal" 实测后写入 (各型号冷态脉冲电流 ADC). */
static const type_cal_t default_typecal =
{
    .magic    = TYPECAL_MAGIC,
    .version  = TYPECAL_VERSION,
    .cur_ref  = {0, 0, 0},
    .crc      = 0,
};

static uint16_t crc16(const uint8_t *p, uint16_t len);
static uint16_t config_calc_crc(const config_t *cfg);
static uint16_t cal_calc_crc(const cal_t *c);
static uint16_t typecal_calc_crc(const type_cal_t *c);
static bool save_page(uint32_t addr, const void *buf, uint16_t len);
static void cal_migrate_v10(const config_v10_t *old);

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

/* 手柄型号显示名 (与 HANDLE_* 索引一一对应) */
static const char *const handle_names[HANDLE_NUM] = {"T12", "T245", "T210"};

const char *Config_GetHandleName(void)
{
    return handle_names[Config_GetHandleIdx()];
}

/* 简单 CRC16 (配置页与校准页共用) */
static uint16_t crc16(const uint8_t *p, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        uint8_t j;
        crc ^= p[i];
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

/* CRC 覆盖范围: 只算 crc 字段之前的字节.
 * 必须用 offsetof(T, crc) 而非 sizeof(T) - sizeof(crc) —— 后者仅在"crc 恰为最后
 * 2 字节且结构体尾部无对齐填充"时才等价. 本组结构体尾部都有填充(config_t 72/crc@68,
 * cal_t 64/60, type_cal_t 16/12), 用 sizeof-2 会越过 crc、把 crc 字段自身也纳入计算,
 * 使"保存写入的 crc"与"校验时算出的 crc"必然不等 → 每次上电都判数据损坏并回落默认
 * (用户设置/校准全丢, 且每次开机多一轮 Flash 擦写). */
static uint16_t config_calc_crc(const config_t *cfg)
{
    return crc16((const uint8_t *)cfg, (uint16_t)offsetof(config_t, crc));
}

static uint16_t cal_calc_crc(const cal_t *c)
{
    return crc16((const uint8_t *)c, (uint16_t)offsetof(cal_t, crc));
}

static uint16_t typecal_calc_crc(const type_cal_t *c)
{
    return crc16((const uint8_t *)c, (uint16_t)offsetof(type_cal_t, crc));
}

/* ===== 编译期布局守卫: cal_t 内 6 个校准数组必须"无填充连续" =====
 * Cal_GetOff/Cal_GetAdc 用 ((T*)&cal_xxx_100)[idx*HANDLE_NUM + h] 跨数组线性寻址,
 * 依赖数组之间没有对齐填充; 一旦编译器插入填充, 寻址会静默越界. 在此编译期拦下
 * (违反即负数组长度 = 编译失败), 与 config.h 的数据页守卫同一手法. */
typedef char guard_cal_off_contiguous[
    (offsetof(cal_t, cal_adc_100) - offsetof(cal_t, cal_off_100)
        == sizeof(int8_t) * CAL_POINT_NUM * HANDLE_NUM) ? 1 : -1];
typedef char guard_cal_adc_contiguous[
    (offsetof(cal_t, crc) - offsetof(cal_t, cal_adc_100)
        == sizeof(uint16_t) * CAL_POINT_NUM * HANDLE_NUM) ? 1 : -1];

/* 落盘一个数据页: 真机擦页后写入; 模拟模式仅保留 RAM 状态(仿真器擦写可能死等).
 * 三个 Save 共用, 避免各写一份 #if 分支导致规则漂移.
 * 返回 true=确实写入成功. 擦除失败时不继续编程(Flash 只能 1→0, 在未擦净区
 * 编程得到的是按位与的脏数据, 比不写更危险), 直接把失败上报给调用方. */
/* 注意: 模拟模式下 save_page 无地址/数据引用, 但本函数被三个 Save 无条件调用,
 * 故不加 #if —— 保持"调用点在两种模式下都编译"的既有约定. */
static bool save_page(uint32_t addr, const void *buf, uint16_t len)
{
#if !HW_SIM_MODE
    if (!Flash_ErasePage(addr))
        return false;
    return Flash_Write(addr, buf, len);
#else
    (void)addr; (void)buf; (void)len;
    return true;   /* 模拟模式不落盘, 视为成功(仅保留 RAM 状态) */
#endif
}

/* v10 旧校准字段整体迁入独立校准页 (升级固件不丢校准).
 * 以默认校准打底(带 magic/version), 仅覆盖 6 偏移 + 6 ADC 数据;
 * 各字段在 v10 与 cal_t 内均连续排布, 分两块整体拷贝. */
static void cal_migrate_v10(const config_v10_t *old)
{
    memcpy(&g_cal, &default_cal, sizeof(g_cal));
    memcpy(g_cal.cal_off_100, old->cal_off_100, sizeof(g_cal.cal_off_100) * CAL_POINT_NUM);
    memcpy(g_cal.cal_adc_100, old->cal_adc_100, sizeof(g_cal.cal_adc_100) * CAL_POINT_NUM);
    (void)Cal_Save();
}

void Config_Init(void)
{
    /* 先清零, 使结构体内对齐填充字节(handle_type 后)在任何存储类/加载路径下都确定为零,
     * 保证 CRC 逐字节计算不依赖"全局/const 零初始化"的隐式约定 */
    memset(&g_config, 0, sizeof(g_config));
    memset(&g_cal, 0, sizeof(g_cal));
    memset(&g_typecal, 0, sizeof(g_typecal));

    Config_Load();
    Cal_Init();
    TypeCal_Init();
}

void Config_Load(void)
{
    uint32_t magic;
    uint16_t version;

    /* 默认值打底: 迁移时未覆盖的字节(新字段/缺失数据)自动落到默认 */
    memcpy(&g_config, &default_config, sizeof(g_config));

    Flash_Read(CONFIG_FLASH_ADDR, &magic, sizeof(magic));
    if (magic != CONFIG_MAGIC)
    {
        (void)Config_Reset();   /* 空页(全FF)/magic 不符 → 出厂默认 */
        return;
    }

    Flash_Read(CONFIG_FLASH_ADDR + 4, &version, sizeof(version));

    if (version == CONFIG_VERSION)
    {
        /* 当前版本: 整读后按 size 迁移. 同版本结构体尾部新增字段时,
         * 校验旧 CRC 后按旧长度覆盖, 新字段留默认; 此后版本迁移需同步更新此分支. */
        config_t old;
        Flash_Read(CONFIG_FLASH_ADDR, &old, sizeof(old));

        if (old.size == sizeof(config_t))
        {
            g_config = old;
        }
        else if (old.size >= 8 && old.size < sizeof(config_t))
        {
            const uint8_t *p = (const uint8_t *)&old;
            uint16_t old_crc = (uint16_t)(p[old.size - 2] | ((uint16_t)p[old.size - 1] << 8));
            if (crc16(p, (uint16_t)(old.size - 2)) != old_crc)
            {
                (void)Config_Reset();
                return;
            }
            memcpy(&g_config, p, old.size);
            (void)Config_Save();   /* 补齐新字段与 CRC, 一次落盘 */
            return;
        }
        else
        {
            (void)Config_Reset();   /* size 异常 → 出厂默认 */
            return;
        }

        if (config_calc_crc(&g_config) != g_config.crc)
            (void)Config_Reset();
    }
    else if (version == 10)
    {
        /* v10 旧布局: 逐字段迁移用户设置, 旧校准字段一并迁入独立校准页 (升级不丢校准).
         * 注: 校验沿用 sizeof-2 的覆盖范围(与旧固件一致). 旧固件同样存在"覆盖范围含
         * crc 自身"的缺陷, 其写出的 v10 数据本就不可能通过校验 —— 即本迁移路径实际
         * 不会命中; 保留原覆盖范围以免"修正后反而认下按旧缺陷写出的数据". */
        config_v10_t old;
        Flash_Read(CONFIG_FLASH_ADDR, &old, sizeof(old));

        if (crc16((const uint8_t *)&old, sizeof(old) - sizeof(old.crc)) != old.crc)
        {
            (void)Config_Reset();
            return;
        }

        g_config.handle_type   = old.handle_type;
        g_config.default_temp  = old.default_temp;
        g_config.sleep_temp    = old.sleep_temp;
        memcpy(g_config.pid_agg_kp,  old.pid_agg_kp,  sizeof(g_config.pid_agg_kp));
        memcpy(g_config.pid_agg_ki,  old.pid_agg_ki,  sizeof(g_config.pid_agg_ki));
        memcpy(g_config.pid_agg_kd,  old.pid_agg_kd,  sizeof(g_config.pid_agg_kd));
        memcpy(g_config.pid_cons_kp, old.pid_cons_kp, sizeof(g_config.pid_cons_kp));
        memcpy(g_config.pid_cons_ki, old.pid_cons_ki, sizeof(g_config.pid_cons_ki));
        memcpy(g_config.pid_cons_kd, old.pid_cons_kd, sizeof(g_config.pid_cons_kd));
        /* temp_max 属型号硬件常量(非用户设置), 不迁移, 取新默认 */
        g_config.sleep_time    = old.sleep_time;
        g_config.shutdown_time = old.shutdown_time;
        g_config.buzzer_enable = old.buzzer_enable;
        g_config.brightness    = old.brightness;

        cal_migrate_v10(&old);   /* 旧校准迁入独立页 (校验通过, 数据可信) */
        (void)Config_Save();   /* 回写当前版本布局 */
    }
    else if (version == 12)
    {
        /* v12 布局(含已移除的 boost_max): 必须逐字段迁移, 不能按旧长度整体拷贝 ——
         * boost_max 已从 config_t 中删除, 整体拷贝会让其后的 sleep_time/shutdown_time/
         * buzzer/brightness 全部错位. temp_max 属型号硬件常量, 不迁移(取新的 450 默认).
         * CRC 覆盖范围与 v12 写入时一致(offsetof(crc)), 故能正确校验. */
        config_v12_t old;
        Flash_Read(CONFIG_FLASH_ADDR, &old, sizeof(old));

        if (crc16((const uint8_t *)&old, (uint16_t)offsetof(config_v12_t, crc)) != old.crc)
        {
            (void)Config_Reset();
            return;
        }

        g_config.handle_type   = old.handle_type;
        g_config.default_temp  = old.default_temp;
        g_config.sleep_temp    = old.sleep_temp;
        memcpy(g_config.pid_agg_kp,  old.pid_agg_kp,  sizeof(g_config.pid_agg_kp));
        memcpy(g_config.pid_agg_ki,  old.pid_agg_ki,  sizeof(g_config.pid_agg_ki));
        memcpy(g_config.pid_agg_kd,  old.pid_agg_kd,  sizeof(g_config.pid_agg_kd));
        memcpy(g_config.pid_cons_kp, old.pid_cons_kp, sizeof(g_config.pid_cons_kp));
        memcpy(g_config.pid_cons_ki, old.pid_cons_ki, sizeof(g_config.pid_cons_ki));
        memcpy(g_config.pid_cons_kd, old.pid_cons_kd, sizeof(g_config.pid_cons_kd));
        g_config.sleep_time    = old.sleep_time;
        g_config.shutdown_time = old.shutdown_time;
        g_config.buzzer_enable = old.buzzer_enable;
        g_config.brightness    = old.brightness;

        (void)Config_Save();   /* 回写新布局 */
    }
    else
    {
        (void)Config_Reset();   /* 无法解析的版本 → 出厂默认 */
    }
}

/* 返回 true=已成功写入 Flash. RAM 副本无论成败都已更新为待写内容(缓存权威),
 * 故失败时立刻重试是安全的; 调用方不应把 false 当作"值没变". */
bool Config_Save(void)
{
    g_config.crc = config_calc_crc(&g_config);
    return save_page(CONFIG_FLASH_ADDR, &g_config, sizeof(g_config));
}

bool Config_Reset(void)
{
    memcpy(&g_config, &default_config, sizeof(g_config));
    return Config_Save();   /* 只重置用户设置, 不触碰校准页; 返回落盘结果供调用方反馈 */
}

/* ==================== 校准数据独立页 ==================== */

/* 校准点按温度点索引 0..5 ↔ 100/200/300/350/400/450.
 * 各字段在 cal_t 内按型号连续排布, 用基址 + idx*HANDLE_NUM + h 统一寻址,
 * 免去 6 行逐字段判断 */

int8_t Cal_GetOff(uint8_t idx, uint8_t h)
{
    if (idx >= CAL_POINT_NUM || h >= HANDLE_NUM)
        return 0;
    return ((int8_t *)&g_cal.cal_off_100)[idx * HANDLE_NUM + h];
}

void Cal_SetOff(uint8_t idx, uint8_t h, int8_t v)
{
    if (idx >= CAL_POINT_NUM || h >= HANDLE_NUM)
        return;
    ((int8_t *)&g_cal.cal_off_100)[idx * HANDLE_NUM + h] = v;
}

uint16_t Cal_GetAdc(uint8_t idx, uint8_t h)
{
    if (idx >= CAL_POINT_NUM || h >= HANDLE_NUM)
        return 0;
    return ((uint16_t *)&g_cal.cal_adc_100)[idx * HANDLE_NUM + h];
}

/* 型号校准有效性: 6 个 ADC 校准点须严格单调递增且首点非 0(对应 adc_to_temp
 * 的防退化判定). 非法时测温会退化线性映射, 读数偏差大, 需重新校准. */
bool Cal_IsHandleValid(uint8_t h)
{
    if (h >= HANDLE_NUM) return false;
    return (Cal_GetAdc(0, h) != 0 &&
            Cal_GetAdc(1, h) > Cal_GetAdc(0, h) &&
            Cal_GetAdc(2, h) > Cal_GetAdc(1, h) &&
            Cal_GetAdc(3, h) > Cal_GetAdc(2, h) &&
            Cal_GetAdc(4, h) > Cal_GetAdc(3, h) &&
            Cal_GetAdc(5, h) > Cal_GetAdc(4, h));
}

/* 校准偏移的范围校验: 六个温度点 × 每型号(共 CAL_POINT_NUM*HANDLE_NUM 个)
 * 必须全部落在 ±CAL_OFF_LIMIT 内.
 * 为何必须校验: 偏移经 cal_offset_interp → spline_interp **直接叠加到读温**上.
 * 越界值(如 -128)会让报温严重偏低, PID 因此持续满功率 → 实际温度远超设定(不安全方向);
 * 页面也会因 '0' + |v|/10 拼出非数字字符.
 * 正常路径(页面编辑 + Cal_SetOff)恒写入钳位后的值, 故越界只可能来自版本迁移
 * (cal_migrate_v10 的 memcpy)或"CRC 自洽但语义越界"的存量数据.
 * 布局前提: 六个偏移数组在 cal_t 内连续无填充 —— 与 Cal_GetOff/Cal_SetOff 的指针
 * 步进访问同一假设, 已由上面的 guard_cal_off_contiguous 编译期守卫保证. */
static bool cal_offsets_valid(void)
{
    const int8_t *off = (const int8_t *)&g_cal.cal_off_100;
    uint8_t i;

    for (i = 0; i < (uint8_t)(CAL_POINT_NUM * HANDLE_NUM); i++)
    {
        if (off[i] > CAL_OFF_LIMIT || off[i] < -CAL_OFF_LIMIT)
            return false;
    }

    return true;
}

void Cal_Init(void)
{
    Cal_Load();

    if (g_cal.magic != CAL_MAGIC || g_cal.version != CAL_VERSION ||
        cal_calc_crc(&g_cal) != g_cal.crc || !cal_offsets_valid())
        Cal_Reset();   /* 空页(全FF)/版本不符/数据损坏/偏移越界 → 出厂校准 */
}

void Cal_Load(void)
{
    Flash_Read(CAL_FLASH_ADDR, &g_cal, sizeof(g_cal));
}

bool Cal_Save(void)
{
    g_cal.crc = cal_calc_crc(&g_cal);
    return save_page(CAL_FLASH_ADDR, &g_cal, sizeof(g_cal));
}

void Cal_Reset(void)
{
    memcpy(&g_cal, &default_cal, sizeof(g_cal));
    (void)Cal_Save();
}

/* ==================== 电流标定数据独立页 ==================== */

void TypeCal_Init(void)
{
    TypeCal_Load();

    if (g_typecal.magic != TYPECAL_MAGIC || g_typecal.version != TYPECAL_VERSION ||
        typecal_calc_crc(&g_typecal) != g_typecal.crc)
        TypeCal_Reset();   /* 空页(全FF)/版本不符/数据损坏 → 全 0 未标定 */
}

void TypeCal_Load(void)
{
    Flash_Read(TYPECAL_FLASH_ADDR, &g_typecal, sizeof(g_typecal));
}

bool TypeCal_Save(void)
{
    g_typecal.crc = typecal_calc_crc(&g_typecal);
    return save_page(TYPECAL_FLASH_ADDR, &g_typecal, sizeof(g_typecal));
}

void TypeCal_Reset(void)
{
    memcpy(&g_typecal, &default_typecal, sizeof(g_typecal));
    (void)TypeCal_Save();
}

uint16_t TypeCal_GetRef(uint8_t h)
{
    if (h >= HANDLE_NUM)
        return 0;
    return g_typecal.cur_ref[h];
}

bool TypeCal_SetRef(uint8_t h, uint16_t v)
{
    if (h >= HANDLE_NUM)
        return false;
    if (g_typecal.cur_ref[h] == v)
        return true;   /* 值未变化: 跳过落盘(省 Flash 擦写), 目标状态已满足 → 视为成功 */
    g_typecal.cur_ref[h] = v;
    return TypeCal_Save();
}
