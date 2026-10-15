#ifndef __CONFIG_H
#define __CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/* 硬件模拟模式开关 (1=模拟, 0=真机): 由 work_task.c 原私有定义提升为全局共享.
 * 模拟模式下所有 Flash 擦写/落盘跳过(见 config.c 三个 Save 函数),
 * 避免仿真器不建模 Flash 控制器时擦写死等 → 主循环卡死 → IWDG 复位死循环. */
/* 默认保守使用模拟模式; Keil target 必须显式定义 HW_SIM_MODE:
 * Debug-Sim = 1, Release-HW = 0. 这样发布构建不依赖开发者手动改源码宏.
 * 若用其它构建系统未定义该宏, 回退模拟模式, 避免意外写 Flash/上真机路径. */
#ifndef HW_SIM_MODE
#define HW_SIM_MODE           1
#endif

#define CONFIG_MAGIC      0x5AA55AA5UL
#define CONFIG_VERSION    13   /* 12→13: 移除 Boost 功能, config_t 删除 boost_max[] 字段;
                                * 同时 T12 常规上限 400→450(三型号统一).
                                * 旧 v12 数据由 config_v12_t 逐字段迁移, 保留 PID 增益/
                                * 温度/时间/蜂鸣/亮度等用户设置, 仅 temp_max 取新默认. */

/* 配置 Flash 存储地址 (使用 App Flash 最后一页, 0x0803F800) */
#define CONFIG_FLASH_ADDR  0x0803F800UL

/* Flash 页大小: STM32F103RC 属高密度器件, 页 = 2KB.
 * 三份数据必须各占一整页 —— save_page() 落盘前先 Flash_ErasePage(addr),
 * 而该函数按"所在整页"擦除, 两地址落在同一页即互相擦除. */
#define CONFIG_FLASH_PAGE_SIZE  0x800UL

/* 温度校准数据 Flash 存储地址 (独立页 0x0803F000)
 * 与配置页分离: Config_Reset() 只重置用户设置, 校准页由 Cal_Init 独立维护 */
#define CAL_MAGIC         0xCA1B4A55UL
#define CAL_VERSION       2   /* 1→2: 校准点由 6 点(100/200/300/350/400/450)改为
                               * 5 点(200/300/350/400/450), cal_t 布局随之变化.
                               * 旧 v1 数据版本不符 → Cal_Init 回落出厂默认表, 需重新校准. */
#define CAL_FLASH_ADDR    0x0803F000UL

/* 温度校准点数 (200/300/350/400/450°C) */
#define CAL_POINT_NUM     5

/* 校准偏移的合法范围 (±°C) —— 单一来源:
 *   - 页面(page/calibrate.c)用它钳位用户可设范围;
 *   - Cal_Init 用它校验从 Flash 载入的数据.
 * 二者必须同源: 页面能设出的值必须能通过加载校验, 反之亦然. */
#define CAL_OFF_LIMIT     50

/* 休眠设置三项的合法范围 —— 单一来源(同 CAL_OFF_LIMIT 的做法):
 *   - 页面(page/settings.c)用它限定用户可调范围;
 *   - Config_Init → config_clamp 用它收敛从 Flash 载入的越界值.
 * 二者必须同源: 页面能设出的值必然通过载入收敛, 反之亦然.
 * sleep_temp 下限 50: 更低已无保温意义; 上限 300 覆盖常见保温区间. */
#define SLEEP_TIME_MIN      0
#define SLEEP_TIME_MAX      60      /* 分钟, 0 = 禁用休眠 */
#define SLEEP_TEMP_MIN      50      /* °C */
#define SLEEP_TEMP_MAX      300     /* °C */
#define SHUTDOWN_TIME_MIN   0
#define SHUTDOWN_TIME_MAX   120     /* 分钟, 0 = 禁用自动关机 */

/* 手柄型号索引 */
#define HANDLE_NUM      3
#define HANDLE_T12      0
#define HANDLE_T245     1
#define HANDLE_T210     2

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;           /* sizeof(config_t), 结构体变化时用于版本迁移 */

    /* 当前手柄型号 (见 HANDLE_*) */
    uint8_t  handle_type;

    /* 温度设置 (通用, 跨型号共享) */
    uint16_t default_temp;      /* 默认目标温度 °C */
    uint16_t sleep_temp;        /* 休眠温度 °C */

    /* PID 参数 (*100), 每型号一套
     * 分段控制: ΔT>50°C 全速, 30~50°C 用激进组(积分限幅 15%), ≤30°C 用保守组(限幅 30%)
     * 积分限幅为内置固定值, 不存入配置 */
    int16_t pid_agg_kp[HANDLE_NUM];   /* 激进 Kp (*100) */
    int16_t pid_agg_ki[HANDLE_NUM];   /* 激进 Ki (*100) */
    int16_t pid_agg_kd[HANDLE_NUM];   /* 激进 Kd (*100) */
    int16_t pid_cons_kp[HANDLE_NUM];  /* 保守 Kp (*100) */
    int16_t pid_cons_ki[HANDLE_NUM];  /* 保守 Ki (*100) */
    int16_t pid_cons_kd[HANDLE_NUM];  /* 保守 Kd (*100) */

    /* 每型号的温度上限 (°C): T12/T245/T210 统一 450 */
    uint16_t temp_max[HANDLE_NUM];

    /* 系统设置 */
    uint16_t sleep_time;        /* 休眠时间 分钟, 0=禁用 */
    uint16_t shutdown_time;     /* 自动关机 分钟, 0=禁用 */
    uint8_t  buzzer_enable;     /* 0=关, 1=开 */
    uint8_t  brightness;        /* 0~255 */

    uint16_t crc;

}config_t;

/* 校准数据独立结构 (独立 flash 页, 恢复默认不清校准) */
typedef struct
{
    uint32_t magic;          /* CAL_MAGIC */
    uint16_t version;        /* CAL_VERSION */

    /* 多点温度校准偏移 (°C, -50 ~ +50), 每型号一套, 按温度区间插值
     * 5 温度点: 200/300/350/400/450, 细分常用焊接带 (300~350) */
    int8_t  cal_off_200[HANDLE_NUM];  /* 200°C 校准偏移 */
    int8_t  cal_off_300[HANDLE_NUM];  /* 300°C 校准偏移 */
    int8_t  cal_off_350[HANDLE_NUM];  /* 350°C 校准偏移 */
    int8_t  cal_off_400[HANDLE_NUM];  /* 400°C 校准偏移 */
    int8_t  cal_off_450[HANDLE_NUM];  /* 450°C 校准偏移 */
    /* 显式填充: 偏移块 5×3=15 B 为奇数, 其后 uint16 块需 2 字节对齐 ——
     * 把编译器本会插入的那 1 B 显式写出, 使布局确定、布局守卫可精确校验(见 config.c). */
    uint8_t cal_off_pad;

    /* 温度校准点 (ADC 值), 每型号一套, 与偏移同温度点对齐 */
    uint16_t cal_adc_200[HANDLE_NUM];  /* 200°C 时的 ADC 值 */
    uint16_t cal_adc_300[HANDLE_NUM];  /* 300°C 时的 ADC 值 */
    uint16_t cal_adc_350[HANDLE_NUM];  /* 350°C 时的 ADC 值 */
    uint16_t cal_adc_400[HANDLE_NUM];  /* 400°C 时的 ADC 值 */
    uint16_t cal_adc_450[HANDLE_NUM];  /* 450°C 时的 ADC 值 */

    uint16_t crc;

}cal_t;

/* 电流标定数据独立结构 (独立 flash 页 0x0803E800, 与校准/配置页分离)
 * 存各型号"冷态发热芯测量脉冲导通电流"的参考 ADC 值 (0=未标定),
 * 供发热芯型号匹配检查 (type_cur_match) 判定 ±30% 区间.
 * 由菜单项 "Cur Cal" 自动实测写入, 出厂全 0 = 匹配检查默认跳过.
 * 注: 旧版地址 0x0803F400 与 CAL_FLASH_ADDR 同处页 126(2KB), 二者落盘互相
 *     擦除(校准丢失要到下次开机 Cal_Init 校验 CRC 失败才显现), 已迁至独立页. */
#define TYPECAL_MAGIC         0x54594341UL   /* "TYCA" */
#define TYPECAL_VERSION       1
#define TYPECAL_FLASH_ADDR    0x0803E800UL

typedef struct
{
    uint32_t magic;             /* TYPECAL_MAGIC */
    uint16_t version;           /* TYPECAL_VERSION */
    uint16_t cur_ref[HANDLE_NUM];  /* 各型号冷态脉冲电流参考 ADC, 0=未标定 */
    uint16_t crc;

}type_cal_t;

/* 旧版 (v10) 配置布局镜像: 校准字段仍在 config 内, 仅供版本迁移解析 */
typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint8_t  handle_type;
    uint8_t  res0;                 /* 对齐填充, 与 v10 实际布局一致 */
    uint16_t default_temp;
    uint16_t sleep_temp;
    int16_t  pid_agg_kp[HANDLE_NUM];
    int16_t  pid_agg_ki[HANDLE_NUM];
    int16_t  pid_agg_kd[HANDLE_NUM];
    int16_t  pid_cons_kp[HANDLE_NUM];
    int16_t  pid_cons_ki[HANDLE_NUM];
    int16_t  pid_cons_kd[HANDLE_NUM];
    uint16_t temp_max[HANDLE_NUM];
    uint16_t boost_max[HANDLE_NUM];
    int8_t   cal_off_100[HANDLE_NUM];
    int8_t   cal_off_200[HANDLE_NUM];
    int8_t   cal_off_300[HANDLE_NUM];
    int8_t   cal_off_350[HANDLE_NUM];
    int8_t   cal_off_400[HANDLE_NUM];
    int8_t   cal_off_450[HANDLE_NUM];
    uint16_t sleep_time;
    uint16_t shutdown_time;
    uint8_t  buzzer_enable;
    uint8_t  brightness;
    uint16_t cal_adc_100[HANDLE_NUM];
    uint16_t cal_adc_200[HANDLE_NUM];
    uint16_t cal_adc_300[HANDLE_NUM];
    uint16_t cal_adc_350[HANDLE_NUM];
    uint16_t cal_adc_400[HANDLE_NUM];
    uint16_t cal_adc_450[HANDLE_NUM];
    uint16_t crc;

}config_v10_t;

/* v12 布局快照(含已删除的 boost_max[]): 仅供 12→13 迁移读取旧数据.
 * 字段顺序/类型与当时的 config_t 完全一致, 编译器填充也随之相同.
 * 注: v12 已有独立校准页(cal_t / type_cal_t), 故本结构不含 cal_* 字段. */
typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint8_t  handle_type;
    uint16_t default_temp;
    uint16_t sleep_temp;
    int16_t  pid_agg_kp[HANDLE_NUM];
    int16_t  pid_agg_ki[HANDLE_NUM];
    int16_t  pid_agg_kd[HANDLE_NUM];
    int16_t  pid_cons_kp[HANDLE_NUM];
    int16_t  pid_cons_ki[HANDLE_NUM];
    int16_t  pid_cons_kd[HANDLE_NUM];
    uint16_t temp_max[HANDLE_NUM];
    uint16_t boost_max[HANDLE_NUM];
    uint16_t sleep_time;
    uint16_t shutdown_time;
    uint8_t  buzzer_enable;
    uint8_t  brightness;
    uint16_t crc;
}config_v12_t;

extern config_t g_config;
extern cal_t    g_cal;
extern type_cal_t g_typecal;

/* ==================== 数据页布局编译期守卫 ====================
 * 三份数据各自落盘时都先整页擦除, 故必须满足两条不变式:
 *   (1) 每个地址 2KB 页对齐 —— 否则会写进邻页(可能属其它记录);
 *   (2) 三个地址的"页号"两两不等 —— 否则一次落盘会擦掉另一份数据.
 * 二者皆为编译期常量表达式, 违反任一条即编译失败(下方负数组长度),
 * 不必等到真机上出现"校准莫名丢失"再回头排查. 新增数据页时在此登记.
 * 未用 C11 _Static_assert: ARMCC V5.06 在 --c99 下不认该关键字.
 *
 * 另: 三页位于 App Flash 顶部, 必须被排除在链接器的 IROM 之外, 否则镜像一旦增长
 * 到该区域会被链接器铺上代码, 落盘擦页即擦掉代码. Project.uvprojx 中 IROM 尺寸已据此
 * 收为 0x36800 (0x08008000 + 0x36800 = 0x0803E800 = TYPECAL 页), 顶部 3 页(0x1800)
 * 全部留作数据页. 调整任一数据页地址时, 需同步复核 IROM 尺寸. */
#define CONFIG_FLASH_PG(a)   ((a) / CONFIG_FLASH_PAGE_SIZE)

#define CONFIG_PAGE_GUARD(name, cond)   typedef char name[(cond) ? 1 : -1]

CONFIG_PAGE_GUARD(guard_cal_page_aligned,
                  (CAL_FLASH_ADDR     % CONFIG_FLASH_PAGE_SIZE) == 0);
CONFIG_PAGE_GUARD(guard_typecal_page_aligned,
                  (TYPECAL_FLASH_ADDR % CONFIG_FLASH_PAGE_SIZE) == 0);
CONFIG_PAGE_GUARD(guard_config_page_aligned,
                  (CONFIG_FLASH_ADDR  % CONFIG_FLASH_PAGE_SIZE) == 0);

CONFIG_PAGE_GUARD(guard_data_pages_distinct,
                  CONFIG_FLASH_PG(CAL_FLASH_ADDR) != CONFIG_FLASH_PG(TYPECAL_FLASH_ADDR) &&
                  CONFIG_FLASH_PG(CAL_FLASH_ADDR) != CONFIG_FLASH_PG(CONFIG_FLASH_ADDR)  &&
                  CONFIG_FLASH_PG(TYPECAL_FLASH_ADDR) != CONFIG_FLASH_PG(CONFIG_FLASH_ADDR));

/* 数据页容量守卫: save_page() 只擦一页再写 sizeof(...) 字节, 故每个结构体必须装得进
 * 一页(2KB). 超出会溢写到相邻页, 且是**静默损坏**:
 *   - CAL 溢出    → 覆盖 CONFIG 页(0x0803F800) → 用户设置丢失
 *   - TYPECAL 溢出 → 覆盖 CAL 页(0x0803F000)    → 温度校准丢失
 *   - CONFIG 溢出  → 越过 Flash 末尾, 写失败
 * 三者都表现为"下次启动 CRC 失败 → 回落出厂值". 当前 64/64/16 字节, 离 2KB 尚远,
 * 加守卫是为了"新增字段撞线时立刻编译失败", 而不是等真机出现数据丢失再回查. */
CONFIG_PAGE_GUARD(guard_config_fits_page,  sizeof(config_t)   <= CONFIG_FLASH_PAGE_SIZE);
CONFIG_PAGE_GUARD(guard_cal_fits_page,     sizeof(cal_t)      <= CONFIG_FLASH_PAGE_SIZE);
CONFIG_PAGE_GUARD(guard_typecal_fits_page, sizeof(type_cal_t) <= CONFIG_FLASH_PAGE_SIZE);

void Config_Init(void);
void Config_Load(void);
/* 三个 Save 均返回 true=已成功写入 Flash(失败多为写保护/超时).
 * RAM 副本无论成败都已更新为待写内容, 调用方可安全重试. */
bool Config_Save(void);
bool Config_Reset(void);   /* 恢复默认并落盘; 返回 false = Flash 写失败(未持久化) */

/* 校准数据独立页管理 (Config_Reset 不触碰) */
void Cal_Init(void);
void Cal_Load(void);
bool Cal_Save(void);
void Cal_Reset(void);

/* 校准点按温度点索引 0..CAL_POINT_NUM-1 (200/300/350/400/450°C) */
int8_t   Cal_GetOff(uint8_t idx, uint8_t h);   /* 取某温度点某型号的偏移 */
void     Cal_SetOff(uint8_t idx, uint8_t h, int8_t v);
uint16_t Cal_GetAdc(uint8_t idx, uint8_t h);   /* 取某温度点某型号的 ADC 值 */

/* 型号校准有效性 (严格单调递增 + 首点非 0) */
bool Cal_IsHandleValid(uint8_t h);

/* 电流标定数据独立页管理 (发热芯型号匹配用, Config_Reset 不触碰) */
void     TypeCal_Init(void);
void     TypeCal_Load(void);
bool     TypeCal_Save(void);
void     TypeCal_Reset(void);

uint16_t TypeCal_GetRef(uint8_t h);   /* 取某型号冷态脉冲电流参考 ADC (0=未标定) */
bool     TypeCal_SetRef(uint8_t h, uint16_t v);   /* 写入并落盘; false = 写失败 */

/* 当前型号的脚柄索引 (越界回落到 T12) */
uint8_t  Config_GetHandleIdx(void);

/* 当前型号显示名 ("T12"/"T245"/"T210", 与索引对应) */
const char *Config_GetHandleName(void);
/* 按索引取型号显示名(越界回落 T12). 供各页面显示"选中/预览的型号"用 ——
 * 有了它, Cur Cal / 换头页就不必各自再存一份型号名表(单一来源). */
const char *Config_GetHandleNameByIdx(uint8_t idx);

/* 当前型号的温度上下限 (°C, 运行时按 handle_type 取值) */
uint16_t Config_GetTempMax(void);   /* 常规设定上限 */

#endif
