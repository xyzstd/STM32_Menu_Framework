# 烙铁控制器 — 架构一页纸

> 单循环裸机（无 RTOS），STM32F103 系列，Keil ARMCC 工程（`Project.uvprojx`）。

## 分层与依赖方向

```
main/           main.c 主循环: Input → Buzzer → WorkTask → UI_Task → Watchdog
  │
  ├─ menu/      内核: 菜单状态机 + 表驱动 + 列表环形渲染
  │     menu.c          事件路由/状态机 (NORMAL=列表 / HANDLER=页面)
  │     menu_display.c  列表环形滚动/窗口跟随渲染
  │     menu_table.c    菜单声明式节点表 (唯一耦合点)
  │
  ├─ page/      页面层: 每个页面 = ops 回调集合, 挂到 menu_table 节点
  │     home(常驻) / temp / handle / ota_wait / calibrate / settings /
  │     pid(快/慢) / autotune(自整定) / cur_cal(电流标定) /
  │     param / confirm(模板) / toast(保存提示框)
  │
  ├─ ctrl/      控制层: 与 UI 完全解耦的纯算法
  │     pid_controller.c   分段 PID (激进/保守) + 继电自整定
  │     heater.c           加热器功率输出 (功率→CCR, 90% 上限)
  │
  └─ bsp/       板级: oled / oled_data / adc / pwm / encoder / key / buzzer / serial / config / flash
```

数据流：**输入(BSP) → input.c(路由为命令) → menu内核(状态推进) → 页面(渲染缓冲) → OLED(差量推送)**；控制流独立：**work_task(状态机) → pid_controller(计算) → heater(输出)**。

## 关键机制

- **菜单表驱动**：`menu_item_t` 含 parent/first_child/flags，`flags` 声明式控制行为——`DIRECT_HANDLER` 常驻页（上电即激活、返回时直接回它）。新增页面 = 表加一行 + 写 ops；页面事件行为一律由 ops 回调裁决。
- **状态机**：NORMAL 态渲染列表，HANDLER 态调用页面 ops；离场方向由 child_num 决定（有子项→下钻，无→回父）。
- **输入统一**：TIM2 1ms 中断内采样（按键 5ms / 编码器 4ms 分频），事件经 SPSC 环形队列交主循环消费；编码器由 TIM4 硬件正交计数，`Encoder_GetSteps()` 按"每满 4 计数一步"折算并保留半格余量，跨页跳转时 `Encoder_RequestResync()` 丢弃残留相位。
- **渲染三层协同**（软 I2C 瓶颈）：输入逐格消费 → 渲染合并(2ms) → `OLED_UpdateDiff` 只标记脏页，主循环每轮 `OLED_FlushStep()` 推一页（分帧，单次 ≤1.5ms，不阻塞输入），静止零推送。
- **控制安全**：上电链路自检；过温 / 热电偶短路开路 / 加热器失效 / 过流 锁存断热（HOME 长按按类型恢复：仅过温直接清，其余重跑自检链路）；手柄断开复位 PID；休眠低温保持；IWDG ≈2s。

## 约定

- 所有含中文源文件 **UTF-8 带 BOM**；ARMCC AC5 严格 C99。
- 文件作用域静态变量用**语义化短名**（`win_first`、`cpu_temp_filt`），跨文件全局保留 `g_` 前缀（`g_config`）。
- 字模统一 **MSB 在上**（bit7=该列最上像素，取模工具默认），整字节写缓冲前经 `OLED_Rev8`。
- 页面渲染统一走 `OLED_UpdateDiff`（周期页除外，如 HOME）。
- **Flash 布局**：App 区 `0x08008000` 起、链接器 IROM 上限 `0x36800`（`Project.uvprojx`）；顶部 3 个 2KB 页为数据页 —— TYPECAL `0x0803E800` / CAL `0x0803F000` / CONFIG `0x0803F800`，须排除在 IROM 之外（落盘先整页擦除）；Boot 区标志页 `0x08007800`。
