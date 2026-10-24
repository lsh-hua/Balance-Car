# 平衡小车固件

## 目录与职责

```text
balance/
|-- .gitignore        # 排除可重建产物与本机工具状态
|-- AGENTS.md         # 项目边界、开发约定与版本管理入口
|-- balance.ioc       # STM32CubeMX 硬件与外设配置
|-- APP/              # 姿态、编码器、PID、平衡控制与电机驱动
|   `-- AGENTS.md     # 应用层运行约束与离线回归入口
|-- Core/
|   |-- Inc/          # 主程序、外设与中断的接口
|   `-- Src/          # 启动调度、外设初始化、中断与遥测
|-- Drivers/
|   |-- CMSIS/        # Cortex-M3 与 STM32F1 设备定义
|   `-- STM32F1xx_HAL_Driver/ # STM32 HAL 外设实现
|-- MDK-ARM/
|   |-- balance.uvprojx # Keil 工程、编译选项与源码清单
|   |-- startup_stm32f103xb.s # 启动向量与栈配置
|   |-- DebugConfig/  # 工程调试配置
|   |-- RTE/          # Keil 运行时组件声明
|   `-- EventRecorderStub.scvd # 事件记录器描述
`-- SYS/              # 系统支持与延时接口
```

`Core/Src/main.c` 调度采样、姿态更新与控制；`APP/balance.c` 消费姿态和轮速，调用 `APP/motor.c` 提交电机输出。应用层通过 Core 提供的外设接口使用 HAL；Drivers 保持厂商边界，SYS 提供底层支持。

## 开发与验证

- 修改 APP、主循环、中断、传感器、控制或电机路径前，读取 `APP/AGENTS.md`，遵守其中的模式、保护与验证约束。
- 人类可见的注释和文档使用中文，代码标识符使用简洁英文；保留既有源文件编码。
- Keil 工程入口为 `MDK-ARM/balance.uvprojx`；硬件配置入口为 `balance.ioc`。新增源码时同步工程清单。
- 每次创建、删除、移动模块或调整职责，同步维护对应目录的 AGENTS.md、目录树、决策原因与变更记录。
- 测试脚本位于本机 `E:\桌面\Phase1`，具体入口见 `APP/AGENTS.md`；该外部目录不属于本固件仓库。

## 版本管理

当前目录是独立 Git 仓库，远端为 `https://github.com/lsh-hua/Balance-Car.git`，首次发布使用 `main`。提交范围限于当前项目，保留厂商许可；编译输出、IDE 用户状态和调试日志由 `.gitignore` 排除。

## 变更记录

- 2026-10-24：为固件建立独立仓库并准备首次 GitHub 提交；新增忽略规则与根目录架构说明，保留源码和既有应用层约束。
