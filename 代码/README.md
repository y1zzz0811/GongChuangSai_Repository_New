# 电控测试程序

本目录保存现阶段实际使用的单项测试工程。三个工程分别编译、下载，功能尚未合并成比赛整车固件。

| 工程 | 芯片 | 用途 |
|---|---|---|
| [F407_MotorBench](F407_MotorBench) | STM32F407VET6 | 当前 A1 电机与麦轮底盘调试：原始协议、单机/多机、位置控制状态机及带转向的正方形路线 |
| [F407_BoardBringup](F407_BoardBringup) | STM32F407VET6 | 主控板 LED、串口、PC1 输入和串口屏调试 |
| [PWM_Servo](PWM_Servo) | STM32F103C8/CB | 早期 TIM2 PWM 舵机测试；上电循环运动，不能直接作为 F407 固件下载 |

**电机及底盘指令只维护这一份：[电机与底盘调试统一操作手册](电机与底盘调试统一操作手册.md)。** 所有原始指令、高级运动指令、停止/故障处理、接线、方向映射及标定方法均以该手册为准。

## 打开与编译

各工程均保留 `Core/`、所需 `Drivers/`、CubeMX `.ioc/.mxproject`、`MDK-ARM/*.uvprojx` 和启动文件。使用 Keil MDK 打开对应 `.uvprojx`，安装匹配的 STM32 设备包后 Rebuild；首次调试根据实际 DAPLink/CMSIS-DAP 配置下载器。仓库遵循根目录 `.gitignore`，不提交编译结果、个人调试选项和备份。驱动库保留构建需要的源文件与头文件，未收录库的无关示例。

F407_MotorBench 的尺寸、轮径、细分、方向及默认速度集中在 [chassis_config.h](F407_MotorBench/Core/Inc/chassis_config.h)。串口临时设置只存在 RAM；持久修改需更新宏并重新编译下载。上电不自动运动。

## 当前验证状态

截至 2026-10-23，用户已完成全部电机单机、M6/M7 双机、M1～M4 四机与底盘直线运动验证。新增 A1 自动路线固件已完成 Keil 编译和主机模拟测试，带转向的方形路线、地面距离/角度及回原点误差仍需实车验证；当前未接入陀螺仪。

主机测试使用 GCC 编译 `F407_MotorBench/Tests/host/test_motorbench.c`，添加 `Tests/host/include` 和 `Core/Inc` 两个头文件搜索目录，标准为 C99。测试模拟 HAL 和电机回复，不连接硬件。源文件按本地现有版本整理，上传未修改控制逻辑。

第三方 HAL/CMSIS 文件保留原版权声明和许可证，具体使用条件见各驱动目录的 `LICENSE.txt`。
