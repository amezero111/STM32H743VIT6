# TLS R1 Full Version Backup Keil

本分支是 RoboCon 2026 车控工程在 2026-06-28 的 Keil 备份版本，用于给队友和下一届同学交接。工程基于 STM32H743VIT6，使用 Keil MDK ARM Compiler 6.24，主要包含底盘、达妙机械臂底部俯仰、幻尔总线舵机吸盘姿态、气路继电器、遥控器解析、USB/协议、BMI088/IMU 等模块。

这份 README 写得很细，目的不是“好看”，而是让接手的人少踩坑。第一次打开工程请先完整看完“当前硬件状态”和“已知问题”，尤其是幻尔总线舵机部分。

## 1. 当前状态一句话版

当前工程已经合并了底盘队友的新版底盘控制：左摇杆控制底盘平移，拨轮 `dial` 控制底盘旋转。

机械臂底部达妙电机使用 CAN2，当前能初始化，进入程序后会缓慢到初始化角 `100 deg`，之后可用右拨杆三档选择 200/400/600 高度 KFS 预设，并用右摇杆 Y 轴微调达妙角度。

气路已经加入：`PC8` 控制气泵继电器，`PC2` 控制电磁阀继电器，高电平导通，低电平关闭。当前用左拨杆 `switch_left == 2` 时气泵和电磁阀同时打开。

幻尔总线舵机代码已经按旧工程的总线协议逻辑整理过，使用 USART2 115200，ID1 为吸盘俯仰，ID2 为吸盘左右旋转。但是注意：截至本 README 写入时，现场测试结果仍然是“集成工程里舵机未成功使能/未运动”。代码里已经保留了详细调试变量，后续需要用示波器或逻辑分析仪确认 STM32 USART2_TX 是否真的发出正确波形。

## 2. 工程如何打开、编译、烧录

Keil 工程文件：

```text
MDK-ARM/VIT6.uvprojx
```

目标名：

```text
VIT6
```

芯片：

```text
STM32H743VITx
```

Keil Pack：

```text
Keil.STM32H7xx_DFP.4.1.3
```

编译器：

```text
ARMCLANG V6.24
```

推荐操作步骤：

1. 用 Keil uVision 打开 `MDK-ARM/VIT6.uvprojx`。
2. 确认当前 Target 为 `VIT6`。
3. 确认 Compiler 显示为 `V6.24`。
4. 点击 Rebuild。
5. 如果没有错误，再进入 Debug 或直接 Download。

命令行编译示例：

```powershell
& 'E:\Keil_Core\UV4\UV4.exe' -b 'D:\STM_Projects\RoboCon\2026\STM32H743VIT6-main\STM32H743VIT6-main\MDK-ARM\VIT6.uvprojx' -j0 -o 'D:\STM_Projects\RoboCon\2026\STM32H743VIT6-main\STM32H743VIT6-main\outputs\keil_build.log'
```

注意：本仓库 `.gitignore` 不提交 `MDK-ARM/VIT6/*.hex`、`*.axf`、`*.o` 等编译产物，因为这些文件经常包含本机绝对路径，也容易因为 Keil 版本或本地路径不同产生无意义冲突。队友拉代码后需要自己 Rebuild。

## 3. 目录结构和主要文件

```text
APPLICATION/
  arm.c / arm.h        机械臂、达妙、幻尔舵机、气路控制
  chasis.c / chassis.h 底盘控制，含舵轮运动学、IMU 修正、遥控映射
  catch.c / catch.h    原抓取/夹爪相关模块
  robot_def.h          遥控器范围、底盘最大速度等全局宏

BSP/
  bsp_fdcan.c / h      FDCAN 注册、过滤器、中断、发送接口
  bsp_dwt.c / h        DWT 计时

Core/
  Src/main.c           CubeMX 主入口，外设初始化，调用各模块 Init
  Src/freertos.c       FreeRTOS 任务创建和任务循环
  Src/fdcan.c          FDCAN1/FDCAN2 底层初始化
  Src/usart.c          USART1/USART2/USART6 初始化
  Src/gpio.c           GPIO 初始化，含 PC8/PC2 继电器输出

MDK-ARM/
  VIT6.uvprojx         Keil 工程
  HEmotor.c / h        幻尔总线舵机协议发送代码
  bsp_usart.c / h      USART BSP 注册与发送接口

MOUDLE/
  motor/
    dji_motor.c / h    大疆 M3508/GM6020/C620 等电机封装
    dm_motor.c / h     达妙电机封装
    feite_motor.c / h  飞特舵机封装
  remote/
    remote.c / h       遥控器数据解析
  BMI088/              BMI088 传感器
  algorithm/           PID 等算法
```

## 4. 系统初始化流程

主入口在 `Core/Src/main.c`。

外设初始化顺序大致为：

```text
HAL_Init
SystemClock_Config
MX_GPIO_Init
MX_DMA_Init
MX_FDCAN1_Init
MX_FDCAN2_Init
MX_SPI1_Init
MX_USART6_UART_Init
MX_SPI4_Init
MX_USART1_UART_Init
MX_USART2_UART_Init
MX_USB_DEVICE_Init
DWT_Init(400)
ChassisInit()
USB_Init()
RemoteControlInit()
CatchInit()
Arm_Init()
osKernelStart()
```

这意味着：

1. FDCAN、USART、GPIO 在各模块 Init 前已经由 CubeMX 初始化完成。
2. 底盘、遥控器、机械臂都在 FreeRTOS 启动前完成实例注册。
3. `Arm_Init()` 里会立刻初始化达妙和幻尔舵机实例，所以只要上电并 Run，机械臂模块就会开始动作/发包，不需要先进入某个遥控模式才初始化。

FreeRTOS 任务在 `Core/Src/freertos.c`：

```text
Chassis_Task  -> ChassisTask()      周期 1 ms
DJIMotor_Task -> DJIMotorControl() + FeiteMotorControl() 周期 1 ms
Remot_Task    -> RemoteControlTask()，等待 USART6 接收事件
Catch_Task    -> catch_start()
Arm_Task      -> Arm_Task()         周期 1 ms
usb_Task      -> USB task
```

机械臂 `Arm_Task()` 内部最后会调用：

```c
DMMotorControl();
HEMotorControl();
```

所以达妙和幻尔舵机的控制帧在机械臂任务里发送；大疆底盘电机控制帧在 `DJIMotor_Task` 和底盘任务相关逻辑里发送。

## 5. 遥控器数据定义

遥控器解析在 `MOUDLE/remote/remote.c` 和 `MOUDLE/remote/remote.h`。

数据结构：

```c
typedef struct {
    uint8_t KEY[4];
    int16_t rocker_l_;     // 左摇杆 X
    int16_t rocker_l1;     // 左摇杆 Y
    int16_t rocker_r_;     // 右摇杆 X
    int16_t rocker_r1;     // 右摇杆 Y
    int16_t dial;          // 拨轮
    uint8_t switch_left;   // 左三档拨杆
    uint8_t switch_right;  // 右三档拨杆
} Remote_Data_s;
```

当前遥控器没有按键可用，机械臂档位临时全部用左右拨杆和右摇杆完成。

遥控器数值范围：

```text
摇杆满量程约为 +/-660
底盘摇杆死区 REMOTE_DEADBAND = 50
机械臂达妙微调死区 J1_REMOTE_DEADBAND = 40
吸盘左右舵机切换死区 HE_REMOTE_DEADBAND = 330
```

## 6. 底盘控制

主要文件：

```text
APPLICATION/chasis.c
APPLICATION/chassis.h
APPLICATION/robot_def.h
```

底盘当前控制映射：

```text
左摇杆 X -> vx
左摇杆 Y -> vy
拨轮 dial -> vw，底盘旋转
```

代码位置在 `ChassisTask()`：

```c
vx = -(float)remote_data->rocker_l_ / REMOTE_STICK_RANGE * REMOTE_MAX_LINEAR;
vy =  (float)remote_data->rocker_l1 / REMOTE_STICK_RANGE * REMOTE_MAX_LINEAR;
vw =  (float)remote_data->dial      / REMOTE_STICK_RANGE * REMOTE_MAX_ANGULAR * 0.5f;
```

底盘最大速度参数在 `APPLICATION/robot_def.h`：

```text
REMOTE_STICK_RANGE = 660.0
REMOTE_MAX_LINEAR  = 18000.0
REMOTE_MAX_ANGULAR = 15000.0
REMOTE_DEADBAND    = 50.0
```

底盘电机：

```text
行走电机：M3508
转向电机：GM6020
当前底盘电机都走 FDCAN2
```

底盘 ID 分配见 `ChassisInit()`：

```text
M3508 行走电机：
  motor_lf 使用 tx_id 3
  motor_rf 使用 tx_id 4
  motor_lb 使用 tx_id 1
  motor_rb 使用 tx_id 2

GM6020 转向电机：
  motor_steering_lf 使用 tx_id 3
  motor_steering_rf 使用 tx_id 4
  motor_steering_lb 使用 tx_id 1
  motor_steering_rb 使用 tx_id 2
```

底盘转向有一个 ID3 上电保护：

```text
CHASSIS_STEERING_ID3_STARTUP_GUARD_ENABLE = 1
CHASSIS_STEERING_STARTUP_HOLD_TICKS = 50
```

含义是左前转向电机 ID3 上电后先停一小段时间，再使能并进入闭环，避免启动瞬间乱动。

IMU 修正在 `chasis.c` 内部完成，使用 BMI088 和 EKF。若底盘出现“自己慢慢偏航”或“拨轮回中仍有旋转”，优先检查：

1. BMI088 是否在线。
2. `chassis_imu_data.status`。
3. 遥控器拨轮 `dial` 是否真正回零。
4. `REMOTE_DEADBAND` 是否需要加大。
5. `CHASSIS_IDLE_YAW_CORRECTION_*` 死区是否太小。

## 7. FDCAN 配置

底层文件：

```text
Core/Src/fdcan.c
BSP/bsp_fdcan.c
```

当前 FDCAN1 和 FDCAN2 都是 Classic CAN：

```text
FrameFormat = FDCAN_FRAME_CLASSIC
Mode        = FDCAN_MODE_NORMAL
AutoRetransmission = ENABLE
```

FDCAN1 引脚：

```text
PD0 -> FDCAN1_RX
PD1 -> FDCAN1_TX
```

FDCAN2 引脚：

```text
PB12 -> FDCAN2_RX
PB13 -> FDCAN2_TX
```

当前达妙和底盘代码都使用 `hfdcan2`。曾经排查达妙时试过 CAN1，但队友成功测试使用的是 CAN2，所以当前工程保持 CAN2。

CAN 线注意事项：

1. CANH/CANL 需要接对。
2. CAN 总线两端需要 120 ohm 终端电阻。
3. 如果总线上只有主控和一个电机，正常 CANH-CANL 之间静态测量常见约 60 ohm 或 120 ohm，具体取决于两端是否都接终端。若测到几十 kOhm，说明大概率没有有效终端。
4. 所有 CAN 设备必须共地。
5. 达妙当前参数按 1 Mbps 使用。

## 8. 达妙机械臂底部俯仰

主要文件：

```text
APPLICATION/arm.c
MOUDLE/motor/dm_motor.c
MOUDLE/motor/dm_motor.h
```

硬件角色：

```text
达妙电机位于机械臂底部，负责机械臂整体俯仰。
现场电机型号曾标为 DM-J4310-2EC。
当前代码 motor_type 写的是 DM4340。
```

如果后续更换达妙型号，必须确认 `dm_motor.c/h` 里的量程、控制模式、ID、反馈解析是否与新型号一致。

当前达妙 CAN 配置在 `Arm_Init()`：

```c
.fdcan_handle = &hfdcan2,
.tx_id = 1,
.rx_id = 0x000,
.motor_type = DM4340,
```

当前达妙控制模式：

```c
DMMotorSetControlMode(motor_j1, DM_MODE_POS_VEL);
```

也就是位置-速度模式。不是 MIT 模式，也不是大疆 C620/M3508 那种电流闭环命令。

上电初始化逻辑：

```text
J1_INIT_TARGET_DEG = 100 deg
J1_INIT_SPEED_RAD_S = 0.8 rad/s
```

程序 Run 后，不需要按键，机械臂会进入 `ARM_KFS_STAGE_INIT100`，缓慢转到 100 deg。到达目标附近：

```text
J1_READY_REACHED_DEG = 3 deg
```

之后进入 `ARM_KFS_STAGE_SELECT`，才开始响应三档 KFS 预设。

达妙角度范围：

```text
J1_TARGET_MIN_DEG = -90 deg
J1_TARGET_MAX_DEG = 130 deg
```

如果调试时发现达妙方向反了，优先看：

```text
J1_MOTOR_SIGN
motor_reverse_flag
feedback_reverse_flag
```

当前 `J1_MOTOR_SIGN = 1.0f`，这是现场调到方向正确后的状态。

## 9. KFS 三档机械臂预设

KFS 预设在 `APPLICATION/arm.c`：

```c
static const Arm_KfsPreset_s arm_kfs_presets[] = {
    {1U, 200U, 85.0f,  ARM_PITCH_KEEP_VERTICAL,   HE_YAW_STATE_FRONT},
    {2U, 400U, 80.0f,  ARM_PITCH_KEEP_HORIZONTAL, HE_YAW_STATE_RIGHT},
    {3U, 600U, 100.0f, ARM_PITCH_KEEP_HORIZONTAL, HE_YAW_STATE_RIGHT},
};
```

右拨杆 `switch_right` 对应关系：

```text
switch_right = 1 -> 200 mm KFS
switch_right = 2 -> 400 mm KFS
switch_right = 3 -> 600 mm KFS
```

三档达妙目标角：

```text
200 mm -> J1 = 85 deg
400 mm -> J1 = 80 deg
600 mm -> J1 = 100 deg
```

吸盘姿态设计：

```text
200 mm：
  吸取顶部，吸盘俯仰保持相对地面垂直
  左右舵机默认正前

400 mm：
  吸取侧面，吸盘俯仰保持相对地面水平
  左右舵机默认正右，可用右摇杆 X 切到正左

600 mm：
  吸取侧面，吸盘俯仰保持相对地面水平
  左右舵机默认正右，可用右摇杆 X 切到正左
```

达妙角度微调：

```text
右摇杆 Y 轴 rocker_r1 -> 微调当前 KFS 档位的达妙目标角
微调速度 J1_PRESET_TRIM_RATE_DEG_S = 240 deg/s
微调最大幅度 J1_PRESET_TRIM_MAX_DEG = +/-15 deg
```

吸盘左右切换：

```text
右摇杆 X 轴 rocker_r_ > +330 -> yaw 正右
右摇杆 X 轴 rocker_r_ < -330 -> yaw 正左
默认回中：
  200 mm 档默认回正前
  400/600 mm 档若已切到左/右则保持侧向，若处于正前则回到默认正右
```

## 10. 幻尔 HTD-45H 总线舵机

主要文件：

```text
APPLICATION/arm.c
MDK-ARM/HEmotor.c
MDK-ARM/HEmotor.h
MDK-ARM/bsp_usart.c
Core/Src/usart.c
```

硬件角色：

```text
ID1：吸盘俯仰舵机
ID2：吸盘左右旋转舵机
```

当前舵机位置常量：

```text
ID1 pitch：
  90  -> 正下
  450 -> 正前
  840 -> 正上

ID2 yaw：
  780 -> 正左
  380 -> 正前
  30  -> 正右
```

当前初始化目标：

```text
ID1 pitch -> 90
ID2 yaw   -> 380
```

USART 配置：

```text
USART2
BaudRate = 115200
PA2 -> USART2_TX
PA3 -> USART2_RX
```

代码发送协议：

```text
帧头：0x55 0x55
格式：0x55 0x55 ID LEN CMD PARAMS CHECKSUM
校验：~(ID + LEN + CMD + PARAMS) 的低 8 位
```

当前用到的幻尔指令：

```text
SERVO_MOVE_TIME_WRITE = 1
SERVO_LOAD_OR_UNLOAD_WRITE = 31
```

`HEmotor.c` 当前做了两件事：

1. 初始化时对使能的舵机发送 `load=1`，然后发送目标位置。
2. 运行中每 10 ms 刷新目标位置，每 500 ms 刷新一次 `load=1`。

一个重要实现细节：

当前 `HEmotor.c` 已经改成“同一条 UART 总线只注册一次，多舵机共享 USART 实例”。原因是旧工程只有一个幻尔舵机，`USARTRegister(&huart2)` 只会调用一次；现在有 ID1/ID2 两个舵机，如果每个舵机都重复注册同一个 `huart2`，会反复启动 USART2 DMA/接收服务，容易引入不可控问题。

### 10.1 当前幻尔舵机未闭环问题

截至 2026-06-28，现场测试结果是：

```text
达妙调试程序可控制舵机运动。
集成工程中，arm_debug 里的发送计数会增加，HAL 状态也可能为 0，但舵机仍未明显使能或运动。
```

这说明问题不一定在 C 代码的上层目标值，而可能在以下位置：

1. USART2_TX 实际波形没有到舵机控制板。
2. USART2 被 DMA/中断/其他模块重新配置。
3. 舵机控制板协议不是当前使用的 0x55 0x55 舵机直连协议，而是另一层控制板协议。
4. 舵机 ID 或波特率和当前假设不一致。
5. 只接单向 TX 时，控制板需要额外的方向控制或使能时序。
6. 舵机电源、共地、控制板供电、电平标准存在问题。

下一步建议不要再盲改代码，优先做硬件信号确认：

```text
1. 用示波器或逻辑分析仪测 PA2/USART2_TX。
2. Run 后看是否每 10 ms 有 0x55 0x55 开头的串口波形。
3. 解码波特率设为 115200, 8N1。
4. 看帧里 ID 是否在 1 和 2 之间交替。
5. 看 CMD 是否出现 31 和 1。
6. 若 TX 有正确波形但舵机不动，再用幻尔官方工具确认 ID/波特率/控制板协议。
7. 若 TX 没有波形，回到 `Core/Src/usart.c`、`MDK-ARM/bsp_usart.c`、`MDK-ARM/HEmotor.c` 排查。
```

推荐临时验证方案：

```text
方案 A：只保留 ID1 舵机发送，排除两个舵机共享总线问题。
方案 B：只保留 ID2 舵机发送。
方案 C：把 ID 改成广播 ID 254，发送 load=1 + move，确认是否是 ID 不匹配。
方案 D：把集成工程里的发送包和幻尔官方上位机/能动版本的发送包用逻辑分析仪对比。
```

注意：不要因为“Debug 变量在变”就认为串口物理层一定成功。Debug 变量只能说明 CPU 执行到了发送函数，不能说明线上的波形、电平和协议一定被舵机接受。

## 11. 气路控制

主要代码在 `APPLICATION/arm.c`。

引脚：

```text
PC8 -> 气泵继电器
PC2 -> 电磁阀继电器
```

电平：

```text
高电平 -> 继电器通
低电平 -> 继电器断
```

当前遥控逻辑：

```text
switch_left == 2 -> 气泵和电磁阀同时打开
其他档位       -> 气泵和电磁阀同时关闭
```

当前代码：

```c
uint8_t air_on = (remote_data->switch_left == AIR_REMOTE_SWITCH_ON) ? 1U : 0U;
Arm_AirSet(air_on, air_on);
```

如果以后要分开控制气泵和电磁阀，可以把 `Arm_AirSet(pump, valve)` 的两个参数分别接到不同遥控器开关、按键或自动流程上。

## 12. 调试变量

机械臂调试变量在 `APPLICATION/arm.h`：

```c
extern volatile Arm_Debug_s arm_debug;
```

Keil Debug 里建议直接 Watch `arm_debug`。

常用字段：

```text
arm_debug.task_count
  Arm_Task 运行次数。若不增加，说明机械臂任务没有跑。

arm_debug.sw
  当前右拨杆 switch_right。

arm_debug.arm_stage
  0 = 上电初始化到 100 deg
  1 = 已进入 KFS 三档选择

arm_debug.arm_ready_reached
  是否已经到达 100 deg 附近并允许进入三档逻辑。

arm_debug.current.j1
  当前达妙相对角度，单位 deg。

arm_debug.target.j1
  当前达妙目标角度，单位 deg。

arm_debug.j1_error_deg
  达妙目标角与反馈角差值。

arm_debug.j1_remote_raw
  右摇杆 Y 原始值 rocker_r1。

arm_debug.j1_trim_deg
  当前 KFS 档位的人工微调角度。

arm_debug.j1_limit_hit
  是否触达软件角度限位。

arm_debug.kfs_mode
  当前 KFS 档位，对应 switch_right。

arm_debug.kfs_height_mm
  当前 KFS 高度：200/400/600。

arm_debug.he_pitch_cmd_pos
  ID1 幻尔俯仰舵机目标值。

arm_debug.he_yaw_cmd_pos
  ID2 幻尔左右舵机目标值。

arm_debug.he_direct_send_count
  幻尔发送计数。若不增加，说明 HEMotorControl 没发包。

arm_debug.he_direct_move_count
  幻尔位置指令计数。

arm_debug.he_direct_load_count
  幻尔 load=1 使能刷新计数。

arm_debug.he_direct_last_id
  最近一次发送的幻尔 ID。

arm_debug.he_direct_last_cmd
  最近一次发送的幻尔指令。1 表示位置，31 表示 load/unload。

arm_debug.he_direct_last_status
  HAL_UART_Transmit 返回值。0 是 HAL_OK。

arm_debug.he_direct_timeout_count
  USART 发送超时次数。

arm_debug.he_direct_busy_count
  USART 忙次数。

arm_debug.air_pump_on
  气泵继电器状态。

arm_debug.air_valve_on
  电磁阀继电器状态。

arm_debug.dm_feedback_initialized
  达妙反馈是否已经初始化。

arm_debug.dm_decode_count
  达妙反馈解析次数。

arm_debug.dm_control_count
  达妙控制发送次数。

arm_debug.dm_last_tx_ok
  最近一次达妙 CAN 发送是否成功。
```

## 13. 达妙发热和“堵转”问题

机械臂长时间维持一个抬起角度时，达妙发热明显，这是正常风险。它不一定是传统意义上的“撞死堵转”，但本质上是电机长时间输出静态力矩来抵抗机械臂重力，所以电流持续存在，热量会累积。

缓解方向：

1. 机械结构上加配重、弹簧、气弹簧或平衡机构，减少电机需要持续扛的重力矩。
2. 在程序上降低保持力矩/电流限制，但会牺牲抗下垂能力。
3. 只在需要抓取/保持时维持角度，空闲时回到低力矩安全位。
4. 加温度监控，温度高时降额或退回安全角。
5. 增加机械刹车或自锁结构。
6. 调整重心，让 100 deg、85 deg、80 deg 这些常用姿态的静态力矩更小。

当前代码还没有加入温度保护或力矩降额逻辑，后续如果比赛长时间运行，建议必须补。

## 14. 大疆 C620/M3508 与达妙控制差异

大疆 M3508 + C620：

```text
常见控制量是电流/速度/角度闭环中的电流输出。
反馈是电调发回编码器角度、速度、电流等。
CAN ID 通常以 0x200/0x1FF 等组帧发送。
电机控制逻辑在 dji_motor.c/h。
```

达妙：

```text
电机内部驱动器直接支持 MIT、位置速度、速度、混合等模式。
当前工程使用 DM_MODE_POS_VEL。
需要设置 tx_id/rx_id、控制模式、位置/速度目标。
控制命令不是大疆 C620 电流命令，不要混用 dji_motor 接口。
控制逻辑在 dm_motor.c/h。
```

现场达妙已知参数：

```text
CAN ID = 0x001
MASTER ID = 0x000
CAN Baud = 1.00 Mbps
Control Mode 曾显示为 position-speed cascade Mode
```

## 15. 常见调试顺序

如果整车上电后底盘不动：

```text
1. 看 remote_data 是否有变化。
2. 看 ChassisTask 是否运行。
3. 看 CAN2 物理层和终端电阻。
4. 看 DJIMotorControl 是否运行。
5. 看 M3508/GM6020 ID 是否与代码一致。
```

如果达妙不动：

```text
1. 看 arm_debug.task_count 是否增加。
2. 看 arm_debug.dm_feedback_initialized 是否为 1。
3. 看 arm_debug.dm_decode_count 是否增加。
4. 看 arm_debug.dm_last_tx_ok 是否为 1。
5. 看 CAN2 是否 1 Mbps，H/L 是否接对，终端电阻是否存在。
6. 确认达妙当前模式是否兼容 DM_MODE_POS_VEL。
7. 确认代码里 fdcan_handle 是否还是 &hfdcan2。
```

如果达妙方向反：

```text
1. 不要立刻乱改 KFS 角度。
2. 先看 J1_MOTOR_SIGN。
3. 再看 motor_reverse_flag 和 feedback_reverse_flag。
4. 改完必须重新验证初始化 100 deg、200/400/600 三档。
```

如果气路不动作：

```text
1. 看 arm_debug.air_pump_on 和 arm_debug.air_valve_on。
2. 看 switch_left 是否等于 2。
3. 万用表测 PC8/PC2 是否输出高电平。
4. 确认继电器板 GND 与主控共地。
5. 确认继电器板输入是高电平有效。
```

如果幻尔舵机不动：

```text
1. 看 arm_debug.he_direct_send_count 是否增加。
2. 看 arm_debug.he_direct_last_status 是否为 0。
3. 看 USART2_TX PA2 是否有 115200 波形。
4. 看帧头是否为 0x55 0x55。
5. 看 ID 是否为 1/2。
6. 看 CMD 是否为 31 或 1。
7. 用官方上位机确认舵机 ID 和波特率。
8. 如果官方能动，记录官方发出的字节流，再和本工程字节流对比。
```

## 16. 不建议随手改的地方

不要随手改这些东西，除非你知道后果：

```text
Core/Src/fdcan.c 的 CAN 位时序
Core/Src/usart.c 的 USART2 波特率
APPLICATION/arm.c 的 J1_MOTOR_SIGN
APPLICATION/arm.c 的 KFS 三档角度
APPLICATION/chasis.c 的底盘电机 ID
MDK-ARM/VIT6.uvprojx 的编译器版本
```

如果必须改，建议一次只改一个变量，烧录验证，再提交。不要一次改方向、ID、波特率、限位、遥控映射，否则现场会很难判断是谁导致的变化。

## 17. 当前关键参数速查

```text
Keil Compiler: ARMCLANG V6.24
MCU: STM32H743VITx
Target: VIT6

FDCAN1:
  PD0 RX
  PD1 TX

FDCAN2:
  PB12 RX
  PB13 TX

达妙 J1:
  CAN2
  tx_id = 1
  rx_id = 0x000
  mode = DM_MODE_POS_VEL
  init target = 100 deg
  software range = -90 deg to 130 deg

幻尔 ID1 pitch:
  USART2
  baud = 115200
  down = 90
  front = 450
  up = 840

幻尔 ID2 yaw:
  USART2
  baud = 115200
  right = 30
  front = 380
  left = 780

气路:
  PC8 pump relay, high active
  PC2 valve relay, high active

遥控:
  left stick -> chassis translation
  dial -> chassis rotation
  right switch 1/2/3 -> KFS 200/400/600
  right stick Y -> J1 trim
  right stick X -> suction yaw left/right
  left switch == 2 -> pump and valve on
```

## 18. Git 使用建议

这个分支是备份分支：

```text
TLS_R1_FullVersion_Backup_Keil
```

建议后续开发流程：

```text
1. 不要直接在 main 上乱推。
2. 新功能从本分支或 main 新开分支。
3. 每次只提交一个明确主题，比如 fix-hiwonder-uart 或 tune-arm-j1-speed。
4. 提交前至少 Rebuild 一次。
5. 硬件验证结果写进 commit message 或 README 的更新记录。
```

本 README 不是最终结论，而是当前交接快照。下一位同学如果修好了幻尔舵机，请务必把“未闭环问题”这一节更新成最终原因和解决方式。这样我们不是在代码里留下谜题，而是在项目里留下路径。
