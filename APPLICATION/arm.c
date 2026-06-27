#include "arm.h"
#include "dm_motor.h"
#include "dji_motor.h"
#include "feite_motor.h"
#include "HEmotor.h"
#include "arm_kinematics.h"
#include "remote.h"
#include "bsp_dwt.h"
#include "bsp_fdcan.h"
#include "main.h"
#include "general_def.h"
#include <math.h>

#define AIR_PUMP_GPIO_PORT  GPIOC
#define AIR_PUMP_GPIO_PIN   GPIO_PIN_8
#define AIR_VALVE_GPIO_PORT GPIOC
#define AIR_VALVE_GPIO_PIN  GPIO_PIN_2
#define AIR_REMOTE_SWITCH_ON 2U

/* ===================== 电机实例 ===================== */

static DMMotor_Instance *motor_j1;    /* 大臂 DM4340 */
static FeiteMotor_Instance *motor_j3; /* 末端舵机, ID=4 */
static HEMotor_Instance *motor_he;    /* 幻尔舵机 */

/* HE 舵机参数 */
#define HE_INIT_POS      94.0f  /* 初始发送值 (垂直) */
#define HE_RAW_PER_DEG   4.17f  /* 达妙每转 1 度，HE 发送值变化 4.17 */

/* J2 减速比 & 同步带耦合系数:
 * M3508 内置 1:19 减速器, 编码器测量的是转子角度 (减速前),
 * total_angle 是转子角度, 需 /19 才得到输出轴角度.
 * 3508 固定在底座上, 通过同步带驱动小臂相对底座的绝对角:
 *   j2_关节 = J2_HOME_ANGLE_DEG + motor_rotor_angle / 19 - J2_COUPLING_RATIO * j1
 *   motor_rotor_ref = (j2_关节 - J2_HOME_ANGLE_DEG + J2_COUPLING_RATIO * j1) * 19
 * home 时大臂水平向前, 小臂反向水平折叠, 所以小臂相对大臂角为 180°. */
#define M3508_REDUCTION_RATIO 19.0f
#define J2_COUPLING_RATIO     1.0f
#define J2_HOME_ANGLE_DEG     180.0f
#define J1_MOTOR_SIGN         -1.0f
#define J1_INIT_POS_RAD       -2.2f
#define J1_POS_VEL_SPEED_RAD_S 5.0f
#define J1_CONTROL_PERIOD_S    0.001f
#define J1_REMOTE_DEADBAND     40
#define J1_REMOTE_FULL_SCALE   660.0f
#define J1_MANUAL_MAX_SPEED_DEG_S 286.0f
#define J1_SPEED_ACCEL_RAD_S2  10.0f
#define J1_TARGET_LEAD_MIN_DEG 4.0f
#define J1_TARGET_LEAD_MAX_DEG 28.0f
#define J1_TARGET_MIN_DEG      -90.0f
#define J1_TARGET_MAX_DEG      90.0f

/* Hiwonder bus servo positions for the suction cup. */
#define HE_SERVO_MOVE_TIME_MS       300U
#define HE_REMOTE_DEADBAND          330

#define HE_PITCH_ID                 1U
#define HE_PITCH_DOWN_POS           90U
#define HE_PITCH_FRONT_POS          450U
#define HE_PITCH_UP_POS             840U

#define HE_YAW_ID                   2U
#define HE_YAW_RIGHT_POS            30U
#define HE_YAW_FRONT_POS            380U
#define HE_YAW_LEFT_POS             780U

static HEMotor_Instance *motor_suction_yaw;
static uint16_t suction_pitch_target = HE_PITCH_DOWN_POS;
static uint16_t suction_yaw_target = HE_YAW_FRONT_POS;
static uint8_t suction_pitch_state = 0U; /* 0=down, 1=front, 2=up */
static uint8_t suction_yaw_state = 1U;   /* 0=left, 1=front, 2=right */
static uint8_t suction_pitch_remote_armed = 0U;

/* ===================== 状态机 ===================== */

typedef enum {
    ARM_STATE_INIT = 0,
    ARM_STATE_SOFT_START,
    ARM_STATE_RUN,
} Arm_State_e;

static volatile Arm_State_e arm_state = ARM_STATE_INIT;
static float soft_start_timestamp = 0.0f;

/* ===================== 零点 & 基准位姿 ===================== */

static float j1_zero_offset_deg = 0.0f;   /* 上电时锁定的 J1 零点 (度) */
static uint8_t j1_zero_inited = 0;
static float j1_zero_abs_rad = J1_INIT_POS_RAD;
static float j1_cmd_abs_rad = J1_INIT_POS_RAD;
static float j1_cmd_speed_rad_s = 0.0f;
static float j1_remote_rate_deg_s = 0.0f;
static float j1_remote_speed_rad_s = 0.0f;
static float j1_speed_cmd_rad_s = 0.0f;
static uint8_t j1_limit_hit = 0U;

static Arm_JointAngles_t home_angles;     /* 上电锁定的 home 关节角 */
static float home_wrist_x = 0.0f;         /* home 时 L2 末端 (腕点) X 坐标 */
static float home_wrist_y = 0.0f;         /* home 时 L2 末端 (腕点) Y 坐标 */

/* ===================== 当前 & 目标 ===================== */

static Arm_JointAngles_t current_angles;  /* 本周期读到的当前角度 */
static Arm_JointAngles_t target_angles;   /* 下发给电机的目标角度 */

/* ===================== 调试观测 ===================== */

volatile Arm_Debug_s arm_debug;
#if 0
static volatile struct {
    Arm_JointAngles_t current;   /* 当前读到的角度 */
    Arm_JointAngles_t target;    /* 本周期下发的目标 */
    Arm_JointAngles_t last_ik;   /* 最近一次 IK 解算结果 (无论成败) */
    float home_wrist_x;          /* home 时记录的腕点 X */
    float home_wrist_y;          /* home 时记录的腕点 Y */
    float wrist_x;               /* sw==2 时解算出的腕点目标 X */
    float wrist_y;               /* sw==2 时解算出的腕点目标 Y */
    float wrist_r;               /* 腕点目标距原点距离 (用于判断是否超出 IK 工作空间) */
    uint8_t last_ik_ret;         /* 最近一次 Arm_IK 返回值 (0=超工作空间) */
    uint8_t last_limits_ok;      /* 最近一次 CheckJointLimits 结果 */
    uint8_t sw;                  /* 本周期读到的拨杆档位 */
    uint16_t ik_ok_count;        /* IK 成功次数 */
    uint16_t ik_fail_count;      /* IK 失败次数 (含超限) */
    float he_cmd_pos;            /* 当前 HE 发送值 */
    float he_follow_deg;         /* HE 跟随使用的 J1 角度 */
    uint16_t he_pitch_cmd_pos;
    uint16_t he_yaw_cmd_pos;
    uint8_t he_pitch_state;
    uint8_t he_yaw_state;
    uint8_t air_pump_on;
    uint8_t air_valve_on;
    float j1_zero_abs_rad;
    float j1_cmd_abs_rad;
    float j1_cmd_speed_rad_s;
    float j1_remote_rate_deg_s;
    int16_t j1_remote_raw;
    uint8_t j1_zero_ready;
    uint8_t j1_limit_hit;
} arm_debug_disabled;
#endif

/* ===================== 辅助函数 ===================== */

/* PC8: pump relay, PC2: valve relay. High level turns the relay on. */
static void Arm_AirSet(uint8_t pump_enable, uint8_t valve_enable)
{
    uint8_t pump_on = (pump_enable != 0U) ? 1U : 0U;
    uint8_t valve_on = (valve_enable != 0U) ? 1U : 0U;

    HAL_GPIO_WritePin(AIR_PUMP_GPIO_PORT,
                      AIR_PUMP_GPIO_PIN,
                      pump_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIR_VALVE_GPIO_PORT,
                      AIR_VALVE_GPIO_PIN,
                      valve_on ? GPIO_PIN_SET : GPIO_PIN_RESET);

    arm_debug.air_pump_on = pump_on;
    arm_debug.air_valve_on = valve_on;
}

static void Arm_SuctionApplyTargets(void)
{
    if (motor_he != NULL) {
        motor_he->ref.position = suction_pitch_target;
        motor_he->ref.time = HE_SERVO_MOVE_TIME_MS;
        motor_he->ref.stop_flag = HE_ENABLED;
    }

    if (motor_suction_yaw != NULL) {
        motor_suction_yaw->ref.position = suction_yaw_target;
        motor_suction_yaw->ref.time = HE_SERVO_MOVE_TIME_MS;
        motor_suction_yaw->ref.stop_flag = HE_ENABLED;
    }

    arm_debug.he_pitch_cmd_pos = suction_pitch_target;
    arm_debug.he_yaw_cmd_pos = suction_yaw_target;
    arm_debug.he_pitch_state = suction_pitch_state;
    arm_debug.he_yaw_state = suction_yaw_state;
}

static void Arm_SuctionUpdateTargetsFromRemote(const Remote_Data_s *remote)
{
    if (remote == NULL) {
        return;
    }

    if (remote->rocker_r_ > HE_REMOTE_DEADBAND) {
        suction_yaw_target = HE_YAW_RIGHT_POS;
        suction_yaw_state = 2U;
    } else if (remote->rocker_r_ < -HE_REMOTE_DEADBAND) {
        suction_yaw_target = HE_YAW_LEFT_POS;
        suction_yaw_state = 0U;
    } else {
        suction_yaw_target = HE_YAW_FRONT_POS;
        suction_yaw_state = 1U;
    }

    if (remote->dial > HE_REMOTE_DEADBAND) {
        suction_pitch_target = HE_PITCH_UP_POS;
        suction_pitch_state = 2U;
        suction_pitch_remote_armed = 1U;
    } else if (remote->dial < -HE_REMOTE_DEADBAND) {
        suction_pitch_target = HE_PITCH_DOWN_POS;
        suction_pitch_state = 0U;
        suction_pitch_remote_armed = 1U;
    } else if (suction_pitch_remote_armed != 0U) {
        suction_pitch_target = HE_PITCH_FRONT_POS;
        suction_pitch_state = 1U;
    }
}

/**
 * @brief 读取 3 个关节的当前角度
 * @note  J1 减去上电锁定的零点偏置, 使 home 位置对应 J1=0
 *        J2 减去 J1 耦合分量, 还原成"小臂相对大臂"的关节角
 */
static float Arm_LimitFloat(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float Arm_ApproachFloat(float current, float target, float step)
{
    if (current < target) {
        current += step;
        if (current > target) {
            current = target;
        }
    } else if (current > target) {
        current -= step;
        if (current < target) {
            current = target;
        }
    }
    return current;
}

static float Arm_J1MotorRadToJointDeg(float motor_rad)
{
    return J1_MOTOR_SIGN * (motor_rad - j1_zero_abs_rad) * RAD_2_DEGREE;
}

static float Arm_J1JointDegToMotorRad(float joint_deg)
{
    return j1_zero_abs_rad + J1_MOTOR_SIGN * joint_deg * DEGREE_2_RAD;
}

static void Arm_J1ApplyTarget(float speed_rad_s)
{
    if (motor_j1 == NULL) {
        return;
    }

    target_angles.j1 = Arm_LimitFloat(target_angles.j1, J1_TARGET_MIN_DEG, J1_TARGET_MAX_DEG);
    j1_cmd_abs_rad = Arm_J1JointDegToMotorRad(target_angles.j1);
    j1_cmd_speed_rad_s = speed_rad_s;
    DMMotorSetPosVelRef(motor_j1, j1_cmd_abs_rad, speed_rad_s);
}

static void Arm_J1LatchZeroIfReady(void)
{
    if ((motor_j1 == NULL) || (motor_j1->feedback_initialized == 0U) || (j1_zero_inited != 0U)) {
        return;
    }

    j1_zero_abs_rad = motor_j1->measure.position_rad;
    j1_zero_offset_deg = j1_zero_abs_rad * RAD_2_DEGREE;
    target_angles.j1 = 0.0f;
    current_angles.j1 = 0.0f;
    j1_zero_inited = 1U;
    Arm_J1ApplyTarget(0.0f);
}

static void Arm_J1UpdateTargetFromRemote(const Remote_Data_s *remote)
{
    float raw;
    float normalized;
    float desired_speed_rad_s;
    float speed_step;
    float lead_deg;
    float target_deg;

    j1_remote_rate_deg_s = 0.0f;
    j1_limit_hit = 0U;

    if ((remote == NULL) || (j1_zero_inited == 0U)) {
        return;
    }

    raw = (float)remote->rocker_r1;
    if (fabsf(raw) < (float)J1_REMOTE_DEADBAND) {
        raw = 0.0f;
    }

    normalized = fabsf(raw) / J1_REMOTE_FULL_SCALE;
    if (normalized > 1.0f) {
        normalized = 1.0f;
    }

    desired_speed_rad_s = raw / J1_REMOTE_FULL_SCALE * J1_POS_VEL_SPEED_RAD_S;
    desired_speed_rad_s = Arm_LimitFloat(desired_speed_rad_s,
                                         -J1_POS_VEL_SPEED_RAD_S,
                                         J1_POS_VEL_SPEED_RAD_S);
    speed_step = J1_SPEED_ACCEL_RAD_S2 * J1_CONTROL_PERIOD_S;
    j1_speed_cmd_rad_s = Arm_ApproachFloat(j1_speed_cmd_rad_s,
                                           desired_speed_rad_s,
                                           speed_step);

    if (raw == 0.0f) {
        j1_speed_cmd_rad_s = 0.0f;
        target_angles.j1 = current_angles.j1;
    } else {
        lead_deg = J1_TARGET_LEAD_MIN_DEG +
                   normalized * (J1_TARGET_LEAD_MAX_DEG - J1_TARGET_LEAD_MIN_DEG);
        target_deg = current_angles.j1 +
                     ((j1_speed_cmd_rad_s > 0.0f) ? lead_deg : -lead_deg);
        target_angles.j1 = Arm_LimitFloat(target_deg, J1_TARGET_MIN_DEG, J1_TARGET_MAX_DEG);
        if (target_angles.j1 != target_deg) {
            j1_limit_hit = 1U;
        }
    }

    j1_remote_rate_deg_s = j1_speed_cmd_rad_s * RAD_2_DEGREE;
    j1_remote_speed_rad_s = fabsf(j1_speed_cmd_rad_s);
}

static void Arm_ReadJointAngles(Arm_JointAngles_t *angles)
{
    if (motor_j1 && motor_j1->feedback_initialized)
        angles->j1 = Arm_J1MotorRadToJointDeg(motor_j1->measure.position_rad);

    if (motor_j3)
        angles->j3 = motor_j3->measure.angle_deg;
}

static void Arm_SetAllRefs(Arm_JointAngles_t angles)
{
    int16_t j3_raw = (int16_t)(angles.j3 / FEITE_DEFAULT_RAW_TO_DEG);
    if (motor_j3) {
        FeiteMotorSetRef(motor_j3, j3_raw);
        FeiteMotorSetSpeed(motor_j3, 500);
        FeiteMotorSetAcc(motor_j3, 20);
    }
}

/**
 * @brief 2 连杆正运动学: (j1, j2) → 腕点 (L2 末端) 坐标
 * @note  用于记录 home 腕点位置; 刻意不调 Arm_FK (3 连杆) 保持与 2 连杆 IK 一致
 */
static void Arm_Compute2LWrist(Arm_JointAngles_t angles, float *x, float *y)
{
    float a1  = angles.j1 * DEGREE_2_RAD;
    float a12 = a1 + angles.j2 * DEGREE_2_RAD;
    *x = ARM_L1 * cosf(a1) + ARM_L2 * cosf(a12);
    *y = ARM_L1 * sinf(a1) + ARM_L2 * sinf(a12);
}

/* ===================== 初始化 ===================== */

void Arm_Init(void)
{
    Arm_AirSet(0U, 0U);

    /* ---- J1: DM4340 大臂电机 (模式) ---- */
    Motor_Init_Config_s dm_config = {
        .can_init_config = {
            .fdcan_handle = &hfdcan2,
            .tx_id = 1,
            .rx_id = 0x000,
        },
        .controller_setting_init_config = {
            .motor_reverse_flag     = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag  = FEEDBACK_DIRECTION_NORMAL,
            .angle_mode             = MOTOR_ANGLE_MODE_SINGLE_TURN,
            .close_loop_type        = OPEN_LOOP, // 设置为开环，绕过 host 端 PID/Profile
        },
        .motor_type = DM4340,
    };

    motor_j1 = DMMotorInit(&dm_config);

		DMMotorSetControlMode(motor_j1, DM_MODE_POS_VEL );
    target_angles.j1 = 0.0f;
    current_angles.j1 = 0.0f;
    j1_zero_abs_rad = J1_INIT_POS_RAD;
    j1_zero_offset_deg = J1_INIT_POS_RAD * RAD_2_DEGREE;
    j1_cmd_abs_rad = J1_INIT_POS_RAD;
    j1_cmd_speed_rad_s = J1_POS_VEL_SPEED_RAD_S;
    DMMotorSetPosVelRef(motor_j1, j1_cmd_abs_rad, j1_cmd_speed_rad_s);
    DMMotorEnable(motor_j1);


    /* ---- J3: 飞特舵机, USART1, ID=4 ---- */
    {
        FeiteMotor_Bus_s *bus = FeiteMotorBusInit(NULL);

        FeiteMotor_Init_Config_s j3_config = {
            .bus = bus,
            .id = 4,
            .model = FEITE_MODEL_HLS_SCS,
            .init_position = 0,
            .init_speed = 500,
            .init_acc = 20,
            .init_torque = 1500,
            .raw_to_deg = FEITE_DEFAULT_RAW_TO_DEG,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        };
        motor_j3 = FeiteMotorInit(&j3_config);
    }

    /* ---- HE: 幻尔舵机, USART1, ID=4 ---- */
    {
        HEMotor_Init_Config_s he_pitch_config = {
            .huart = &huart2,
            .motor_config = { .id = HE_PITCH_ID, .cmd = SERVO_MOVE_TIME_WRITE },
            .motor_ref = {
                .position = HE_PITCH_DOWN_POS,
                .time = HE_SERVO_MOVE_TIME_MS,
                .stop_flag = HE_ENABLED,
            },
        };
        HEMotor_Init_Config_s he_yaw_config = {
            .huart = &huart2,
            .motor_config = { .id = HE_YAW_ID, .cmd = SERVO_MOVE_TIME_WRITE },
            .motor_ref = {
                .position = HE_YAW_FRONT_POS,
                .time = HE_SERVO_MOVE_TIME_MS,
                .stop_flag = HE_ENABLED,
            },
        };

        motor_he = HEMotorInit(&he_pitch_config);
        motor_suction_yaw = HEMotorInit(&he_yaw_config);
        suction_pitch_target = HE_PITCH_DOWN_POS;
        suction_yaw_target = HE_YAW_FRONT_POS;
        suction_pitch_state = 0U;
        suction_yaw_state = 1U;
        suction_pitch_remote_armed = 0U;
        Arm_SuctionApplyTargets();
    }
}

/* ===================== 控制任务 ===================== */

void Arm_Task(void)
{
    uint8_t sw = 0U;

    if (remote_data != NULL) {
        sw = remote_data->switch_right;
        uint8_t air_on = (remote_data->switch_left == AIR_REMOTE_SWITCH_ON) ? 1U : 0U;
        Arm_AirSet(air_on, air_on);
    } else {
        Arm_AirSet(0U, 0U);
    }

    Arm_ReadJointAngles(&current_angles);
    arm_debug.task_count++;
    arm_debug.current = current_angles;
    arm_debug.sw = sw;
    arm_debug.dm_decode_count = g_dm_motor_debug.decode_count;
    arm_debug.dm_control_count = g_dm_motor_debug.control_count;
    arm_debug.dm_last_tx_id = g_dm_motor_debug.last_tx_std_id;
    arm_debug.dm_last_rx_id = g_dm_motor_debug.last_rx_id;
    arm_debug.dm_last_tx_ok = g_dm_motor_debug.last_tx_ok;
    if (motor_j1 != NULL) {
        arm_debug.dm_feedback_initialized = motor_j1->feedback_initialized;
        arm_debug.dm_stop_flag = (uint8_t)motor_j1->stop_flag;
        arm_debug.dm_state = motor_j1->measure.state;
        arm_debug.dm_motor_id = motor_j1->measure.motor_id;
        arm_debug.dm_position_rad = motor_j1->measure.position_rad;
        arm_debug.dm_velocity_rad_s = motor_j1->measure.velocity_rad_s;
        arm_debug.dm_torque_nm = motor_j1->measure.torque_nm;
        arm_debug.dm_pos_ref_rad = motor_j1->pos_ref;
        arm_debug.dm_vel_ref_rad_s = motor_j1->vel_ref;
    }

    /* 简化逻辑：不再使用复杂的初始化状态机，直接进入运行模式 */
    Arm_J1LatchZeroIfReady();
    j1_remote_rate_deg_s = 0.0f;
    j1_limit_hit = 0U;

    /* 遥控器控制: sw1=停止, sw2=手动控制达妙电机(J1), sw3=保持当前位置 */
    if ((sw == 1U) && (j1_zero_inited != 0U)) {
        target_angles.j1 = current_angles.j1;
        j1_cmd_abs_rad = motor_j1->measure.position_rad;
        j1_cmd_speed_rad_s = 0.0f;
        j1_speed_cmd_rad_s = 0.0f;
        j1_remote_speed_rad_s = 0.0f;
        DMMotorSetPosVelRef(motor_j1, j1_cmd_abs_rad, j1_cmd_speed_rad_s);
    } else if ((sw == 2U) && (remote_data != NULL)) {
        Arm_J1UpdateTargetFromRemote(remote_data);
        Arm_J1ApplyTarget(j1_remote_speed_rad_s);
    } else {
        if (j1_zero_inited != 0U) {
            target_angles.j1 = current_angles.j1;
            j1_speed_cmd_rad_s = 0.0f;
            j1_remote_speed_rad_s = 0.0f;
            Arm_J1ApplyTarget(0.0f);
        } else {
            Arm_J1ApplyTarget(0.0f);
        }
    }

    arm_debug.target = target_angles;
    arm_debug.j1_zero_abs_rad = j1_zero_abs_rad;
    arm_debug.j1_cmd_abs_rad = j1_cmd_abs_rad;
    arm_debug.j1_cmd_speed_rad_s = j1_cmd_speed_rad_s;
    arm_debug.j1_remote_rate_deg_s = j1_remote_rate_deg_s;
    arm_debug.j1_remote_speed_rad_s = j1_remote_speed_rad_s;
    arm_debug.j1_error_deg = target_angles.j1 - current_angles.j1;
    arm_debug.j1_remote_raw = (remote_data != NULL) ? remote_data->rocker_r1 : 0;
    arm_debug.j1_zero_ready = j1_zero_inited;
    arm_debug.j1_limit_hit = j1_limit_hit;

    /* HE 舵机按 J1 目标角度同向跟随：若方向仍不对，再改回减号即可 */
    if ((sw == 2) && (remote_data != NULL)) {
        Arm_SuctionUpdateTargetsFromRemote(remote_data);
    }
    Arm_SuctionApplyTargets();

    /* DM 电机控制发送 (位置速度模式正常遥控) */
    DMMotorControl();
    HEMotorControl();
}
