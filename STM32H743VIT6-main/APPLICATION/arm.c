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

#define AIR_MODULE_1_GPIO_PORT GPIOC
#define AIR_MODULE_1_GPIO_PIN  GPIO_PIN_8
#define AIR_MODULE_2_GPIO_PORT GPIOD
#define AIR_MODULE_2_GPIO_PIN  GPIO_PIN_3
#define AIR_MODULE_ON_LEVEL    GPIO_PIN_SET
#define AIR_MODULE_OFF_LEVEL   GPIO_PIN_RESET
#define AIR_REMOTE_SWITCH_ON   2U

/* ===================== 电机实例 ===================== */

static DMMotor_Instance *motor_j1;    /* 大臂 DM4340 */
static DMMotor_Instance *motor_j2;    /* 新增达妙电机 */
static HEMotor_Instance *motor_j3;    /* 4号幻尔舵机, USART3 */
static HEMotor_Instance *motor_he;    /* 3号幻尔舵机, USART3 */
static HEMotor_Instance *motor_h1;    /* 1号幻尔舵机, USART6 */
static HEMotor_Instance *motor_h2;    /* 2号幻尔舵机, USART6 */

/* 舵机与达妙控制参数 */
#define SERVO3_INIT_POS      470U
#define SERVO4_INIT_POS      685U
#define SERVO3_PRESET_1_POS  115U
#define SERVO4_PRESET_1_POS  661U
#define SERVO3_PRESET_2_POS  107U
#define SERVO4_PRESET_2_POS  288U
#define SERVO3_PRESET_3_POS  100U
#define SERVO4_PRESET_3_POS  685U
#define SERVO_MOVE_TIME_MS   100U
#define J1_INIT_POS_RAD      (-2.2f)
#define J1_RC_STEP_RAD       0.03f
#define J1_MAX_VEL_RAD_S     12.0f
#define J1_PRESET_1_DEG      (-40.336974f)
#define J1_PRESET_2_DEG      (-44.817675f)
#define J1_PRESET_3_DEG      0.0f
#define H1_INIT_POS          390U
#define H2_INIT_POS          440U
#define H1_PRESET_1_POS      388U
#define H2_PRESET_1_POS      86U
#define H1_PRESET_2_POS      781U
#define H2_PRESET_2_POS      604U
#define H1_PRESET_3_POS      780U
#define H2_PRESET_3_POS      450U
#define J2_MAX_VEL_RAD_S     12.0f
#define J2_RC_STEP_RAD       0.03f
#define J2_PRESET_1_DEG      85.930549f
#define J2_PRESET_2_DEG      80.444420f
#define J2_PRESET_3_DEG      0.0f
#define J2_MOTOR_SIGN        1.0f

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

static Arm_JointAngles_t home_angles;     /* 上电锁定的 home 关节角 */
static float home_wrist_x = 0.0f;         /* home 时 L2 末端 (腕点) X 坐标 */
static float home_wrist_y = 0.0f;         /* home 时 L2 末端 (腕点) Y 坐标 */

/* ===================== 当前 & 目标 ===================== */

static Arm_JointAngles_t current_angles;  /* 本周期读到的当前角度 */
static Arm_JointAngles_t target_angles;   /* 下发给电机的目标角度 */
static float target_j2_deg = 0.0f;
static uint8_t arm_last_left_switch = 0U;
static uint8_t arm_last_right_switch = 0U;
static uint8_t arm_last_key_state[4] = {0U, 0U, 0U, 0U};

/* ===================== 调试观测 ===================== */

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
    uint8_t air_pc8_on;
    uint8_t air_pd3_on;
} arm_debug;

/* ===================== 辅助函数 ===================== */

/* Air outputs: high = on, low = off. */
static void Arm_AirOutputSet(GPIO_TypeDef *gpio_port, uint16_t gpio_pin, uint8_t enable)
{
    uint8_t on = (enable != 0U) ? 1U : 0U;

    HAL_GPIO_WritePin(gpio_port,
                      gpio_pin,
                      on ? AIR_MODULE_ON_LEVEL : AIR_MODULE_OFF_LEVEL);
}

static void Arm_AirModule1Set(uint8_t enable)
{
    uint8_t on = (enable != 0U) ? 1U : 0U;

    Arm_AirOutputSet(AIR_MODULE_1_GPIO_PORT, AIR_MODULE_1_GPIO_PIN, on);
    arm_debug.air_pc8_on = on;
}

static void Arm_AirModule2Set(uint8_t enable)
{
    uint8_t on = (enable != 0U) ? 1U : 0U;

    Arm_AirOutputSet(AIR_MODULE_2_GPIO_PORT, AIR_MODULE_2_GPIO_PIN, on);
    arm_debug.air_pd3_on = on;
}

static void Arm_ProcessAirKeys(uint8_t left_sw)
{
    uint8_t key_now[4] = {0U, 0U, 0U, 0U};

    if (remote_data != NULL) {
        key_now[0] = (remote_data->KEY[0] != 0U) ? 1U : 0U;
        key_now[1] = (remote_data->KEY[1] != 0U) ? 1U : 0U;
        key_now[2] = (remote_data->KEY[2] != 0U) ? 1U : 0U;
        key_now[3] = (remote_data->KEY[3] != 0U) ? 1U : 0U;
    }

    if ((left_sw == AIR_REMOTE_SWITCH_ON) && (remote_data != NULL)) {
        if ((key_now[0] != 0U) && (arm_last_key_state[0] == 0U)) {
            Arm_AirModule1Set(1U);
        }
        if ((key_now[1] != 0U) && (arm_last_key_state[1] == 0U)) {
            Arm_AirModule1Set(0U);
        }
        if ((key_now[2] != 0U) && (arm_last_key_state[2] == 0U)) {
            Arm_AirModule2Set(1U);
        }
        if ((key_now[3] != 0U) && (arm_last_key_state[3] == 0U)) {
            Arm_AirModule2Set(0U);
        }
    }

    arm_last_key_state[0] = key_now[0];
    arm_last_key_state[1] = key_now[1];
    arm_last_key_state[2] = key_now[2];
    arm_last_key_state[3] = key_now[3];
}

static void Arm_SetServoTargets(uint16_t servo3_pos, uint16_t servo4_pos)
{
    if (motor_he != NULL) {
        motor_he->ref.position = servo3_pos;
        motor_he->ref.time = SERVO_MOVE_TIME_MS;
    }
    if (motor_j3 != NULL) {
        motor_j3->ref.position = servo4_pos;
        motor_j3->ref.time = SERVO_MOVE_TIME_MS;
    }
}

static void Arm_SetServoTargets2(uint16_t h1_pos, uint16_t h2_pos)
{
    if (motor_h1 != NULL) {
        motor_h1->ref.position = h1_pos;
        motor_h1->ref.time = SERVO_MOVE_TIME_MS;
    }
    if (motor_h2 != NULL) {
        motor_h2->ref.position = h2_pos;
        motor_h2->ref.time = SERVO_MOVE_TIME_MS;
    }
}

static float Arm_GetPresetJ1Deg(uint8_t right_sw)
{
    switch (right_sw) {
        case 1U: return J1_PRESET_1_DEG;
        case 2U: return J1_PRESET_2_DEG;
        case 3U: return J1_PRESET_3_DEG;
        default: return target_angles.j1;
    }
}

static void Arm_ApplyPreset(uint8_t right_sw)
{
    switch (right_sw) {
        case 1U: Arm_SetServoTargets(SERVO3_PRESET_1_POS, SERVO4_PRESET_1_POS); break;
        case 2U: Arm_SetServoTargets(SERVO3_PRESET_2_POS, SERVO4_PRESET_2_POS); break;
        case 3U: Arm_SetServoTargets(SERVO3_PRESET_3_POS, SERVO4_PRESET_3_POS); break;
        default: Arm_SetServoTargets(SERVO3_INIT_POS, SERVO4_INIT_POS); return;
    }
    target_angles.j1 = Arm_GetPresetJ1Deg(right_sw);
}

static void Arm_ApplyPreset2(uint8_t right_sw)
{
    switch (right_sw) {
        case 1U: Arm_SetServoTargets2(H1_PRESET_1_POS, H2_PRESET_1_POS); target_j2_deg = J2_PRESET_1_DEG; break;
        case 2U: Arm_SetServoTargets2(H1_PRESET_2_POS, H2_PRESET_2_POS); target_j2_deg = J2_PRESET_2_DEG; break;
        case 3U: Arm_SetServoTargets2(H1_PRESET_3_POS, H2_PRESET_3_POS); target_j2_deg = J2_PRESET_3_DEG; break;
        default: Arm_SetServoTargets2(H1_INIT_POS, H2_INIT_POS); break;
    }
}

/**
 * @brief 读取 3 个关节的当前角度
 * @note  J1 减去上电锁定的零点偏置, 使 home 位置对应 J1=0
 *        J2 减去 J1 耦合分量, 还原成"小臂相对大臂"的关节角
 */
static void Arm_ReadJointAngles(Arm_JointAngles_t *angles)
{
    if (motor_j1 && motor_j1->feedback_initialized)
        angles->j1 = J1_MOTOR_SIGN *
                     (motor_j1->measure.position_rad * RAD_2_DEGREE - j1_zero_offset_deg);

    /* HE 舵机为开环控制，目前不读取反馈角度 */
    angles->j3 = 0.0f; 
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
    Arm_AirModule1Set(0U);
    Arm_AirModule2Set(0U);

    /* ---- J1: DM4340 大臂电机 (模式) ---- */
    Motor_Init_Config_s dm_config = {
        .can_init_config = {
            .fdcan_handle = &hfdcan2,
            .tx_id = 3,
            .rx_id = 0x13,
        },
        .controller_setting_init_config = {
            .motor_reverse_flag     = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag  = FEEDBACK_DIRECTION_NORMAL,
            .angle_mode             = MOTOR_ANGLE_MODE_SINGLE_TURN,
            .close_loop_type        = OPEN_LOOP, 
        },
        .motor_type = DM4340,
    };

    motor_j1 = DMMotorInit(&dm_config);

		DMMotorSetControlMode(motor_j1, DM_MODE_POS_VEL );
    DMMotorEnable(motor_j1);

    target_angles.j1 = J1_MOTOR_SIGN * (J1_INIT_POS_RAD * RAD_2_DEGREE);
    DMMotorSetPosVelRef(motor_j1, J1_INIT_POS_RAD, 4.0f); // 初始位置 -2.2rad

    /* ---- 4号幻尔舵机 ---- */
    {
        HEMotor_Init_Config_s j3_config = {
            .huart = &huart3,
            .motor_config = { .id = 4, .cmd = SERVO_MOVE_TIME_WRITE },
            .motor_ref = { .position = SERVO4_INIT_POS, .time = SERVO_MOVE_TIME_MS, .stop_flag = HE_ENABLED },
        };
        motor_j3 = HEMotorInit(&j3_config);
    }

    /* ---- 3号幻尔舵机 ---- */
    {
        HEMotor_Init_Config_s he_config = {
            .huart = &huart3,
            .motor_config = { .id = 3, .cmd = SERVO_MOVE_TIME_WRITE },
            .motor_ref = { .position = SERVO3_INIT_POS, .time = SERVO_MOVE_TIME_MS, .stop_flag = HE_ENABLED },
        };
        motor_he = HEMotorInit(&he_config);
    }

    /* ---- 新增达妙电机 ---- */
    {
        Motor_Init_Config_s dm2_config = dm_config;
        dm2_config.can_init_config.tx_id = 2;
        dm2_config.can_init_config.rx_id = 0x12;
        motor_j2 = DMMotorInit(&dm2_config);
        DMMotorSetControlMode(motor_j2, DM_MODE_POS_VEL);
        DMMotorEnable(motor_j2);
        DMMotorSetPosVelRef(motor_j2, 0.0f, 0.0f);
    }

    { HEMotor_Init_Config_s cfg = {
		.huart = &huart2,
    .motor_config = { .id = 1, .cmd = SERVO_MOVE_TIME_WRITE },
		.motor_ref = { .position = H1_INIT_POS, .time = SERVO_MOVE_TIME_MS, .stop_flag = HE_ENABLED }
		};
		motor_h1 = HEMotorInit(&cfg); }
    { HEMotor_Init_Config_s cfg = {
		.huart = &huart2,
    .motor_config = { .id = 2, .cmd = SERVO_MOVE_TIME_WRITE },
		.motor_ref = { .position = H2_INIT_POS, .time = SERVO_MOVE_TIME_MS, .stop_flag = HE_ENABLED } };
		motor_h2 = HEMotorInit(&cfg);
		}
}

/* ===================== 控制任务 ===================== */

void Arm_Task(void)
{
    uint8_t left_sw = 0U;
    uint8_t right_sw = 0U;

    if (remote_data != NULL) {
        left_sw = remote_data->switch_left;
        right_sw = remote_data->switch_right;
    }

    Arm_ProcessAirKeys(left_sw);

    Arm_ReadJointAngles(&current_angles);
    arm_debug.current = current_angles;
    arm_debug.sw = right_sw;

    if (motor_j1 && motor_j1->feedback_initialized && !j1_zero_inited) {
        j1_zero_offset_deg = 0.0f;
        j1_zero_inited = 1;
        target_angles.j1 = J1_MOTOR_SIGN * (J1_INIT_POS_RAD * RAD_2_DEGREE);
        target_angles.j2 = current_angles.j2;
        target_angles.j3 = current_angles.j3;
    }

    if ((left_sw == 2U) && (remote_data != NULL)) {
        if ((arm_last_left_switch != 2U) || (arm_last_right_switch != right_sw)) Arm_ApplyPreset(right_sw);
        {
            float j1_step_rad = (float)remote_data->rocker_r1 / 660.0f * J1_RC_STEP_RAD;
            if (fabsf(j1_step_rad) < 0.0002f) j1_step_rad = 0.0f;
            target_angles.j1 += j1_step_rad * RAD_2_DEGREE;
            if (target_angles.j1 > 180.0f) target_angles.j1 = 180.0f;
            if (target_angles.j1 < -180.0f) target_angles.j1 = -180.0f;
        }
        DMMotorSetPosVelRef(motor_j1, J1_MOTOR_SIGN * target_angles.j1 * DEGREE_2_RAD, J1_MAX_VEL_RAD_S);
        if (motor_j2 && motor_j2->feedback_initialized) DMMotorSetPosVelRef(motor_j2, motor_j2->measure.position_rad, 0.0f);
    } else if ((left_sw == 3U) && (remote_data != NULL)) {
        if ((arm_last_left_switch != 3U) || (arm_last_right_switch != right_sw)) Arm_ApplyPreset2(right_sw);
        {
            float j2_step_rad = (float)remote_data->rocker_r1 / 660.0f * J2_RC_STEP_RAD;
            if (fabsf(j2_step_rad) < 0.0002f) j2_step_rad = 0.0f;
            target_j2_deg += j2_step_rad * RAD_2_DEGREE;
            if (target_j2_deg > 180.0f) target_j2_deg = 180.0f;
            if (target_j2_deg < -180.0f) target_j2_deg = -180.0f;
        }
        DMMotorSetPosVelRef(motor_j2, J2_MOTOR_SIGN * target_j2_deg * DEGREE_2_RAD, J2_MAX_VEL_RAD_S);
        target_angles.j1 = current_angles.j1;
        DMMotorSetPosVelRef(motor_j1, J1_MOTOR_SIGN * target_angles.j1 * DEGREE_2_RAD, 0.0f);
    } else {
        target_angles.j1 = current_angles.j1;
        DMMotorSetPosVelRef(motor_j1, J1_MOTOR_SIGN * target_angles.j1 * DEGREE_2_RAD, 0.0f);
        if (motor_j2 && motor_j2->feedback_initialized) DMMotorSetPosVelRef(motor_j2, motor_j2->measure.position_rad, 0.0f);
    }

    arm_last_left_switch = left_sw;
    arm_last_right_switch = right_sw;
    arm_debug.target = target_angles;

    DMMotorControl();
    HEMotorControl();
}
