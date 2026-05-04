#include "arm.h"
#include "dm_motor.h"
#include "bsp_dwt.h"
#include "bsp_fdcan.h"
#include "main.h"
#include "general_def.h"
#include <math.h>

/* ===================== 电机实例 ===================== */

static DMMotor_Instance *motor_j1;  // 大臂 DM4340
static uint8_t base_locked = 0;
static float base_angle_deg = 0.0f;
static const float arm_osc_amp_deg = 5.0f;
static const float arm_osc_freq_hz = 0.2f;

/* ===================== 初始化 ===================== */

void Arm_Init(void)
{

    Motor_Init_Config_s dm_config = {
        .can_init_config = {
            .fdcan_handle = &hfdcan1,
            .tx_id      = 1,   // SlaveID, 需与达妙上位机中设置的一致
            .rx_id      = 0x11,   // MasterID = SlaveID + 0x10, 需与上位机一致
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp   = 40.0f,
                .Ki   = 0.05f,
                .Kd   = 0.0f,
                .MaxOut = 12.5f,   // 输出限幅在 ±p_max
            },
            .speed_PID = {
                .Kp   = 2.0f,
                .Ki   = 0.01f,
                .Kd   = 0.0f,
                .MaxOut = 30.0f,  // 输出限幅在 ±v_max
            },
            .current_PID = {
                .Kp   = 1.0f,
                .Ki   = 0.0f,
                .Kd   = 0.0f,
                .MaxOut = 10.0f,  // 输出限幅在 ±t_max
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source  = MOTOR_FEED,
            .speed_feedback_source  = MOTOR_FEED,
            .outer_loop_type        = ANGLE_LOOP,
            .close_loop_type        = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag     = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag  = FEEDBACK_DIRECTION_NORMAL,
            .feedforward_flag       = FEEDFORWARD_NONE,
            .angle_mode             = MOTOR_ANGLE_MODE_TOTAL,
        },
        .motor_type = DM4340,
    };

    motor_j1 = DMMotorInit(&dm_config);

    /* 设置 MIT 模式电机侧 KP/KD
     * KP=40: 位置刚度, KD=0.05: 速度阻尼
     * 这些值来自队友 arm.c 中 dm_mit_PID 的配置 */
    DMMotorSetMITParams(motor_j1, 40.0f, 0.05f);

}

/* ===================== 控制任务 ===================== */

void Arm_Task(void)
{
    
    

    /* 首次运行: 记录当前位置为基准角 */
    if (!base_locked && motor_j1->feedback_updated) {
        base_angle_deg = motor_j1->measure.position_rad * RAD_2_DEGREE;
        base_locked = 1;
    }

    /* 小幅度往复,用于验证 MIT 控制生效 */
    if (base_locked) {
        float t = DWT_GetTimeline_s();
        float ref = base_angle_deg + arm_osc_amp_deg * sinf(2.0f * PI * arm_osc_freq_hz * t);
        DMMotorSetRef(motor_j1, ref);
    }

    /* 统一发送所有 DM 电机控制帧
     * 内部会运行 3 环 PID 串级计算 */
    DMMotorControl();

    /* 调试提示:
     * 在调试器中观察 g_dm_motor_debug:
     *   - decode_count > 0    说明 CAN 接收正常
     *   - tx_ok_count > 0     说明 CAN 发送正常
     *   - last_total_angle    当前电机累计角度
     *   - last_control_mode   应为 0 (MIT 模式)
     *
     * 如果 decode_count 始终为 0:
     *   1. 检查 tx_id/rx_id 是否与达妙上位机一致
     *   2. 检查 CAN 总线接线和终端电阻
     *   3. 检查 FDCAN 时钟和波特率配置
     */
}
