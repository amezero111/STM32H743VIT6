#ifndef ARM_APP_H
#define ARM_APP_H

#include <stdint.h>
#include "arm_kinematics.h"

typedef struct {
    Arm_JointAngles_t current;
    Arm_JointAngles_t target;
    Arm_JointAngles_t last_ik;
    float home_wrist_x;
    float home_wrist_y;
    float wrist_x;
    float wrist_y;
    float wrist_r;
    uint8_t last_ik_ret;
    uint8_t last_limits_ok;
    uint8_t sw;
    uint16_t ik_ok_count;
    uint16_t ik_fail_count;
    float he_cmd_pos;
    float he_follow_deg;
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
    float j1_remote_speed_rad_s;
    float j1_error_deg;
    int16_t j1_remote_raw;
    uint8_t j1_zero_ready;
    uint8_t j1_limit_hit;
    uint32_t task_count;
    uint8_t dm_feedback_initialized;
    uint8_t dm_stop_flag;
    uint8_t dm_state;
    uint8_t dm_motor_id;
    float dm_position_rad;
    float dm_velocity_rad_s;
    float dm_torque_nm;
    float dm_pos_ref_rad;
    float dm_vel_ref_rad_s;
    uint32_t dm_decode_count;
    uint32_t dm_control_count;
    uint16_t dm_last_tx_id;
    uint16_t dm_last_rx_id;
    uint8_t dm_last_tx_ok;
} Arm_Debug_s;

extern volatile Arm_Debug_s arm_debug;

void Arm_Init(void);
void Arm_Task(void);

#endif
