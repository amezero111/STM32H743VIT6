/**
 * @file robot_def.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief   锟斤拷锟斤拷锟剿讹拷锟斤拷,锟斤拷锟斤拷锟斤拷锟斤拷锟剿的革拷锟街诧拷锟斤拷
 * @version 0.1
 * @date 2024-01-09
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __ROBOT_DEF_H__
#define __ROBOT_DEF_H__

#include "stdint.h"


#define RAD_2_DEGREE        57.2957795f    // 180/pi
#define ANGLE_LIMIT_360_TO_180_ABS(angle) \
    do { \
        while ((angle) > 180.0f) { (angle) -= 360.0f; } \
        while ((angle) < -180.0f) { (angle) += 360.0f; } \
    } while (0)

// 锟斤拷锟斤拷锟剿硷拷锟轿尺寸定锟斤拷
// 锟斤拷410mm, 锟斤拷320mm. 锟斤拷锟借长为前锟斤拷锟斤拷(锟斤拷锟?, 锟斤拷为锟斤拷锟揭凤拷锟斤拷(锟街撅拷)
#define CHASSIS_WHEEL_BASE      0.410f      // 锟斤拷锟?前锟斤拷锟街撅拷锟斤拷, X锟结方锟斤拷)
#define CHASSIS_WHEEL_TRACK     0.320f      // 锟街撅拷(锟斤拷锟斤拷锟街撅拷锟斤拷, Y锟结方锟斤拷)
#define CHASSIS_HALF_BASE       (CHASSIS_WHEEL_BASE / 2.0f)
#define CHASSIS_HALF_TRACK      (CHASSIS_WHEEL_TRACK / 2.0f)

// 锟斤拷锟斤拷锟轿讹拷锟街革拷锟斤拷锟斤拷锟斤拷锟斤拷牡锟斤拷锟斤拷锟斤拷锟斤拷锟?(锟斤拷位: 锟斤拷锟斤拷祝锟斤拷锟斤拷锟?30/816/816锟斤拷锟斤拷贸锟?
// 3锟斤拷锟街ｏ拷锟斤拷锟斤拷 (锟斤拷前)
#define W3_X  (0.4865f)
#define W3_Y  (0.0f)   

// 1锟斤拷锟街ｏ拷锟阶憋拷锟襟顶碉拷 (锟斤拷锟? -> 锟斤拷锟斤拷1为锟斤拷2为锟斤拷
#define W1_X  (-0.2433f)
#define W1_Y  (-0.3650f)

// 2锟斤拷锟街ｏ拷锟阶憋拷锟揭讹拷锟斤拷 (锟揭猴拷)
#define W2_X  (-0.2433f)
#define W2_Y  (0.3650f)

// 锟斤拷锟斤拷俣锟阶拷锟斤拷锟截宏定锟斤拷 
#define MOTOR_REDUCTION_RATIO    19.2032f     // 锟斤拷锟斤拷锟斤拷俦锟?1:19.2302
#define WHEEL_RADIUS_M           0.081f     // 锟斤拷锟接半径(锟斤拷) - 锟斤拷锟斤拷实锟斤拷锟斤拷锟接尺达拷锟斤拷锟?

// 锟斤拷锟劫讹拷转锟斤拷为3508锟斤拷锟斤拷俣然锟斤拷锟斤拷锟斤拷锟阶拷锟较碉拷锟?
// 转锟斤拷锟斤拷式: 锟斤拷锟絉PM = 锟斤拷锟劫讹拷(m/s) * 转锟斤拷系锟斤拷
// 锟狡碉拷锟斤拷锟斤拷:
// 1. 锟斤拷锟斤拷转锟斤拷(rpm) = 锟斤拷锟劫讹拷(m/s) * 60 / (2 * 锟斤拷 * 锟斤拷锟接半径)
// 2. 锟斤拷锟阶拷锟?rpm) = 锟斤拷锟斤拷转锟斤拷(rpm) * 锟斤拷锟劫憋拷
// 3. 转锟斤拷系锟斤拷 = 60 * 锟斤拷锟劫憋拷 / (2 * 锟斤拷 * 锟斤拷锟接半径)
#define LINEAR_VELOCITY_TO_MOTOR_RPM    (60.0f * MOTOR_REDUCTION_RATIO / (2.0f * 3.14159f * WHEEL_RADIUS_M)) // 约锟斤拷锟斤拷 8732.24

// 锟斤拷锟斤拷转锟斤拷锟斤拷锟接碉拷锟斤拷俣锟阶拷锟轿拷锟斤拷俣锟?(锟斤拷锟斤拷锟劫度凤拷锟斤拷锟斤拷示锟斤拷)
#define MOTOR_SPEED_TO_LINEAR_VEL(motor_speed)   ((motor_speed) / LINEAR_VELOCITY_TO_MOTOR_RPM)

// DJ6020锟斤拷锟斤拷锟斤拷俣锟阶拷锟斤拷锟截宏定锟斤拷
#define GM6020_REDUCTION_RATIO    1.0f        // GM6020通锟斤拷为直锟斤拷锟斤拷锟斤拷锟劫憋拷1:1
#define GM6020_ECD_RANGE          8192.0f     // 锟斤拷锟斤拷锟斤拷锟斤拷围 0-8191

// 锟斤拷锟劫讹拷(rad/s)转锟斤拷为GM6020锟斤拷锟絉PM
// 转锟斤拷锟斤拷式: 锟斤拷锟絉PM = 锟斤拷锟劫讹拷(rad/s) * 60 / (2 * 锟斤拷) * 锟斤拷锟劫憋拷
#define ANGULAR_VELOCITY_TO_GM6020_RPM    (60.0f * GM6020_REDUCTION_RATIO / (2.0f * 3.14159f))

// 锟斤拷锟斤拷转锟斤拷锟斤拷锟斤拷GM6020锟斤拷锟絉PM转锟斤拷为锟斤拷锟劫讹拷(rad/s)
#define GM6020_RPM_TO_ANGULAR_VEL(rpm)    ((rpm) / ANGULAR_VELOCITY_TO_GM6020_RPM)


#define STEERING_CHASSIS_ALIGN_ECD_LF   5800// 锟斤拷锟斤拷 A 4锟斤拷锟斤拷锟斤拷值锟斤拷锟斤拷锟叫伙拷械锟侥讹拷锟斤拷要锟睫革拷7848-
#define STEERING_CHASSIS_ALIGN_ECD_LB   5610 // 锟斤拷锟斤拷 B 2锟斤拷锟斤拷锟斤拷值锟斤拷锟斤拷锟叫伙拷械锟侥讹拷锟斤拷要锟睫革拷3562+
#define STEERING_CHASSIS_ALIGN_ECD_RF   1734// 锟斤拷锟斤拷 C 1锟斤拷锟斤拷锟斤拷值锟斤拷锟斤拷锟叫伙拷械锟侥讹拷锟斤拷要锟睫革拷7878+
#define STEERING_CHASSIS_ALIGN_ECD_RB   4389 // 锟斤拷锟斤拷 D 3锟斤拷锟斤拷锟斤拷值锟斤拷锟斤拷锟叫伙拷械锟侥讹拷锟斤拷要锟睫革拷6437-

#define STEERING_CHASSIS_ALIGN_ANGLE_LF STEERING_CHASSIS_ALIGN_ECD_LF / 8192.f * 360.f // 锟斤拷锟斤拷 A 锟斤拷锟斤拷嵌锟?
#define STEERING_CHASSIS_ALIGN_ANGLE_LB STEERING_CHASSIS_ALIGN_ECD_LB / 8192.f * 360.f // 锟斤拷锟斤拷 B 锟斤拷锟斤拷嵌锟?
#define STEERING_CHASSIS_ALIGN_ANGLE_RF STEERING_CHASSIS_ALIGN_ECD_RF / 8192.f * 360.f // 锟斤拷锟斤拷 C 锟斤拷锟斤拷嵌锟?
#define STEERING_CHASSIS_ALIGN_ANGLE_RB STEERING_CHASSIS_ALIGN_ECD_RB / 8192.f * 360.f // 锟斤拷锟斤拷 D 锟斤拷锟斤拷嵌锟?

// ================== 锟斤拷锟斤拷锟街讹拷锟斤拷锟斤拷锟?(锟斤拷锟斤拷锟斤拷实锟绞诧拷锟斤拷值) ==================
// 锟斤拷锟借：1锟斤拷为锟斤拷锟?锟斤拷为锟揭猴拷3锟斤拷为前锟斤拷锟斤拷
#define STEERING_CHASSIS_ALIGN_ECD_1   6900 // 锟斤拷锟斤拷锟斤拷1锟斤拷锟街憋拷锟斤拷锟斤拷值700
#define STEERING_CHASSIS_ALIGN_ECD_2   0 // 锟斤拷锟斤拷锟斤拷2锟斤拷锟街憋拷锟斤拷锟斤拷值8011
#define STEERING_CHASSIS_ALIGN_ECD_3   2270 // 锟斤拷锟斤拷锟斤拷4锟斤拷锟街憋拷锟斤拷锟斤拷值1240

#define STEERING_CHASSIS_ALIGN_ANGLE_1 STEERING_CHASSIS_ALIGN_ECD_1 / 8192.f * 360.f
#define STEERING_CHASSIS_ALIGN_ANGLE_2 STEERING_CHASSIS_ALIGN_ECD_2 / 8192.f * 360.f
#define STEERING_CHASSIS_ALIGN_ANGLE_3 STEERING_CHASSIS_ALIGN_ECD_3 / 8192.f * 360.f

#pragma pack(1) // 压锟斤拷锟结构锟斤拷,取锟斤拷锟街节讹拷锟斤拷,锟斤拷锟斤拷锟斤拷锟斤拷荻锟斤拷锟斤拷鼙锟斤拷锟斤拷锟?



#pragma pack() // 取锟斤拷压锟斤拷
#endif

