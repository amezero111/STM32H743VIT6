#include "arm_kinematics.h"
#include "general_def.h"
#include <math.h>

/* ===================== 正运动学 ===================== */

Arm_Position_t Arm_FK(Arm_JointAngles_t angles)
{
    Arm_Position_t pos;

    float a1  = angles.j1 * DEGREE_2_RAD;
    float a2  = angles.j2 * DEGREE_2_RAD;
    float a3  = angles.j3 * DEGREE_2_RAD;
    float a12 = a1 + a2;
    float a123 = a12 + a3;

    pos.x   = ARM_L1 * cosf(a1) + ARM_L2 * cosf(a12) + ARM_L3 * cosf(a123);
    pos.y   = ARM_L1 * sinf(a1) + ARM_L2 * sinf(a12) + ARM_L3 * sinf(a123);
    pos.phi = Arm_NormalizeAngle(angles.j1 + angles.j2 + angles.j3);

    return pos;
}

/* ===================== 逆运动学 ===================== */

uint8_t Arm_IK(Arm_Position_t target, Arm_JointAngles_t *angles_out,
               Arm_Elbow_Config_e elbow)
{
    if (angles_out == NULL)
        return 0;

    float phi_rad = target.phi * DEGREE_2_RAD;

    /* Step 1: 根据姿态反推腕部 (舵机转轴中心) 位置 */
    float x_w = target.x - ARM_L3 * cosf(phi_rad);
    float y_w = target.y - ARM_L3 * sinf(phi_rad);

    /* Step 2: 腕部 2 连杆逆解 (l1=大臂, l2=小臂) */
    float r2 = x_w * x_w + y_w * y_w;
    float r  = sqrtf(r2);

    if (r > ARM_L1 + ARM_L2 + 0.01f || r < fabsf(ARM_L1 - ARM_L2) - 0.01f)
        return 0;  /* 目标超出工作空间 */

    float cos_j2 = (r2 - ARM_L1 * ARM_L1 - ARM_L2 * ARM_L2) /
                   (2.0f * ARM_L1 * ARM_L2);
    if (cos_j2 > 1.0f)  cos_j2 = 1.0f;
    if (cos_j2 < -1.0f) cos_j2 = -1.0f;

    float j2_rad;
    if (elbow == ARM_ELBOW_DOWN)
        j2_rad = acosf(cos_j2);      /* θ2 > 0, 正解 */
    else
        j2_rad = -acosf(cos_j2);     /* θ2 < 0, 负解 */

    float j1_rad = atan2f(y_w, x_w) -
                   atan2f(ARM_L2 * sinf(j2_rad),
                          ARM_L1 + ARM_L2 * cosf(j2_rad));

    /* Step 3: 由姿态方程反推 θ3 */
    float j3_rad = phi_rad - j1_rad - j2_rad;

    angles_out->j1 = j1_rad * RAD_2_DEGREE;
    angles_out->j2 = j2_rad * RAD_2_DEGREE;
    angles_out->j3 = j3_rad * RAD_2_DEGREE;

    return 1;
}

/* ===================== 关节限位检查 ===================== */

uint8_t Arm_CheckJointLimits(Arm_JointAngles_t angles)
{
    if (angles.j1 < ARM_J1_MIN_DEG || angles.j1 > ARM_J1_MAX_DEG) return 0;
    if (angles.j2 < ARM_J2_MIN_DEG || angles.j2 > ARM_J2_MAX_DEG) return 0;
    if (angles.j3 < ARM_J3_MIN_DEG || angles.j3 > ARM_J3_MAX_DEG) return 0;
    return 1;
}

/* ===================== 角度归一化 ===================== */

float Arm_NormalizeAngle(float angle_deg)
{
    float a = fmodf(angle_deg, 360.0f);
    if (a >  180.0f) a -= 360.0f;
    if (a < -180.0f) a += 360.0f;
    return a;
}
