#include "HEmotor.h"

#define SERVO_LOAD_OR_UNLOAD_WRITE 31U
#define HE_LOAD_REFRESH_PERIOD_MS  500U

static uint8_t he_idx = 0;
static HEMotor_Instance *he_motor_instances[HE_MOTOR_CNT] = {NULL};

static void HEMotorLoadOrUnloadWriteInternal(HEMotor_Instance *motor, uint8_t load);

HEMotor_Instance *HEMotorInit(HEMotor_Init_Config_s *config)
{
    if ((config == NULL) || (he_idx >= HE_MOTOR_CNT)) {
        return NULL;
    }

    HEMotor_Instance *motor = (HEMotor_Instance *)malloc(sizeof(HEMotor_Instance));
    if (motor == NULL) {
        return NULL;
    }

    memset(motor, 0, sizeof(HEMotor_Instance));

    USART_Init_Config_s bsp_usart_config = {
        .usart_handle = config->huart,
        .recv_buff_size = HE_MAX_BUFFSIZE,
        .module_callback = NULL,
    };

    motor->usart_instance = USARTRegister(&bsp_usart_config);
    motor->config = config->motor_config;
    motor->ref = config->motor_ref;

    he_motor_instances[he_idx++] = motor;

    if (motor->ref.stop_flag == HE_ENABLED) {
        HEMotorLoadOrUnloadWriteInternal(motor, 1U);
        HEMotorMoveTimeWrite(motor, motor->ref.position, motor->ref.time);
    }

    return motor;
}

static void HEMotorSendPacket(HEMotor_Instance *motor,
                              uint8_t cmd,
                              uint8_t *params,
                              uint8_t param_len)
{
    uint8_t length = param_len + 3U;
    uint8_t total_len = length + 3U;
    uint8_t *buf = motor->send_buff;
    uint32_t checksum_total = 0U;

    buf[0] = 0x55U;
    buf[1] = 0x55U;
    buf[2] = motor->config.id;
    buf[3] = length;
    buf[4] = cmd;

    checksum_total = (uint32_t)buf[2] + buf[3] + buf[4];
    for (uint8_t i = 0U; i < param_len; i++) {
        buf[5U + i] = params[i];
        checksum_total += params[i];
    }

    buf[5U + param_len] = (uint8_t)(~(checksum_total & 0xFFU));

    USARTSend(motor->usart_instance, buf, total_len, USART_TRANSFER_BLOCKING);
}

void HEMotorMoveTimeWrite(HEMotor_Instance *motor, uint16_t pos, uint16_t time)
{
    if ((motor == NULL) || (motor->ref.stop_flag == HE_STOP)) {
        return;
    }

    uint8_t params[4];
    params[0] = (uint8_t)(pos & 0xFFU);
    params[1] = (uint8_t)((pos >> 8) & 0xFFU);
    params[2] = (uint8_t)(time & 0xFFU);
    params[3] = (uint8_t)((time >> 8) & 0xFFU);

    HEMotorSendPacket(motor, SERVO_MOVE_TIME_WRITE, params, 4U);
}

static void HEMotorLoadOrUnloadWriteInternal(HEMotor_Instance *motor, uint8_t load)
{
    if (motor == NULL) {
        return;
    }

    uint8_t params[1];
    params[0] = (load != 0U) ? 1U : 0U;

    HEMotorSendPacket(motor, SERVO_LOAD_OR_UNLOAD_WRITE, params, 1U);
}

void HEMotorControl(void)
{
    static uint32_t last_send_tick = 0U;
    static uint32_t last_load_tick = 0U;
    uint32_t now = HAL_GetTick();
    uint8_t refresh_load = 0U;

    if ((now - last_send_tick) < 10U) {
        return;
    }
    last_send_tick = now;

    if ((now - last_load_tick) >= HE_LOAD_REFRESH_PERIOD_MS) {
        last_load_tick = now;
        refresh_load = 1U;
    }

    for (uint8_t i = 0U; i < he_idx; i++) {
        if ((he_motor_instances[i] != NULL) &&
            (he_motor_instances[i]->ref.stop_flag == HE_ENABLED)) {
            if (refresh_load != 0U) {
                HEMotorLoadOrUnloadWriteInternal(he_motor_instances[i], 1U);
            }

            HEMotorMoveTimeWrite(he_motor_instances[i],
                                 he_motor_instances[i]->ref.position,
                                 he_motor_instances[i]->ref.time);
        }
    }
}
