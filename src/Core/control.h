/**
 * windwolf
 */
#ifndef __CORE_MOTOR_CONTROL_H
#define __CORE_MOTOR_CONTROL_H

#include "stdint.h"
#include "motor.h"
#include "encoder.h"
#include "conf.h"
#include "math.h"

#define MOTOR_SPEED_FACTOR (DECODER_BASE_PPR * 4 * MOTOR_SLOWDOWN / (MOTOR_WHEEL_DIAMETER * PI)) // 外轮一圈的编码计数 / 外轮直径(mm) = 每前进1mm的编码计数。线速(mm/s) x 这个因子 = 每秒编码数
#define MOTOR_MIN_PLUSE_SPPED (DECODER_BASE_PPR * 4 * MOTOR_MIN_SPEED)                            // 最小编码速
#define MOTOR_MIN_LINE_SPPED (MOTOR_WHEEL_DIAMETER * PI * MOTOR_MIN_SPEED / MOTOR_SLOWDOWN)       // 最小外轮线速
#define MOTOR_MAX_PLUSE_SPPED (MOTOR_DECODER_PPR * 4 * MOTOR_MAX_SPEED)                           // 最大编码速
#define MOTOR_MAX_LINE_SPPED (MOTOR_WHEEL_DIAMETER * PI * MOTOR_MAX_SPEED / MOTOR_SLOWDOWN)       // 最大外轮线速

typedef struct {
    Motor_StatusTypeDef *motor;
    Encoder_StatusTypeDef *encoder;
    float expectSpeed; // 期望的最终线速;
    LiteQueueTypeDef errors;
    float delta;
} Control_EncoderMotorStatusTypeDef;

void Control_Init();



void Control_SetExpectLineSpeed(float speed);

void Control_PidControl();

#endif