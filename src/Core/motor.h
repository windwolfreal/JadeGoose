/**
 * windwolf
 */
#ifndef __CORE_MOTOR_H
#define __CORE_MOTOR_H

#include "stdint.h"
#include "conf.h"

#define MOTOR_COUNT WHEEL_COUNT
#define MOTOR_PWM_FREQ 1000               // PWM频率 hz
#define MOTOR_PWM_TIM_CLOCK_FREQ 9000000 // TIM的输入时钟频率 hz
#define MOTIR_PWN_STEP 1000     // PWM步数，也是油门精度，在最大油门和最小油门之间分多少格。也是TIM的CNT


typedef uint8_t Motor_Direction_t;

// typedef struct
// {
//     Motor_Direction_t direction;
//     uint32_t speed; // 0-100
// } Motor_CommmandTypeDef;

typedef struct
{
    uint8_t inited;
    float throttle;

} Motor_StatusTypeDef;

void Motor_Init();

Motor_StatusTypeDef *Motor_GetStatus(uint8_t no);
/**
 * @no 电机编号
 * @throttle 油门大小。-1 ~ 1之间。0等于刹车。
 */
void Motor_UpdateThrottle(uint8_t no, float throttle);

#endif
