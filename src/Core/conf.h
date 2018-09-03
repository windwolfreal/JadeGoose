/**
 * windwolf
 */
#ifndef __CORE_CONF_H
#define __CORE_CONF_H

#define WHEEL_COUNT 2
#define USE_FREERTOS 0

#define MOTOR_SLOWDOWN 50       // 电机减速比
#define MOTOR_WHEEL_DIAMETER 80 // 轮子直径 mm
#define MOTOR_MIN_SPEED 10      // 电机最小转速（本体，而非减速后） r/s
#define MOTOR_MAX_SPEED 600     // 电机最小转速（本体） r/s

#define DECODER_BASE_PPR 11 // 电机编码器基础脉冲数

#if defined(PI)
#else
#define PI 3.14159265358979f
#endif 

#endif
