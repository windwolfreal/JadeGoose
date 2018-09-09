#include "control.h"
#include "stdint.h"
#include "motor.h"
#include "encoder.h"
#include "conf.h"
#include "math.h"

Control_EncoderMotorStatusTypeDef encoderMotors[WHEEL_COUNT];

#define P 0.001f
#define I 0.0f
#define D 0.0f
#define INC_K0 (P + I + D)
#define INC_K1 (-P - 2 * D)
#define INC_K2 (D)

void Control_Init()
{
    Encoder_Init();
    Motor_Init();
    for (int i = 0; i < WHEEL_COUNT; i++)
    {
        Control_EncoderMotorStatusTypeDef *encoderMotor = &encoderMotors[i];
        encoderMotor->motor = Motor_GetStatus(i + 1);
        encoderMotor->encoder = Encoder_GetStatus(i + 1);
        LiteQueue_Create(&(encoderMotor->errors), 3);
    }
}

void Control_SetExpectLineSpeed(float speed)
{
    float absSpeed = fabs(speed);
    if (absSpeed > MOTOR_MAX_LINE_SPPED)
    {
        absSpeed = MOTOR_MAX_LINE_SPPED;
    }
    for (int i = 0; i < WHEEL_COUNT; i++)
    {
        Control_EncoderMotorStatusTypeDef *encoderMotor = &encoderMotors[i];
        encoderMotor->expectSpeed = speed;
    }
}

void Control_PidControl()
{
    for (int i = 0; i < WHEEL_COUNT; i++)
    {
        Control_EncoderMotorStatusTypeDef *encoderMotor = &encoderMotors[i];
        float measuredLineSpeed = encoderMotor->encoder->deltaPulse * DECODER_PLUSE_DETECTION_FREQ / MOTOR_SPEED_FACTOR;  // 测量线速
        float e0 = encoderMotor->expectSpeed - measuredLineSpeed;                                                         // 线速误差
        LiteQueue_Append_f(&(encoderMotor->errors), e0);
        float e1 = LiteQueue_Peek(&(encoderMotor->errors), 1);
        float e2 = LiteQueue_Peek(&(encoderMotor->errors), 2);
        float delta = INC_K0 * e0 + INC_K1 * e1 + INC_K2 * e2;
        encoderMotor->delta = delta;
        float throttle = encoderMotor->motor->throttle;                        // 当前油门
        throttle += delta; //
        if (throttle > 1.0f)
        {
            throttle = 1.0f;
        }
        else if (throttle < -1.0f)
        {
            throttle = -1.0f;
        }
        Motor_UpdateThrottle(i + 1, throttle);
    }
}