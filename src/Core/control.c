#include "control.h"
#include "stdint.h"
#include "motor.h"
#include "encoder.h"
#include "conf.h"
#include "math.h"

Control_EncoderMotorStatusTypeDef encoderMotors[WHEEL_COUNT];

static float P = 0.003f;
static float I = 0.0f;
static float D = 0.0f;

void Control_Init()
{
    Encoder_Init();
    Motor_Init();
    for (int i = 0; i < WHEEL_COUNT; i++)
    {
        Control_EncoderMotorStatusTypeDef *encoderMotor = &encoderMotors[i];
        encoderMotor->motor = Motor_GetStatus(i + 1);
        encoderMotor->encoder = Encoder_GetStatus(i + 1);
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
        float measuredLineSpeed = encoderMotor->encoder->deltaPulse * DECODER_PLUSE_DETECTION_FREQ / MOTOR_SPEED_FACTOR; // 测量线速
        float err = encoderMotor->expectSpeed - measuredLineSpeed;                                                       // 线速误差
        float throttle = encoderMotor->motor->throttle;                                                                  // 当前油门
        throttle += P * err;                                                                                             //
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