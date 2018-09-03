
#include "stdint.h"
#include "motor.h"
#include "stm32f1xx_hal.h"
#include "math.h"


#define MOTOR_CALC_PULSE_SPEED(delta) (delta * MOTOR_PLUSE_DETECTION_FREQ)



Motor_StatusTypeDef motors[MOTOR_COUNT];

/* TIM1 init function */
static void MX_TIM1_Init(void)
{
  LL_TIM_InitTypeDef TIM_InitStruct;
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct;
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct;

  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /**TIM1 GPIO Configuration  
  PE9   ------> TIM1_CH1
  PE11   ------> TIM1_CH2 
  */
  GPIO_InitStruct.Pin = MOTOR1_PWM_Pin | MOTOR2_PWM_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  LL_GPIO_AF_EnableRemap_TIM1();

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* TIM1 interrupt Init */
  NVIC_SetPriority(TIM1_UP_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
  NVIC_EnableIRQ(TIM1_UP_IRQn);

  TIM_InitStruct.Prescaler = MOTOR_PWM_TIM_CLOCK_FREQ / (MOTIR_PWN_STEP * MOTOR_PWM_FREQ) - 1; //TIM1的APB时钟是9MHZ。预分配后计数频率为1MHZ
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = MOTIR_PWN_STEP - 1;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);

  LL_TIM_EnableARRPreload(TIM1);

  // LL_TIM_SetClockSource(TIM1, LL_TIM_CLOCKSOURCE_INTERNAL);

  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);

  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_LOW;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_HIGH;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);

  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);

  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);

  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);

  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);

  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);

  LL_TIM_DisableMasterSlaveMode(TIM1);

  LL_TIM_EnableAllOutputs(TIM1);
  LL_TIM_EnableIT_UPDATE(TIM1);
}



static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct;
  
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOE);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_GPIO_ResetOutputPin(GPIOD, MOTOR1_DIR1_Pin | MOTOR1_DIR2_Pin | MOTOR2_DIR1_Pin | MOTOR2_DIR2_Pin);

  /**/
  GPIO_InitStruct.Pin = MOTOR1_DIR1_Pin | MOTOR1_DIR2_Pin | MOTOR2_DIR1_Pin | MOTOR2_DIR2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void Motor_Init()
{
  for (uint32_t i = 0; i < MOTOR_COUNT; i++)
  {
    Motor_StatusTypeDef *motor = &motors[i];
    motor->inited = 1;
    motor->throttle = 0.0f;
  }

  MX_GPIO_Init();
  MX_TIM1_Init();

  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_EnableCounter(TIM1);
}

Motor_StatusTypeDef *Motor_GetStatus(uint8_t no) {
  return &motors[no - 1];
}

void Motor_UpdateThrottle(uint8_t no, float throttle)
{ 
  if (no > MOTOR_COUNT) {
    return ;
  }
  Motor_StatusTypeDef *motor = &motors[no-1];
  motor->throttle = throttle;
  float absTh = fabs(throttle);
  uint8_t isBreak = absTh < 0.0001;
  int32_t pwm = throttle * MOTIR_PWN_STEP;
  switch (no)
  {
  case 1:
  {
    if (isBreak)
    {
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
      TIM1->CCR1 = 0;
    }
    else if (pwm > 0)
    {
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
    }
    else if (pwm < 0)
    {
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
    }
    TIM1->CCR1 = abs(pwm);
    return;
    }
    case 2: {
      if (isBreak)
      {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
        TIM1->CCR1 = 0;
      }
      else if (pwm > 0)
      {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
      }
      else if (pwm < 0)
      {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
      }
      TIM1->CCR2 = abs(pwm);
      return;
    }
  }
}

