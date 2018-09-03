#include "encoder.h"
#include "stdint.h"
#include "stm32f1xx_hal.h"

Encoder_StatusTypeDef encoders[ENCODER_COUNT];
uint32_t pdScalerCount = 0;

/* TIM2 init function */
static void MX_TIM2_Init(void)
{
    LL_TIM_InitTypeDef TIM_InitStruct;

    LL_GPIO_InitTypeDef GPIO_InitStruct;

    LL_TIM_OC_InitTypeDef OC_InitStruct;

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

    /**TIM2 GPIO Configuration  
  PA0-WKUP   ------> TIM2_CH1
  PA1   ------> TIM2_CH2 
  */
    GPIO_InitStruct.Pin = MOTOR1_FEEDBACK1_Pin | MOTOR1_FEEDBACK2_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* TIM2 interrupt Init */
    NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(TIM2_IRQn);

    LL_TIM_SetEncoderMode(TIM2, LL_TIM_ENCODERMODE_X4_TI12);

    LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);

    LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);

    LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);

    LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);

    LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);

    LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);

    LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1);

    LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);

    TIM_InitStruct.Prescaler = 8;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 65535;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(TIM2, &TIM_InitStruct);

    LL_TIM_DisableARRPreload(TIM2);

    LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);

    LL_TIM_DisableMasterSlaveMode(TIM2);

    LL_TIM_EnableIT_UPDATE(TIM2);
}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

    LL_TIM_InitTypeDef TIM_InitStruct;

    LL_GPIO_InitTypeDef GPIO_InitStruct;

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

    /**TIM3 GPIO Configuration  
  PA6   ------> TIM3_CH1
  PA7   ------> TIM3_CH2 
  */
    GPIO_InitStruct.Pin = MOTOR2_FEEDBACK1_Pin | MOTOR2_FEEDBACK2_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* TIM3 interrupt Init */
    NVIC_SetPriority(TIM3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(TIM3_IRQn);

    LL_TIM_SetEncoderMode(TIM3, LL_TIM_ENCODERMODE_X4_TI12);

    LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);

    LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);

    LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);

    LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);

    LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);

    LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);

    LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1);

    LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);

    TIM_InitStruct.Prescaler = 8;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 65535;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(TIM3, &TIM_InitStruct);

    LL_TIM_DisableARRPreload(TIM3);

    LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);

    LL_TIM_DisableMasterSlaveMode(TIM3);

    LL_TIM_EnableIT_UPDATE(TIM3);
}

static void MX_GPIO_Init(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
}

Encoder_StatusTypeDef *Encoder_GetStatus(uint8_t no) {
    return &encoders[no - 1];
}

void Encoder_Init() {

    for (uint32_t i = 0; i < ENCODER_COUNT; i++)
    {
        Encoder_StatusTypeDef *encoder = &encoders[i];
        LiteQueue_Create(&encoder->historyPulses, 4);
        encoder->deltaPulse = 0;
        encoder->pulseOverflow = 0;
        encoder->inited = 1;
    }

    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();

    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
    LL_TIM_EnableCounter(TIM2);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2);
    LL_TIM_EnableCounter(TIM3);
}

void Encoder_Overflow(int8_t no)
{
    if (no == 1)
    {
        if (encoders[0].inited)
        {
            if (LL_TIM_GetDirection(TIM2) == LL_TIM_COUNTERDIRECTION_UP)
            {
                encoders[0].pulseOverflow++;
            }
            else
            {
                encoders[0].pulseOverflow--;
            }
        }
    }
    if (no == 2)
    {
        if (encoders[1].inited)
        {
            if (LL_TIM_GetDirection(TIM3) == LL_TIM_COUNTERDIRECTION_UP)
            {
                encoders[1].pulseOverflow++;
            }
            else
            {
                encoders[1].pulseOverflow--;
            }
        }
    }
}

// 触发计算速度的中断和触发计算溢出的中断要报告同样的抢占优先级，以保证速度计算过程中，位移溢出值的完整性。
void Encoder_UpdatePulseHistory()
{
    // 到达分频数，触发计算。
    pdScalerCount++;
    if (pdScalerCount != DECODER_PLUSE_DETECTION__PRESCALER)
    {
        return;
    }
    pdScalerCount = 0;

    int32_t currentDisplacement;
    int32_t lastDisplacement;
    if (encoders[0].inited)
    {
        currentDisplacement = (encoders[0].pulseOverflow << 16) + TIM2->CNT;
        lastDisplacement = LiteQueue_Peek(&encoders[0].historyPulses);
        encoders[0].deltaPulse = currentDisplacement - lastDisplacement;
        LiteQueue_Append(&encoders[0].historyPulses, currentDisplacement);
    }
    if (encoders[1].inited)
    {
        currentDisplacement = (encoders[1].pulseOverflow << 16) + TIM3->CNT;
        lastDisplacement = LiteQueue_Peek(&encoders[1].historyPulses);
        encoders[1].deltaPulse = currentDisplacement - lastDisplacement;
        LiteQueue_Append(&encoders[1].historyPulses, currentDisplacement);
    }
}