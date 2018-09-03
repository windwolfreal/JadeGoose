#include "stdint.h"
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "core_bsp.h"
#include "motor.h"
#include "encoder.h"

void BSP_DelayMs(uint32_t duration)
{
#if USE_FREERTOS == 1
    vTaskDelay(duration);
#else
    HAL_Delay(duration);
#endif
}

/**
* @brief This function handles TIM1 update interrupt.
*/
void TIM1_UP_IRQHandler(void)
{
    /* USER CODE BEGIN TIM1_UP_IRQn 0 */

    if (LL_TIM_IsActiveFlag_UPDATE(TIM1) && LL_TIM_IsEnabledIT_UPDATE(TIM1))
    {
        LL_TIM_ClearFlag_UPDATE(TIM1);
        Encoder_UpdatePulseHistory();
    }

    /* USER CODE END TIM1_UP_IRQn 0 */
    /* USER CODE BEGIN TIM1_UP_IRQn 1 */

    /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
    /* USER CODE BEGIN TIM2_IRQn 0 */
    if (LL_TIM_IsActiveFlag_UPDATE(TIM2)) {
        LL_TIM_ClearFlag_UPDATE(TIM2);
        Encoder_Overflow(1);
    }
    /* USER CODE END TIM2_IRQn 0 */
    /* USER CODE BEGIN TIM2_IRQn 1 */

    /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
    /* USER CODE BEGIN TIM3_IRQn 0 */
    if (LL_TIM_IsActiveFlag_UPDATE(TIM3))
    {
        LL_TIM_ClearFlag_UPDATE(TIM3);
        Encoder_Overflow(2);
    }
    /* USER CODE END TIM3_IRQn 0 */
    /* USER CODE BEGIN TIM3_IRQn 1 */

    /* USER CODE END TIM3_IRQn 1 */
}