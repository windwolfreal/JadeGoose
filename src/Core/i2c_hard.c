
#include "stdint.h"
#include "i2c.h"
#include "stm32f1xx_hal.h"

#if USE_HARDWARE_I2C == 1
    I2C_HandleTypeDef hi2c1;
    DMA_HandleTypeDef hdma_i2c1_rx;
    DMA_HandleTypeDef hdma_i2c1_tx;
    /** 
     * Enable DMA controller clock
     */
    /** 
  * Enable DMA controller clock
  */
    static void MX_DMA_Init(void)
    {
        /* Init with LL driver */
        /* DMA controller clock enable */
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

        /* DMA interrupt init */
        /* DMA1_Channel6_IRQn interrupt configuration */
        NVIC_SetPriority(DMA1_Channel6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
        NVIC_EnableIRQ(DMA1_Channel6_IRQn);
        /* DMA1_Channel7_IRQn interrupt configuration */
        NVIC_SetPriority(DMA1_Channel7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
        NVIC_EnableIRQ(DMA1_Channel7_IRQn);
    }

    /* I2C1 init function */
    static void MX_I2C1_Init(void)
    {

        LL_I2C_InitTypeDef I2C_InitStruct;

        LL_GPIO_InitTypeDef GPIO_InitStruct;

        /**I2C1 GPIO Configuration  
  PB6   ------> I2C1_SCL
  PB7   ------> I2C1_SDA 
  */
        GPIO_InitStruct.Pin = IMU_I2C_SCL_Pin | IMU_I2C_SDA_Pin;
        GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
        GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
        LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* Peripheral clock enable */
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

        /* I2C1 DMA Init */

        /* I2C1_RX Init */
        LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_7, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

        LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PRIORITY_LOW);

        LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MODE_NORMAL);

        LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PERIPH_NOINCREMENT);

        LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MEMORY_INCREMENT);

        LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PDATAALIGN_BYTE);

        LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MDATAALIGN_BYTE);

        /* I2C1_TX Init */
        LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_6, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

        LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PRIORITY_LOW);

        LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MODE_NORMAL);

        LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PERIPH_NOINCREMENT);

        LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MEMORY_INCREMENT);

        LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PDATAALIGN_BYTE);

        LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MDATAALIGN_BYTE);

        /* I2C1 interrupt Init */
        NVIC_SetPriority(I2C1_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
        NVIC_EnableIRQ(I2C1_ER_IRQn);

        /**I2C Initialization 
    */
        LL_I2C_DisableOwnAddress2(I2C1);

        LL_I2C_DisableGeneralCall(I2C1);

        LL_I2C_EnableClockStretching(I2C1);

        I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
        I2C_InitStruct.ClockSpeed = 400000;
        I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
        I2C_InitStruct.OwnAddress1 = 0;
        I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
        I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
        LL_I2C_Init(I2C1, &I2C_InitStruct);

        LL_I2C_SetOwnAddress2(I2C1, 0);
    }

    void I2C_Init()
    {

        MX_DMA_Init();
        MX_I2C1_Init();
    
}

uint8_t I2C_Send(uint8_t deviceAddr, uint8_t *sendBuf, uint16_t sendLength);

uint16_t I2C_SendAndReceive(uint8_t deviceAddr, uint8_t *sendBuf, uint16_t sendLength, uint8_t *receiveBuf, uint16_t receiveLength);

/**
* @brief This function handles I2C1 error interrupt.
*/
void I2C1_ER_IRQHandler(void)
{
    /* USER CODE BEGIN I2C1_ER_IRQn 0 */

    /* USER CODE END I2C1_ER_IRQn 0 */
    HAL_I2C_ER_IRQHandler(&hi2c1);
    /* USER CODE BEGIN I2C1_ER_IRQn 1 */

    /* USER CODE END I2C1_ER_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel6 global interrupt.
*/
void DMA1_Channel6_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

    /* USER CODE END DMA1_Channel6_IRQn 0 */
    //HAL_DMA_IRQHandler(&hdma_i2c1_tx);
    /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

    /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel7 global interrupt.
*/
void DMA1_Channel7_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */

    /* USER CODE END DMA1_Channel7_IRQn 0 */
    //HAL_DMA_IRQHandler(&hdma_i2c1_rx);
    /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */

    /* USER CODE END DMA1_Channel7_IRQn 1 */
}

#endif