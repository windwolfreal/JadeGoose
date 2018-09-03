#include "mpu6050.h"
#include "i2c.h"

#define devAddr 0xD0
/*

#define MPU6000_ACCEL_DEFAULT_RANGE_G			8
#define MPU6000_ACCEL_DEFAULT_RATE			1000
#define MPU6000_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	30

#define MPU6000_GYRO_DEFAULT_RANGE_G			8
#define MPU6000_GYRO_DEFAULT_RATE			1000
#define MPU6000_GYRO_DEFAULT_DRIVER_FILTER_FREQ		30

#define MPU6000_DEFAULT_ONCHIP_FILTER_FREQ		42

#define MPU6000_ONE_G					9.80665f
*/

static void MPU6050_ReadRegs(uint8_t regAddr, uint8_t *readBuf, uint16_t length)
{
    I2C_SendAndReceive(devAddr, &regAddr, 1, readBuf, length);
}

static void MPU6050_WriteReg(uint8_t regAddr, uint8_t value)
{
    I2C_SendTwo(devAddr, regAddr, value);
}

static uint8_t MPU6050_TestConnection(void)
{
    // if (MPU6050_getDeviceID() == 0x68) //0b01101000;
    return 1;
    // else
    //     return 0;
}

void MPU6050_Init(MPU6050_InitTypeDef *initData)
{
    I2C_Init();
    uint8_t whoami = 0;
    MPU6050_ReadRegs(0x75, &whoami, 1);
    MPU6050_WriteReg(0x6b, 0x80); // reset
    BSP_DelayMs(100);

    MPU6050_WriteReg(0x6b, 0x00);
    // MPU6050_WriteReg(0x19, initData->simpleRateDivider);
    // MPU6050_WriteReg(0x6b, initData->clockSource);
    // MPU6050_WriteReg(0x1a, initData->lowPassFilter);
    // MPU6050_WriteReg(0x1b, initData->gyroScale << 3);
    // MPU6050_WriteReg(0x1c, initData->accelScale << 3);
    MPU6050_ReadRegs(0x75, &whoami, 1);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_Check()
*功　　能:	  检测IIC总线上的MPU6050是否存在
*******************************************************************************/
uint8_t MPU6050_Check(void)
{
    uint8_t result = MPU6050_TestConnection();
    switch (result)
    {
    case 0:
        printf("MPU6050 not found...\r\n");
        break;
    case 1:
        printf("MPU6050 check success...\r\n");
        break;
    }
    return result;
}

void MPU6050_ReadData(MPU6050_DataTypeDef *data)
{
    int8_t tempData[14];
    
    MPU6050_ReadRegs(0x3b, tempData, 14);
    data->accelX = ((tempData[0] << 8) + tempData[1]);
    data->accelY = ((tempData[2] << 8) + tempData[3]);
    data->accelZ = ((tempData[4] << 8) + tempData[5]);
    data->temp = ((tempData[6] << 8) + tempData[7]);
    data->gyroX = ((tempData[8] << 8) + tempData[9]);
    data->gyroY = ((tempData[10] << 8) + tempData[11]);
    data->gyroZ = ((tempData[12] << 8) + tempData[13]);
}