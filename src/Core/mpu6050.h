#ifndef __MPU6050_H_
#define __MPU6050_H_

#include "stm32f1xx.h"



typedef struct  {
    uint8_t simpleRateDivider; //采样率分频：0-255. 采样频率=陀螺仪输出频率/（ 1+SMPLRT_DIV）
    uint8_t lowPassFilter;     // 低通滤波 0:不滤波; 1:184hz,188hz; 2:94hz,98hz; 3:44hz,42hz; 4:21hz,20hz; 5:10hz,10hz; 6:5hz,5hz; 7:--;
    uint8_t gyroScale;     // 陀螺仪量程 0:250; 1:500; 2:1000; 3:2000;
    uint8_t accelScale;    // 加速度计量程 0:2G; 1:4G; 2:8G; 3:16G;
    uint8_t clockSource;   // 内部时钟源 0:内部8MHZ; 1:陀螺仪X; 2:陀螺仪Y; 3:陀螺仪Z; 4:外部32.768khz; 5:外部19.2MHZ; 6:--; 7:时钟停止;

} MPU6050_InitTypeDef;
typedef struct {
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    int16_t temp;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
} MPU6050_DataTypeDef;

void MPU6050_Init(MPU6050_InitTypeDef *initData); //初始化

void MPU6050_ReadData(MPU6050_DataTypeDef *data);

uint8_t MPU6050_Check(void);
#endif
