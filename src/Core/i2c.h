#ifndef __CORE_I2C_H
#define __CORE_I2C_H

#include "stdint.h"
#include "core_bsp.h"

#define USE_HARDWARE_I2C 0

void I2C_Init();

uint8_t I2C_Send(uint8_t deviceAddr, uint8_t *sendBuf, uint16_t sendLength);
uint8_t I2C_SendOne(uint8_t deviceAddr, uint8_t data);
uint8_t I2C_SendTwo(uint8_t deviceAddr, uint8_t data1, uint8_t data2);

uint16_t I2C_SendAndReceive(uint8_t deviceAddr, uint8_t *sendBuf, uint16_t sendLength, uint8_t *receiveBuf, uint16_t receiveLength);

#endif 