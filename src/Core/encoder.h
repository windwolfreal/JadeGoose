/**
 * windwolf
 */
#ifndef __CORE_ENCODER_H
#define __CORE_ENCODER_H
#include "lite_queue.h"
#include "stdint.h"
#include "conf.h"

#define ENCODER_COUNT WHEEL_COUNT


#define DECODER_PLUSE_DETECTION_BASE_FREQ 1000                                            // 编码探测采样频率
#define DECODER_PLUSE_DETECTION__PRESCALER 20                                             // 编码探测分频数。
#define DECODER_PLUSE_DETECTION_FREQ (DECODER_PLUSE_DETECTION_BASE_FREQ / DECODER_PLUSE_DETECTION__PRESCALER) // 实际的编码探测频率 = 编码探测采样频率 / 分频数

typedef struct
{
    LiteQueueTypeDef historyPulses; // 最近四次测得编码数，加上了overflow。
    int32_t deltaPulse;             // 最近采样的编码数差
    int16_t pulseOverflow;          // 编码数溢出次数
    uint8_t inited;                 // 是否初始化
} Encoder_StatusTypeDef;

void Encoder_Init();

Encoder_StatusTypeDef *Encoder_GetStatus(uint8_t no);

/**
 * 编码器溢出。
 * @no 编码器编号
 * */
void Encoder_Overflow(int8_t no);

void Encoder_UpdatePulseHistory();

#endif
