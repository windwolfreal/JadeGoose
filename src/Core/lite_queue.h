#ifndef __QUEUE_H_
#define __QUEUE_H_

#include "stdint.h"
typedef struct
{
    int32_t *data;  
    uint8_t head;      // 指向当前最新的数据
    uint8_t maxsize;
} LiteQueueTypeDef;

void LiteQueue_Create(LiteQueueTypeDef *queue, uint8_t maxsize);
void LiteQueue_Append(LiteQueueTypeDef *queue, int32_t val);
int32_t LiteQueue_Peek(LiteQueueTypeDef *queue, uint8_t index);

void LiteQueue_Append_f(LiteQueueTypeDef *queue, float val);
float LiteQueue_Peek_f(LiteQueueTypeDef *queue, uint8_t index);
#endif

