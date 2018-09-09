#include "stdint.h"
#include "lite_queue.h"
#include "stdlib.h"
//#include "portable.h"

void LiteQueue_Create(LiteQueueTypeDef *queue, uint8_t maxsize)
{
    queue->maxsize = maxsize;
    // queue->data = (int32_t *)pvPortMalloc(sizeof(int32_t) * maxsize);
    queue->data = (int32_t *)malloc(sizeof(int32_t) * maxsize);
    queue->head = maxsize - 1;
    for (int i = 0; i < maxsize; i++) {
        queue->data[i] = 0;
    }
}

inline void LiteQueue_Append(LiteQueueTypeDef *queue, int32_t val)
{
    int head = queue->head + 1;
    if (head >= queue->maxsize)
    {
        head -= queue->maxsize;
    }
    queue->data[head] = val;
    queue->head = head;
}

inline int32_t LiteQueue_Peek(LiteQueueTypeDef *queue, uint8_t index)
{
    
    int head = queue->head - index;
    if (head < 0) {
        head += queue->maxsize;
    }
    return queue->data[head];
}

inline void LiteQueue_Append_f(LiteQueueTypeDef *queue, float val)
{
    LiteQueue_Append(queue, *(int32_t *)&val);
}

inline float LiteQueue_Peek_f(LiteQueueTypeDef *queue, uint8_t index)
{
    int32_t rtn = LiteQueue_Peek(queue, index);
    return *(float *)&rtn;
}