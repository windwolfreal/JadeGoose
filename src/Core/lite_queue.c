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

inline int32_t LiteQueue_Peek(LiteQueueTypeDef *queue)
{
    return queue->data[queue->head];
}
