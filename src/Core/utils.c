#include<stdint.h>

inline int32_t absInt32(int32_t val) {
    if (val < 0) {
        return -val;
    }
    else {
        return val;
    }
}

inline float absFloat(float val)
{
    if (val < 0)
    {
        return -val;
    }
    else
    {
        return val;
    }
}