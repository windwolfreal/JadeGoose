#ifndef __CORE_BSP_H
#define __CORE_BSP_H

#include "stdint.h"






void BSP_DelayMs(uint32_t duration);

#define BSP_DelayUs(duration)                     \
    do                                            \
    {                                             \
        asm volatile("MOV R0,%[loops]\n\t"        \
                     "1: \n\t"                    \
                     "SUB R0, #1\n\t"             \
                     "CMP R0, #0\n\t"             \
                     "BNE 1b \n\t"                \
                     :                            \
                     : [loops] "r"(13 * duration) \
                     : "memory");                 \
    } while (0)
#endif 