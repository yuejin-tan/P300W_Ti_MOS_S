/*
 * bsp_inline2.h
 *
 *  Created on: 2023年10月13日
 *      Author: t
 */

#ifndef SHARE_CPU2_BSP_INLINE2_H_
#define SHARE_CPU2_BSP_INLINE2_H_

#include "F28x_Project.h"
#include "stdint.h"

// timer 0

static inline int16_t bsp_tim0_polling_OF(void)
{
    return CpuTimer0Regs.TCR.bit.TIF;
}

static inline void bsp_tim0_clearFlg_OF(void)
{
    CpuTimer0Regs.TCR.bit.TIF = 1;
}

// timer 1

static inline int16_t bsp_tim1_polling_OF(void)
{
    return CpuTimer1Regs.TCR.bit.TIF;
}

static inline void bsp_tim1_clearFlg_OF(void)
{
    CpuTimer1Regs.TCR.bit.TIF = 1;
}


#endif /* SHARE_CPU2_BSP_INLINE2_H_ */