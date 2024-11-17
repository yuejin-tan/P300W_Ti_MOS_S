/*
 * encoder.h
 *
 *  Created on: 2024年9月7日
 *      Author: t
 */

#ifndef MCU_CODE_CPU1_ENCODER_H_
#define MCU_CODE_CPU1_ENCODER_H_

#include "F28x_Project.h"
#include "stdint.h"
#include "math.h"

#define EQEP_TYJDEV_REGS EQep2Regs
#define EQEP_TYJDEV_BASE EQEP2_BASE

typedef struct {
    float raw2u16_ratio;
    float spdCalc_ratio;
}encoder_struct;

static inline void encoder_init(encoder_struct* hEncoder, uint32_t ppr)
{
    hEncoder->raw2u16_ratio = 65536.0f / (ppr * 4);

    // default UPEVENT DIV = 4
    hEncoder->spdCalc_ratio = (float)(200e6 * 2.0 * M_PI) / ppr;
}

static inline uint16_t encoder_u16Read(encoder_struct* hEncoder, uint32_t qposcnt)
{
    return qposcnt * hEncoder->raw2u16_ratio;
}

static inline float encoder_lowSpdCalc(encoder_struct* hEncoder, uint16_t qcprdlat)
{
    if (qcprdlat == 0)
    {
        // 防止除零
        qcprdlat = 1;
    }
    return hEncoder->spdCalc_ratio / qcprdlat;
}

#endif /* MCU_CODE_CPU1_ENCODER_H_ */