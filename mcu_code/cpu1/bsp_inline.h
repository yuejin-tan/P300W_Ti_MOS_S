/*
 * bsp_inline.h
 *
 *  Created on: 2022年6月23日
 *      Author: tyj
 */

#ifndef INCX_BSP_INLINE_H_
#define INCX_BSP_INLINE_H_

#include "F28x_Project.h"
#include "stdint.h"

 // HW Fault
static inline int bsp_GPIO_ifHWFaultSigSet()
{
    if (GpioDataRegs.GPBDAT.bit.GPIO56)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

// POWER EN
static inline void bsp_POWER_EN_CH1(int level)
{
    if (level)
    {
        GpioDataRegs.GPASET.bit.GPIO26 = 1;
    }
    else
    {
        GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;
    }
}

// LED D9 RED
static inline void bsp_LED_D9_RED_CTRL(int sta)
{
    if (sta == 1)
    {
        GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
    }
    else if (sta == 0)
    {
        GpioDataRegs.GPBSET.bit.GPIO34 = 1;
    }
    else
    {
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }
}

// ePWM

#include "SVPWM.h"

static inline int16_t bsp_epwm_ch1_is_up_cnt()
{
    return EPwm4Regs.TBSTS.bit.CTRDIR;
}

static inline void bsp_epwm_ch1_reg_set(struct SVPWM_struct* hSVPWM)
{
    EPwm4Regs.CMPA.bit.CMPA = __min(__max(hSVPWM->epwmU, 0), vPWM_LOAD_VAL_I);
    EPwm5Regs.CMPA.bit.CMPA = __min(__max(hSVPWM->epwmV, 0), vPWM_LOAD_VAL_I);
    EPwm6Regs.CMPA.bit.CMPA = __min(__max(hSVPWM->epwmW, 0), vPWM_LOAD_VAL_I);
}

static inline void bsp_epwm_ch1_reg_init()
{
    EPwm4Regs.CMPA.bit.CMPA = vPWM_CMP_DEFAILT_VAL_I;
    EPwm5Regs.CMPA.bit.CMPA = vPWM_CMP_DEFAILT_VAL_I;
    EPwm6Regs.CMPA.bit.CMPA = vPWM_CMP_DEFAILT_VAL_I;
}

#define TYJ_AQSFRC_LOW 0b01
#define TYJ_AQSFRC_HIGH 0b10

static inline void bsp_epwm_ch1_on()
{
    // 开启CH1 IGBT
    EPwm4Regs.AQCSFRC.bit.CSFA = 0;
    EPwm4Regs.AQCSFRC.bit.CSFB = 0;

    EPwm5Regs.AQCSFRC.bit.CSFA = 0;
    EPwm5Regs.AQCSFRC.bit.CSFB = 0;

    EPwm6Regs.AQCSFRC.bit.CSFA = 0;
    EPwm6Regs.AQCSFRC.bit.CSFB = 0;

    bsp_POWER_EN_CH1(1);
}

static inline void bsp_epwm_ch1_off()
{
    // 关闭CH1 IGBT
    EPwm4Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm4Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;

    EPwm5Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm5Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;

    EPwm6Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm6Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;

    bsp_POWER_EN_CH1(0);
}

static inline void bsp_epwm_ch1_LSSC()
{
    // 开启CH1 下桥 IGBT
    EPwm4Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm4Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_LOW;

    EPwm5Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm5Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_LOW;

    EPwm6Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm6Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_LOW;
}

// adc A

static inline int16_t bsp_get_CH1_Iw_adcRaw(void)
{
    return AdcaResultRegs.ADCRESULT0;
}

static inline int16_t bsp_get_CH1_Va_adcRaw(void)
{
    return AdcaResultRegs.ADCRESULT1;
}

static inline int16_t bsp_get_CH1_Udc_adcRaw(void)
{
    return AdcaResultRegs.ADCRESULT2;
}


// adc B

static inline int16_t bsp_get_CH1_Iv_adcRaw(void)
{
    return AdcbResultRegs.ADCRESULT0;
}

static inline int16_t bsp_get_CH1_Vc_adcRaw(void)
{
    return AdcbResultRegs.ADCRESULT1;
}

// adc C

static inline int16_t bsp_get_CH1_Iu_adcRaw(void)
{
    return AdccResultRegs.ADCRESULT0;
}

static inline int16_t bsp_get_CH1_Vb_adcRaw(void)
{
    return AdccResultRegs.ADCRESULT1;
}


// adc D

static inline int16_t bsp_get_diff1_adcRaw(void)
{
    return AdcdResultRegs.ADCRESULT0;
}

static inline int16_t bsp_get_diff2_adcRaw(void)
{
    return AdcdResultRegs.ADCRESULT1;
}

// adc sw trig

static inline void bsp_adc_swTrig_ntc(void)
{
    AdcdRegs.ADCSOCFRC1.all = 0b111100u;
}

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

#endif /* INCX_BSP_INLINE_H_ */
