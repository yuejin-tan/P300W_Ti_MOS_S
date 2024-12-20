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
    if (GpioDataRegs.GPADAT.bit.GPIO19)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

static inline int bsp_GPIO_ifHWFaultSigSet2()
{
    if (GpioDataRegs.GPEDAT.bit.GPIO139)
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
    // 低使能
    if (level)
    {
        GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;
    }
    else
    {
        GpioDataRegs.GPASET.bit.GPIO16 = 1;
    }
}

static inline void bsp_POWER_EN_CH2(int level)
{
    // 高使能
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

// group1
static inline int16_t bsp_epwm_ch1_is_up_cnt()
{
    return EPwm1Regs.TBSTS.bit.CTRDIR;
}

static inline void bsp_epwm_ch1_reg_set(struct SVPWM_struct* hSVPWM)
{
    EPwm1Regs.CMPA.bit.CMPA = __min(__max(hSVPWM->epwmU, 0), vPWM_LOAD_VAL_I);
    EPwm2Regs.CMPA.bit.CMPA = __min(__max(hSVPWM->epwmV, 0), vPWM_LOAD_VAL_I);
    EPwm3Regs.CMPA.bit.CMPA = __min(__max(hSVPWM->epwmW, 0), vPWM_LOAD_VAL_I);
}

static inline void bsp_epwm_ch1_reg_init()
{
    EPwm1Regs.CMPA.bit.CMPA = vPWM_CMP_DEFAILT_VAL_I;
    EPwm2Regs.CMPA.bit.CMPA = vPWM_CMP_DEFAILT_VAL_I;
    EPwm3Regs.CMPA.bit.CMPA = vPWM_CMP_DEFAILT_VAL_I;
}

#define TYJ_AQSFRC_LOW 0b01
#define TYJ_AQSFRC_HIGH 0b10

static inline void bsp_epwm_ch1_on()
{
    // 开启 IGBT
    EPwm1Regs.AQCSFRC.bit.CSFA = 0;
    EPwm1Regs.AQCSFRC.bit.CSFB = 0;

    EPwm2Regs.AQCSFRC.bit.CSFA = 0;
    EPwm2Regs.AQCSFRC.bit.CSFB = 0;

    EPwm3Regs.AQCSFRC.bit.CSFA = 0;
    EPwm3Regs.AQCSFRC.bit.CSFB = 0;
}

static inline void bsp_epwm_ch1_off()
{
    // 关闭 IGBT
    EPwm1Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm1Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;

    EPwm2Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm2Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;

    EPwm3Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm3Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;
}

static inline void bsp_epwm_ch1_LSSC()
{
    // 开启 下桥 IGBT
    EPwm1Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm1Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_LOW;

    EPwm2Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm2Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_LOW;

    EPwm3Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm3Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_LOW;
}

// mode : 0@DOWN 1@UP; bit0 A, bit1 B, bit2 C
static inline void bsp_epwm_ch1_LSSC2(int16_t mode)
{
    if (mode & 0x1)
    {
        EPwm1Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_HIGH;
        EPwm1Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;
    }
    else
    {
        EPwm1Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
        EPwm1Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_LOW;
    }

    if (mode & 0x2)
    {
        EPwm2Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_HIGH;
        EPwm2Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;
    }
    else
    {
        EPwm2Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
        EPwm2Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_LOW;
    }

    if (mode & 0x4)
    {
        EPwm3Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_HIGH;
        EPwm3Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;
    }
    else
    {
        EPwm3Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
        EPwm3Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_LOW;
    }
}


// group2
static inline int16_t bsp_epwm_ch2_is_up_cnt()
{
    return EPwm4Regs.TBSTS.bit.CTRDIR;
}

static inline void bsp_epwm_ch2_reg_set(struct SVPWM_struct* hSVPWM)
{
    EPwm4Regs.CMPA.bit.CMPA = __min(__max(hSVPWM->epwmU, 0), vPWM_LOAD_VAL_I);
    EPwm5Regs.CMPA.bit.CMPA = __min(__max(hSVPWM->epwmV, 0), vPWM_LOAD_VAL_I);
    EPwm6Regs.CMPA.bit.CMPA = __min(__max(hSVPWM->epwmW, 0), vPWM_LOAD_VAL_I);
}

static inline void bsp_epwm_ch2_reg_init()
{
    EPwm4Regs.CMPA.bit.CMPA = vPWM_CMP_DEFAILT_VAL_I;
    EPwm5Regs.CMPA.bit.CMPA = vPWM_CMP_DEFAILT_VAL_I;
    EPwm6Regs.CMPA.bit.CMPA = vPWM_CMP_DEFAILT_VAL_I;
}

static inline void bsp_epwm_ch2_on()
{
    // 开启 IGBT
    EPwm4Regs.AQCSFRC.bit.CSFA = 0;
    EPwm4Regs.AQCSFRC.bit.CSFB = 0;

    EPwm5Regs.AQCSFRC.bit.CSFA = 0;
    EPwm5Regs.AQCSFRC.bit.CSFB = 0;

    EPwm6Regs.AQCSFRC.bit.CSFA = 0;
    EPwm6Regs.AQCSFRC.bit.CSFB = 0;
}

static inline void bsp_epwm_ch2_off()
{
    // 关闭 IGBT
    EPwm4Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm4Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;

    EPwm5Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm5Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;

    EPwm6Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm6Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;
}

static inline void bsp_epwm_ch2_LSSC()
{
    // 开启 下桥 IGBT
    EPwm4Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm4Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_LOW;

    EPwm5Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm5Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_LOW;

    EPwm6Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwm6Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_LOW;
}

// mode : 0@DOWN 1@UP; bit0 A, bit1 B, bit2 C
static inline void bsp_epwm_ch2_LSSC2(int16_t mode)
{
    if (mode & 0x1)
    {
        EPwm4Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_HIGH;
        EPwm4Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;
    }
    else
    {
        EPwm4Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
        EPwm4Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_LOW;
    }

    if (mode & 0x2)
    {
        EPwm5Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_HIGH;
        EPwm5Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;
    }
    else
    {
        EPwm5Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
        EPwm5Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_LOW;
    }

    if (mode & 0x4)
    {
        EPwm6Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_HIGH;
        EPwm6Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;
    }
    else
    {
        EPwm6Regs.AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
        EPwm6Regs.AQCSFRC.bit.CSFB = TYJ_AQSFRC_LOW;
    }
}

// adc A

static inline int16_t bsp_get_CH2_Iw_adcRaw(void)
{
    return AdcaResultRegs.ADCRESULT0;
}

static inline int16_t bsp_get_CH2_Va_adcRaw(void)
{
    return AdcaResultRegs.ADCRESULT1;
}

static inline int16_t bsp_get_CH2_Udc_adcRaw(void)
{
    return AdcaResultRegs.ADCRESULT2;
}

static inline int16_t bsp_get_CH1_Iw_adcRaw(void)
{
    return AdcaResultRegs.ADCRESULT3;
}


// adc B

static inline int16_t bsp_get_CH2_Iv_adcRaw(void)
{
    return AdcbResultRegs.ADCRESULT0;
}

static inline int16_t bsp_get_CH2_Vc_adcRaw(void)
{
    return AdcbResultRegs.ADCRESULT1;
}

static inline int16_t bsp_get_CH1_Iv_adcRaw(void)
{
    return AdcbResultRegs.ADCRESULT2;
}

static inline int16_t bsp_get_CH1_Udc_adcRaw(void)
{
    return AdcbResultRegs.ADCRESULT3;
}

// adc C

static inline int16_t bsp_get_CH2_Iu_adcRaw(void)
{
    return AdccResultRegs.ADCRESULT0;
}

static inline int16_t bsp_get_CH2_Vb_adcRaw(void)
{
    return AdccResultRegs.ADCRESULT1;
}

static inline int16_t bsp_get_CH1_Iu_adcRaw(void)
{
    return AdccResultRegs.ADCRESULT2;
}

// adc D

static inline uint16_t bsp_get_diff1_adcRaw(void)
{
    return AdcdResultRegs.ADCRESULT0;
}

static inline uint16_t bsp_get_ntc1_adcRaw(void)
{
    return AdcdResultRegs.ADCRESULT1;
}

static inline uint16_t bsp_get_ntcRef1_adcRaw(void)
{
    return AdcdResultRegs.ADCRESULT2;
}

// adc sw trig
static inline void bsp_adc_swTrig_ntc(void)
{
    AdcdRegs.ADCSOCFRC1.all = 0b110u;
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
