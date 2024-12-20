/*
 * isr.c
 *
 *  Created on: 2022年6月24日
 *      Author: tyj
 */

#include "F28x_Project.h"
#include "stdint.h"

#include "algo_code_config.h"

#include "proj.h"
#include "isr.h"

 // 观测主中断执行时间
uint16_t isr_start_pwm_cnt = 0;
uint16_t isr_end_pwm_cnt = 0;

uint16_t isr_start_pwm_cnt2 = 0;
uint16_t isr_end_pwm_cnt2 = 0;

#pragma CODE_SECTION(adca1_isr, MEM_MACRO);
interrupt void adca1_isr(void)
{
    // 查看执行时间
    volatile struct EPWM_REGS* EPwmxRegs = &EPwm7Regs;
    isr_start_pwm_cnt2 = EPwmxRegs->TBCTR;
    // if (EPwmxRegs->TBSTS.bit.CTRDIR == 0u)
    // {
    //     isr_start_pwm_cnt2 += 10000u;
    // }

    // 主动清零IPC，保证中断触发
    // IpcRegs.IPCCLR.bit.IPC0 = 1;
    // 在峰值处算一个周期，此次跳过

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;   //clear INT1 flag for ADC-A
    if (1 == AdcaRegs.ADCINTOVF.bit.ADCINT1) //ADCINT overflow occurred
    {
        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //Clear overflow flag
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //Re-clear ADCINT flag
    }
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    // ch2 drv8305 方案
    mainIsrProcess2();

    // 同步CPU2 用IPC0
    // IpcRegs.IPCSET.bit.IPC0 = 1;
    // 在峰值处算一个周期，此次跳过

    // 查看执行时间
    isr_end_pwm_cnt2 = EPwmxRegs->TBCTR;
    // if (EPwmxRegs->TBSTS.bit.CTRDIR == 0u)
    // {
    //     isr_end_pwm_cnt2 += 10000u;
    // }
}

// 载波峰值
#pragma CODE_SECTION(adcb1_isr, MEM_MACRO);
interrupt void adcb1_isr(void)
{
    // 查看执行时间
    volatile struct EPWM_REGS* EPwmxRegs = &EPwm7Regs;
    isr_start_pwm_cnt = EPwmxRegs->TBCTR;
    // if (EPwmxRegs->TBSTS.bit.CTRDIR == 0u)
    // {
    //     isr_start_pwm_cnt += 10000u;
    // }

    // 主动清零IPC，保证中断触发
    IpcRegs.IPCCLR.bit.IPC0 = 1;

    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;   //clear INT1 flag for ADC-A
    if (1 == AdcbRegs.ADCINTOVF.bit.ADCINT1) //ADCINT overflow occurred
    {
        AdcbRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //Clear overflow flag
        AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //Re-clear ADCINT flag
    }
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    // ch1 IGBT IPM 方案
    mainIsrProcess();

    // 同步CPU2 用IPC0
    IpcRegs.IPCSET.bit.IPC0 = 1;

    // 查看执行时间
    isr_end_pwm_cnt = EPwmxRegs->TBCTR;
    // if (EPwmxRegs->TBSTS.bit.CTRDIR == 0u)
    // {
    //     isr_end_pwm_cnt += 10000u;
    // }
}

// Unused ISR
interrupt void NOTUSED_ISR_TYJ(void)
{
    asm("      ESTOP0");
    for (;;);
}

