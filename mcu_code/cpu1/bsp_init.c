/*
 * bsp.c
 *
 *  Created on: 2022年6月17日
 *      Author: tyj
 */

#include "F2837xD_Ipc_drivers.h"
#include "F28x_Project.h"

#include "string.h"

#include "bsp_cfg.h"
#include "bsp_init.h"
#include "bsp_runtime.h"
#include "bsp_inline.h"

#include "isr.h"
#include "scd_inc.h"
#include "bootcpu2.h"
#include "cdb_cpu1.h"

#include "proj.h"

#include "drv8305.h"

 // 最小化引入 driverlib
// #include "can.h"
#include "gpio.h"
#include "encoder.h"
#include "eqep.h"
#include "pin_map.h"


// 将魔改过的底驱全部保存至自己的源文件
void initSysCtrl()
{
    // Disable the watchdog
    volatile Uint16 temp;
    // Grab the clock config first so we don't clobber it
    EALLOW;
    temp = WdRegs.WDCR.all & 0x0007;
    WdRegs.WDCR.all = 0x0068 | temp;
    EDIS;

#ifdef _FLASH
    // Copy time critical code and Flash setup code to RAM. This includes the
    // following functions: InitFlash()
    // The  RamfuncsLoadStart, RamfuncsLoadSize, and RamfuncsRunStart
    // symbols are created by the linker. Refer to the device .cmd file.
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);

    // Call Flash Initialization to setup flash waitstates. This function must
    // reside in RAM.
    InitFlash();
#endif

    // The Device_cal function, which copies the ADC & oscillator calibration
    // values from TI reserved OTP into the appropriate trim registers, occurs
    // automatically in the Boot ROM. If the boot ROM code is bypassed during
    // the debug process, the following function MUST be called for the ADC and
    // oscillators to function according to specification. The clocks to the
    // ADC MUST be enabled before calling this function.
    // See the device data manual and/or the ADC Reference Manual for more
    // information.
#ifdef CPU1
    EALLOW;

    // Enable pull-ups on unbonded IOs as soon as possible to reduce power
    // consumption.
    GPIO_EnableUnbondedIOPullups();

    CpuSysRegs.PCLKCR13.bit.ADC_A = 1;
    CpuSysRegs.PCLKCR13.bit.ADC_B = 1;
    CpuSysRegs.PCLKCR13.bit.ADC_C = 1;
    CpuSysRegs.PCLKCR13.bit.ADC_D = 1;

    // Check if device is trimmed
    if (*((Uint16*)0x5D1B6) == 0x0000) {
        //
        // Device is not trimmed--apply static calibration values
        //
        AnalogSubsysRegs.ANAREFTRIMA.all = 31709;
        AnalogSubsysRegs.ANAREFTRIMB.all = 31709;
        AnalogSubsysRegs.ANAREFTRIMC.all = 31709;
        AnalogSubsysRegs.ANAREFTRIMD.all = 31709;
    }

    CpuSysRegs.PCLKCR13.bit.ADC_A = 0;
    CpuSysRegs.PCLKCR13.bit.ADC_B = 0;
    CpuSysRegs.PCLKCR13.bit.ADC_C = 0;
    CpuSysRegs.PCLKCR13.bit.ADC_D = 0;
    EDIS;

    // Initialize the PLL control: SYSPLLMULT and SYSCLKDIVSEL.
    // Defined options to be passed as arguments to this function are defined
    // in F2837xD_Examples.h.
    // Note: The internal oscillator CANNOT be used as the PLL source if the
    // PLLSYSCLK is configured to frequencies above 194 MHz.
    // PLLSYSCLK = (XTAL_OSC) * (IMULT + FMULT) / (PLLSYSCLKDIV)
    InitSysPll(XTAL_OSC, IMULT_20, FMULT_0, PLLCLK_BY_1);

#endif // CPU1

    // Turn on all peripherals basic
    EALLOW;

    CpuSysRegs.PCLKCR0.bit.CLA1 = 1;
    CpuSysRegs.PCLKCR0.bit.DMA = 1;
    CpuSysRegs.PCLKCR0.bit.CPUTIMER0 = 1;
    CpuSysRegs.PCLKCR0.bit.CPUTIMER1 = 1;
    CpuSysRegs.PCLKCR0.bit.CPUTIMER2 = 1;

#ifdef CPU1
    CpuSysRegs.PCLKCR0.bit.HRPWM = 0;
#endif

    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;

#ifdef CPU1
    CpuSysRegs.PCLKCR1.bit.EMIF1 = 0;
    CpuSysRegs.PCLKCR1.bit.EMIF2 = 0;
#endif

    CpuSysRegs.PCLKCR2.bit.EPWM1 = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM3 = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM4 = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM5 = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM6 = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM7 = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM8 = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM9 = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM10 = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM11 = 0;
    CpuSysRegs.PCLKCR2.bit.EPWM12 = 0;

    CpuSysRegs.PCLKCR3.bit.ECAP1 = 0;
    CpuSysRegs.PCLKCR3.bit.ECAP2 = 0;
    CpuSysRegs.PCLKCR3.bit.ECAP3 = 0;
    CpuSysRegs.PCLKCR3.bit.ECAP4 = 0;
    CpuSysRegs.PCLKCR3.bit.ECAP5 = 0;
    CpuSysRegs.PCLKCR3.bit.ECAP6 = 0;

    CpuSysRegs.PCLKCR4.bit.EQEP1 = 0;
    CpuSysRegs.PCLKCR4.bit.EQEP2 = 0;
    CpuSysRegs.PCLKCR4.bit.EQEP3 = 0;

    CpuSysRegs.PCLKCR6.bit.SD1 = 0;
    CpuSysRegs.PCLKCR6.bit.SD2 = 0;

    CpuSysRegs.PCLKCR7.bit.SCI_A = 0;
    CpuSysRegs.PCLKCR7.bit.SCI_B = 0;
    CpuSysRegs.PCLKCR7.bit.SCI_C = 0;
    CpuSysRegs.PCLKCR7.bit.SCI_D = 0;

    CpuSysRegs.PCLKCR8.bit.SPI_A = 0;
    CpuSysRegs.PCLKCR8.bit.SPI_B = 0;
    CpuSysRegs.PCLKCR8.bit.SPI_C = 0;

    CpuSysRegs.PCLKCR9.bit.I2C_A = 0;
    CpuSysRegs.PCLKCR9.bit.I2C_B = 0;

    CpuSysRegs.PCLKCR10.bit.CAN_A = 0;
    CpuSysRegs.PCLKCR10.bit.CAN_B = 0;

    CpuSysRegs.PCLKCR11.bit.McBSP_A = 0;
    CpuSysRegs.PCLKCR11.bit.McBSP_B = 0;

#ifdef CPU1
    CpuSysRegs.PCLKCR11.bit.USB_A = 0;

    CpuSysRegs.PCLKCR12.bit.uPP_A = 0;
#endif

    CpuSysRegs.PCLKCR13.bit.ADC_A = 0;
    CpuSysRegs.PCLKCR13.bit.ADC_B = 0;
    CpuSysRegs.PCLKCR13.bit.ADC_C = 0;
    CpuSysRegs.PCLKCR13.bit.ADC_D = 0;

    CpuSysRegs.PCLKCR14.bit.CMPSS1 = 0;
    CpuSysRegs.PCLKCR14.bit.CMPSS2 = 0;
    CpuSysRegs.PCLKCR14.bit.CMPSS3 = 0;
    CpuSysRegs.PCLKCR14.bit.CMPSS4 = 0;
    CpuSysRegs.PCLKCR14.bit.CMPSS5 = 0;
    CpuSysRegs.PCLKCR14.bit.CMPSS6 = 0;
    CpuSysRegs.PCLKCR14.bit.CMPSS7 = 0;
    CpuSysRegs.PCLKCR14.bit.CMPSS8 = 0;

    CpuSysRegs.PCLKCR16.bit.DAC_A = 0;
    CpuSysRegs.PCLKCR16.bit.DAC_B = 0;
    CpuSysRegs.PCLKCR16.bit.DAC_C = 0;

    EDIS;

    // 配置LSPCLK为200MHz
    EALLOW;
    ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 0;
    EDIS;

    return;
}

static void init_uart_regs(volatile struct SCI_REGS* SciXRegs)
{
    (*SciXRegs).SCICTL1.bit.SWRESET = 0;
    (*SciXRegs).SCIFFTX.bit.TXFIFORESET = 0;
    (*SciXRegs).SCIFFRX.bit.RXFIFORESET = 0;

    // 8数据位，1停止位，无校验
    (*SciXRegs).SCICCR.all = 0b00000111;
    (*SciXRegs).SCICTL1.all = 0b00000011;
    // 关闭接收错误中断
    (*SciXRegs).SCICTL1.bit.RXERRINTENA = 0;
    // 退出复位模式
    (*SciXRegs).SCICTL1.bit.SWRESET = 1;
    // 关闭发送模式唤醒
    (*SciXRegs).SCICTL1.bit.TXWAKE = 0;
    // 关闭睡眠模式
    (*SciXRegs).SCICTL1.bit.SLEEP = 0;
    // 发送使能 on
    (*SciXRegs).SCICTL1.bit.TXENA = 1;
    // 接收使能 off
    (*SciXRegs).SCICTL1.bit.RXENA = 1;

    // RegVal=LSPCLK/(baudRate*8)-1
    // 波特率5M
    // (*SciXRegs).SCILBAUD.bit.BAUD = 0x4;
    // (*SciXRegs).SCIHBAUD.bit.BAUD = 0x0;
    // 波特率1M
    // (*SciXRegs).SCILBAUD.bit.BAUD = 0x18;
    // (*SciXRegs).SCIHBAUD.bit.BAUD = 0x0;
    // 波特率460800
    (*SciXRegs).SCILBAUD.bit.BAUD = 0x35;
    (*SciXRegs).SCIHBAUD.bit.BAUD = 0x0;

    // 0 Bit间隔
    (*SciXRegs).SCIFFCT.bit.FFTXDLY = 0;
    // 启用FIFO
    (*SciXRegs).SCIFFTX.bit.SCIFFENA = 1;
    // 还剩2字节时中断
    (*SciXRegs).SCIFFTX.bit.TXFFIL = 2;
    // 不启用发送FIFO中断
    (*SciXRegs).SCIFFTX.bit.TXFFIENA = 0;
    // 收到4字节时中断
    (*SciXRegs).SCIFFRX.bit.RXFFIL = 4;
    // 不启用接收FIFO中断
    (*SciXRegs).SCIFFRX.bit.RXFFIENA = 0;
    // 调试行为不影响
    (*SciXRegs).SCIPRI.all = 0b11000;

    // 中断配置，都不使能
    (*SciXRegs).SCICTL2.bit.TXINTENA = 0;
    (*SciXRegs).SCICTL2.bit.RXBKINTENA = 0;

    // 退出复位模式
    (*SciXRegs).SCICTL1.bit.SWRESET = 1;
    // TXFIFO退出复位状态
    (*SciXRegs).SCIFFTX.bit.TXFIFORESET = 1;
    // RXFIFO退出复位状态
    (*SciXRegs).SCIFFRX.bit.RXFIFORESET = 1;

    return;
}

static void init_uart_A()
{
    EALLOW;
    // 启动SCI-A的时钟
    CpuSysRegs.PCLKCR7.bit.SCI_A = 1;
    EDIS;

    // 配置引脚
    GPIO_SetupPinMux(42, GPIO_MUX_CPU1, 15);
    GPIO_SetupPinOptions(42, GPIO_INPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(43, GPIO_MUX_CPU1, 15);
    GPIO_SetupPinOptions(43, GPIO_OUTPUT, GPIO_ASYNC);

    init_uart_regs(&SciaRegs);
    return;
}

// 配置CHx一相的epwm
static void cfg_epwm_1pha(volatile struct EPWM_REGS* EPwmxRegs, int syncMode)
{
    // 配置时基模块
    EPwmxRegs->TBPRD = vPWM_LOAD_VAL_I;
    EPwmxRegs->TBPHS.bit.TBPHS = 0x0;
    EPwmxRegs->TBCTR = 0x0;
    EPwmxRegs->TBCTL.bit.FREE_SOFT = 0;
    EPwmxRegs->TBCTL.bit.PHSDIR = TB_UP;
    EPwmxRegs->TBCTL.bit.CLKDIV = TB_DIV1;
    EPwmxRegs->TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwmxRegs->TBCTL.bit.SWFSYNC = 0;
    if (syncMode == 0)
    {
        // 0代表ePWM1，取其零点产生同步信号
        EPwmxRegs->TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
    }
    else
    {
        // 其他ePWM传播同步信号即可
        EPwmxRegs->TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    }
    EPwmxRegs->TBCTL.bit.PRDLD = TB_SHADOW;
    if (syncMode == 0)
    {
        EPwmxRegs->TBCTL.bit.PHSEN = TB_DISABLE;
    }
    else
    {
        EPwmxRegs->TBCTL.bit.PHSEN = TB_ENABLE;
    }
    EPwmxRegs->TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;

    EPwmxRegs->TBCTL2.bit.PRDLDSYNC = 1;
    EPwmxRegs->TBCTL2.bit.SYNCOSELX = 0;

    // EPwmxRegs->TBSTS 状态读取寄存器，不用配置

    EPwmxRegs->CMPA.bit.CMPA = vPWM_CMP_DEFAILT_VAL_I;
    EPwmxRegs->CMPB.bit.CMPB = vPWM_CMP_DEFAILT_VAL_I;

    EPwmxRegs->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwmxRegs->CMPCTL.bit.SHDWBMODE = CC_SHADOW;
#if MATLAB_PARA_ctrl_freq_mul == 2
    EPwmxRegs->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO_PRD;
    EPwmxRegs->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO_PRD;
    EPwmxRegs->CMPCTL2.bit.LOADCMODE = CC_CTR_ZERO_PRD;
    EPwmxRegs->CMPCTL2.bit.LOADDMODE = CC_CTR_ZERO_PRD;
#else
    EPwmxRegs->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwmxRegs->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwmxRegs->CMPCTL2.bit.LOADCMODE = CC_CTR_ZERO;
    EPwmxRegs->CMPCTL2.bit.LOADDMODE = CC_CTR_ZERO;
#endif

    // 不使用预装载
    EPwmxRegs->AQCTL.all = 0;
    // 装载信号选择也不必考虑
    EPwmxRegs->AQTSRCSEL.all = 0;

    EPwmxRegs->AQCTLA.all = 0;
    EPwmxRegs->AQCTLA.bit.CAU = AQ_SET;
    EPwmxRegs->AQCTLA.bit.CAD = AQ_CLEAR;
    // 无需用高级触发
    EPwmxRegs->AQCTLA2.all = 0;

    EPwmxRegs->AQCTLB.all = 0;
    EPwmxRegs->AQCTLB.bit.CAU = AQ_SET;
    EPwmxRegs->AQCTLB.bit.CAD = AQ_CLEAR;
    // 无需用高级触发
    EPwmxRegs->AQCTLB2.all = 0;

    EPwmxRegs->AQSFRC.bit.RLDCSF = 3; // 不使用shadow
    EPwmxRegs->AQSFRC.bit.ACTSFA = AQ_CLEAR;
    EPwmxRegs->AQSFRC.bit.ACTSFB = AQ_SET;

#define TYJ_AQSFRC_LOW 0b01
#define TYJ_AQSFRC_HIGH 0b10

    // 先关闭IGBT
    EPwmxRegs->AQCSFRC.bit.CSFA = TYJ_AQSFRC_LOW;
    EPwmxRegs->AQCSFRC.bit.CSFB = TYJ_AQSFRC_HIGH;

#define TYJ_DB_LOAD_ZERO 0
#define TYJ_DB_LOAD_PRD 1
#define TYJ_DB_LOAD_ZERO_PRD 2
#define TYJ_DB_LOAD_FREEZE 3

    // EPwmxRegs->DBxx
    EPwmxRegs->DBCTL.bit.HALFCYCLE = 0;
    EPwmxRegs->DBCTL.bit.DEDB_MODE = 0;
    EPwmxRegs->DBCTL.bit.OUTSWAP = 0;
    EPwmxRegs->DBCTL.bit.SHDWDBFEDMODE = 0;
    EPwmxRegs->DBCTL.bit.SHDWDBREDMODE = 0;
    EPwmxRegs->DBCTL.bit.LOADFEDMODE = TYJ_DB_LOAD_ZERO_PRD;
    EPwmxRegs->DBCTL.bit.LOADREDMODE = TYJ_DB_LOAD_ZERO_PRD;
    EPwmxRegs->DBCTL.bit.IN_MODE = DBA_RED_DBB_FED;

    // DRV8305 高有效
    EPwmxRegs->DBCTL.bit.POLSEL = DB_ACTV_HIC;

    EPwmxRegs->DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwmxRegs->DBCTL2.bit.SHDWDBCTLMODE = 0;
    EPwmxRegs->DBCTL2.bit.LOADDBCTLMODE = TYJ_DB_LOAD_ZERO_PRD;
    // 死区时间配置
    EPwmxRegs->DBRED.bit.DBRED = PWM_DEADBAND_TICKS;
    EPwmxRegs->DBFED.bit.DBFED = PWM_DEADBAND_TICKS;

    // GLD与EPWMXLINK不用

    // EPwm6Regs.PCCTL
    // 斩波模块不用

    // EPwmxRegs->TZ ，该硬件板硬件保护电路有误，暂不使用

    // EPwmxRegs->ETxx ，在外部配置，配置1相即可
    return;
}

static void init_epwm_CH1()
{

    // 设置时钟
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    // 100MHz 分频
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;

    CpuSysRegs.PCLKCR2.bit.EPWM4 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM5 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM6 = 1;
    EDIS;

    // epwm1 用于计算CPU1的中断时间
    cfg_epwm_1pha(&EPwm1Regs, 0);
    // epwm2 用于触发CPU1的CLA1, 及CLA1的软件计时
    cfg_epwm_1pha(&EPwm2Regs, 1);

    cfg_epwm_1pha(&EPwm4Regs, 1);
    cfg_epwm_1pha(&EPwm5Regs, 1);
    cfg_epwm_1pha(&EPwm6Regs, 1);


    // 配置EPWM的CH1的触发事件
#if 1
    // 在CH1的a相上，只在数到零时触发
    volatile struct EPWM_REGS* EPwmxRegs = &EPwm4Regs;
    EPwmxRegs->ETSEL.all = 0;
    EPwmxRegs->ETSEL.bit.SOCAEN = 1;

#if MATLAB_PARA_ctrl_freq_mul == 2
    EPwmxRegs->ETSEL.bit.SOCASEL = ET_CTR_PRDZERO;
#else
    EPwmxRegs->ETSEL.bit.SOCASEL = ET_CTR_ZERO;
#endif

    EPwmxRegs->ETPS.all = 0;
    EPwmxRegs->ETPS.bit.SOCAPRD = 1;
    // 无需使用高级特性

#endif

    // 配置与epwm相关的GPIO
    // 上电后GPIO被配置成上拉（关闭）态，因此最后配置GPIO即可

    // 魔改 InitEPwmxGpio();

    EALLOW;
    // Disable pull-up
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;

    // Configure GPIOx
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;
    EDIS;

    return;
}

static void init_adc_power_up()
{
    EALLOW;
    // 开启时钟
    CpuSysRegs.PCLKCR13.bit.ADC_A = 1;
    CpuSysRegs.PCLKCR13.bit.ADC_B = 1;
    CpuSysRegs.PCLKCR13.bit.ADC_C = 1;
    CpuSysRegs.PCLKCR13.bit.ADC_D = 1;
    // adc 50MHz
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6;
    AdccRegs.ADCCTL2.bit.PRESCALE = 6;
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6;
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    // late中断
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    // 不使用burst
    AdcaRegs.ADCBURSTCTL.all = 0;
    AdcbRegs.ADCBURSTCTL.all = 0;
    AdccRegs.ADCBURSTCTL.all = 0;
    AdcdRegs.ADCBURSTCTL.all = 0;
    // 上电
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    EDIS;

    return;
}

static void init_adc_a()
{
    EALLOW;
    // 使用3高优先级, 确保中断发生时需要的都采到了
    AdcaRegs.ADCSOCPRICTL.bit.SOCPRIORITY = 3;

    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 4;   // ch1 Ic
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = ADC_SAMP_TICKS;  //sample time
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 11; //trigger on ePWM4 SOCA/C

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 15;  // ch1 Va
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = ADC_SAMP_TICKS;  //sample time
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 11; //trigger on ePWM4 SOCA/C

    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 5;   // ch1 Udc
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = ADC_SAMP_TICKS;  //sample time
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 11; //trigger on ePWM4 SOCA/C

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 2;  //end of SOC2 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 1; //允许重复中断
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;    //enable INT1 flag
    EDIS;

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag

    return;
}

static void init_adc_b()
{
    EALLOW;
    // 使用2高优先级
    AdcbRegs.ADCSOCPRICTL.bit.SOCPRIORITY = 2;

    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 4;   // ch1 Ib
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = ADC_SAMP_TICKS;  //sample time
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 11; //trigger on ePWM4 SOCA/C

    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 5;   // ch1 Vc
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = ADC_SAMP_TICKS;  //sample time
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 11; //trigger on ePWM4 SOCA/C

    EDIS;

    return;
}

static void init_adc_c()
{
    EALLOW;
    // 使用2高优先级
    AdccRegs.ADCSOCPRICTL.bit.SOCPRIORITY = 2;

    AdccRegs.ADCSOC0CTL.bit.CHSEL = 4;   // ch1 Ia
    AdccRegs.ADCSOC0CTL.bit.ACQPS = ADC_SAMP_TICKS;  //sample time
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 11; //trigger on ePWM4 SOCA/C

    AdccRegs.ADCSOC1CTL.bit.CHSEL = 5;   // ch1 Vb
    AdccRegs.ADCSOC1CTL.bit.ACQPS = ADC_SAMP_TICKS;  //sample time
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 11; //trigger on ePWM4 SOCA/C

    EDIS;

    return;
}

static void init_adc_d()
{
    EALLOW;
    // 使用2高优先级
    AdcdRegs.ADCSOCPRICTL.bit.SOCPRIORITY = 2;

    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 4;   // diff1
    AdcdRegs.ADCSOC0CTL.bit.ACQPS = ADC_SAMP_TICKS;  //sample time
    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 11; //trigger on ePWM4 SOCA/C

    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 5;   // diff2
    AdcdRegs.ADCSOC1CTL.bit.ACQPS = ADC_SAMP_TICKS;  //sample time
    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 11; //trigger on ePWM4 SOCA/C

    AdcdRegs.ADCSOC2CTL.bit.CHSEL = 0;   // resv
    AdcdRegs.ADCSOC2CTL.bit.ACQPS = ADC_SAMP_TICKS;  //sample time
    AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = 0; //trigger sw

    AdcdRegs.ADCSOC3CTL.bit.CHSEL = 1;  // resv
    AdcdRegs.ADCSOC3CTL.bit.ACQPS = ADC_SAMP_TICKS;  //sample time
    AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = 0; //trigger sw

    AdcdRegs.ADCSOC4CTL.bit.CHSEL = 2;  // resv
    AdcdRegs.ADCSOC4CTL.bit.ACQPS = ADC_SAMP_TICKS;  //sample time
    AdcdRegs.ADCSOC4CTL.bit.TRIGSEL = 0; //trigger sw

    AdcdRegs.ADCSOC5CTL.bit.CHSEL = 3;  // resv
    AdcdRegs.ADCSOC5CTL.bit.ACQPS = ADC_SAMP_TICKS;  //sample time
    AdcdRegs.ADCSOC5CTL.bit.TRIGSEL = 0; //trigger sw

    EDIS;

    return;
}

static void init_eqep2(int ifInvQaQb)
{
    // 设置时钟
    EALLOW;
    CpuSysRegs.PCLKCR4.bit.EQEP2 = 1;
    EDIS;

    // gpio cfg 
    // no pull, qual 6, mux to eqep
    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1;
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1;
    GpioCtrlRegs.GPBPUD.bit.GPIO57 = 1;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO57 = 2;
    GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 1;
    GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 1;
    GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 1;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO54 = 1;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO55 = 1;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO57 = 1;
    EDIS;

    // Sets the polarity of the eQEP module's input signals.
    EQEP_setInputPolarity(EQEP_TYJDEV_BASE, true, true, true, false);
    // Configures eQEP module's quadrature decoder unit.
    if (ifInvQaQb)
        EQEP_setDecoderConfig(EQEP_TYJDEV_BASE, (EQEP_CONFIG_QUADRATURE | EQEP_CONFIG_2X_RESOLUTION | EQEP_CONFIG_SWAP | EQEP_CONFIG_IGATE_DISABLE));
    else
        EQEP_setDecoderConfig(EQEP_TYJDEV_BASE, (EQEP_CONFIG_QUADRATURE | EQEP_CONFIG_2X_RESOLUTION | EQEP_CONFIG_NO_SWAP | EQEP_CONFIG_IGATE_DISABLE));
    // Set the emulation mode of the eQEP module.
    EQEP_setEmulationMode(EQEP_TYJDEV_BASE, EQEP_EMULATIONMODE_RUNFREE);
    // Configures eQEP module position counter unit.
    EQEP_setPositionCounterConfig(EQEP_TYJDEV_BASE, EQEP_POSITION_RESET_IDX, ENCO_PPR * 4 - 1);
    // Sets the current encoder position.
    EQEP_setPosition(EQEP_TYJDEV_BASE, 0U);
    // Disables the eQEP module unit timer.
    EQEP_disableUnitTimer(EQEP_TYJDEV_BASE);
    // Disables the eQEP module watchdog timer.
    EQEP_disableWatchdog(EQEP_TYJDEV_BASE);
    // Configures the quadrature modes in which the position count can be latched.
    EQEP_setLatchMode(EQEP_TYJDEV_BASE, (EQEP_LATCH_CNT_READ_BY_CPU | EQEP_LATCH_RISING_STROBE | EQEP_LATCH_FALLING_INDEX));
    // Configures the mode in which the position counter is initialized.
    EQEP_setPositionInitMode(EQEP_TYJDEV_BASE, (EQEP_INIT_DO_NOTHING));
    // Sets the software initialization of the encoder position counter.
    EQEP_setSWPositionInit(EQEP_TYJDEV_BASE, false);
    // Sets the init value for the encoder position counter.
    EQEP_setInitialPosition(EQEP_TYJDEV_BASE, 0U);
    // Enables the eQEP module.
    EQEP_enableModule(EQEP_TYJDEV_BASE);
    // Configures eQEP module edge-capture unit.
    EQEP_setCaptureConfig(EQEP_TYJDEV_BASE, EQEP_CAPTURE_CLK_DIV_1, EQEP_UNIT_POS_EVNT_DIV_4);
    // Enables the eQEP module edge-capture unit.
    EQEP_enableCapture(EQEP_TYJDEV_BASE);
}

static void init_board_gpio()
{
    // DRV8305 fault gpio139
    GPIO_SetupPinMux(139, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(139, GPIO_INPUT, GPIO_PULLUP | GPIO_QUAL6);

    // DRV8305 WAKE gpio27
    // 不考虑低功耗
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_WritePin(27, 1);

    // POWER EN gpio26 上电后复位
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    bsp_POWER_EN_CH1(0);

    // LEDS
    // LED D9 RED
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    bsp_LED_D9_RED_CTRL(0);

    // LED D10 BLUE
    GPIO_SetupPinMux(31, GPIO_MUX_CPU2, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);

    return;
}

static void init_cpu_timers()
{
    // tim_0 1ms 主循环低优先级任务，滴答计数

    // Initialize timer period, 200MHz * 1000us -> 1ms
    CpuTimer0Regs.PRD.all = (200ul * 1000ul - 1);
    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT):
    CpuTimer0Regs.TPR.all = 0;
    CpuTimer0Regs.TPRH.all = 0;

    CpuTimer0Regs.TCR.all = 0;
    // Make sure timer is stopped:
    CpuTimer0Regs.TCR.bit.TSS = 1;
    // Reload all counter register with period value:
    CpuTimer0Regs.TCR.bit.TRB = 1;

    // tim_1 100ms cdb轮询任务，滴答计数

    // Initialize timer period, 200MHz * 4000us -> 4ms
    CpuTimer1Regs.PRD.all = (200ul * 1000ul * 100ul - 1);
    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT):
    CpuTimer1Regs.TPR.all = 0;
    CpuTimer1Regs.TPRH.all = 0;

    CpuTimer1Regs.TCR.all = 0;
    // Make sure timer is stopped:
    CpuTimer1Regs.TCR.bit.TSS = 1;
    // Reload all counter register with period value:
    CpuTimer1Regs.TCR.bit.TRB = 1;
    return;
}

static void init_canB_cpu1()
{
    // init GPIO
    GPIO_setPinConfig(GPIO_12_CANTXB);
    GPIO_setPinConfig(GPIO_17_CANRXB);

    return;
}

void bsp_init_chip_devs()
{
    init_board_gpio();
    init_uart_A();
    init_canB_cpu1();
    init_epwm_CH1();
    init_adc_power_up();
    DELAY_US(1000);
    init_adc_a();
    init_adc_b();
    init_adc_c();
    init_adc_d();
    init_cpu_timers();
    init_eqep2(1);

    return;
}

void bsp_init_board_devs()
{
    drv8305_init();

    return;
}

void bsp_interrupt_cfg()
{
    // 魔改 InitPieCtrl();

    // Disable Interrupts at the CPU level
    DINT;

    // Disable the PIE
    PieCtrlRegs.PIECTRL.bit.ENPIE = 0;

    // Clear all PIEIER registers:
    PieCtrlRegs.PIEIER1.all = 0;
    PieCtrlRegs.PIEIER2.all = 0;
    PieCtrlRegs.PIEIER3.all = 0;
    PieCtrlRegs.PIEIER4.all = 0;
    PieCtrlRegs.PIEIER5.all = 0;
    PieCtrlRegs.PIEIER6.all = 0;
    PieCtrlRegs.PIEIER7.all = 0;
    PieCtrlRegs.PIEIER8.all = 0;
    PieCtrlRegs.PIEIER9.all = 0;
    PieCtrlRegs.PIEIER10.all = 0;
    PieCtrlRegs.PIEIER11.all = 0;
    PieCtrlRegs.PIEIER12.all = 0;

    // Clear all PIEIFR registers:
    PieCtrlRegs.PIEIFR1.all = 0;
    PieCtrlRegs.PIEIFR2.all = 0;
    PieCtrlRegs.PIEIFR3.all = 0;
    PieCtrlRegs.PIEIFR4.all = 0;
    PieCtrlRegs.PIEIFR5.all = 0;
    PieCtrlRegs.PIEIFR6.all = 0;
    PieCtrlRegs.PIEIFR7.all = 0;
    PieCtrlRegs.PIEIFR8.all = 0;
    PieCtrlRegs.PIEIFR9.all = 0;
    PieCtrlRegs.PIEIFR10.all = 0;
    PieCtrlRegs.PIEIFR11.all = 0;
    PieCtrlRegs.PIEIFR12.all = 0;

    IER = 0x0000;
    IFR = 0x0000;

    // 魔改 InitPieVectTable();
    Uint16 i;
    Uint32* Dest = (Uint32*)(void*)&PieVectTable;

    //
    // Do not write over first 3 32-bit locations (these locations are
    // initialized by Boot ROM with boot variables)
    //
    Dest = Dest + 3;

    EALLOW;
    for (i = 0; i < 221; i++)
    {
        *Dest++ = (Uint32)NOTUSED_ISR_TYJ;
    }
    EDIS;

    //
    // Enable the PIE Vector Table
    //
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;

    EALLOW;
    PieVectTable.ADCA1_INT = &adca1_isr; //function for ADCA interrupt 1
    EDIS;

    //Enable group 1 interrupts
    IER |= M_INT1;

    // enable PIE interrupt
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    // Enables PIE to drive a pulse into the CPU
    PieCtrlRegs.PIEACK.all = 0xFFFF;

    EINT; // Enable Global interrupt INTM
    return;
}

void bsp_start()
{
    // 启动scd调试支持
#ifdef _SCD_ENABLE
    // 初始化
    scd_init_1();

#endif

    // 起动DRV8305
    bsp_POWER_EN_CH1(1);
    // 等待10ms，保证drv8305完成初始化
    DELAY_US(10000);

    // 启动pwm波
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    // 启动 tim_0 主循环低优先级任务滴答计时器
    CpuTimer0Regs.TCR.bit.TSS = 0;

    // 启动 tim_1 cdb轮询任务滴答计时器
    CpuTimer1Regs.TCR.bit.TSS = 0;

    // 在该控制板上，先要等强电上电，才能校准偏置
    while (CH1_Udc < CH1_Udc_LThd)
    {
#ifdef _SCD_ENABLE
        // 保证调试功能正常
        scd_call_in_mainLoop();
        cdb_ack_cpu1();
#endif

    }
    // 等待10ms，保证drv8305完成初始化
    DELAY_US(10000);

    // 初始化adc偏置
    adcOffset_init(4000);

#ifdef _DEBUG
    // Enable Global realtime interrupt DBGM
    ERTM;
#endif

    return;
}

void bsp_init_cla()
{
#ifdef _FLASH
    // 初始化cla内存
    extern uint16_t Cla1funcsRunStart, Cla1funcsLoadStart, Cla1funcsLoadSize;
    extern uint16_t Cla1ConstRunStart, Cla1ConstLoadStart, Cla1ConstLoadSize;

    memcpy((uint32_t*)&Cla1funcsRunStart, (uint32_t*)&Cla1funcsLoadStart,
        (uint32_t)&Cla1funcsLoadSize);

    memcpy((uint32_t*)&Cla1ConstRunStart, (uint32_t*)&Cla1ConstLoadStart,
        (uint32_t)&Cla1ConstLoadSize);
#endif //_FLASH

    EALLOW;

    // Initialize and wait for CLA1ToCPUMsgRAM
    MemCfgRegs.MSGxINIT.bit.INIT_CLA1TOCPU = 1;
    while (MemCfgRegs.MSGxINITDONE.bit.INITDONE_CLA1TOCPU != 1) {};

    // Initialize and wait for CPUToCLA1MsgRAM
    MemCfgRegs.MSGxINIT.bit.INIT_CPUTOCLA1 = 1;
    while (MemCfgRegs.MSGxINITDONE.bit.INITDONE_CPUTOCLA1 != 1) {};

    // program space
    MemCfgRegs.LSxMSEL.bit.MSEL_LS0 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS0 = 1;

    // data space
    MemCfgRegs.LSxMSEL.bit.MSEL_LS1 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS1 = 0;

    EDIS;

    // Compute all CLA task vectors
    // On Type-1 CLAs the MVECT registers accept full 16-bit task addresses as
    // opposed to offsets used on older Type-0 CLAs
    EALLOW;
    // Cla1Regs.MVECT1 = (uint16_t)(&Cla1Task1_ad2s1210Read);

    // Enable the IACK instruction to start a task on CLA in software
    // for all  8 CLA tasks. Also, globally enable all 8 tasks (or a
    // subset of tasks) by writing to their respective bits in the MIER register

    // Cla1Regs.MCTL.bit.IACKE = 1;
    // Cla1Regs.MIER.all = 0x1;

    // Configure the vectors for the end-of-task interrupt for all 8 tasks
    // PieVectTable.CLA1_1_INT = &cla1Isr1;

    // 不使用中断机制
    PieCtrlRegs.PIEIER11.all = 0x0;
    // IER |= (M_INT11);

    EDIS;

    return;
}

void bsp_init_all()
{
    DINT;
    bsp_init_chip_devs();
    bsp_init_board_devs();
    bsp_interrupt_cfg();

    bsp_init_cla();

    // 启动cpu2
    bootCpu2();
}

void bsp_init_cpu2dev()
{
    // 移交CANA、CANB控制权
    EALLOW;
    DevCfgRegs.CPUSEL8.bit.CAN_A = 1;
    DevCfgRegs.CPUSEL8.bit.CAN_B = 1;
    EDIS;

    // 移交内存控制权
    EALLOW;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS0 = 1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS1 = 1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS2 = 1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS3 = 1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS4 = 1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS5 = 1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS6 = 1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS7 = 1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS8 = 1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS9 = 1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS10 = 1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS11 = 1;
    // 12属于cpu1
    // MemCfgRegs.GSxMSEL.bit.MSEL_GS12 = 1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS13 = 1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS14 = 1;
    MemCfgRegs.GSxMSEL.bit.MSEL_GS15 = 1;
    EDIS;
}
