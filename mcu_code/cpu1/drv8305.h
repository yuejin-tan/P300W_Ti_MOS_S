/*
 * drv8305.h
 *
 *  Created on: 2024年11月17日
 *      Author: t
 */

#ifndef MCU_CODE_CPU1_DRV8305_H_
#define MCU_CODE_CPU1_DRV8305_H_

#include "F28x_Project.h"
#include "stdint.h"

static inline void drv8305_init()
{
    // pin66 cs pin
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PULLUP);
    GPIO_WritePin(66, 1);

    // pin63 SPISIMOA pin
    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15);
    GPIO_SetupPinOptions(63, GPIO_OUTPUT, GPIO_PULLUP | GPIO_ASYNC);
    // pin64 SPISOMIA pin
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15);
    GPIO_SetupPinOptions(64, GPIO_INPUT, GPIO_PULLUP | GPIO_ASYNC);
    // pin65 SPICLKA pin
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15);
    GPIO_SetupPinOptions(65, GPIO_OUTPUT, GPIO_PULLUP | GPIO_ASYNC);

    // spi外设配置
    // 打开 SPI-B 时钟
    EALLOW;
    CpuSysRegs.PCLKCR8.bit.SPI_B = 1;

    // 确保复位
    SpibRegs.SPICCR.bit.SPISWRESET = 0;
    // rising edge, no delay
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0;
    SpibRegs.SPICTL.bit.CLK_PHASE = 0;

    // 关闭环回测试模式
    SpibRegs.SPICCR.bit.SPILBK = 0;
    // 使用最大16bit数据位数
    SpibRegs.SPICCR.bit.SPICHAR = 16 - 1;

    // 关闭overrun中断
    SpibRegs.SPICTL.bit.OVERRUNINTENA = 0;
    // 主机模式
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1;
    // 开启传输
    SpibRegs.SPICTL.bit.TALK = 1;
    // 关闭中断
    SpibRegs.SPICTL.bit.SPIINTENA = 0;

    // 波特率2MHz
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 99;

    // 使spi外设退出复位状态
    SpibRegs.SPICCR.bit.SPISWRESET = 1;
    EDIS;

    // 似乎可以摆烂，用上电默认值
}


#endif /* MCU_CODE_CPU1_DRV8305_H_ */
