/*
 * bsp_cfg.h
 *
 *  Created on: 2023年10月12日
 *      Author: t
 */

#ifndef MCU_CODE_CPU1_BSP_CFG_H_
#define MCU_CODE_CPU1_BSP_CFG_H_

#include "stdint.h"

#include "algo_code_config.h"
#include "topCfg.h"

 // 计算死区填充值
#define PWM_DEADBAND_TICKS (MATLAB_PARA_tick_dead)

// 母线电压
#define UDC_INIT_VAL (80)

// 保护参数设置
// 电流保护
#define CUR_PRCT_THD_H (120 * 1.2)
#define CUR_PRCT_THD_L (-CUR_PRCT_THD_H)
#define CUR_PRCT_INTG_BASE_UTIL (110)
#define CUR_PRCT_INTG_BASE (CUR_PRCT_INTG_BASE_UTIL * CUR_PRCT_INTG_BASE_UTIL)
#define CUR_PRCT_INTG_MAX (0.2 * CUR_PRCT_INTG_BASE * 0.01)
#define CUR_PRCT_FOLW_DMAX (20.0)
#define CUR_PRCT_FOLW_INTG_MAX (CUR_PRCT_FOLW_DMAX * 0.01)
// 转速保护
#define SPD_PRCT_THD_L (-2000.0 / 60.0 * 2.0 * M_PI)
#define SPD_PRCT_THD_H (10000.0 / 60.0 * 2.0 * M_PI)
#define SPD_PRCT_INTG_BASE_UTIL (0)
#define SPD_PRCT_INTG_BASE (SPD_PRCT_INTG_BASE_UTIL * SPD_PRCT_INTG_BASE_UTIL)
#define SPD_PRCT_INTG_MAX (0)
#define SPD_PRCT_FOLW_DMAX (50.0)
#define SPD_PRCT_FOLW_INTG_MAX (SPD_PRCT_FOLW_DMAX * 0.1)
// 电压保护
#define UDC_PRCT_THD_L (-10.0)
#define UDC_PRCT_THD_H (330.0)
#define UDC_PRCT_INTG_BASE_UTIL (0)
#define UDC_PRCT_INTG_BASE (UDC_PRCT_INTG_BASE_UTIL * UDC_PRCT_INTG_BASE_UTIL)
#define UDC_PRCT_INTG_MAX (0)
#define UDC_PRCT_FOLW_DMAX (20)
#define UDC_PRCT_FOLW_INTG_MAX (SPD_PRCT_FOLW_DMAX *  0.1)
// 温度保护
#define TEMP_PRCT_THD_L (-50)
#define TEMP_PRCT_THD_H (100)
#define TEMP_PRCT_INTG_BASE_UTIL (0)
#define TEMP_PRCT_INTG_BASE (TEMP_PRCT_INTG_BASE_UTIL * TEMP_PRCT_INTG_BASE_UTIL)
#define TEMP_PRCT_INTG_MAX (0)
#define TEMP_PRCT_FOLW_DMAX (0)
#define TEMP_PRCT_FOLW_INTG_MAX (0)

// 采样时间设置
// 150 sysclk = 750ns
#define ADC_SAMP_TICKS (150)

#endif /* MCU_CODE_CPU1_BSP_CFG_H_ */
