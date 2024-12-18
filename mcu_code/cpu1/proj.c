/*
 * proj.c
 *
 *  Created on: 2022年6月25日
 *      Author: tyj
 */
#include "F28x_Project.h"
#include "math.h"

#include "bsp_cfg.h"
#include "transform.h"
#include "thetaCal.h"
#include "SVPWM.h"
#include "pid.h"
#include "waveGen.h"
#include "samp.h"
#include "filter.h"
#include "speed_cal.h"
#include "comp.h"
#include "interp_tab.h"
#include "deadBandComp.h"
#include "oeca.h"
#include "adrc.h"

#include "proj.h"
#include "bsp_inline.h"
#include "protect.h"
#include "encoder.h"
#include "proj2.h"

#include "drv8305.h"

struct SVPWM_struct CH1_svpwm;

struct ThataCal_struct CH1_thetaI;

struct ThataCal_struct CH1_thetaU;

struct ThataCal_struct thetaMTPA;

struct Trans_struct CH1_Utar;

struct PIctrl_struct UdcPI;
struct PIctrl_struct PdcPI;

float Ld4Icomp = MATLAB_PARA_Ld;
float Lq4Icomp = MATLAB_PARA_Lq;
float Faif4Icomp = MATLAB_PARA_faif;

struct ADRC_struct UdcAdrc;
uint16_t ADRC_mode = 0;

struct PIctrl_struct omegaPI;

struct PIctrl_struct CH1_IdPI;

struct PIctrl_struct CH1_IqPI;

struct SpeedCal_struct omegaEcal;

struct protect_struct curProtect_CH1;
struct protect_struct UdcProtect_CH1;

struct protect_struct speedProtect;
struct protect_struct tempProtect;

struct LPF_Ord1_2_struct CH1_IdFilt;
struct LPF_Ord1_2_struct CH1_IqFilt;
struct LPF_Ord1_2_struct CH1_I0Filt;

struct LPF_Ord1_2_struct CH1_IdcFilt;
struct LPF_Ord1_2_struct CH1_UdcFilt;

struct LPF_Ord1_2_struct omegaFilt;

struct LPF_Ord1_2_struct PdcFilt;

struct Goertz_struct Goertz1;
struct Goertz_struct Goertz2;
struct OECA_struct OECA1;
struct LPF_Ord1_2_struct OECA_f1;

struct encoder_struct encoder1;
struct DRV8305_struct drv8305_1;

enum ANGLE_MODE
{
    AM_manual = 0,
    AM_self_inc,
    AM_sync,
    AM_sync_comp,
};

enum CURRENT_MODE
{
    CM_error = -1,
    CM_stop = 0,
    CM_svpwm,
    CM_I_loop,
    CM_LSSC,
};

enum SPEED_MODE
{
    SM_err = -1,
    SM_stop = 0,
    SM_Is_ramp,
    SM_Te_ramp,
    SM_OECA_exp,
    SM_speed_loop_N,
    SM_speed_loop_ramp_N,
    SM_Pdc_loop,
    SM_Pdc_loop_ramp,
    SM_Udc_loop,
    SM_Udc_loop_ramp,
    SM_Udc_loop2,
    SM_Udc_loop_ramp2,
};

enum TORQUE_MODE
{
    TM_id0 = 0,
    TM_MTPA,
};

enum OUT_MODE
{
    OM_stop = 0,
    OM_CH1, // 1
    OM_free, // 2
};

enum WAVE_MODE
{
    WM_trig = 0,
    WM_sine,
};

// 采样值
int16_t CH1_Udc_raw = 0;
int16_t CH1_Idc_raw = (MATLAB_PARA_I_bus_sense_offset * MATLAB_PARA_adc_gain);
int16_t CH1_Iu_raw = (MATLAB_PARA_Isense_offset * MATLAB_PARA_adc_gain);
int16_t CH1_Iv_raw = (MATLAB_PARA_Isense_offset * MATLAB_PARA_adc_gain);
int16_t CH1_Iw_raw = (MATLAB_PARA_Isense_offset * MATLAB_PARA_adc_gain);

uint16_t thetaEnco_raw = 0;
int16_t thetaDiff_raw = 0;

uint16_t thetaEnco_raw_offset = 16500;

float spdEnco = 0;

// SI 采样值、原始值
float CH1_Udc_SI = 0.0f;
float CH1_Idc_SI = 0.0f;

struct Trans_struct CH1_Ifbk;
struct Trans_struct CH1_Ifilt;

// 处理(滤波)后值
float CH1_Udc = 0.0f;
float CH1_Idc = 0.0f;

float Pdc = 0.0f;

float CH1_Udc_LThd = 6.0f;

// 转速环
float omegaMfbk = 0.0f;

// 波形产生
int16_t targetWaveMode = 0;
float targetWaveFreq = 1.0f;
float targetWaveAmp = 1.0f;
float targetWaveOffset = 0;

// 斜坡给定
float targetRampVal = 0.0f;
float targetRampGrad = 1.0f;

// 控制参数

float CH1_Idc_raw_offset = (MATLAB_PARA_I_bus_sense_offset * MATLAB_PARA_adc_gain);
float CH1_Iu_raw_offset = (MATLAB_PARA_Isense_offset * MATLAB_PARA_adc_gain);
float CH1_Iv_raw_offset = (MATLAB_PARA_Isense_offset * MATLAB_PARA_adc_gain);
float CH1_Iw_raw_offset = (MATLAB_PARA_Isense_offset * MATLAB_PARA_adc_gain);

int16_t CH1_angle_mode = 0;

int16_t CH1_cur_mode = 0;
int16_t CH1_cur_mode2 = 0;

uint16_t filtHWFaultCnt = 0;

// bit 0 1: udc var
// bit 1 2: Udq comp with fbk val
// bit 3 4: deadBand comp
int16_t CH1_ext_fcn = 0;
float db_Ithd_1 = 1.0f / MATLAB_PARA_db_Ithd;
float db_cmp_tick = MATLAB_PARA_tick_db_comp;
float db_cmp_vds = MATLAB_PARA_Vdf + MATLAB_PARA_Vsat;

int16_t speed_mode = 0;
int16_t torque_mode = 0;
int16_t channel_mode = 0;

float targetTe = 0.0f;
float targetIs = 0.0f;

float targetThetaMTPA = 0.3333333f;

float targetN = 0.0f;
float targetOmegaM = 0.0f;
float targetUdc = 0.0f;
float targetPdc = 0.0f;

float targetId_CH1 = 0.0f;
float targetIq_CH1 = 0.0f;
float targetThetaE_CH1 = 0.0f;

float CH1_Ud_comp = 0.0f;
float CH1_Uq_comp = 0.0f;

float thetaEInc = 1e-5;

// 保护配置
int16_t IProtectFlg_CH1 = 0;
int16_t SPDProtectFlg = 0;
int16_t UdcProtectFlg_CH1 = 0;
int16_t tempProtectFlg = 0;

#define M_1Ord_LPF_Kahan(coeff, oldVal, newVal) \
do{\
 static volatile float errSum = 0 ;\
 volatile float deltaNew = ( newVal - oldVal ) * coeff - errSum;\
 volatile float ans = oldVal + deltaNew;\
 volatile float mustCalcFirst = ans - oldVal;\
 errSum = mustCalcFirst - deltaNew;\
 oldVal = ans;\
}while(0)

void adcOffset_init(int32_t avgNum)
{

    int32_t sumsTab[4] = { 0,0,0,0 };
    float avgNum_1 = 1.0f / avgNum;

    DELAY_US(100);

    const int delay_tick_tmp = (float)(1e6 / 25.0) / vCTRL_FREQ;

#pragma UNROLL ( 1 )
    for (int32_t i = 0;i < avgNum;i++)
    {
        F28x_usDelay(delay_tick_tmp);
        sumsTab[1] += CH1_Iu_raw;
        sumsTab[2] += CH1_Iv_raw;
        sumsTab[3] += CH1_Iw_raw;
    }

    CH1_Iu_raw_offset = sumsTab[1] * avgNum_1;
    CH1_Iv_raw_offset = sumsTab[2] * avgNum_1;
    CH1_Iw_raw_offset = sumsTab[3] * avgNum_1;
}

static void cfg_clk_util(uint16_t pwm_freq)
{
    algo_clk_cfg(pwm_freq);

    LPF_Ord1_2_cfg(&CH1_IdFilt, LPF_ORD_2_t, vCTRL_TS, vCTRL_FREQ * MATLAB_PARA_Idq_filter_factor, MATLAB_PARA_Idq_filter_yita);
    LPF_Ord1_2_cfg(&CH1_IqFilt, LPF_ORD_2_t, vCTRL_TS, vCTRL_FREQ * MATLAB_PARA_Idq_filter_factor, MATLAB_PARA_Idq_filter_yita);
    LPF_Ord1_2_cfg(&CH1_I0Filt, LPF_ORD_2_t, vCTRL_TS, vCTRL_FREQ * MATLAB_PARA_Idq_filter_factor, MATLAB_PARA_Idq_filter_yita);
    LPF_Ord1_2_cfg(&CH1_UdcFilt, LPF_ORD_2_t, vCTRL_TS, MATLAB_PARA_outer_filter_bw, MATLAB_PARA_outer_filter_yita);
    LPF_Ord1_2_cfg(&CH1_IdcFilt, LPF_ORD_2_t, vCTRL_TS, MATLAB_PARA_outer_filter_bw, MATLAB_PARA_outer_filter_yita);

    LPF_Ord1_2_cfg(&PdcFilt, LPF_ORD_2_t, vCTRL_TS, MATLAB_PARA_outer_filter_bw, MATLAB_PARA_outer_filter_yita);
    LPF_Ord1_2_cfg(&omegaFilt, LPF_KAHAN_2_t, vCTRL_TS, MATLAB_PARA_omega_filter_bw, NULL);

    PIctrl_Iloop_cfg(&CH1_IdPI, MATLAB_PARA_Iloop_bw_factor, MATLAB_PARA_Ld, MATLAB_PARA_Rall, MATLAB_PARA_Upi_max, -MATLAB_PARA_Upi_max);
    PIctrl_Iloop_cfg(&CH1_IqPI, MATLAB_PARA_Iloop_bw_factor, MATLAB_PARA_Lq, MATLAB_PARA_Rall, MATLAB_PARA_Upi_max, -MATLAB_PARA_Upi_max);
}

void ctrl_init()
{
    cfg_clk_util(PWM_FREQ);

    SVPWM_init(&CH1_svpwm, UDC_INIT_VAL);

    thetaCal_init(&CH1_thetaI);
    thetaCal_init(&CH1_thetaU);

    thetaCal_init(&thetaMTPA);

    PIctrl_init(&omegaPI, MATLAB_PARA_kp_wloop, MATLAB_PARA_ki_wloop, MATLAB_PARA_ki_wloop, MATLAB_PARA_Wpi_max, MATLAB_PARA_Wpi_min);
    PIctrl_init(&UdcPI, MATLAB_PARA_kp_vloop, MATLAB_PARA_ki_vloop, MATLAB_PARA_ki_vloop, MATLAB_PARA_Pdcpi_max, MATLAB_PARA_Pdcpi_min);
    PIctrl_init(&PdcPI, MATLAB_PARA_kp_ploop, MATLAB_PARA_ki_ploop, MATLAB_PARA_ki_ploop, MATLAB_PARA_Pdcpi_max, MATLAB_PARA_Pdcpi_min);

    adrc_init(&UdcAdrc, MATLAB_PARA_ADRC_gamma, MATLAB_PARA_ADRC_c_u1, MATLAB_PARA_ADRC_c_u2, MATLAB_PARA_ADRC_beta1, MATLAB_PARA_ADRC_beta2,
        MATLAB_PARA_Wdcpi_max, MATLAB_PARA_Wdcpi_min, MATLAB_PARA_ADRC_SMCk);

    speedCal_init(&omegaEcal, &omegaFilt, 0);

    protect_init(&curProtect_CH1, CUR_PRCT_THD_L, CUR_PRCT_THD_H, CUR_PRCT_INTG_BASE, CUR_PRCT_INTG_MAX, CUR_PRCT_FOLW_DMAX, CUR_PRCT_FOLW_INTG_MAX);
    protect_init(&UdcProtect_CH1, UDC_PRCT_THD_L, UDC_PRCT_THD_H, UDC_PRCT_INTG_BASE, UDC_PRCT_INTG_MAX, UDC_PRCT_FOLW_DMAX, UDC_PRCT_FOLW_INTG_MAX);

    protect_init(&speedProtect, SPD_PRCT_THD_L, SPD_PRCT_THD_H, SPD_PRCT_INTG_BASE, SPD_PRCT_INTG_MAX, SPD_PRCT_FOLW_DMAX, SPD_PRCT_FOLW_INTG_MAX);
    protect_init(&tempProtect, TEMP_PRCT_THD_L, TEMP_PRCT_THD_H, TEMP_PRCT_INTG_BASE, TEMP_PRCT_INTG_MAX, TEMP_PRCT_FOLW_DMAX, TEMP_PRCT_FOLW_INTG_MAX);

    // proposed method
    OECA_init(&OECA1, &OECA_f1);
    Goertz_init(&Goertz1, 1, OECA1.sweepMax);
    Goertz_init(&Goertz2, 2, OECA1.sweepMax);

    encoder_init(&encoder1, ENCO_PPR);
}

static inline void angleModeTask()
{
    switch (CH1_angle_mode)
    {
    case AM_sync_comp:
        thetaCal_setTheta_Uint(&CH1_thetaI, getThetaEUint(thetaEnco_raw - thetaEnco_raw_offset));
        thetaCal_setTheta(&CH1_thetaU, thetaComp(thetaCal_getTheta(&CH1_thetaI), omegaEcal.omegaE));
        targetThetaE_CH1 = thetaCal_getTheta(&CH1_thetaI);
        break;

    case AM_sync:
        thetaCal_setTheta_Uint(&CH1_thetaI, getThetaEUint(thetaEnco_raw - thetaEnco_raw_offset));
        thetaCal_setTheta_noWrap(&CH1_thetaU, thetaCal_getTheta(&CH1_thetaI));
        targetThetaE_CH1 = thetaCal_getTheta(&CH1_thetaI);
        break;

    case AM_self_inc:
        thetaCal_setTheta(&CH1_thetaU, thetaCal_getTheta(&CH1_thetaU) + thetaEInc);
        thetaCal_setTheta_noWrap(&CH1_thetaI, thetaCal_getTheta(&CH1_thetaU));
        targetThetaE_CH1 = thetaCal_getTheta(&CH1_thetaI);
        break;

    case AM_manual:
    default:
        thetaCal_setTheta(&CH1_thetaU, targetThetaE_CH1);
        thetaCal_setTheta(&CH1_thetaI, targetThetaE_CH1);
        break;
    }
}

static inline void sigSampTask()
{
    // 摆烂没有使用DMA或CLA来读取 eqep, 故需尽早访问寄存器, 以保持延迟一致性
    thetaEnco_raw = encoder_u16Read(&encoder1, EQEP_TYJDEV_REGS.QPOSCNT);
    spdEnco = encoder_lowSpdCalc(&encoder1, EQEP_TYJDEV_REGS.QCPRDLAT);
    // thetaDiff_raw = thetaEnco_raw;

    // ADC signals
    CH1_Iu_raw = bsp_get_CH1_Iu_adcRaw();
    CH1_Iv_raw = bsp_get_CH1_Iv_adcRaw();
    CH1_Iw_raw = bsp_get_CH1_Iw_adcRaw();
    CH1_Ifbk.U = getCurSI(CH1_Iu_raw, CH1_Iu_raw_offset);
    CH1_Ifbk.V = getCurSI(CH1_Iv_raw, CH1_Iv_raw_offset);
    CH1_Ifbk.W = getCurSI(CH1_Iw_raw, CH1_Iw_raw_offset);

    // CH1_Idc_raw = bsp_get_CH1_Idc_adcRaw();
    // CH1_Idc_SI = getCurSI(CH1_Idc_raw, CH1_Idc_raw_offset);
    // CH1_Idc = LPF_Ord2_update(&CH1_IdcFilt, CH1_Idc_SI);

    CH1_Udc_raw = bsp_get_CH1_Udc_adcRaw();
    CH1_Udc_SI = getBusVoltSI(CH1_Udc_raw);
    CH1_Udc = LPF_Ord2_update(&CH1_UdcFilt, CH1_Udc_SI);

    // Pdc = LPF_Ord2_update(&PdcFilt, CH1_Udc * CH1_Idc);

    // calculate omega 可以用上次的角度值，区别不大
    omegaMfbk = speedCal_update(&omegaEcal, getThetaEUint(thetaEnco_raw)) * (float)(1.0 / MATLAB_PARA_RDC2ELE_RATIO);

    // 角度计算
    angleModeTask();

    // 电流变换
    trans3_uvw2dq0(&CH1_Ifbk, &CH1_thetaI);
    CH1_Ifilt.d = LPF_Ord2_update(&CH1_IdFilt, CH1_Ifbk.d);
    CH1_Ifilt.q = LPF_Ord2_update(&CH1_IqFilt, CH1_Ifbk.q);
    CH1_Ifilt.abdq0 = LPF_Ord2_update(&CH1_I0Filt, CH1_Ifbk.abdq0);

    // 反变换得到滤波后的三相基波电流，此处不考虑零序
    // 使用电压的角度，以补偿控制造成的延后（有待证明？）
    trans2_dq2uvw(&CH1_Ifilt, &CH1_thetaU);
}

static inline void protectIsrTask()
{
    // 任何时候进行电流绝对值保护和有效值保护
    IProtectFlg_CH1 |=
        protectCalc_THD_HL(&curProtect_CH1, CH1_Ifbk.U) |
        protectCalc_THD_HL(&curProtect_CH1, CH1_Ifbk.V) |
        protectCalc_THD_HL(&curProtect_CH1, CH1_Ifbk.W) |
        protectCalc_INTG2(&curProtect_CH1, CH1_Ifbk.q * CH1_Ifbk.q + CH1_Ifbk.d * CH1_Ifbk.d) |
        0;

    // 部分模式进行电流跟踪保护
    if (CH1_cur_mode == CM_I_loop)
    {
        IProtectFlg_CH1 |=
            protectCalc_FOLW2(&curProtect_CH1, CH1_Ifbk.q, targetIq_CH1) |
            protectCalc_FOLW2(&curProtect_CH1, CH1_Ifbk.d, targetId_CH1) |
            0;
    }
    else
    {
        protect_FOLW_Clear(&curProtect_CH1);
    }

    // 功率模块硬件故障保护
    if (bsp_GPIO_ifHWFaultSigSet())
    {
        filtHWFaultCnt++;
        if (filtHWFaultCnt > 3)
        {
            IProtectFlg_CH1 |= 0x10u;
            // 触发读取错误位
            drv8305_1.ii = 0;
        }
    }
    else
    {
        filtHWFaultCnt = 0;
    }

    // 任何时候进行转速绝对值保护
    SPDProtectFlg |=
        protectCalc_THD_HL(&speedProtect, omegaMfbk) |
        0;

    // 部分模式进行转速跟踪保护
    if (speed_mode >= SM_speed_loop_N && speed_mode <= SM_speed_loop_ramp_N)
    {
        SPDProtectFlg |=
            protectCalc_FOLW2(&speedProtect, omegaMfbk, targetOmegaM) |
            0;
    }
    else
    {
        protect_FOLW_Clear(&speedProtect);
    }

    // 任何时候进行母线电压绝对值保护
    UdcProtectFlg_CH1 |=
        protectCalc_THD_HL(&UdcProtect_CH1, CH1_Udc) |
        0;


    // 部分模式下进行电压跟踪保护
    if (speed_mode >= SM_Udc_loop)
    {
        UdcProtectFlg_CH1 |=
            protectCalc_FOLW2(&UdcProtect_CH1, CH1_Udc, targetUdc) |
            0;
    }
    else
    {
        protect_FOLW_Clear(&UdcProtect_CH1);
    }

    // 保护结果判定
    if (IProtectFlg_CH1 | UdcProtectFlg_CH1 | tempProtectFlg)
    {
        CH1_cur_mode = CM_error;
        channel_mode &= 0b11111110;
        speed_mode = SM_err;
    }


    if (SPDProtectFlg | tempProtectFlg | UdcProtectFlg_CH1)
    {
        speed_mode = SM_err;
        channel_mode = OM_stop;
    }

    // 手动停机
    // if (bsp_GPIO_ifStopSigSet())
    // {
    //     speed_mode = SM_stop;
    //     channel_mode = OM_stop;
    //     CH1_cur_mode = CM_stop;
    //     IProtectFlg_CH1 |= 0x20u;
    // }

}

static inline void curLoopTask()
{
    switch (CH1_cur_mode)
    {
    case CM_LSSC:
        bsp_epwm_ch1_LSSC2(CH1_cur_mode2);
        break;

    case CM_I_loop:
        if (CH1_ext_fcn & 0x2u)
        {
            CH1_Ud_comp = -CH1_Ifilt.q * Lq4Icomp * omegaEcal.omegaE;
            CH1_Uq_comp = (CH1_Ifilt.d * Ld4Icomp + Faif4Icomp) * omegaEcal.omegaE;
        }
        else
        {
            CH1_Ud_comp = 0;
            CH1_Uq_comp = 0;
        }

        CH1_Utar.d = PIctrl_update_clamp2(&CH1_IdPI, targetId_CH1 - CH1_Ifilt.d) + CH1_Ud_comp;
        CH1_Utar.q = PIctrl_update_clamp2(&CH1_IqPI, targetIq_CH1 - CH1_Ifilt.q) + CH1_Uq_comp;

    case CM_svpwm:
        if (CH1_ext_fcn & 0x1u)
        {
            float tempUdc1 = CH1_Udc;
            if (CH1_ext_fcn & 0x4u)
            {
                // 死区补偿 part1
                tempUdc1 = dbComp2_UdcOffset(CH1_Udc, db_cmp_vds);
            }
            SVPWM_setUdc(&CH1_svpwm, tempUdc1);
            // 考虑母线电压缓变性及抗饱和保护的不敏感性，在这里进行抗饱和参数的变化
            const float SQRT_3_DIV_3 = 0.5773503f;
            float Usv_max = CH1_Udc * SQRT_3_DIV_3;
            PIctrl_svpwmBoundSet(&CH1_IdPI, Usv_max);
            PIctrl_svpwmBoundSet(&CH1_IqPI, Usv_max);
        }
        trans2_dq2albe(&CH1_Utar, &CH1_thetaU);
        SVPWM_dutyCal(&CH1_svpwm, CH1_Utar.al, CH1_Utar.be);
        if (CH1_ext_fcn & 0x4u)
        {
            // 死区补偿 part2
#if MATLAB_PARA_ctrl_freq_mul == 2
            if (bsp_epwm_ch1_is_up_cnt())
            {
                dbComp2_down(&CH1_svpwm, &CH1_Ifilt, db_Ithd_1, db_cmp_tick);
            }
            else
            {
                dbComp2_up(&CH1_svpwm, &CH1_Ifilt, db_Ithd_1, db_cmp_tick);
            }
#else
            dbComp2_all(&CH1_svpwm, &CH1_Ifilt, db_Ithd_1, db_cmp_tick);
#endif
        }
        bsp_epwm_ch1_reg_set(&CH1_svpwm);
        bsp_epwm_ch1_on();
        break;

    default:
    case CM_stop:
        // 归零指令值
        CH1_Utar.d = 0;
        CH1_Utar.q = 0;
        targetId_CH1 = 0;
        targetIq_CH1 = 0;
        CH1_Ud_comp = 0;
        CH1_Uq_comp = 0;
        // 清空控制器
        PIctrl_setIntg(&CH1_IdPI, 0);
        PIctrl_setIntg(&CH1_IqPI, 0);
        // 归零电压
        bsp_epwm_ch1_reg_init();

    case CM_error:
        // 封锁驱动
        bsp_epwm_ch1_off();
        break;
    }
}

static inline void outerLoopTask()
{
    // 目标给定
    switch (speed_mode)
    {

    case SM_speed_loop_N:
        targetOmegaM = targetN * (float)(2.0 * M_PI / 60.0);
        break;

    case SM_speed_loop_ramp_N:
        targetN = targetRamp(targetRampVal, targetN, targetRampGrad * vCTRL_TS);
        targetOmegaM = targetN * (float)(2.0 * M_PI / 60.0);
        break;

    case SM_Pdc_loop_ramp:
        targetPdc = targetRamp(targetRampVal, targetPdc, targetRampGrad * vCTRL_TS);
        break;

    case SM_Udc_loop_ramp:
    case SM_Udc_loop_ramp2:
        targetUdc = targetRamp(targetRampVal, targetUdc, targetRampGrad * vCTRL_TS);
        break;
    }

    // 闭环计算
    switch (speed_mode)
    {
    case SM_Udc_loop2:
    case SM_Udc_loop_ramp2:
        if (ADRC_mode)
        {
            targetTe = adrc_update2(&UdcAdrc, targetUdc, CH1_Udc, omegaMfbk);
        }
        else
        {
            targetTe = adrc_update(&UdcAdrc, targetUdc, CH1_Udc, omegaMfbk);
        }
        break;

    case SM_Udc_loop:
    case SM_Udc_loop_ramp:
        targetTe = PIctrl_update_clamp_comp2(&UdcPI, getDeltaCapEnSI(CH1_Udc, targetUdc), Pdc) * omegaEcal.omegaE_inv * MATLAB_PARA_p0;
        break;

    case SM_Pdc_loop:
    case SM_Pdc_loop_ramp:
        targetTe = PIctrl_update_clamp2(&PdcPI, targetPdc - Pdc) * omegaEcal.omegaE_inv * MATLAB_PARA_p0;
        break;

    case SM_speed_loop_N:
    case SM_speed_loop_ramp_N:
        targetTe = PIctrl_update_bCalc2(&omegaPI, targetOmegaM - omegaMfbk);
        break;

    case SM_Te_ramp:
        targetTe = targetRamp(targetRampVal, targetTe, targetRampGrad * vCTRL_TS);
        break;

    case SM_Is_ramp:
        targetIs = targetRamp(targetRampVal, targetIs, targetRampGrad * vCTRL_TS);
        break;

    case SM_OECA_exp:
        // TODO OECA
        switch (OECA1.status)
        {
        default:
        case OECA_ready:
            targetIs = 0;
            targetThetaMTPA = 0;
            OECA1.sweepCnt = 0;
            OECA1.sampCnt = 0;
            OECA1.timeCnt = 0;
            OECA1.dbgCnt = 0;
            OECA1.dbgPtr = 0;
            OECA1.sampDiv = 30e3f * OECA1.sweepTime / OECA1.sweepMax + 0.5f;
            OECA1.timeCntMax = 30e3f * OECA1.sweepTime + 0.5f;
            Goertz_init(&Goertz1, 1, OECA1.sweepMax);
            Goertz_init(&Goertz2, 2, OECA1.sweepMax);
            OECA_algoStaSet(&OECA1, 0.0f);
            OECA_omegaOB(&OECA1, thetaEnco_raw * (1.0f / 65536.0f));
            break;

        case OECA_Isweep0:
        {
            targetIs = OECA1.sweepCurrent;
            targetThetaMTPA = OECA_getSweepTheta(&OECA1);
            OECA1.sampCnt++;
            OECA1.timeCnt++;
            OECA1.dbgCnt++;
            float thetaRdcRawNorm = thetaEnco_raw * (1.0f / 65536.0f);
            OECA_omegaOB(&OECA1, thetaRdcRawNorm);
            if (OECA1.dbgCnt >= OECA1.dbgDiv)
            {
                OECA1.dbgCnt = 0;
                // dbg 记录
                OECA1.hDbgBuff[OECA1.dbgPtr * 2 + 0] = thetaRdcRawNorm;
                OECA1.hDbgBuff[OECA1.dbgPtr * 2 + 1] = OECA1.omegaOB;
                OECA1.dbgPtr++;
            }
            if (OECA1.sampCnt >= OECA1.sampDiv)
            {
                OECA1.sampCnt = 0;
                Goertz_iter(&Goertz1, OECA1.omegaOB);
                Goertz_iter(&Goertz2, OECA1.omegaOB);

                OECA1.sweepCnt++;
            }
            if (OECA1.sweepCnt >= OECA1.sweepMax)
            {
                // 跳转状态，关闭输出
                OECA1.status = OECA_Isweep1;
                targetIs = 0;
                // 计算结果，只算一次
                Goertz_getAns(&Goertz1);
                Goertz_getAns(&Goertz2);
                OECA1.estAns1 = OECA_util_angle_norm(Goertz1.AnsArg - 0.5f);
            }
        }

        break;

        case OECA_Isweep1:
            targetIs = 0.0f;
            targetThetaMTPA = 0.0f;

            OECA_omegaOB(&OECA1, thetaEnco_raw * (1.0f / 65536.0f));
            // 参数设计
            if (OECA1.modeCfg & OECA_M_AUTO_PI_PARA)
            {
                {
                    float omegaSlop = (float)(2.0 * M_PI * 30e3f) / (1.0f * OECA1.sweepMax * OECA1.sampDiv);
                    float factor = __divf32(OECA1.PICurrent, OECA1.sweepCurrent);
                    float aaa = (Goertz1.AnsAbs * factor * omegaSlop + 4.0f * Goertz2.AnsAbs * factor * factor * omegaSlop) * OECA1.J_coeff;
                    OECA1.kp_eComp = __divf32(OECA1.h_eComp + 1.0f, (float)(2.0 * 2.0 * M_PI) * aaa * OECA1.h_eComp * OECA1.T_sigma_eComp);
                    OECA1.ki_eComp = __divf32(OECA1.kp_eComp, CTRL_FREQ * OECA1.h_eComp * OECA1.T_sigma_eComp);
                }

                OECA1.modeCfg ^= OECA_M_AUTO_PI_PARA;
            }

            // 初值设定
            if (OECA1.modeCfg & OECA_M_AUTO_INIT_VAL)
            {
                OECA_algoStaSet(&OECA1, OECA1.estAns1);
                OECA1.modeCfg ^= OECA_M_AUTO_INIT_VAL;
            }

            // 自动跳转
            if (OECA1.modeCfg & OECA_M_AUTOALGO)
            {
                // 确保速度较小时再跳转
                if (fabsf(OECA1.omegaOB) < OECA1.maxOmegaMThd)
                {
                    OECA1.status = OECA_algo0;
                }
            }
            if (OECA1.modeCfg & OECA_M_AUTO_GOBACK)
            {
                OECA1.status = OECA_ready;
            }
            break;

        case OECA_algo0:
            targetIs = OECA1.PICurrent;
            OECA_omegaOB(&OECA1, thetaEnco_raw * (1.0f / 65536.0f));
            targetThetaMTPA = OECA_PICalc(&OECA1);
            OECA_pllCalc(&OECA1);
            break;

        case OECA_stop:
            targetIs = 0;
            break;
        }
        break;


    default:
    case SM_stop:
        // 归零指令值
        targetOmegaM = omegaMfbk;
        targetN = targetOmegaM * (float)(60.0 / 2.0 * M_PI);
        targetTe = 0;
        targetIs = 0;
        targetUdc = CH1_Udc;
        targetPdc = Pdc;
        // 清空控制器
        PIctrl_setIntg(&omegaPI, 0);
        PIctrl_setIntg(&UdcPI, 0);
        PIctrl_setIntg(&PdcPI, 0);

        adrc_clear(&UdcAdrc);
        break;

    case SM_err:
        // 保留现场
        break;
    }

    // 输出环
    switch (channel_mode)
    {
    default:
    case OM_stop:
        targetId_CH1 = 0;
        targetIq_CH1 = 0;
        break;

    case OM_CH1:
        switch (torque_mode)
        {
        case TM_id0:
            targetIq_CH1 = targetTe * (1.0f / MATLAB_PARA_faif / MATLAB_PARA_p0 / 1.5f);
            break;

        default:
        case TM_MTPA:
        {
            float te_tab = targetTe * 1.0f + TE_IDQ_MTPA_2WAY_TAB_OFFSET;
            targetIq_CH1 = lookUp_1d_lin_puX(te_tab, TE_IQ_MTPA_2WAY_TAB, TE_IDQ_MTPA_2WAY_TAB_MAX);
            targetId_CH1 = lookUp_1d_lin_puX(te_tab, TE_ID_MTPA_2WAY_TAB, TE_IDQ_MTPA_2WAY_TAB_MAX);
            thetaCal_setTheta(&thetaMTPA, targetThetaMTPA);
            break;
        }

        }
        break;

    case OM_free:
        switch (torque_mode)
        {
        case TM_id0:
            targetIq_CH1 = targetIs;
            break;

        default:
        case TM_MTPA:
            thetaCal_setTheta(&thetaMTPA, targetThetaMTPA);
            targetIq_CH1 = targetIs * thetaCal_getSinVal(&thetaMTPA);
            targetId_CH1 = fabsf(targetIs) * thetaCal_getCosVal(&thetaMTPA);
            break;

        }
        break;

    }
}

#pragma CODE_SECTION(mainIsrProcess, MEM_MACRO);
void mainIsrProcess()
{
    sigSampTask();
    protectIsrTask();
    outerLoopTask();
    curLoopTask();

    return;
}

void mainLoopProcess()
{
    if (bsp_tim0_polling_OF())
    {
        // 1ms间隔已到
        // 清空标志位
        bsp_tim0_clearFlg_OF();

        drv8305_idleErrCheck(&drv8305_1);
    }

    return;
}
