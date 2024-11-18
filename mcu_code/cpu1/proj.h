/*
 * proj.h
 *
 *  Created on: 2022年6月25日
 *      Author: tyj
 */

#ifndef INCX_PROJ_H_
#define INCX_PROJ_H_

void adcOffset_init(int32_t avgNum);

void ctrl_init();

void mainIsrProcess();

void mainLoopProcess();

extern struct SVPWM_struct CH1_svpwm;

extern struct ThataCal_struct CH1_thetaI;

extern struct ThataCal_struct CH1_thetaU;

extern struct Trans_struct CH1_Utar;

extern struct PIctrl_struct UdcPI;
extern struct PIctrl_struct PdcPI;

extern struct PIctrl_struct omegaPI;

extern struct PIctrl_struct CH1_IdPI;

extern struct PIctrl_struct CH1_IqPI;

extern struct ADRC_struct UdcAdrc;
extern uint16_t ADRC_mode;

extern struct SpeedCal_struct omegaEcal;

extern struct protect_struct curProtect_CH1;
extern struct protect_struct UdcProtect_CH1;

extern struct protect_struct speedProtect;
extern struct protect_struct tempProtect;

extern struct LPF_Ord1_2_struct CH1_IdFilt;

extern struct LPF_Ord1_2_struct CH1_IqFilt;

extern struct LPF_Ord1_2_struct CH1_I0Filt;

extern struct LPF_Ord1_2_struct CH1_IdcFilt;

extern struct LPF_Ord1_2_struct CH1_UdcFilt;

extern struct LPF_Ord1_2_struct omegaFilt;

extern struct LPF_Ord1_2_struct PdcFilt;

extern struct Goertz_struct Goertz1;
extern struct Goertz_struct Goertz2;
extern struct OECA_struct OECA1;

// 采样值
extern int16_t CH1_Udc_raw;
extern int16_t CH1_Idc_raw;
extern int16_t CH1_Iu_raw;
extern int16_t CH1_Iv_raw;
extern int16_t CH1_Iw_raw;

extern uint16_t thetaEnco_raw;
extern int16_t thetaDiff_raw;

extern uint16_t thetaEnco_raw_offset;

extern float spdEnco;

// SI 采样值、原始值
extern float CH1_Udc_SI;
extern float CH1_Idc_SI;

extern struct Trans_struct CH1_Ifbk;
extern struct Trans_struct CH1_Ifilt;

// 处理(滤波)后值
extern float CH1_Udc;
extern float CH1_Idc;

extern float Pdc;

extern float CH1_Udc_LThd;

// 转速环
extern float omegaMfbk;

// 波形产生
extern int16_t targetWaveMode;
extern float targetWaveFreq;
extern float targetWaveAmp;
extern float targetWaveOffset;

// 斜坡给定
extern float targetRampVal;
extern float targetRampGrad;

// 控制参数

extern float CH1_Iu_raw_offset;
extern float CH1_Iv_raw_offset;
extern float CH1_Iw_raw_offset;
extern float CH1_Idc_raw_offset;


extern int16_t CH1_angle_mode;

extern int16_t CH1_cur_mode;
extern int16_t CH1_cur_mode2;

extern uint16_t filtHWFaultCnt;

extern int16_t CH1_ext_fcn;

extern float db_Ithd_1;
extern float db_cmp_tick;
extern float db_cmp_vds;

extern int16_t speed_mode;
extern int16_t torque_mode;
extern int16_t channel_mode;

extern float targetTe;
extern float targetIs;
extern float targetThetaMTPA;

extern float targetN;
extern float targetOmegaM;
extern float targetUdc;

extern float targetId_CH1;
extern float targetIq_CH1;
extern float targetThetaE_CH1;

extern float thetaEInc;


// 保护配置
extern int16_t IProtectFlg_CH1;
extern int16_t SPDProtectFlg;
extern int16_t UdcProtectFlg_CH1;
extern int16_t tempProtectFlg;

// 观测主中断执行时间
extern uint16_t isr_start_pwm_cnt;
extern uint16_t isr_end_pwm_cnt;

// 旋变fault信号读取
extern uint16_t ad2s1210ErrGPIO;

#endif /* INCX_PROJ_H_ */
