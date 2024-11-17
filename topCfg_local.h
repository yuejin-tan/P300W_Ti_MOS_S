/*
 * topCfg_local.h
 *
 *  Created on: 2023年10月13日
 *      Author: t
 */

#ifndef TOPCFG_LOCAL_H_
#define TOPCFG_LOCAL_H_

// 数组大小 512k*16bit 合 256k float
#define RAMBUFFSIZE (256ul * 1024ul)

// EMIF CS3 外置SRAM
#define CDB_BUFF_ADDR ((void*)0x300000ul)

// CDB 端口
#define CAN_CDB_BASE CANB_BASE

#endif /* TOPCFG_LOCAL_H_ */
