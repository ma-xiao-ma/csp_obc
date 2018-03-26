#ifndef _BSP_H_
#define _BSP_H_


#include <stdio.h>			/* ��Ϊ�õ���printf���������Ա����������ļ� */
#include "string.h"
#include "stdint.h"
#include "stm32f4xx.h"





void bsp_Init(void);
void bsp_DelayUS(uint32_t _ulDelayTime);
void BSP_Tick_Init (void);
uint32_t  BSP_CPU_ClkFreq (void);
static void NVIC_Configuration(void);
void SoftReset(void);
#define ENABLE_INT()	__set_PRIMASK(0)	/* ʹ��ȫ���ж� */
#define DISABLE_INT()	__set_PRIMASK(1)	/* ��ֹȫ���ж� */

#define SD_FILESYSTEM_ENABLE   0

void DEBUG_LOG(const char *format, ...);
#endif
