/**
  ******************************************************************************
  * @file    wc2015/include/speed_control.h
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.0
  * @date    27-Abril-2015
  * @brief   Cabeçalho para o módulo speed_control.c
  ******************************************************************************
  */

/* Define para previnir a inclusão recursiva ---------------------------------*/
#ifndef __SPEED_CONTROL_H
#define __SPEED_CONTROL_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <stdbool.h>
#include "encoders.h"
#include "motores.h"
#include "sensores.h"

/* Constantes ----------------------------------------------------------------*/
#define KP_X 30//0.05
#define KD_X 0
#define KP_W 0.5
#define KD_W 2

#define TS 1	// Tempo de atualização em [ms]

// Timer para base de tempo de atualização
#define TIM_CLK			__TIM3_CLK_ENABLE()
#define TIM				TIM3
#define TIM_PERIOD		999
#define TIM_PRESCALER	83
#define TIMx_IRQn		TIM3_IRQn
#define TIMx_IRQHandler	TIM3_IRQHandler


#define CNT_PER_1000MM 142400	// = (1000*PPR) / (W_DIAMETER*PI)
#define CNT_PER_360DEG 37120	// = ((2*PI*W_DISTANCE)*CNT_PER_1000MM)/(2*1000)


#define GYRO_SCALE 1
#define SENSOR_SCALE 1


/* Macros --------------------------------------------------------------------*/
#define MM_TO_COUNTS(mm)	(((mm) * CNT_PER_1000MM) / 1000)
#define SPEEDX_TO_COUNTS(speed)	((CNT_PER_1000MM * (speed * 2) * TS) / 1000000)
#define ACCX_TO_COUNTS(acc)		(SPEEDX_TO_COUNTS(acc / 2) / TS)
#define COUNTS_TO_MM(cnt)	(((cnt) * 1000) / CNT_PER_1000MM)

#define DEG_TO_COUNTS(deg)	(((deg) * CNT_PER_360DEG) / 360)
#define SPEEDW_TO_COUNTS(speed)	((CNT_PER_360DEG * (speed * 2) * TS) / 360000)
#define ACCW_TO_COUNTS(acc)		(SPEEDW_TO_COUNTS(acc / 2) / TS)
#define COUNTS_TO_DEG(cnt)	(((cnt) * 360) / CNT_PER_360DEG)


/* Protótipos das Funções --------------------------------------------------- */
void speedControlConfig(void);

void speedProfile(void);
void getEncoderStatus(void);
void updateCurrentSpeed(void);
void calculateMotorPwm(void);
int32_t needToDecelerate(int32_t, int32_t, int32_t);
void resetProfile(void);


/* Variáveis externas --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern int32_t distanceLeft, distance_mm;
extern int32_t targetSpeedX, targetSpeedW;
extern int32_t endSpeedX, endSpeedW;
extern int32_t accX, decX, accW, decW;
extern bool onlyUseEncoderFeedback;


#endif /* __SPEED_CONTROL_H */
