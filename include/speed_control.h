/**
  ******************************************************************************
  * @file    wc2015/include/speed_control.h
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.0
  * @date    27-Abril-2015
  * @brief   Cabe�alho para o m�dulo speed_control.c
  ******************************************************************************
  */

/* Define para previnir a inclus�o recursiva ---------------------------------*/
#ifndef __SPEED_CONTROL_H
#define __SPEED_CONTROL_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "main.h"

/* Constantes ----------------------------------------------------------------*/
#define KP_X 0.05
#define KD_X 0.3
#define KP_W 0.2
#define KD_W 0

#define TS 1	// Tempo de atualiza��o em [ms]

// Timer para base de tempo de atualiza��o
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


#define MINIMAL_SX_STRAIGHT 5

#define SIZE_BUFFER_SECTORS		20

#define SEARCH_RUN		0
#define FAST_RUN1		1
#define FAST_RUN2		2

#define GOAL_OK			4
#define RUN_OK			5
#define WAIT			6


/* Macros --------------------------------------------------------------------*/
#define MM_TO_COUNTS(mm)	(((mm) * CNT_PER_1000MM) / 1000)
#define SPEEDX_TO_COUNTS(speed)	((CNT_PER_1000MM * (speed * 2) * TS) / 1000000)
#define ACCX_TO_COUNTS(acc)		(SPEEDX_TO_COUNTS(acc / 2) / TS)
#define COUNTS_TO_MM(cnt)	(((cnt) * 1000) / CNT_PER_1000MM)

#define DEG_TO_COUNTS(deg)	(((deg) * CNT_PER_360DEG) / 360)
#define SPEEDW_TO_COUNTS(speed)	((CNT_PER_360DEG * (speed * 2) * TS) / 360000)
#define ACCW_TO_COUNTS(acc)		(SPEEDW_TO_COUNTS(acc / 2) / TS)
#define COUNTS_TO_DEG(cnt)	(((cnt) * 360) / CNT_PER_360DEG)

#define ACCC_TO_COUNTS(acc) (float)((float)acc * 7296.0f) / 2133.3f)


/* Prot�tipos das Fun��es --------------------------------------------------- */
void speedControlConfig(void);

void speedProfile(void);
void getEncoderStatus(void);
void updateCurrentSpeed(void);
void calculateMotorPwm(void);
int32_t needToDecelerate(int32_t, int32_t, int32_t);
void resetProfile(void);

void recordsSectors(void);
void changeRuns(void);
void changeSpeedProfile(void);


/* Vari�veis externas --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern int32_t distanceLeft, distance_mm;
extern int32_t targetSpeedX, targetSpeedW;
extern int32_t endSpeedX, endSpeedW;
extern int32_t accX, decX, accW, decW;
extern bool useEncoderFeedback;
extern bool useGyroFeedback;
extern bool useSensorFeedback;
extern uint8_t num_run;
extern int32_t buf_temp[3 * SIZE_BUFFER_SECTORS];
extern bool fflash;


#endif /* __SPEED_CONTROL_H */
