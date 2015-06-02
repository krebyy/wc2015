/**
  ******************************************************************************
  * @file    umart_lite_plus_teste/include/main.h
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.1
  * @date    20-Abril-2015
  * @brief   Inclus�o das bibliotecas de usu�rio e defini��o de par�metros
  ******************************************************************************
  */

/* Define para previnir a inclus�o recursiva ---------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H


/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"
#include "delays.h"
#include "buzzer.h"
#include "leds.h"
#include "botao.h"
#include "motores.h"
#include "sensores.h"
#include "encoders.h"
#include "usart1.h"
#include "usb_user.h"
#include "flash.h"
#include "speed_control.h"
#include "rodinhas.h"


/* Defini��es ----------------------------------------------------------------*/
#define STDIO_UART	// STDIO_UART ou STDIO_USB: direciona as fun��es de escrita
				// e leitura (printf, scanf, putc...) para a UART ou para a USB

//#define DEBUG_PRINTS

/* Constantes ----------------------------------------------------------------*/
#define N_PARAMETROS 11

/* Macros --------------------------------------------------------------------*/
/* Prot�tipos das Fun��es --------------------------------------------------- */
void systick(void);
void perifericosConfig(void);
void init_parametros(void);
void initializeRun(void);
void recordSectors(void);

extern int32_t param_speedX_med, param_topSpeed1, param_topSpeed2;
extern int32_t param_pid_kp, param_pid_kd, param_scale_sensor, param_scale_gyro;
extern int32_t param_accX1, param_accC1, param_accX2, param_accC2;


#endif /* __MAIN_H */
