/**
  ******************************************************************************
  * @file    umart_lite_plus_teste/include/main.h
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.1
  * @date    20-Abril-2015
  * @brief   Inclusão das bibliotecas de usuário e definição de parâmetros
  ******************************************************************************
  */

/* Define para previnir a inclusão recursiva ---------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H


/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
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


/* Definições ----------------------------------------------------------------*/
#define STDIO_UART	// STDIO_UART ou STDIO_USB: direciona as funções de escrita
				// e leitura (printf, scanf, putc...) para a UART ou para a USB

/* Constantes ----------------------------------------------------------------*/
#define N_PARAMETROS 11

/* Macros --------------------------------------------------------------------*/
/* Protótipos das Funções --------------------------------------------------- */
void systick(void);
void perifericosConfig(void);
void init_parametros(void);


#endif /* __MAIN_H */
