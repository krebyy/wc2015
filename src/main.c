/**
  ******************************************************************************
  * @file    wc2015/src/main.c
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.1
  * @date    17-Maio-2015
  * @brief   Programa do Rob� uMaRT Lite+ - Winter Challenge 2015
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Defini��es do Programa ----------------------------------------------------*/
#define DEBUG_PRINTS


/* Vari�veis Globais ---------------------------------------------------------*/
int32_t erro = 0, erro_a = 0;
bool run = false;
uint32_t ticks = 0;

uint32_t param_speedX_med, param_speedX_min, param_speedX_max;
uint32_t param_accX, param_accC, param_a, param_b;
int32_t param_pid_kp, param_pid_ki, param_pid_kd, param_pid_offset;

int32_t trecho = 0;


// #############################################################################

// ################################ MAIN () ####################################

/**
  * @brief Programa Principal
  */
int main (void)
{
	int32_t d0 = 0;

	// Configura��o e inicializa��o dos perif�ricos ----------------------------
	perifericosConfig();


	// Inicio do programa ------------------------------------------------------
	// As mensagens enviadas por printf podem ser destinadas � USB ou � UART,
	// para isto altere a defini��o no arquivo main.h
	printf("Winter Challenge 2015 - uMaRT LITE+ V1.1\r\n");
	delay_ms(100);


	// Inicializa��o dos par�metros guardados na Flash
	init_parametros();


	// Caso inicie o rob� com SW1 pressionado: altera��o dos par�metros pelas rodinhas
	if (getSW1() == HIGH)
	{
		menu_rodinhas();
	}


	// Loop enquanto o bot�o SW1 n�o � pressionado
	// Neste momento que � realizada a leitura/grava��o dos par�metros do rob�
	HAL_UART_Receive_IT(&huart1, &RxByte, 1);
	while (getSW1() == LOW)
	{
		if (comandosUART() == 1) break;
	}

	delay_ms(1000);


	targetSpeedX = SPEEDX_TO_COUNTS(1000);//param_speedX_med);
	accX = decX = ACCX_TO_COUNTS(1000);//param_accX);
	distanceLeft = MM_TO_COUNTS(720);

	resetEncoderEsquerda();
	resetEncoderDireita();
	speedControlConfig();
	run = true;


	// Loop principal ----------------------------------------------------------
	HAL_UART_Receive_IT(&huart1, &RxByte, 1);
	while (1)
	{
		int32_t c = comandosUART();
		if (c == 0)
		{
			run = false;
			beep(50);
			setMotores(0, 0);
		}
		else if (c == 1)
		{
			run = true;
		}


		// Mensagens peri�dicas (1 segundo) para depura��o
#ifdef DEBUG_PRINTS
		if (ticks >= 1000 && run == true)
		{
			toggleLED(LED5);
			printf("Distacia percorrida: %d\r\n", distance_mm);
			printf("Velocidade media: %d\r\n", distance_mm - d0);
			printf("\r\n");
			d0 = distance_mm;

			ticks = 0;
		}
#endif
	}
}



// #############################################################################

// ############################## CONTROLADOR ##################################

// IRQ do Timer de controle de velocidade e speedProfile -----------------------
/**
  * @brief  Call back do timer - Per�odo definido em speed_control.h
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (run == true) speedProfile();
}



// #############################################################################

// ########################## FUN��ES AUXILIARES ###############################

/**
  * @brief Esta fun��o � chamada a cada 1ms por interrup��o
  */
void systick (void)
{
	ticks++;
}


/**
  * @brief Configura��o e inicializa��o dos perif�ricos
  */
void perifericosConfig(void)
{
	usart1Config();
	usbConfig();
	buzzerConfig();
	ledsConfig();
	botaoConfig();
	motoresConfig();
	sensoresConfig();
	encodersConfig();
}


/**
  * @brief Inicializa os par�metros com os valores guardados na FLASH
  */
void init_parametros(void)
{
	uint32_t buf[N_PARAMETROS];
	readFlash(buf, N_PARAMETROS);

	param_speedX_med = buf[0];
	param_speedX_min = buf[1];
	param_speedX_max = buf[2];

	param_pid_kp = buf[3];
	param_pid_ki = buf[4];
	param_pid_kd = buf[5];
	param_pid_offset = buf[6];

	param_accX = buf[7];
	param_accC = buf[8];

	param_a = buf[9];
	param_b = buf[10];
}

