/**
  ******************************************************************************
  * @file    wc2015/src/main.c
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.1
  * @date    17-Maio-2015
  * @brief   Programa do Robô uMaRT Lite+ - Winter Challenge 2015
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Definições do Programa ----------------------------------------------------*/



/* Variáveis Globais ---------------------------------------------------------*/
int32_t erro = 0, erro_a = 0;
bool run = false;
uint32_t ticks = 0;

int32_t param_speedX_med, param_speedX_min, param_speedX_max;
int32_t param_accX, param_accC, param_a, param_b;
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

	// Configuração e inicialização dos periféricos ----------------------------
	perifericosConfig();


	// Inicio do programa ------------------------------------------------------
	// As mensagens enviadas por printf podem ser destinadas à USB ou à UART,
	// para isto altere a definição no arquivo main.h
#ifdef DEBUG_PRINTS
	printf("Winter Challenge 2015 - uMaRT LITE+ V1.1\r\n");
#endif
	delay_ms(100);


	// Inicialização dos parâmetros guardados na Flash
	init_parametros();


	// Caso inicie o robô com SW1 pressionado: alteração dos parâmetros pelas rodinhas
	if (getSW1() == HIGH)
	{
		//menu_rodinhas();

		while (getSW1() == HIGH);
		beeps(2, 50, 100);
		delay_ms(500);

		flag_run = RUN_OK;
		num_run = FAST_RUN1;

	}


	// Loop enquanto o botão SW1 não é pressionado
	// Neste momento que é realizada a leitura/gravação dos parâmetros do robô
	HAL_UART_Receive_IT(&huart1, &RxByte, 1);
	while (getSW1() == LOW)
	{
		if (comandosUART() == 1) break;
	}

	delay_ms(1000);

	if (num_run == SEARCH_RUN)
	{
		targetSpeedX = SPEEDX_TO_COUNTS(param_speedX_med);//param_speedX_med);
		accX = decX = ACCX_TO_COUNTS(param_accX);//param_accX);
		distanceLeft = MM_TO_COUNTS(13155);
	}
	else if (num_run == FAST_RUN1)
	{
		useEncoderFeedback = true;
		useSensorFeedback = true;
		//useGyroFeedback = true;

		changeSpeedProfile();
	}

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

		if (fflash == true)
		{
			writeFlash(ADDR_FLASH_SECTOR_10, buf_temp, 3 * SIZE_BUFFER_SECTORS);
			fflash = false;

			uint32_t buf[SIZE_BUFFER_SECTORS * 3];
			readFlash(ADDR_FLASH_SECTOR_10, buf, SIZE_BUFFER_SECTORS * 3);

			for (uint32_t i = 0; i < SIZE_BUFFER_SECTORS * 3; i++)
			{
				printf("[%ld]: %ld\r\n", i, buf[i]);
			}
		}


		// Mensagens periódicas (1 segundo) para depuração
#ifdef DEBUG_PRINTS
		if (ticks >= 1000 && run == true)
		{
			toggleLED(LED5);
			printf("Distacia percorrida: %ld\r\n", distance_mm);
			printf("Velocidade media: %ld\r\n", distance_mm - d0);
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
  * @brief  Call back do timer - Período definido em speed_control.h
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (run == true) speedProfile();
}



// #############################################################################

// ########################## FUNÇÕES AUXILIARES ###############################

/**
  * @brief Esta função é chamada a cada 1ms por interrupção
  */
void systick (void)
{
	ticks++;
}


/**
  * @brief Configuração e inicialização dos periféricos
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
  * @brief Inicializa os parâmetros com os valores guardados na FLASH
  */
void init_parametros(void)
{
	uint32_t buf[N_PARAMETROS];
	readFlash(ADDR_FLASH_SECTOR_11, buf, N_PARAMETROS);

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

