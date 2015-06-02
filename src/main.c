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

int32_t param_speedX_med, param_topSpeed1, param_topSpeed2;
int32_t param_pid_kp, param_pid_kd, param_scale_sensor, param_scale_gyro;
int32_t param_accX1, param_accC1, param_accX2, param_accC2;

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

		flag_run = 0;
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


	// Seleciona os parâmetros da respectiva corrida (searchRun, fastRun1 e fastRun2)
	initializeRun();

	resetEncoderEsquerda();
	resetEncoderDireita();
	speedControlConfig();
	run = true;


	// Loop principal ----------------------------------------------------------
	while (1)
	{
		switch (num_run)
		{
			case SEARCH_RUN:	// Corrida de de reconhecimento ****************
				if (flag_run == RUN_OK)
				{
					run = false;
					recordSectors();

					calculateSpeedProfile(param_topSpeed1, param_accC1);
					resetProfile();

					num_run = FAST_RUN1;
					flag_run = PAUSE;
				}
				break;


			case FAST_RUN1:	// Corrida rápida 1 ********************************
				if (run == false)
				{
					while (getSW1() == LOW);

					beeps(2, 50, 50);
					delay_ms(1000);

					useEncoderFeedback = true;
					useSensorFeedback = true;
					useGyroFeedback = false;

					valid_marker = false;

					changeSpeedProfile();

					flag_run = 0;
					run = true;
				}

				if (flag_run == RUN_OK)
				{
					run = false;
					calculateSpeedProfile(param_topSpeed2, param_accC2);
					resetProfile();

					num_run = FAST_RUN2;
					flag_run = PAUSE;
				}
				break;


			case FAST_RUN2:	// Corrida rápida 2 ********************************
				if (run == false)
				{
					while (getSW1() == LOW);

					beeps(3, 50, 50);
					delay_ms(1000);

					useEncoderFeedback = true;
					useSensorFeedback = true;
					useGyroFeedback = false;

					valid_marker = false;

					changeSpeedProfile();

					flag_run = 0;
					run = true;
				}
				break;


			case STOP:
				run = false;
				break;
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

	// Velocidades
	param_speedX_med = buf[0];
	param_topSpeed1 = buf[1];
	param_topSpeed2 = buf[2];

	// Controlador
	param_pid_kp = buf[3];
	param_pid_kd = buf[4];
	param_scale_sensor = buf[5];
	param_scale_gyro = buf[6];

	// Acelerações
	param_accX1 = buf[7];
	param_accC1 = buf[8];
	param_accX2 = buf[9];
	param_accC2 = buf[10];
}


void initializeRun(void)
{
	accX = decX = 2;//ACCX_TO_COUNTS(500);//param_accX);
	accW = decW = 2;

	switch (num_run)
	{
		case SEARCH_RUN:	// Corrida de de reconhecimento ********************
			useEncoderFeedback = false;
			useGyroFeedback = false;
			useSensorFeedback = true;

			targetSpeedX = SPEEDX_TO_COUNTS(param_speedX_med);
			distanceLeft = MM_TO_COUNTS(10000);
			break;


		case FAST_RUN1:	// Corrida rápida 1 ************************************
			useEncoderFeedback = true;
			useSensorFeedback = true;
			useGyroFeedback = false;

			updateBufferSpeedProfile();
			calculateSpeedProfile(param_topSpeed1, param_accC1);
			changeSpeedProfile();
			break;


		case FAST_RUN2:	// Corrida rápida 2 ************************************
			useEncoderFeedback = true;
			useSensorFeedback = true;
			useGyroFeedback = false;

			updateBufferSpeedProfile();
			calculateSpeedProfile(param_topSpeed2, param_accC2);
			changeSpeedProfile();
			break;
	}
}


void recordSectors(void)
{
	writeFlash(ADDR_FLASH_SECTOR_10, buf_temp, 2 * SIZE_BUFFER_SECTORS);

#ifdef DEBUG_PRINTS
	uint32_t buf[SIZE_BUFFER_SECTORS * 2];
	readFlash(ADDR_FLASH_SECTOR_10, buf, SIZE_BUFFER_SECTORS * 2);

	for (uint32_t i = 0; i < SIZE_BUFFER_SECTORS * 2; i++)
	{
		printf("[%ld]: %ld\r\n", i, buf[i]);
	}
#endif
}
