/**
  ******************************************************************************
  * @file    umart_lite_plus_teste/src/sensores.c
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.1
  * @date    20-Abril-2015
  * @brief   Funções para acionamento do Botão SW1
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sensores.h"


/** @defgroup Variáveis privadas
  * @{
  */
GPIO_TypeDef* EMISSORES_PORT[N_EMISSORES] =
			{L_LINE_PORT, R_LINE_PORT, L_MARK_E_PORT, R_MARK_E_PORT};
const uint16_t EMISSORES_PIN[N_EMISSORES] =
			{L_LINE_PIN, R_LINE_PIN, L_MARK_E_PIN, R_MARK_E_PIN};

GPIO_TypeDef* RECEPTORES_PORT[N_RECEPTORES] =
			{LINE1_PORT, LINE2_PORT, LINE3_PORT, LINE4_PORT,
			LINE5_PORT, LINE6_PORT, LINE7_PORT, LINE8_PORT,
			L_MARK_R_PORT, R_MARK_R_PORT};
const uint16_t RECEPTORES_PIN[N_RECEPTORES] =
			{LINE1_PIN, LINE2_PIN, LINE3_PIN, LINE4_PIN,
			LINE5_PIN, LINE6_PIN, LINE7_PIN, LINE8_PIN,
			L_MARK_R_PIN, R_MARK_R_PIN};

GPIO_TypeDef* ANALOGICAS_PORT[N_ANALOGICAS] =
			{G_OUTZ_PORT, G_VREF_PORT, VBAT_PORT};
const uint16_t ANALOGICAS_PIN[N_ANALOGICAS] =
			{G_OUTZ_PIN, G_VREF_PIN, VBAT_PIN};

ADC_HandleTypeDef hadc1;
/**
  * @}
  */


/* Variáveis externas --------------------------------------------------------*/
bool valid_marker = false;
int32_t flag_run = 0;
int32_t acumulator_aSpeed = 0, angle = 0;
int32_t numSamplesGyro = 0;


/**
  * @brief Configuração dos GPIOs e ADCs dos sensores
  * @param Nenhum
  * @return Nenhum
  */
void sensoresConfig(void)
{
	SENSORES_CLK;	// Habilita o barramento de clock do GPIO dos Sensores
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_ChannelConfTypeDef sConfig;

	// Configura os GPIOs dos emissores como saída push/pull
	for (int i = 0; i < N_EMISSORES; i++)
	{
		GPIO_InitStructure.Pin = EMISSORES_PIN[i];;
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStructure.Pull = GPIO_PULLUP;
		GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
		HAL_GPIO_Init(EMISSORES_PORT[i], &GPIO_InitStructure);
		HAL_GPIO_WritePin(EMISSORES_PORT[i], EMISSORES_PIN[i], LOW);
	}


	// Configura os GPIOs dos receptores como entrada sem resistor interno
	for (int i = 0; i < N_RECEPTORES; i++)
	{
		GPIO_InitStructure.Pin = RECEPTORES_PIN[i];
		GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
		GPIO_InitStructure.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(RECEPTORES_PORT[i], &GPIO_InitStructure);
	}


	// Configura os pinos analógicos
	for (int i = 0; i < N_ANALOGICAS; i++)
	{
		GPIO_InitStructure.Pin = ANALOGICAS_PIN[i];
		GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStructure.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(ANALOGICAS_PORT[i], &GPIO_InitStructure);
	}

	// Configuração do ADC
	__ADC1_CLK_ENABLE();
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION12b;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
	HAL_ADC_Init(&hadc1);


	// Leitura da Bateria, gera um alerta quando a tensão for menor que 7,00V
	if (getTensao() > VBAT_ALERTA)
	{
#ifdef DEBUG_PRINTS
		printf("Bateria: %d mV\r\n", getTensao());
#endif
		beeps(1, 50, 50);
	}
	else
	{
#ifdef DEBUG_PRINTS
		printf("Bateria BAIXA!\r\n");
#endif
		beeps(5, 50, 50);
		beep(500);
	}
}



/**
  * @brief Realiza várias leituras dos sensores de linha e retorna a média
  * @param Nenhum
  * @return erro Valores negativos (delocado para direita), valores positivos
  * (deslocado para esquerda), INFINITO caso não tenha detectado linha
  */
int32_t getSensorError(void)
{
	int32_t erro = 0, soma = 0, n = 0;

	for(int i = 50; i <= 100; i += 10)
	{
		uint32_t t0 = micros();

		// Habilita os emissores
		HAL_GPIO_WritePin(L_LINE_PORT, L_LINE_PIN, HIGH);
		HAL_GPIO_WritePin(R_LINE_PORT, R_LINE_PIN, HIGH);
		if (i == 100)
		{
			HAL_GPIO_WritePin(L_MARK_E_PORT, L_MARK_E_PIN, HIGH);
			HAL_GPIO_WritePin(R_MARK_E_PORT, R_MARK_E_PIN, HIGH);
		}
		elapse_us(i, t0);

		// Leitura do giroscópio
		acumulator_aSpeed += getRawGyro();
		numSamplesGyro++;

		// Realiza a leitura de todos os sensores de linha, os sensores das
		// extremidades pussuem peso maior, no final é realizada a média ponderada
		if(HAL_GPIO_ReadPin(LINE1_PORT, LINE1_PIN) == LINHA)
		{
			soma += -2000;
			n++;
		}
		if(HAL_GPIO_ReadPin(LINE2_PORT, LINE2_PIN) == LINHA)
		{
			soma += -1333;
			n++;
		}
		if(HAL_GPIO_ReadPin(LINE3_PORT, LINE3_PIN) == LINHA)
		{
			soma += -667;
			n++;
		}
		if(HAL_GPIO_ReadPin(LINE4_PORT, LINE4_PIN) == LINHA)
		{
			soma += -167;
			n++;
		}
		if(HAL_GPIO_ReadPin(LINE5_PORT, LINE5_PIN) == LINHA)
		{
			soma += 167;
			n++;
		}
		if(HAL_GPIO_ReadPin(LINE6_PORT, LINE6_PIN) == LINHA)
		{
			soma += 667;
			n++;
		}
		if(HAL_GPIO_ReadPin(LINE7_PORT, LINE7_PIN) == LINHA)
		{
			soma += 1333;
			n++;
		}
		if(HAL_GPIO_ReadPin(LINE8_PORT, LINE8_PIN) == LINHA)
		{
			soma += 2000;
			n++;
		}

		// Desabilita os emissores
		HAL_GPIO_WritePin(L_LINE_PORT, L_LINE_PIN, LOW);
		HAL_GPIO_WritePin(R_LINE_PORT, R_LINE_PIN, LOW);
		if (i == 100)
		{
			readMarkers();
			HAL_GPIO_WritePin(L_MARK_E_PORT, L_MARK_E_PIN, LOW);
			HAL_GPIO_WritePin(R_MARK_E_PORT, R_MARK_E_PIN, LOW);
		}
		elapse_us(i * 2, t0);


		// Leitura do giroscópio
		acumulator_aSpeed += getRawGyro();
		numSamplesGyro++;
	}


	if(n != 0)
	{
		erro = soma / n;
	}
	else
	{
		erro = INFINITO;
	}

	return erro;
}


/**
  * @brief
  * @param
  * @return
  */
void readMarkers(void)
{
	static int32_t marker_corner = 0, marker_start_goal = 0;
	static bool marker_intersection = false, fmarker = false;

	// Detecção das marcas de partida/chegada
	if (HAL_GPIO_ReadPin(R_MARK_R_PORT, R_MARK_R_PIN) == GPIO_PIN_SET)
	{
		marker_start_goal++;
	}
	else if (marker_start_goal > 0)
	{
		marker_start_goal--;
	}

	// corner and start/goal marker detection
	if (HAL_GPIO_ReadPin(L_MARK_R_PORT, L_MARK_R_PIN) == GPIO_PIN_SET)
	{
		marker_corner++;
	}
	else if (marker_corner > 0)
	{
		marker_corner--;
	}

	 // detection of intersection, both marker - ignore intersection
	if (marker_start_goal > 1 && marker_corner > 1)
	{
		marker_intersection = true;
	}
	if (marker_intersection == true && marker_start_goal == 0 && marker_corner == 0)
	{
		marker_intersection = false;
	}


    // corner marker check
    if (marker_intersection == true)
    {
    	fmarker = false;
    }
    if (marker_intersection == false && fmarker == false && marker_corner > MARKER_TH)
    {	// corner marker detect
    	fmarker = true;
    }
    if (marker_intersection == false && fmarker == true && marker_corner == 0)
    {	// corner marker fix
		fmarker = false;
		beep(50);

		valid_marker = true;
    }

    // start/goal marker check
    if (flag_run == 0 && marker_start_goal > MARKER_TH)
    {	// start marker detect
    	flag_run = 1;
    }
    if (flag_run == 1 && marker_start_goal == 0)
    {	// start marker fix
		flag_run=2;
		beep(200);

		valid_marker = true;
    }
    if (flag_run == 2 && marker_start_goal > MARKER_TH)
    {	// goal marker detect
		flag_run = 3;
    }
    if (flag_run == 3 && marker_intersection == true)
    {	// ignore intersection
		flag_run = 2;
    }
    if (flag_run == 3 && marker_start_goal == 0)
    {	// goal marker fix
		flag_run = GOAL_OK;
		beep(200);
		distanceLeft = MM_TO_COUNTS(150);
		endSpeedX = 0;

		valid_marker = true;
    }
}

/**
  * @brief Verifica a leitura do giroscópio
  * @param Nenhum
  * @return w: Velocidade angular
  */
int32_t getRawGyro()
{
	int32_t w = 0;

	w = getRawADC(G_OUTZ_CH) - getRawADC(G_VREF_CH);

	return w;
}

int32_t getGyro()
{
	int32_t raw_aSpeed = acumulator_aSpeed / numSamplesGyro;

	acumulator_aSpeed = 0;
	numSamplesGyro = 0;

	return (int32_t)T_GYRO(raw_aSpeed);
}


/**
  * @brief Verifica a tensão da bateria
  * @param Nenhum
  * @return Tensão da bateria em mV
  */
int32_t getTensao()
{
	return ((getRawADC(VBAT_CH) * VBAT_V) / VBAT_RAW);
}


/**
  * @brief Realiza a conversão de um canal analógico
  * @param canal: Canal analógico a ser realizado a conversão
  * @return rawADC: Resultado da conversão (valor de 12 bits)
  */
uint32_t getRawADC(uint32_t canal)
{
	uint32_t rawADC;
	ADC_ChannelConfTypeDef sConfig;

	sConfig.Channel = canal;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	rawADC = HAL_ADC_GetValue(&hadc1);

	return rawADC;
}
