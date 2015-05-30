/**
  ******************************************************************************
  * @file    wc2015/src/sensores.c
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.0
  * @date    27-Abril-2015
  * @brief   Funções para controle de velocidade dos motores
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "speed_control.h"


TIM_HandleTypeDef htim3;

/* Variáveis privadas --------------------------------------------------------*/
int32_t leftEncoderChange = 0, rightEncoderChange = 0;
int32_t encoderChange = 0, encoderCount = 0;
int32_t leftEncoderOld = 0, rightEncoderOld = 0;
int32_t leftEncoderCount = 0, rightEncoderCount = 0;
int32_t distance = 0;

int32_t oldPosErrorX = 0, posErrorX = 0;
int32_t oldPosErrorW = 0, posErrorW = 0;

int32_t oldSensorError = 0;

int32_t curSpeedX = 0, curSpeedW = 0;

int32_t bufferCounts[201];
uint8_t index_buffer_counts = 1;

int32_t bufferDistances[20] = {0};
int32_t bufferSpeedsWm[20] = {0};
uint8_t index_buffer_sector = 0;
int32_t accumulatorSpeedW = 0, numSpeedW = 0;

int32_t bufferSpeedXout[20] = {0};
int32_t bufferSpeedWout[20] = {0};


/* Variáveis externas --------------------------------------------------------*/
int32_t distanceLeft = 0, distance_mm = 0;
int32_t targetSpeedX = 0, targetSpeedW = 0;
int32_t endSpeedX = 0, endSpeedW = 0;
int32_t accX = 0, decX = 0, accW = 0, decW = 0;

bool onlyUseEncoderFeedback = false;
bool onlyUseGyroFeedback = false;
bool onlyUseSensorFeedback = false;

//#define CNTS_PRINTS
#define RUN1_PRINTS


/**
  * @brief Configuração da Base de Tempo de atualização do speedControl
  * @param Nenhum
  * @return Nenhum
  */
void speedControlConfig(void)
{
	TIM_CLK;

	// Configuração da base de tempo do speedProfile
	htim3.Instance = TIM;
	htim3.Init.Period = TIM_PERIOD;
	htim3.Init.Prescaler = TIM_PRESCALER;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	HAL_TIM_Base_Init(&htim3);

	HAL_TIM_Base_Start_IT(&htim3);

	HAL_NVIC_SetPriority(TIMx_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIMx_IRQn);

	HAL_TIM_IRQHandler(&htim3);
}


void speedProfile(void)
{
	getEncoderStatus();
	updateCurrentSpeed();
	calculateMotorPwm();

	// Registra a distância do trecho e o SpeedW_médio
	if (valid_marker == true)
	{
		bufferDistances[index_buffer_sector] = distance - bufferDistances[index_buffer_sector - 1];
		bufferSpeedsWm[index_buffer_sector] = accumulatorSpeedW / numSpeedW;

#ifdef RUN1_PRINTS
		printf("D[%d] = %ld\r\n", index_buffer_sector, bufferDistances[index_buffer_sector]);
		printf("W[%d] = %ld\r\n", index_buffer_sector, bufferSpeedsWm[index_buffer_sector]);
#endif

		index_buffer_sector++;
		accumulatorSpeedW = 0;
		numSpeedW = 0;
		valid_marker = false;
	}

	// Quando o robô parar na linha de chegada: calcula as velocidades do speedProfile
	if (frun == 4 && curSpeedX == 0)
	{
		printf("\r\n");

		for (uint8_t i = 0; i < index_buffer_sector; i++)
		{
			if (abs(bufferSpeedsWm[i]) < MINIMAL_SX_STRAIGHT)
			{	// Reta
				bufferSpeedXout[i] = SPEEDX_TO_COUNTS(param_speedX_max);
				bufferSpeedWout[i] = 0;
			}
			else
			{	// Curva
				float ray = (float)SPEEDX_TO_COUNTS(param_speedX_med) / (float)bufferSpeedsWm[i];
				bufferSpeedXout[i] = (int32_t)(sqrtf(ACCC_TO_COUNTS(param_accC) * abs(ray));
				bufferSpeedWout[i] = (int32_t)(bufferSpeedXout[i] / ray);
			}

			printf("SX[%d] = %ld\r\n", i, bufferSpeedXout[i]);
			printf("SW[%d] = %ld\r\n", i, bufferSpeedWout[i]);
		}

		frun = 5;
	}

#ifdef CNTS_PRINTS
	// Envia as velocidades (pacotes de 100 contagens - a cada 100ms)
	bufferCounts[index_buffer_counts] = leftEncoderChange;
	bufferCounts[index_buffer_counts + 1] = rightEncoderChange;
	index_buffer_counts += 2;
	if (index_buffer_counts == 201)
	{
		bufferCounts[0] = 0xAAAAAAAA;
		HAL_UART_DMAResume(&huart1);
		HAL_UART_Transmit_DMA(&huart1, (uint8_t*)bufferCounts, 804);
		index_buffer_counts = 1;
	}
#endif
}


void getEncoderStatus(void)
{
	int32_t leftEncoder, rightEncoder;

	leftEncoder = getEncoderEsquerda();
	rightEncoder = getEncoderDireita();

	leftEncoderChange = leftEncoder - leftEncoderOld;
	rightEncoderChange = rightEncoder - rightEncoderOld;
	encoderChange = (leftEncoderChange + rightEncoderChange) / 2;

	leftEncoderOld = leftEncoder;
	rightEncoderOld = rightEncoder;

	leftEncoderCount += leftEncoderChange;
	rightEncoderCount += rightEncoderChange;
	encoderCount =  (leftEncoderCount + rightEncoderCount) / 2;

	distanceLeft -= encoderChange;// update distanceLeft
	distance += encoderChange;
	distance_mm = COUNTS_TO_MM(distance);
}


void updateCurrentSpeed(void)
{
	if (targetSpeedW == 0)
	{
		if (needToDecelerate(distanceLeft, curSpeedX, endSpeedX) > decX)
		{
			targetSpeedX = endSpeedX;
		}
		if(curSpeedX < targetSpeedX)
		{
			curSpeedX += accX;
			if(curSpeedX > targetSpeedX)
				curSpeedX = targetSpeedX;
		}
		else if(curSpeedX > targetSpeedX)
		{
			curSpeedX -= decX;
			if(curSpeedX < targetSpeedX)
				curSpeedX = targetSpeedX;
		}
	}
	else
	{
		if (needToDecelerate(distanceLeft, curSpeedW, endSpeedW) > decW)
		{
			targetSpeedW = endSpeedW;
		}
		if(curSpeedW < targetSpeedW)
		{
			curSpeedW += accW;
			if(curSpeedW > targetSpeedW)
				curSpeedW = targetSpeedW;
		}
		else if(curSpeedW > targetSpeedW)
		{
			curSpeedW -= decW;
			if(curSpeedW < targetSpeedW)
				curSpeedW = targetSpeedW;
		}
	}
}


void calculateMotorPwm(void) // encoder PD controller
{
	int32_t gyroFeedback;
	int32_t rotationalFeedback;
	int32_t sensorFeedback;

	int32_t encoderFeedbackX, encoderFeedbackW;
	int32_t posPwmX, posPwmW;

    /* simple PD loop to generate base speed for both motors */
	encoderFeedbackX = rightEncoderChange + leftEncoderChange;
	encoderFeedbackW = rightEncoderChange - leftEncoderChange;

	accumulatorSpeedW += encoderFeedbackW;
	numSpeedW++;

	//gyroFeedback = getGyro() / GYRO_SCALE;
	gyroFeedback = 0;

	sensorFeedback = getSensorError();
	if (sensorFeedback == INFINITO) sensorFeedback = oldSensorError;
	oldSensorError = sensorFeedback;
	sensorFeedback /= SENSOR_SCALE;

	/*if(onlyUseGyroFeedback == true)
		rotationalFeedback = gyroFeedback;
	else if(onlyUseEncoderFeedback == true)
		rotationalFeedback = encoderFeedbackW;
	else
		rotationalFeedback = encoderFeedbackW + gyroFeedback + sensorFeedback;*/
	rotationalFeedback = sensorFeedback;

	posErrorX += curSpeedX - encoderFeedbackX;
	posErrorW = curSpeedW - rotationalFeedback;

	posPwmX = KP_X * posErrorX + KD_X * (posErrorX - oldPosErrorX);
	posPwmW = ((posErrorW * param_pid_kp) / 128) +  (((posErrorW - oldPosErrorW) * param_pid_kd) / 128);

	oldPosErrorX = posErrorX;
	oldPosErrorW = posErrorW;

	if (sensorFeedback < 0) setLED(LED1, HIGH);
	else setLED(LED1, LOW);

	if (sensorFeedback > 0) setLED(LED3, HIGH);
	else setLED(LED3, LOW);

	setMotores(posPwmX - posPwmW, posPwmX + posPwmW);
}


int32_t needToDecelerate(int32_t dist, int32_t curSpd, int32_t endSpd)
{
	if (curSpd<0) curSpd = -curSpd;
	if (endSpd<0) endSpd = -endSpd;
	if (dist<0) dist = 1;//-dist;
	if (dist == 0) dist = 1;  //prevent divide by 0

	return (abs((curSpd*curSpd - endSpd*endSpd) / (2*dist)));
	//calculate deceleration rate needed with input distance, input current
	//speed and input targetspeed to determind if the deceleration is needed
	//use equaltion 2*a*S = Vt^2 - V0^2  ==>  a = (Vt^2-V0^2)/(2*S)
}


void resetProfile(void)
{
	//curSpeedX = curSpeedW = 0;
	posErrorX = posErrorW = 0;
	oldPosErrorX = oldPosErrorW = 0;
}
