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

int32_t rotationalFeedback = 0;
int32_t oldPosErrorX = 0, posErrorX = 0;
int32_t oldPosErrorW = 0, posErrorW = 0;

int32_t oldSensorError = 0;

int32_t curSpeedX = 0, curSpeedW = 0;

int32_t bufferLFT[201];
uint8_t index_buffer_lft = 1;
//uint8_t c_aux = 0;

int32_t bufferDistances[20] = {0};
int32_t bufferSpeedsWm[20] = {0};
uint8_t index_buffer_sector = 0;
int32_t accumulatorSpeedW = 0, numSpeedW = 0;

int32_t bufferSpeedXout[SIZE_BUFFER_SECTORS] = {0};
int32_t bufferSpeedWout[SIZE_BUFFER_SECTORS] = {0};




/* Variáveis externas --------------------------------------------------------*/
int32_t distanceLeft = 0, distance_mm = 0;
int32_t targetSpeedX = 0, targetSpeedW = 0;
int32_t endSpeedX = 0, endSpeedW = 0;
int32_t accX = 0, decX = 0, accW = 0, decW = 0;

bool useEncoderFeedback = false;
bool useGyroFeedback = false;
bool useSensorFeedback = true;

uint8_t num_run = SEARCH_RUN;

int32_t buf_temp[3 * SIZE_BUFFER_SECTORS];
bool fflash = false;

/* Definições do programa ----------------------------------------------------*/
//#define LFT_PRINTS	// Habilita o envio dos valores para o software LFTrakking
//#define SEARCH_RUN_PRINTS	// Habilita mensagens de debug da searchRun
//#define FAST_RUNS_PRINTS	// Habilita mensagens de debug para as fastRuns


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

	recordsSectors();	// Registra a distância do trecho e o SpeedW_médio
	changeRuns();		// Tratamento das voltas (searchRun, fastRun1 e fastRun2)



#ifdef LFT_PRINTS
	// Envia as velocidades (pacotes de 100 contagens - a cada 100ms)
	//bufferLFT[index_buffer_lft] = leftEncoderChange;
	//bufferLFT[index_buffer_lft + 1] = rightEncoderChange;

	// Envia a resposta da velocidade SpeedX
	bufferLFT[index_buffer_lft] = curSpeedX;
	bufferLFT[index_buffer_lft + 1] = 2 * encoderChange;

	// Envia a resposta da velocidade SpeedW
	//bufferLFT[index_buffer_lft] = curSpeedW;
	//bufferLFT[index_buffer_lft + 1] = rotationalFeedback;

	index_buffer_lft += 2;
	if (index_buffer_lft == 201)// && c_aux < 100)
	{
		bufferLFT[0] = 0xAAAAAAAA;
		HAL_UART_DMAResume(&huart1);
		HAL_UART_Transmit_DMA(&huart1, (uint8_t*)bufferLFT, 804);
		index_buffer_lft = 1;

		//c_aux++;
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
	//if (targetSpeedW == 0)
	//{
		if (needToDecelerate(distanceLeft, curSpeedX, endSpeedX) > decX)
		{
			targetSpeedX = endSpeedX;
			targetSpeedW = endSpeedW;
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
	//}
	//else
	//{
		//if (needToDecelerate(distanceLeft, curSpeedW, endSpeedW) > decW)
		//{
		//	targetSpeedW = endSpeedW;
		//}
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
	//}
}


void calculateMotorPwm(void) // encoder PD controller
{
	int32_t gyroFeedback;
	int32_t sensorFeedback;

	int32_t encoderFeedbackX, encoderFeedbackW;
	int32_t posPwmX, posPwmW;

	rotationalFeedback = 0;

    // Feedbacks dos encoders
	encoderFeedbackX = rightEncoderChange + leftEncoderChange;
	encoderFeedbackW = rightEncoderChange - leftEncoderChange;

	accumulatorSpeedW += encoderFeedbackW;
	numSpeedW++;

	// Leitura do giroscópio
	//gyroFeedback = getGyro() / GYRO_SCALE;
	gyroFeedback = 0;

	// Leitura dos sensores de linha
	sensorFeedback = getSensorError();
	if (sensorFeedback == INFINITO) sensorFeedback = oldSensorError;
	oldSensorError = sensorFeedback;
	sensorFeedback /= SENSOR_SCALE;
	if (num_run != SEARCH_RUN) sensorFeedback /= 10;

	// Habilita os feedback selecionados
	if (useEncoderFeedback == true) rotationalFeedback += encoderFeedbackW;
	if (useGyroFeedback == true) rotationalFeedback += gyroFeedback;
	if (useSensorFeedback == true) rotationalFeedback += sensorFeedback;

	// Calculo do erro
	posErrorX += curSpeedX - encoderFeedbackX;
	if (num_run == SEARCH_RUN) posErrorW = curSpeedW - rotationalFeedback;
	else if (flag_run < RUN_OK) posErrorW += curSpeedW - rotationalFeedback;
	else posErrorW = 0;
	//posErrorW += curSpeedW - rotationalFeedback;

	// Controladores PDs para ambos motores
	posPwmX = KP_X * posErrorX + KD_X * (posErrorX - oldPosErrorX);
	if (num_run == SEARCH_RUN)
	{
		posPwmW = ((posErrorW * param_pid_kp) / 128) +  (((posErrorW - oldPosErrorW) * param_pid_kd) / 128);
	}
	else
	{
		posPwmW = KP_W * posErrorW + KD_W * (posErrorW - oldPosErrorW);
	}

	oldPosErrorX = posErrorX;
	oldPosErrorW = posErrorW;

	// Indica através dos LEDs o sinal do erro
	if (sensorFeedback < 0) setLED(LED1, HIGH);
	else setLED(LED1, LOW);
	if (sensorFeedback > 0) setLED(LED3, HIGH);
	else setLED(LED3, LOW);

	// Aciona os motores
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
	curSpeedX = curSpeedW = 0;
	posErrorX = posErrorW = 0;
	oldPosErrorX = oldPosErrorW = 0;
	distanceLeft = 0;
}


void recordsSectors(void)
{
	static int32_t oldDistance = 0;

	// Registra a distância do trecho e o SpeedW_médio
	if (num_run == SEARCH_RUN && valid_marker == true)
	{

		bufferDistances[index_buffer_sector] = distance - oldDistance;
		bufferSpeedsWm[index_buffer_sector] = accumulatorSpeedW / numSpeedW;
		oldDistance = distance;

#ifdef SEARCH_RUN_PRINTS
		printf("D[%d] = %ld\r\n", index_buffer_sector, bufferDistances[index_buffer_sector]);
		printf("W[%d] = %ld\r\n", index_buffer_sector, bufferSpeedsWm[index_buffer_sector]);
#endif

		index_buffer_sector++;
		accumulatorSpeedW = 0;
		numSpeedW = 0;
		valid_marker = false;
	}
}

void changeRuns(void)
{
	static uint32_t delay_start = 0;

	switch (num_run)
	{
		case SEARCH_RUN:
			// Quando o robô parar na linha de chegada: calcula as velocidades do speedProfile
			if (flag_run == GOAL_OK && curSpeedX == 0)
			{
#ifdef SEARCH_RUN_PRINTS
				printf("\r\n");
#endif

				// Calculo dos parametros do speedProfile
				for (uint8_t i = 0; i < index_buffer_sector; i++)
				{
					if (abs(bufferSpeedsWm[i]) < MINIMAL_SX_STRAIGHT)
					{	// Reta
						bufferSpeedXout[i] = SPEEDX_TO_COUNTS(param_speedX_max);
						bufferSpeedWout[i] = 0;
					}
					else
					{	// Curva
						float ray = ((float)SPEEDX_TO_COUNTS(param_speedX_med) / (float)bufferSpeedsWm[i]);
						bufferSpeedXout[i] = (int32_t)(sqrtf(ACCC_TO_COUNTS(param_accC) * abs(ray * W_2)));
						if (bufferSpeedXout[i] > SPEEDX_TO_COUNTS(param_speedX_max)) bufferSpeedXout[i] = SPEEDX_TO_COUNTS(param_speedX_max);
						bufferSpeedWout[i] = (int32_t)(bufferSpeedXout[i] / ray);
					}
#ifdef SEARCH_RUN_PRINTS
					printf("SX[%d] = %ld\r\n", i, bufferSpeedXout[i]);
					printf("SW[%d] = %ld\r\n", i, bufferSpeedWout[i]);
#endif
				}

				// Cocatena os buffers e grava na flash
				uint32_t count = 0;
				memcpy(&buf_temp[count], bufferSpeedXout, 4 * SIZE_BUFFER_SECTORS);
				count += SIZE_BUFFER_SECTORS;
				memcpy(&buf_temp[count], bufferSpeedWout, 4 * SIZE_BUFFER_SECTORS);
				count += SIZE_BUFFER_SECTORS;
				memcpy(&buf_temp[count], bufferDistances, 4 * SIZE_BUFFER_SECTORS);
				fflash = true;
				setMotores(0, 0);

				// Atualiza o estado
				flag_run = RUN_OK;
				num_run = FAST_RUN1;

				resetProfile();
			}
			break;


		case FAST_RUN1:
			if (flag_run == RUN_OK)
			{
				if (getSW1() == HIGH)
				{
					delay_start = 0;
					flag_run = WAIT;
					beep(300);
				}
			}

			if (flag_run == WAIT)
			{
				if (++delay_start == 1000)
				{
					useEncoderFeedback = true;
					useSensorFeedback = true;
					//useGyroFeedback = true;

					valid_marker = false;
					index_buffer_sector = 0;

					changeSpeedProfile();

					flag_run = 0;
				}
			}

			if (valid_marker == true)
			{
				index_buffer_sector++;
				changeSpeedProfile();

				valid_marker = false;
			}

			if (flag_run == GOAL_OK && curSpeedX == 0)
			{
				flag_run = RUN_OK;
				num_run = FAST_RUN2;
			}
			break;

		case FAST_RUN2:
			break;
	}
}


void changeSpeedProfile(void)
{
	targetSpeedX = bufferSpeedXout[index_buffer_sector];
	endSpeedX = bufferSpeedXout[index_buffer_sector + 1];

	targetSpeedW = bufferSpeedWout[index_buffer_sector];
	endSpeedW = bufferSpeedWout[index_buffer_sector + 1];

	distanceLeft = bufferDistances[index_buffer_sector];
}


void updateBufferSpeedProfile(void)
{
	uint32_t buf[SIZE_BUFFER_SECTORS * 3];
	uint32_t count = 0;

	readFlash(ADDR_FLASH_SECTOR_10, buf, SIZE_BUFFER_SECTORS * 3);

	memcpy(bufferSpeedXout, &buf[count], 4 * SIZE_BUFFER_SECTORS);
	count += SIZE_BUFFER_SECTORS;
	memcpy(bufferSpeedWout, &buf[count], 4 * SIZE_BUFFER_SECTORS);
	count += SIZE_BUFFER_SECTORS;
	memcpy(bufferDistances, &buf[count], 4 * SIZE_BUFFER_SECTORS);
}
