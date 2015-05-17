/**
  ******************************************************************************
  * @file    umart_lite_plus_teste/src/usart1.c
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.1
  * @date    20-Abril-2015
  * @brief   Funções para configuração e inicialização da USART1
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart1.h"

/* Variáveis Externas ------------------------------------------------------- */
UART_HandleTypeDef huart1;
char RxBuffer[BUFFER_SIZE];
char RxByte;
uint32_t rx_available = 0;


/**
  * @brief Configuração da USART1
  * @param Nenhum
  * @return Nenhum
  */
void usart1Config(void)
{
	__GPIOA_CLK_ENABLE();	// Habilita o barramento de clock do GPIOA
	__USART1_CLK_ENABLE();	// Habilita o barramento de clock da USART1

	// Configura os GPIOs da USART1 como Alternate Function
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Configuração do periférico USART
	huart1.Instance = USART1;
	huart1.Init.BaudRate = BAUD_RATE;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&huart1);

	HAL_NVIC_SetPriority(USARTx_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(USARTx_IRQn);
}


/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of IT Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
    static uint32_t pos = 0;

	RxBuffer[pos] = RxByte;

	HAL_UART_Receive_IT(UartHandle, &RxByte, 1);

	pos++;
	if (RxBuffer[pos - 1] == '\n')
	{
		RxBuffer[pos] = '\0';
		rx_available = pos;
		pos = 0;
	}

}


int32_t comandosUART(void)
{
	uint32_t buf[N_PARAMETROS + 1];
	uint32_t checksum = 0;
	static uint32_t i = 0;

	if(rx_available > 0)
	{
		// Envia os dados do robô para o celular
		if (strcmp(RxBuffer, "GET\n") == 0)
		{
			readFlash(buf, N_PARAMETROS);
			checksum = 0;
			for (i = 0; i < N_PARAMETROS; i++) checksum += buf[i];

			printf("%d_%d_%d_%d_%d_%d_%d_%d_%d_%d_%d_%d\n", buf[0], buf[1],
					buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8],
					buf[9], buf[10], checksum);

			//printf("Dados enviados com sucesso, checksum = %d\r\n", checksum);
		}

		// Recebe os dados do celular e grava no robô
		else if (strcmp(RxBuffer, "SET\n") == 0)
		{
			i = 1;
		}
		else if (i == 1)
		{
			char* token;
			i = 0;

			memset(buf, 0, N_PARAMETROS + 1);
			token = strtok (RxBuffer,"_");
			while (token != NULL)
			{
				buf[i] = atoi(token);
				i++;
				token = strtok (NULL, "_");
			}

			checksum = 0;
			for (i = 0; i < N_PARAMETROS; i++) checksum += buf[i];

			if ((checksum == buf[N_PARAMETROS]) && (checksum != 0))
			{
				writeFlash(buf, N_PARAMETROS);
				init_parametros();
				printf("Dados recebidos com sucesso, checksum = %d\r\n", checksum);
				beep(100);
			}
			else
			{
				printf("Checksum INCORRETO!\r\n");
			}
		}

		// Freia o robô
		else if (strcmp(RxBuffer, "STOP\n") == 0)
		{
			printf("Freia o robo!\r\n");
			rx_available = 0;
			memset(RxBuffer, 0, BUFFER_SIZE);

			return 0;
		}

		// Reinicia o robô
		else if (strcmp(RxBuffer, "START\n") == 0)
		{
			printf("Reinicia o robo!\r\n");
			rx_available = 0;
			memset(RxBuffer, 0, BUFFER_SIZE);

			return 1;
		}

		// Parâmetro desconhecido
		else
		{
			printf("ERRO\r\n");
		}

		rx_available = 0;
		memset(RxBuffer, 0, BUFFER_SIZE);
	}

	//delay_ms(1);

	return -1;
}



#ifdef STDIO_UART

/**
  * @brief Redefinição da função de escrita para uso da função printf - stdio.h
  */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart1, ptr, len, USART_TIMEOUT);

	return len;
}


/**
  * @brief Redefinição da função de leitura para uso da função scanf - stdio.h
  */
int _read (int file, char *ptr, int len)
{
	HAL_UART_Receive(&huart1, ptr, len, USART_TIMEOUT);

	return len;
}

#endif
