
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "string.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t UART_NewMessage = 0;
char rxString[MAXSTRING];
uint8_t UART1_rxBuffer = '\000';
char* _index;
uint8_t pwmChannel[10];

const char helpString[] = {
		"\n\rCommands for use:\n\r"
		"help: - This page. \n\r"
		"off  - Set all channels to zero.\n\r"
		"cfg: - Config channel. \n\r"
		"       Example:\n\r"
		"         cfg:ch02=37,ch05=a8\n\r"
		"         ......\n\r"
		"         cfg:ch00=ff,ch01=e0,ch02=d0,ch03=c0,ch04=b0,ch05=a0,ch06=90,ch07=80,ch08=70,ch09=60\n\r"
		"         ...cfg:chXX=YY - set pwm channel XX = YY - constant in hex format."
		"---\n\rlog:\n\r"};

const char versionTitle[] = {
		"Quantium laser led driver, v1.0 (040520) by dragosha2000@gmx.net, HatSoft (c)\n\rLutsk,UA,2020\n\r" };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void StartTimers(void)
{
	// pinouts for PCB revision 1.0 build:160219)
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1 );
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1 );
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
}

//const uint16_t PWM_CHANNELS[10] = {TIM2->CCR4,TIM2->CCR3,TIM3->CCR4,TIM3->CCR3,TIM3->CCR2,TIM3->CCR1,TIM4->CCR1,TIM4->CCR2,TIM2->CCR1,TIM2->CCR2};
void setPwmChannel(uint8_t channel, uint8_t data) {
	switch (channel) {
	data = data*4;
	case 0: TIM2->CCR4 = data; break;
	case 1: TIM2->CCR3 = data; break;
	case 2: TIM3->CCR4 = data; break;
	case 3: TIM3->CCR3 = data; break;
	case 4: TIM3->CCR2 = data; break;
	case 5: TIM3->CCR1 = data; break;
	case 6: TIM4->CCR1 = data; break;
	case 7: TIM4->CCR2 = data; break;
	case 8: TIM2->CCR1 = data; break;
	case 9: TIM2->CCR2 = data; break;
	}
}

//const uint16_t PWM_CHANNELS[10] = {TIM2->CCR4,TIM2->CCR3,TIM3->CCR4,TIM3->CCR3,TIM3->CCR2,TIM3->CCR1,TIM4->CCR1,TIM4->CCR2,TIM2->CCR1,TIM2->CCR2};
void setPwmChannelFromPwmValues() {
	TIM2->CCR4 = pwmChannel[0];
	TIM2->CCR3 = pwmChannel[1];
	TIM3->CCR4 = pwmChannel[2];
	TIM3->CCR3 = pwmChannel[3];
	TIM3->CCR2 = pwmChannel[4];
	TIM3->CCR1 = pwmChannel[5];
	TIM4->CCR1 = pwmChannel[6];
	TIM4->CCR2 = pwmChannel[7];
	TIM2->CCR1 = pwmChannel[8];
	TIM2->CCR2 = pwmChannel[9];
}

uint32_t hexToInt(char* hex) {
	uint32_t val = 0;
    while (*hex) {
        uint8_t byte = *hex++;
        if (byte >= '0' && byte <= '9') byte = byte - '0';
        else if (byte >= 'a' && byte <='f') byte = byte - 'a' + 10;
        else if (byte >= 'A' && byte <='F') byte = byte - 'A' + 10;
        val = (val << 4) | (byte & 0xF);
    }
    return val;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
  SendMessage("\n\rKernel started on ....\n\r");
  SendMessage(versionTitle);
  SendMessage("\n\rtesting hardware...");
  for (uint8_t ch=0;ch<10;ch++) {
	  setPwmChannel(ch, 0);
  }
  StartTimers();
/*
  for (uint8_t ch=0;ch<10;ch++) {
	  for (uint8_t data = 0;data<255;data++) {
		  setPwmChannel(ch, data);
		  HAL_Delay(0);
	  }
	  for (uint8_t data = 255;data>0;data--) {
		  setPwmChannel(ch, data);
		  HAL_Delay(0);
	  }
	  setPwmChannel(ch, 0);
  }
*/
  for (uint8_t a=0;a<4;a++) {
	  for (uint8_t ch=0;ch<10;ch++) {
		  setPwmChannel(ch, 255);
	  }

	  HAL_Delay(100);

	  for (uint8_t ch=0;ch<10;ch++) {
		  setPwmChannel(ch, 0);
	  }
	  HAL_Delay(300);
  }

  SendMessage("-ok...\n\r");
  OutputHelpString();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  if (UART_NewMessage == 1) {
		  UartCommandProcessor();
	  }
	  HAL_Delay(1);
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1024;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1024;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1024;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void UartCommandProcessor() {

	char string_temp[50];
	uint8_t message_id;
//	char binaryString[(sizeof(int) << 3) + 1];
//	char* temp_str;

		snprintf(string_temp,sizeof(string_temp),"<UART0> incoming message: %s\n\r", rxString);
		SendMessage(string_temp);
		message_id = processUartMessage(rxString);
		if (message_id>-1) {
			SendMessage("- check command success.\n\r");
			switch (message_id) {
			case UART_CMD_HELP:
				OutputHelpString();
				break;
			case UART_CMD_CONFIG_CHANNEL:
				for (uint8_t x=0; x<10; x++) {
					setPwmChannel(x, pwmChannel[x]);
				}
//				setPwmChannelFromPwmValues();
				break;
			case UART_CMD_OFF_ALL:
				for (uint8_t x=0; x<10; x++) {
					pwmChannel[x] = 0;
					setPwmChannel(x, 0);
				}
			}
		} else {
			SendMessage("incorrect command !\n\r");
		}
		UART_NewMessage = 0;
}

uint8_t processUartMessage(char* string) {

//	char str1[6];
	int8_t id = -1;
	uint8_t x;
	char str1[3] = {0,0,0};

	_index = strstr(string,"help");
    if(_index!=NULL) return UART_CMD_HELP;
    else {
    	_index = strstr(string,"cfg:");
    	if (_index!=NULL) {
    		for (x=0; x<10; x++) {
    			char string_temp[5];
    			snprintf(string_temp,sizeof(string_temp),"ch%02X", x+1);
    			_index = strstr(string,string_temp);
    			if (_index != NULL) {
    	    		memcpy(str1, _index  + 5, 2);
    				pwmChannel[x] = (uint8_t)hexToInt(str1);
    			}
    		}
    		id = UART_CMD_CONFIG_CHANNEL;
    	} else {
        	_index = strstr(string,"off");
        	if (_index!=NULL) {
        		id = UART_CMD_OFF_ALL;
        	}
    	}
    	return id;
    }
}

void OutputHelpString() {
	SendMessage(helpString);
}

void SendMessage(char string[]) {

//	while (UART_TX_Busy){};
//	UART_TX_Busy = 1;
//	snprintf(string_out,sizeof(string_out),string);
	HAL_UART_Transmit(&huart1,(uint8_t*)string,strlen(string), HAL_MAX_DELAY);
//	CDC_Transmit_FS((uint8_t)string, (uint16_t)strlen(string));
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static short int UART1_rxindex = 0;
	static uint8_t UART1_ErrorFlag = UART_PACKET_OK;

		UART1_rxBuffer = (uint8_t)huart->Instance->DR;
		if (UART1_rxBuffer == 8 || UART1_rxBuffer == 127) // If Backspace or del
		{
			UART1_rxindex--;
			if (UART1_rxindex < 0) UART1_rxindex = 0;
		}

		else if (UART1_rxBuffer == '\n' || UART1_rxBuffer == '\r' || UART1_rxBuffer == '\0') // If Enter
		{
			if (UART1_ErrorFlag == UART_PACKET_OK && UART1_rxindex)
			{
				rxString[UART1_rxindex] = 0;
				UART1_rxindex = 0;
				UART_NewMessage = 1;
			}
			else
			{
				SendMessage("ERROR > UART1 packet too long\n\r");
				UART1_ErrorFlag = UART_PACKET_OK; // reset error state
			}
		}

		else
		{
			if (UART1_rxBuffer != '\r' && UART1_ErrorFlag == UART_PACKET_OK) // Ignore return
			{
				rxString[UART1_rxindex] = UART1_rxBuffer; // Add that character to the string
				UART1_rxindex++;
				if (UART1_rxindex >= MAXSTRING) // User typing too much, we can't have commands that big
				{
					UART1_ErrorFlag = UART_PACKET_TOO_LONG;
					UART1_rxindex = 0;
					rxString[UART1_rxindex] = '\000';
					UART_NewMessage = 1;
				}
			}
		}
		__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
