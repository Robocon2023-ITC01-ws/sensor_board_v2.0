/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define true 1
#define false 0

uint16_t Enc_count[3];
uint8_t BTN;
uint8_t BTN2;
uint8_t old_BTN;
uint8_t old_BTN2;
uint8_t flag = 0;
uint8_t flag_exti = 0;
uint8_t cntt;

uint16_t AD_RES[2];
uint16_t pub_adc = 0;

uint16_t time_read;

uint8_t noise;

uint8_t buzzer;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
CAN_TxHeaderTypeDef TxHeader;
CAN_TxHeaderTypeDef TxHeader2;
uint8_t TxData[8];
uint8_t TxData2[2];
uint32_t TxMailbox;

float map(float Input, float Min_Input, float Max_Input, float Min_Output, float Max_Output)
{

	return (float)((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData);
	cntt++;
	while (cntt - 100 > 0)
	{
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		cntt = 0;
	}
	if(RxHeader.StdId == 0x111){
		flag = 1;
	}

}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  HAL_ADCEx_Calibration_Start(&hadc1);
  	HAL_ADC_Start_DMA(&hadc1, &AD_RES, 2);


  	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

	TxHeader.DLC = 8; // data length
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = 0x333; // ID

	TxHeader2.DLC = 2; // data length
	TxHeader2.IDE = CAN_ID_STD;
	TxHeader2.RTR = CAN_RTR_DATA;
	TxHeader2.StdId = 0x444; // ID
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  	pub_adc = AD_RES[1];

	  	Enc_count[0] = TIM3->CNT;
	  	Enc_count[1] = TIM4->CNT;
	  	Enc_count[2] = TIM1->CNT;

		TxData[0] = ((Enc_count[0] & 0xFF00) >> 8);
		TxData[1] = (Enc_count[0] & 0x00FF);
		TxData[2] = ((Enc_count[1] & 0xFF00) >> 8);
		TxData[3] = (Enc_count[1] & 0x00FF);
		TxData[4] = ((Enc_count[2] & 0xFF00) >> 8);
		TxData[5] = (Enc_count[2] & 0x00FF);
		TxData[6] = ((pub_adc & 0xFF00) >> 8);
		TxData[7] = (pub_adc & 0x00FF);

		if(flag == 1){
			HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
			flag = 0;
		}

		if (flag_exti == 1 && (BTN != old_BTN || BTN2 != old_BTN2)){  ///  && HAL_GetTick() - time_read > 200
			HAL_CAN_AddTxMessage(&hcan, &TxHeader2, TxData2, &TxMailbox);
			flag_exti = 0;


			old_BTN = BTN;
			old_BTN2 = BTN2;
		}

		if(HAL_GetTick() - time_read > 100 && buzzer == 1){
			buzzer = 0;
			time_read = HAL_GetTick();
		}


		if (buzzer == 1){
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
		}
		else {
			HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
		}




  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// not working
	noise+=1;
	buzzer = 1;
	time_read = HAL_GetTick();
//	if(GPIO_Pin == BTN1_Pin){
//
//			uint8_t bit = HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin);
//			if(bit == true){
//				BTN |= 1 <<0;//set bit 0 to high(1)
//			}
//			else BTN &= ~(1 << 0);//clear bit 0 to low(0)
//	}
	if(GPIO_Pin == BTN1_Pin  || BTN2_Pin){
			uint8_t bit1 =! HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin);
			uint8_t bit2 =! HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin);
			if(bit1 == true && bit2 != true){
				BTN |= 1 <<0;	//set bit 0 to high(1)
			}
			else if (bit1 != true && bit2 == true){
				BTN &= ~(1 << 0);	//clear bit 0 to low(0)
			}
	}
	if(GPIO_Pin == BTN3_Pin  || BTN5_Pin){
		if(GPIO_Pin == BTN3_Pin){
			BTN &= ~(1 << 1);
			BTN &= ~(1 << 2);
		}
		else if(GPIO_Pin == BTN5_Pin){
			uint8_t bit3 = HAL_GPIO_ReadPin(BTN5_GPIO_Port, BTN5_Pin);
			if(bit3 == true){
				BTN |= 1 <<1;	//set bit 0 to high(1)
				BTN &= ~(1 << 2);
			}
			else if (bit3 != true){
				BTN &= ~(1 << 1);	//set bit 0 to high(1)
				BTN |= 1 <<2;	//set bit 0 to high(1)
			}
		}

	}
	if(GPIO_Pin == BTN4_Pin  || BTN6_Pin){
		if(GPIO_Pin == BTN4_Pin){
			BTN &= ~(1 << 3);
			BTN &= ~(1 << 4);
		}
		else if(GPIO_Pin == BTN6_Pin) {
			uint8_t bit4 = HAL_GPIO_ReadPin(BTN6_GPIO_Port, BTN6_Pin);
			if(bit4 == true){
				BTN |= 1 <<3;	//set bit 0 to high(1)
				BTN &= ~(1 << 4);
			}
			else if (bit4 != true){
				BTN &= ~(1 << 3);
				BTN |= 1 <<4;	//set bit 0 to high(1)
			}
		}
	}
	TxData2[0] = BTN;

	flag_exti = true;


}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
