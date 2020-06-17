/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t usartBuf = 0;
State ComSate = Stop,NowSate = Stop,LastSate = Stop;
Speed ComSpeed = LOW ;
uint8_t StartOnce = 1;//第一次启动标志
uint32_t count = 0;
unsigned long Pressure = 0;

uint8_t maopiOnce = 1;//第一次启动标志
double maopi = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void StepStop(void)
{
		HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);//失能
		HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);//停止输出脉冲
		StartOnce = 1;//电机停止后置为第一次启动
		count = 0;
}

void StepStart(uint16_t speed,uint8_t DIR)
{
	 if( StartOnce == 1)
	 {
		__HAL_TIM_SET_AUTORELOAD(&htim2,speed);//设置频率
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,speed/2);
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, !DIR);//设置方向
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);//开启脉冲+中断
		HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);//使能
		 StartOnce = 0;
	 }
}

double map(long x, long in_min, long in_max, double out_min, double out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
} 


void DateSend(uint32_t distence,unsigned long pressure)
{
			long ADC = pressure - 0x7FFFFF;
			double KG = map(ADC,-3607101,3607101,-50.0f,50.0f)/1.2;
			static int times = 0;
			if(maopiOnce == 1)
			{
				if(times < 5)
				{
					times++;
					return;
				}
				else if(times < 15)
				{
					times++;
					maopi += KG;
					return;
				}
				else
				{
					maopi = maopi/10;
					maopiOnce = 0;
				}
			}

		static uint32_t NowTime = 0,LastTime = 0;
		NowTime = HAL_GetTick();
		if(NowTime - LastTime <= 150)
		{
			printf("%.2f*%.2f",distence*0.003125,KG-maopi);
			//printf("%f-%f",0.01,0.01);
			LastTime = NowTime;
		}
		else 
		{
			LastTime = NowTime;
		}

}
void HC711UpDate()
{
	 

    HAL_GPIO_WritePin(HC_CLK_GPIO_Port, HC_CLK_Pin, GPIO_PIN_RESET);//使能
		if(HAL_GPIO_ReadPin(HC_DAT_GPIO_Port,HC_DAT_Pin))//AD转换未结束则等待，否则开始读取
			return;
		HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);//停止输出脉冲
		unsigned char i;
		unsigned long Count;
		Count = 0;
    for (i = 0; i < 24; i++)

    {
				HAL_GPIO_WritePin(HC_CLK_GPIO_Port, HC_CLK_Pin, GPIO_PIN_SET);//PD_SCL 置高（发送脉冲）
        Count=Count<<1; //下降沿来时变量Count左移一位，右侧补零
        HAL_GPIO_WritePin(HC_CLK_GPIO_Port, HC_CLK_Pin, GPIO_PIN_RESET); //PD_SCL 置低
        if(HAL_GPIO_ReadPin(HC_DAT_GPIO_Port,HC_DAT_Pin)) Count++;//如果是读数据线高电平就置位
    }

    HAL_GPIO_WritePin(HC_CLK_GPIO_Port, HC_CLK_Pin, GPIO_PIN_SET);//PD_SCL 置高（发送脉冲）
    Count=Count^0x800000;//第25个脉冲下降沿来时，转换数据
    HAL_GPIO_WritePin(HC_CLK_GPIO_Port, HC_CLK_Pin, GPIO_PIN_RESET); //PD_SCL 置低
		DateSend(count,Count);
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);//开启脉冲+中断
}
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_UART_Receive_IT(&huart1,&usartBuf,1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

			switch(NowSate)
			{
				case Stop:
				if(ComSate != Stop)
					NowSate = ComSate;
				else
					StepStop();
					break;
				
				//慢速前进
				case SlowForward:
				if(ComSate == Stop || HAL_GPIO_ReadPin(SwitchR_GPIO_Port,SwitchR_Pin) == 0 )//串口命令停止或到右限位，电机停止
				{
					NowSate = Stop;
					ComSate = Stop;
				}
				else 
				{
					if(ComSpeed == LOW)
					StepStart(37500,1);
					else if(ComSpeed == MID)
					StepStart(18750,1);
					else if(ComSpeed == FAS)
					StepStart(9375,1);
					HC711UpDate();
					
				}
					break;
			
				//快速前进
				case FastForward:
				if(ComSate == Stop || HAL_GPIO_ReadPin(SwitchR_GPIO_Port,SwitchR_Pin) == 0)
				{
					NowSate = Stop;
					ComSate = Stop;
				}
				else if(ComSpeed == LOW)
					StepStart(1000,1);
				else if(ComSpeed == MID)
					StepStart(500,1);
				else if(ComSpeed == FAS)
					StepStart(200,1);
					break;
				
				//慢速后退
				case SlowBehind:
				if(ComSate == Stop || HAL_GPIO_ReadPin(SwitchL_GPIO_Port,SwitchL_Pin) == 0  )
				{
					NowSate = Stop;
					ComSate = Stop;
				}
				else 
				{
				if(ComSpeed == LOW)
					StepStart(37500,0);
				else if(ComSpeed == MID)
					StepStart(18750,0);
				else if(ComSpeed == FAS)
					StepStart(9375,0);
					HC711UpDate();
				}
					break;
				
				//快速后退
				case FastBehind:
				if(ComSate == Stop || HAL_GPIO_ReadPin(SwitchL_GPIO_Port,SwitchL_Pin) == 0 )
				{
					NowSate = Stop;
					ComSate = Stop;
				}
				else if(ComSpeed == LOW)
					StepStart(1000,0);
				else if(ComSpeed == MID)
					StepStart(500,0);
				else if(ComSpeed == FAS)
					StepStart(200,0);
					break;
				
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	switch(usartBuf)
	{
		case 0xFF:
			ComSate = Stop;
			break;
		case 0x01:
			ComSate = SlowForward;
			ComSpeed = LOW;
			break;
		case 0x02:
			ComSate = SlowForward;
			ComSpeed = MID;
			break;
		case 0x03:
			ComSate = SlowForward;
			ComSpeed = FAS;
			break;
		case 0x11:
			ComSate = SlowBehind;
			ComSpeed = LOW;
			break;
		case 0x12:
			ComSate = SlowBehind;
			ComSpeed = MID;
			break;
		case 0x13:
			ComSate = SlowBehind;
			ComSpeed = FAS;
			break;
		case 0x21:
			ComSate = FastForward;
			ComSpeed = LOW;
			break;
		case 0x22:
			ComSate = FastForward;
			ComSpeed = MID;
			break;
		case 0x23:
			ComSate = FastForward;
			ComSpeed = FAS;
			break;
		case 0x31:
			ComSate = FastBehind;
			ComSpeed = LOW;
			break;
		case 0x32:
			ComSate = FastBehind;
			ComSpeed = MID;
			break;
		case 0x33:
			ComSate = FastBehind;
			ComSpeed = FAS;
			break;
	}
		HAL_UART_Receive_IT(&huart1,&usartBuf,1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if( htim->Instance == TIM2)
		count++;
}

int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
