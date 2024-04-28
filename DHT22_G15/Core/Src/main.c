/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lib_lcd.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static rgb_lcd lcddata;
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
uint8_t Humidite_octet_1 , Humidite_octet_2 , Temperature_octet_1 , Temperature_octet_2;
uint16_t somme , Temperature , Humidite;
uint8_t Reception=0;
float Temp=0.0;
float Humi=0.0;

//Fonction pour le timer en microseconde
void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim6,0);  // initialiser le timer a 0
	while (__HAL_TIM_GET_COUNTER(&htim6)<us);  /*attendre que le compteur atteingne la
												valeur desirée mise en parametre*/
}

// Initialise la broche GPIO en mode sortie
void GPIO_INIT_OUTPUT(void){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

// Initialise la broche GPIO en mode entrée
void GPIO_INIT_INPUT(void){
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Pin = GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

// Démarre la communication avec le capteur DHT22
void start_DHT22(void){
	GPIO_INIT_OUTPUT();
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
	delay_us(35);
	GPIO_INIT_INPUT();
}

// Vérifie la réponse du capteur DHT22
uint8_t DHT22_Verification_Reponse(void){
	GPIO_INIT_INPUT();
	uint8_t Reponse = 0;
	delay_us(30);
	if (!(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)))
	{
		delay_us(80);
			if ((HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_3))) Reponse=1;
			else Reponse= -1;
	}
	while((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3))){}

			return Reponse;

}

// Lit les données du capteur DHT22
uint8_t Lecture_DHT22(void){
	uint8_t octet_mesure,j;
	for(j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3))){}
		delay_us(40);
		if (!(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)))
		{
			octet_mesure &= ~(1<<(7-j));

		}
		else
		{
			octet_mesure |= (1<<(7-j));
		}
		while((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3))){}
	}
	return octet_mesure;
}

// Affiche la température sur l'écran LCD
void affichage_temperature(float Temperature){
	char txt[20]={0};
	sprintf(txt,"Temp:%.1f C",Temperature);
	clearlcd();
	lcd_position(&hi2c1,0,0);
	lcd_print(&hi2c1,txt);
}

// Affiche la l'humidité sur l'écran LCD
void affichage_humidite(float Humidite){
	char txt[20]={0};
	sprintf(txt,"Humi:%.1f",Humidite);
	lcd_position(&hi2c1,0,1);
	lcd_print(&hi2c1,txt);
	lcd_position(&hi2c1,10,1);
		lcd_print(&hi2c1,"%");

}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	/*void INIT_timer3(){
		RCC->APB1ENR1 |= (1<<1);//activation du clk de TIM3
		TIM3->PSC = 15;//prescaler du TIM3 pour avoir f=1Mhz ceci signie aussi que le timer va s'incrementer tous les microsecondes
		TIM3->ARR = 20;// la valeur de reload du TIM3 (1Mhz /20)=50Khz, t=20us a fin de faire une temporisation de 20 us
		TIM3->CNT = 0;//initialisation du comptage de timer
		TIM3->CR1 |= (1<<0);//activation du TIM3 tempo=20us
		while((TIM3->SR & (1<<0))==0){}//attente de 20us
		if((TIM3->SR & (1<<0))==1){// test si 20us sont terminer
		TIM3->SR &= ~(0x1); } // remise a zero du flag de temporisation
	}*/



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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim6);
    clearlcd();
    lcd_init(&hi2c1,&lcddata);
    lcd_position(&hi2c1,1,1);
    lcd_print(&hi2c1,"Initialisation");
    HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  start_DHT22();
	  Reception=DHT22_Verification_Reponse();
	  Humidite_octet_1 = Lecture_DHT22();
	  Humidite_octet_2 = Lecture_DHT22();
	  Temperature_octet_1 = Lecture_DHT22();
	  Temperature_octet_2  = Lecture_DHT22();
	  somme = Lecture_DHT22();
	  Temperature= (Temperature_octet_1<<8)|Temperature_octet_2;
	  Humidite=(Humidite_octet_1<<8)|Humidite_octet_2;
	  Temp=(float)(Temperature/10.0);
	  Humi=(float)(Humidite/10.0);
	  affichage_temperature(Temp);
	  affichage_humidite(Humi);
	  HAL_Delay(2500);
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
