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
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "usb.h"
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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/////*****CAPTEUR HC SR04********//////
#define TRIG_PIN GPIO_PIN_10 //definition du port en sortie
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_6 // defnition du port en entree
#define ECHO_PORT GPIOC

uint32_t Value1 = 0; // l'instant quand echo passe à 1
uint32_t Value2 = 0; // l'instant quand echo passe à 0
uint16_t Distance  = 0;  // en cm

/////*****MOTEUR L298P********//////
const uint16_t pwm =100; // pour ajuster la vitesse des roues
//****Moteur Gauche********
#define IN1_PIN GPIO_PIN_0 //definition du port en sortie
#define IN1_PORT GPIOC
#define IN2_PIN GPIO_PIN_1 //definition du port en sortie
#define IN2_PORT GPIOC

//****Moteur Droite********
#define IN3_PIN GPIO_PIN_1 //definition du port en sortie
#define IN3_PORT GPIOA
#define IN4_PIN GPIO_PIN_0 //definition du port en sortie
#define IN4_PORT GPIOA

/* *****NOTE*********
PWM pour IN1 et IN2 -> PWM_ENA (Timer 1 Channel 1) // Roue Gauche
PWM pour IN2 et IN3 -> PWM_ENB (Timer 2 Channel 1) // Roue Droite
******************************************* */
// ********FIN NOTE*************

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Fonction pour mesurer la distance entre le robot et un obstacle
uint16_t Mesure_Distance(void)
{
HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
__HAL_TIM_SET_COUNTER(&htim16, 0);
while (__HAL_TIM_GET_COUNTER(&htim16) < 10);  // wait for 10 us
HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

while (!(HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) ); //Attendre que echo passe à l'état haut
Value1 = __HAL_TIM_GET_COUNTER(&htim16); // valeur du timer à laquelle echo passe à l'état haut

while ((HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN))); //Attendre que echo passe à l'état bas
Value2 = __HAL_TIM_GET_COUNTER(&htim16); // valeur du timer à laquelle echo passe à l'état bas

// Value2-Value1 : durée pendant laquelle echo reste en etat haut
// On multiplie la moitié de cette durée par la vitesse de l'ultrason pour avoir la distance

Distance = ((Value2-Value1)* 0.034)/2;
HAL_Delay(50);
return Distance;
}

//Fonction pour clignoter la LED verte
void Blink_Green(void)
{
HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
HAL_Delay(500);					/* Insert delay 500 ms */
HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
HAL_Delay(200);					/* Insert delay 200 ms */
}

//Fonction pour clignoter la LED rouge
void Blink_Red(void)
{
HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);
HAL_Delay(500);					/* Insert delay 500 ms */
HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
HAL_Delay(200);					/* Insert delay 200 ms */
}


// ********LES DIFFERENTS MOUVEMENTS DU ROBOT*******************

// fonction pour initialiser le moteur
void Init_Moteur(void)
{
	HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
	// Ajustez le rapport cyclique pour contrôler la vitesse
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm); //

	HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
	// Ajustez le rapport cyclique pour contrôler la vitesse
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm); //

	HAL_Delay(500); // Attendre 0,5 s
}

//fonction pour avancer 1001 (IN1=1 ; IN2=0, IN3=0 ; IN4=1)
void Move_Forward(void)
{
	HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
	// Ajustez le rapport cyclique pour contrôler la vitesse
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);

	HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_SET);
	// Ajustez le rapport cyclique pour contrôler la vitesse
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);

	HAL_Delay(500); // Attendre 0,5 s
}

//fonction pour reculer 0110 (IN1=0 ; IN2=1, IN3=1 ; IN4=0)
void Move_Backward(void)
{
	HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_SET);
	// Ajustez le rapport cyclique pour contrôler la vitesse
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);

	HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
	// Ajustez le rapport cyclique pour contrôler la vitesse
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);

	HAL_Delay(500); // Attendre 0,5 s
}

//fonction pour arreter (IN1=0 ; IN2=0, IN3=0 ; IN4=0)
void Stop(void)
{
	HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
	// Ajustez le rapport cyclique pour contrôler la vitesse
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
	HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
	// Ajustez le rapport cyclique pour contrôler la vitesse
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);

	HAL_Delay(500); // Attendre 0,5 s
}

//fonction pour tourner a droite 1000 (IN1=1 ; IN2=0, IN3=0 ; IN4=0)
void Turn_Right(void)
{
	HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
	// Ajustez le rapport cyclique pour contrôler la vitesse
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);

	HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_RESET);
	// Ajustez le rapport cyclique pour contrôler la vitesse
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);

	HAL_Delay(500); // Attendre 0,5 s
}

//fonction pour tourner a gauche 0001 (IN1=0 ; IN2=0, IN3=0 ; IN4=1)
void Turn_Left(void)
{
	HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, GPIO_PIN_RESET);
	// Ajustez le rapport cyclique pour contrôler la vitesse
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);

	HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, GPIO_PIN_SET);
	// Ajustez le rapport cyclique pour contrôler la vitesse
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm);

	HAL_Delay(500); // Attendre 0,5 s
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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim16);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low
  Init_Moteur();

  //uint16_t Distance1 = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  uint16_t distance = 0; // Initialisation de la distance

	      // Mesure la distance pendant que le robot avance
	      while (1)
	      {
	          distance = Mesure_Distance(); // Mesurer la distance

	          if (distance <= 10)
	          {
	              // Obstacle détecté à moins de 10cm, déclenche l'évitement
	              // Séquence : reculer, tourner, vérifier à nouveau la distance
	              Blink_Red();
	              Move_Backward();
	              HAL_Delay(50);
	              Turn_Right();
	              HAL_Delay(50);
	              distance = Mesure_Distance();
	              if (distance <= 10)
	              {
	                  continue; // Répéter l'évitement si l'obstacle est toujours détecté
	              }
	              Move_Forward();
	              HAL_Delay(50);
	          } else {
	              // Pas d'obstacle, avance normalement
	              Blink_Green();
	              Move_Forward();
	              HAL_Delay(50);
	          }
	      }
  } // end while
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV8;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
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
