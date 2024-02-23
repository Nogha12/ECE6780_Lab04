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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f0xx_it.h"
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
char uartInputChar;
int inputReceivedFlag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void txCharacter(char data);
void txString(char* data);
char rxCharacter(void);
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
  int clkSpeed;
  int targetBaud = 115200;
  int activePin;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enable GPIO B clock
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable GPIO C clock
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // Enable USART 3 clock
  
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  // Configure GPIO B pins 10 and 11 to connect to USART 3
  GPIOB->MODER &= ~((0x3 << 10*2) | (0x3 << 11*2));
  GPIOB->MODER |= (0x2 << 10*2) | (0x2 << 11*2); // alternate mode
  GPIOB->OTYPER &= ~((0x1 << 10) | (0x1 << 11)); // push-pull
  GPIOB->OSPEEDR &= ~((0x3 << 10*2) | (0x3 << 11*2)); // low-speed
  GPIOB->PUPDR &= ~((0x3 << 10*2) | (0x3 << 11*2)); // no pull-up/pull-down
  GPIOB->AFR[1] &= ~((0xF << ((10 - 8)*4)) | (0xF << ((11 - 8)*4)));
  GPIOB->AFR[1] |= (0x4 << ((10 - 8)*4)) | (0x4 << ((11 - 8)*4)); // alternate function 4
  
  // Configure GPIO C pins 6, 7, 8, and 9 (LED pins)
  GPIOC->MODER &= !((0x3 << 6*2) | (0x3 << 7*2) | (0x3 << 8*2) | (0x3 << 9*2));
  GPIOC->MODER |= (0x1 << 6*2) | (0x1 << 7*2) | (0x1 << 8*2) | (0x1 << 9*2); // output mode
  GPIOC->OTYPER &= !((0x1 << 6) | (0x1 << 7) | (0x1 << 8) | (0x1 << 9)); // push-pull
  GPIOC->OSPEEDR &= !((0x3 << 6*2) | (0x3 << 7*2) | (0x3 << 8*2) | (0x3 << 9*2)); // low-speed
  GPIOC->PUPDR &= !((0x3 << 6*2) | (0x3 << 7*2) | (0x3 << 8*2) | (0x3 << 9*2)); // no pull-up/pull-down

  clkSpeed = HAL_RCC_GetHCLKFreq();
  USART3->CR1 |= (0x1 << 2) | (0x1 << 3); // enable TX and RX
  USART3->CR1 |= (0x1 << 5); // enable interrupts from receive register not empty
  USART3->BRR &= 0x0;
  USART3->BRR |= clkSpeed / targetBaud; // set baud rate clock divisor
  
  NVIC_EnableIRQ(USART3_4_IRQn); // enable USART 3 interrupts
  NVIC_SetPriority(USART3_4_IRQn, 2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  USART3->CR1 |= (0x1 << 0); // enable USART 3
  txString("Awaiting input: \n\r");
  while (1)
  {
    /* USER CODE END WHILE */
    
    /* USER CODE BEGIN 3 */
    //txString("Noah Lomu\n\r");
    //HAL_Delay(1000);
    /* Old code
    switch(rxCharacter())
    {
      case 'r':
        GPIOC->ODR ^= (0x1 << 6); // Toggle pin C6 (red LED)
        break;
      case 'b':
        GPIOC->ODR ^= (0x1 << 7); // Toggle pin C7 (blue LED)
        break;
      case 'o':
        GPIOC->ODR ^= (0x1 << 8); // Toggle pin C8 (orange LED)
        break;
      case 'g':
        GPIOC->ODR ^= (0x1 << 9); // Toggle pin C9 (green LED)
        break;
      default:
        txString("Error, key not valid.\n\r");
        break;
    }
    */
    //txString("inputReceivedFlag = ");
    //txCharacter(inputReceivedFlag + 48);
    //txString(".\n\r");
    
    HAL_Delay(1); // only check every milisecond
    
    if (inputReceivedFlag == 1)
    {
      switch(uartInputChar)
      {
        case 'r':
          activePin = 6; // Set active pin to C6 (red LED)
          break;
        case 'b':
          activePin = 7; // Set active pin to C7 (blue LED)
          break;
        case 'o':
          activePin = 8; // Set active pin to C8 (orange LED)
          break;
        case 'g':
          activePin = 9; // Set active pin to C9 (green LED)
          break;
        case '0':
          if (activePin < 6 || activePin > 9)
          {
            txString("Error: please first choose a pin (r, b, o, or g).\n\r");
            break;
          }
          GPIOC->ODR &= ~(0x1 << activePin); // Turn off active pin
          txString("Pin ");
          txCharacter(activePin + 48);
          txString(" was turned off.\n\r");
          txString("\nAwaiting input: \n\r");
          activePin = -1;
          break;
        case '1':
          if (activePin < 6 || activePin > 9)
          {
            txString("Error: please first choose a pin (r, b, o, or g).\n\r");
            break;
          }
          GPIOC->ODR |= (0x1 << activePin); // Turn on active pin
          txString("Pin ");
          txCharacter(activePin + 48);
          txString(" was turned on.\n\r");
          txString("\nAwaiting input: \n\r");
          activePin = -1;
          break;
        case '2':
          if (activePin < 6 || activePin > 9)
          {
            txString("Error: please first choose a pin (r, b, o, or g).\n\r");
            break;
          }
          GPIOC->ODR ^= (0x1 << activePin); // Toggle active pin
          txString("Pin ");
          txCharacter(activePin + 48);
          txString(" was toggled.\n\r");
          txString("\nAwaiting input: \n\r");
          activePin = -1;
          break;
        default:
          txString("Error: Please enter r, b, o, or g followed by 0, 1, or 2.\n\r");
          activePin = -1;
          break;
      }
      inputReceivedFlag = 0;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief Transmits a character via USART 3
  */
void txCharacter(char data)
{
  while ((USART3->ISR & (0x1 << 7)) == 0x0);
  USART3->TDR = data;
  return;
}

/**
  * @brief Transmits a string via USART 3
  */
void txString(char* data)
{
  for (int i = 0; data[i] != '\0'; i++)
  {
    txCharacter(data[i]);
  }
  return;
}

/**
  * @brief Receives a character via USART 3
  */
char rxCharacter(void)
{
  while ((USART3->ISR & (0x1 << 5)) == 0x0);
  return USART3->RDR;
}

/**
  * @brief This function handles USART 3 interrupts.
  */
void USART3_4_IRQHandler(void)
{
  inputReceivedFlag = 1;
  uartInputChar = USART3->RDR;
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

