/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
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
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void usb_transmit_byte(uint8_t transmit_int);
void usb_transmit_int(int32_t transmit_int);
void usb_transmit_uint(uint32_t transmit_uint);
void usb_transmit_float(float transmit_float);
void usb_transmit_char(char transmit_char);  // test this
void usb_transmit_string(char *transmit_string);  // test this
void toggle_LED(void);
void set_LED(uint8_t status);
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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  uint8_t sendstuff[] = "Testprint\n\r";
  char txbuf[10];
  CDC_Transmit_FS(sendstuff, sizeof(sendstuff));
  int starttime, stoptime, difftime;

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  starttime = HAL_GetTick();
	  HAL_Delay(1000);
	  toggle_LED();
	  stoptime = HAL_GetTick();
	  difftime = stoptime - starttime;


//	  snprintf(txbuf, 9, "%d", difftime);
	  sprintf(txbuf, "%d", difftime);
	  usb_transmit_string("\nTime:");
	  usb_transmit_int(difftime);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// debug functions for usb printing.
void usb_transmit_byte(uint8_t transmit_byte)
{
	CDC_Transmit_FS(transmit_byte, 1);
}
void usb_transmit_int(int32_t transmit_int)  // ok but sprintf is dangerous
{
	uint8_t stringbuf[10];
	sprintf(stringbuf,  "%d", transmit_int);
	CDC_Transmit_FS(stringbuf, strlen(stringbuf));
}
void usb_transmit_uint(uint32_t transmit_uint)  // ok but sprintf is dangerous
{
	uint8_t stringbuf[10];
	sprintf(stringbuf,  "%d", transmit_uint);

	CDC_Transmit_FS(stringbuf, strlen(stringbuf));
}
void usb_transmit_char(char transmit_char)
{
	CDC_Transmit_FS(transmit_char, 1);
}
void usb_transmit_string( char *transmit_string)
{
	CDC_Transmit_FS( (uint8_t*) transmit_string, strlen(transmit_string));
}
void usb_transmit_float(float transmit_float)
{
	char fullstring[20];
	char *tmpSign = (transmit_float < 0) ? "-" : " ";
	float tmpVal = (transmit_float < 0) ? -transmit_float : transmit_float;
	int tmpInt1 = tmpVal;                  // Get the integer (9876543210).
	float tmpFrac = tmpVal - tmpInt1;      // Get fraction (0.012).
	int tmpInt2 = (tmpFrac * 100);  // Turn into integer (12).
	sprintf (fullstring, "%s%d.%2d", tmpSign, tmpInt1, tmpInt2);

	CDC_Transmit_FS( (uint8_t*) fullstring, strlen(fullstring));
}
void toggle_LED(void)
{
	HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
}
void set_LED(uint8_t status)
{
	HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, status);
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
