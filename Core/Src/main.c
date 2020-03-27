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
#include "eth.h"
#include "ltdc.h"
#include "quadspi.h"
#include "usb_device.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32746g_sdram.h"
#include "stm32746g_qspi.h"
#include "debug_console.h"

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
  MX_LTDC_Init();
  MX_ETH_Init();
  MX_FMC_Init();
  MX_QUADSPI_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  /* SDRAM device configuration */
  BSP_SDRAM_Init();
#ifdef _JEIL_DEBUG_H_
  DebugInit();
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#ifndef _JEIL_DEBUG_H_
    HAL_GPIO_WritePin(LED_Out_GPIO_Port, LED_Out_Pin, GPIO_PIN_SET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(LED_Out_GPIO_Port, LED_Out_Pin, GPIO_PIN_RESET);
    HAL_Delay(250);
#else
    HAL_Delay(500);
#endif
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;

  /* Add a freq to guarantee minimum wait */
  if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)(uwTickFreq);
  }

  while ((HAL_GetTick() - tickstart) < wait)
  {
#ifdef _JEIL_DEBUG_H_
	  DebugTask();
#endif
  }
}

void SDRAM_Test(void)
{
	#define SDRAM_BUFFER_SIZE   ((uint32_t)0x1000)
	#define WRITE_READ_ADDR     ((uint32_t)0x0800)
	/* Read/Write Buffers */
	uint32_t aTxBuffer[SDRAM_BUFFER_SIZE];
	uint32_t aRxBuffer[SDRAM_BUFFER_SIZE];
	/* Status variables */
	__IO uint32_t uwWriteReadStatus = 0;

	printf("\r\n SDRAM example !!!\r\n");
	/* Program the SDRAM external device */
	/*##-1- Configure the SDRAM device #########################################*/
	HAL_Delay(10);

	/*##-2- SDRAM memory read/write access #####################################*/
	/* Fill the buffer to write */
	for (int i = 0; i < SDRAM_BUFFER_SIZE; i++) {
		aTxBuffer[i] = 0xC178A562 + i; /* TxBuffer init */
	}

	/* Write data to the SDRAM memory */
	BSP_SDRAM_WriteData(SDRAM_DEVICE_ADDR + WRITE_READ_ADDR, aTxBuffer,	SDRAM_BUFFER_SIZE);
	printf(" /* Write data to the SDRAM memory */\r\n");
//	for (int i = 0; i < BUFFER_SIZE; i++) {
//		printf("%02X:0x%08lX ", i, aTxBuffer[i]);
//		HAL_Delay(2);
//	}
//	printf("\r\n");
	HAL_Delay(100);

	/* Read back data from the SDRAM memory */
	BSP_SDRAM_ReadData(SDRAM_DEVICE_ADDR + WRITE_READ_ADDR, aRxBuffer, SDRAM_BUFFER_SIZE);
	printf(" /* Read back data from the SDRAM memory */\r\n");
//	for (int i = 0; i < BUFFER_SIZE; i++) {
//		printf("%02X:0x%08lX ", i, aRxBuffer[i]);
//		HAL_Delay(2);
//	}
//	printf("\r\n");
	HAL_Delay(100);

	/*##-3- Checking data integrity ############################################*/
	for (int i = 0; (i < SDRAM_BUFFER_SIZE); i++) {
		if (aRxBuffer[i] != aTxBuffer[i]) {
			uwWriteReadStatus++;
		}
	}
	if (uwWriteReadStatus == 0) /* check date */
		printf(" SDRAM Test OK\r\n");
	else
		printf(" SDRAM Test False\r\n");
}

void W25Q_QUADSPI_Test(void)
{
	#define	QSPI_BUFFER_SIZE	0x100

	uint8_t wData[QSPI_BUFFER_SIZE];
	uint8_t rData[QSPI_BUFFER_SIZE];
	uint8_t pData[3];

	/*##-1- Initialize W25Q128FV  ###########################################*/
	if (BSP_QSPI_Init()==QSPI_OK) {
		printf("\r\n  W25Q256JV QuadSPI Test....\r\n");
		HAL_Delay(10);
	}

	/*##-2-Read Device ID Test    ###########################################*/
	if (BSP_QSPI_Read_Device_ID(READ_ID_CMD, pData)==QSPI_OK)
		printf("-SPI I/0 Read Device ID : 0x%2X 0x%2X\r\n", pData[0], pData[1]);
	HAL_Delay(10);

	if (BSP_QSPI_Read_Device_ID(DUAL_READ_ID_CMD, pData)==QSPI_OK)
		printf("-Dual I/O Read Device ID : 0x%2X 0x%2X\r\n", pData[0], pData[1]);
	HAL_Delay(10);

	if (BSP_QSPI_Read_Device_ID(QUAD_READ_ID_CMD, pData)==QSPI_OK)
		printf("-Quad I/O Read Device ID : 0x%2X 0x%2X\r\n", pData[0], pData[1]);
	HAL_Delay(10);
	if (BSP_QSPI_Read_Device_ID(READ_JEDEC_ID_CMD, pData)==QSPI_OK)
		printf(" Read JEDEC ID :  0x%2X 0x%2X 0x%2X\r\n", pData[0], pData[1], pData[2]);
	HAL_Delay(10);

	/*##-3-QSPI Erase/Write/Read Test    ###########################################*/
	/* fill buffer */
	for (uint32_t i = 0; i < 0x100; i++) {
		wData[i] = i;
		rData[i] = 0;
	}

	if (BSP_QSPI_Erase_Block(0) == QSPI_OK)
		printf(" QSPI Erase Block OK\r\n");
	else
		printf(" QSPI Erase Block FAIL\r\n");
	HAL_Delay(10);

	if (BSP_QSPI_Write(wData, 0x00, QSPI_BUFFER_SIZE) == QSPI_OK)
		printf(" QSPI Write OK\r\n");
	else
		printf(" QSPI Write FAIL\r\n");
	HAL_Delay(10);

	if (BSP_QSPI_Read(rData, 0x00, QSPI_BUFFER_SIZE) == QSPI_OK)
		printf(" QSPI Read OK\r\n");
	else
		printf(" QSPI Read FAIL\r\n");
	HAL_Delay(10);
#if 0
	printf("QSPI Read Data : \r\n");
	for (uint32_t i = 0; i < 0x100; i++) {
		printf("0x%02X  ", rData[i]);
		HAL_Delay(1);
	}
	printf("\r\n");

	for (uint32_t i = 0; i < 0x100; i++) {
		if (rData[i] != wData[i]) {
			printf("0x%02X 0x%02X ", wData[i], rData[i]);
			HAL_Delay(1);
		}
	}
	printf("\r\n");
	HAL_Delay(100);
#endif
	/* check data */
#if 1
	uint8_t wdat;
	uint8_t rdat;
	uint8_t verify=1;
	for (uint16_t i = 0; i < QSPI_BUFFER_SIZE; i++) {
		wdat = wData[i];
		rdat = rData[i];
		if (wdat!=rdat) {
			printf("At %3d Byte miss match between write=%02X & read=%02X \r\n", i, wdat, rdat);
			HAL_Delay(10);
			verify = 0;
			break;
		}
	}
	if (verify)
		printf(" QSPI Verify OK\r\n");
	else
		printf(" QSPI Verify FAIL\r\n");
#endif
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
	printf("\r\n Error Handler.\r\n");
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
