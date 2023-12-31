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
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BluetoothClassicComm.h"
#include "mpu6050.h"
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
uint32_t RecMpuDataCounter = 0;

uint8_t gyroCalibValues[6] = {0, 193, 0, 21, 255, 241};

MpuData_t MpuData;

BLU_ImuBaseDataReport_t BluImuBaseDataReport;

// As the sensor is set in init, the sensor takes a sample every 5ms (200hz),
// creates an interrupt, then puts the dma data in the buffer, 
// then the data is scaled and the orientations are calculated.
// This process is repeated automatically every 5ms. So dt is 0.005 seconds
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  MPU6050_ReadDmaDataEndCallBack(&MpuData);
  RecMpuDataCounter++;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  MPU6050_Read_DMA();
}

void ReportBaseImuData(void)
{
  BluImuBaseDataReport.gyroX               = MpuData.gyroX;
  BluImuBaseDataReport.gyroY               = MpuData.gyroY;
  BluImuBaseDataReport.gyroZ               = MpuData.gyroZ;
  BluImuBaseDataReport.accX                = MpuData.accX;
  BluImuBaseDataReport.accY                = MpuData.accY;
  BluImuBaseDataReport.accZ                = MpuData.accZ;
  BluImuBaseDataReport.normalizedAccX      = MpuData.normalizedAccX;
  BluImuBaseDataReport.normalizedAccY      = MpuData.normalizedAccY;
  BluImuBaseDataReport.normalizedAccZ      = MpuData.normalizedAccZ;
  BluImuBaseDataReport.fildAccX            = MpuData.fildAccX;
  BluImuBaseDataReport.fildAccY            = MpuData.fildAccY;
  BluImuBaseDataReport.fildAccZ            = MpuData.fildAccZ;
  BluImuBaseDataReport.jerkX               = MpuData.jerkX;
  BluImuBaseDataReport.jerkY               = MpuData.jerkY;
  BluImuBaseDataReport.jerkZ               = MpuData.jerkZ;
  BluImuBaseDataReport.roll                = MpuData.roll; 
  BluImuBaseDataReport.pitch               = MpuData.pitch; 
  BluImuBaseDataReport.yaw                 = MpuData.yaw;
  BluImuBaseDataReport.velX                = MpuData.velX;
  BluImuBaseDataReport.velY                = MpuData.velY;
  BluImuBaseDataReport.velZ                = MpuData.velZ;
  BluImuBaseDataReport.posX                = MpuData.posX;
  BluImuBaseDataReport.posY                = MpuData.posZ; /*!!!Intended!!!*/
  BluImuBaseDataReport.posZ                = MpuData.posY; 

  BLU_ReportImuData(&BluImuBaseDataReport);

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  
  BLU_Init();

  MPU6050_Init(&hi2c1);
  // while (MPU6050_Init(&hi2c1) == 1){
  //   // MPU6050_DeviceReset(0);
  //   // HAL_Delay(1000);
  //   // MPU6050_DeviceReset(1);
  // }

  // uint8_t *NewCalibData = MPU6050_Calibrate_Gyro();
  MPU6050Set_Calibrate_Gyro(gyroCalibValues);
  MPU6050_Start_IRQ();
  MPU6050_Read_DMA();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static uint32_t LogAccelTimer = 0;

    if( (HAL_GetTick() - LogAccelTimer > 499 ) ) //
    {
      LogAccelTimer = HAL_GetTick();

      // BLU_DbgMsgTransmit("posX %.2f posY: %.2f posZ: %.2f", 
      //                               MpuData.posX, 
      //                               MpuData.posY,
      //                               MpuData.posZ
      //                               );

      
      // BLU_DbgMsgTransmit("RecDataCnt %d ",RecMpuDataCounter);

    }

    ReportBaseImuData();
    BLU_Task();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
