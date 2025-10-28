#include "main.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE BEGIN PV */

/* USER CODE END PV */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t uartBusy = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef MPU6050_Init(void);
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
	MPU6050_Init();
  /* USER CODE BEGIN 2 */
/* USER CODE BEGIN 2 */
printf("Testing MPU6050 with blocking read...\r\n");

// 先进行阻塞读取测试
int16_t testData[3];
HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, 
                                           (uint8_t*)testData, 6, 100);

if(status == HAL_OK) {
    printf("Blocking read - X=%6d, Y=%6d, Z=%6d\r\n", testData[0], testData[1], testData[2]);
} else {
    printf("Blocking read failed: %d\r\n", status);
}

// 然后启动DMA
printf("Starting MPU6050 DMA Read...\r\n");
status = HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT,
                              (uint8_t*)rawBuf[bufIdx], SMP_LEN*2); 

printf("DMA Start Status: %d\r\n", status);
printf("I2C State: %d\r\n", HAL_I2C_GetState(&hi2c1));
/* USER CODE END 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
while (1)
{		
    if(bufFull){
        uartBusy = 1;
        
        printf("[Main] Processing buffer %d\r\n", 1-bufIdx);
        printf("[Main] Raw Data - X=%6d, Y=%6d, Z=%6d\r\n", 
               rawBuf[1-bufIdx][0], rawBuf[1-bufIdx][1], rawBuf[1-bufIdx][2]);
        
        // 手动计算RMS（简单可靠）
        int32_t sum_sq = (int32_t)rawBuf[1-bufIdx][0] * rawBuf[1-bufIdx][0] + 
                         (int32_t)rawBuf[1-bufIdx][1] * rawBuf[1-bufIdx][1] + 
                         (int32_t)rawBuf[1-bufIdx][2] * rawBuf[1-bufIdx][2];
        
        float rms = sqrtf((float)sum_sq / 3.0f);
        float rms_g = rms / 16384.0f;  // 转换为g单位
        
        printf("RMS=%.6f g\r\n", rms_g);
        
        bufFull = 0;
        uartBusy = 0;
        
    } else {
        static uint32_t noDataCount = 0;
        noDataCount++;
        if(noDataCount % 10 == 0) {
            uartBusy = 1;
            printf("[Main %lu] Waiting for data...\r\n", noDataCount);
            uartBusy = 0;
        }
    }
    HAL_Delay(1000);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c->Instance == I2C1){
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);  // LED指示
        bufFull = 1;
        bufIdx = 1 - bufIdx;
        
        // 重启DMA
        HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT,
                            (uint8_t*)&rawBuf[bufIdx][0], SMP_LEN*2);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    printf("I2C Error: %lu\n", hi2c->ErrorCode);
    // 可以在这里重启I2C或DMA
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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




HAL_StatusTypeDef MPU6050_Init(void) {
    uint8_t check, data;
    HAL_StatusTypeDef status;
    
    status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 100);
    if(status != HAL_OK) {
        printf("MPU6050 WHO_AM_I read failed: %d\r\n", status);
        return status;
    }
    if(check != 0x68) {
        printf("MPU6050 WHO_AM_I check failed: 0x%02X\r\n", check);
        return HAL_ERROR;  
    }
    printf("MPU6050 detected, WHO_AM_I=0x%02X\r\n", check);

    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 100);
    HAL_Delay(10);

    data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 100);

    data = 0x03;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, CONFIG_REG, 1, &data, 1, 100);

    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 100);

    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 100);
    printf("MPU6050 initialized successfully.\r\n");
    return HAL_OK;
}