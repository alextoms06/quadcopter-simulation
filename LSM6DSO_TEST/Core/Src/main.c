/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : LSM6DSO I2C device detection test
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and SysTick */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();

    /* ---------------------------------------------------------
       LSM6DSO I2C ADDRESS TEST
       --------------------------------------------------------- */

    HAL_StatusTypeDef addr6A;
    HAL_StatusTypeDef addr6B;

    /*
     * STM32 HAL expects the 7-bit I2C address shifted left by 1.
     *
     * LSM6DSO possible addresses:
     *     0x6A
     *     0x6B
     */

    addr6A = HAL_I2C_IsDeviceReady(
        &hi2c1,
        (0x6A << 1),
        3,
        100
    );

    addr6B = HAL_I2C_IsDeviceReady(
        &hi2c1,
        (0x6B << 1),
        3,
        100
    );

    /*
     * LED indication:
     *
     * LED ON  = LSM6DSO detected at 0x6A or 0x6B
     * LED OFF = sensor not detected
     */

    if ((addr6A == HAL_OK) || (addr6B == HAL_OK))
    {
        HAL_GPIO_WritePin(
            LD2_GPIO_Port,
            LD2_Pin,
            GPIO_PIN_SET
        );
    }
    else
    {
        HAL_GPIO_WritePin(
            LD2_GPIO_Port,
            LD2_Pin,
            GPIO_PIN_RESET
        );
    }

    uint8_t who_am_i_6A = 0;
    uint8_t who_am_i_6B = 0;

    HAL_I2C_Mem_Read(
        &hi2c1,
        (0x6A << 1),
        0x0F,
        I2C_MEMADD_SIZE_8BIT,
        &who_am_i_6A,
        1,
        100
    );

    HAL_I2C_Mem_Read(
        &hi2c1,
        (0x6B << 1),
        0x0F,
        I2C_MEMADD_SIZE_8BIT,
        &who_am_i_6B,
        1,
        100
    );

    /* Infinite loop */
    while (1)
    {
        /*
         * Keep the program running so that you can inspect
         * addr6A and addr6B in the debugger.
         */

        HAL_Delay(100);
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Configure the main internal regulator output voltage */
    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /* Initializes the RCC Oscillators */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;

    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* Initializes CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief I2C1 Initialization Function
  * @retval None
  */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;

    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;

    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;

    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;

    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief USART2 Initialization Function
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;

    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;

    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief GPIO Initialization Function
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure LD2 LED output level */
    HAL_GPIO_WritePin(
        LD2_GPIO_Port,
        LD2_Pin,
        GPIO_PIN_RESET
    );

    /* Configure B1 User Button */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    HAL_GPIO_Init(
        B1_GPIO_Port,
        &GPIO_InitStruct
    );

    /* Configure LD2 LED */
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        LD2_GPIO_Port,
        &GPIO_InitStruct
    );
}

/**
  * @brief  Error Handler
  * @retval None
  */
void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
        /* Error state */
    }
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and line number
  * @param  file: source file name
  * @param  line: line number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* Add implementation if required */
    /* USER CODE END 6 */
}

#endif /* USE_FULL_ASSERT */
