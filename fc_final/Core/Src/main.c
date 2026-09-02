/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* LSM6DSO Registers */
#define LSM6DSO_REG_WHO_AM_I   0x0F
#define LSM6DSO_CHIP_ID        0x6C
#define LSM6DSO_REG_CTRL1_XL   0x10
#define LSM6DSO_REG_CTRL2_G    0x11
#define LSM6DSO_REG_CTRL3_C    0x12
#define LSM6DSO_REG_OUTX_L_G   0x22
#define LSM6DSO_REG_OUTX_L_A   0x28

/* BMP581 Registers */
#define BMP581_REG_CHIP_ID     0x01
#define BMP581_CHIP_ID         0x50
#define BMP581_REG_TEMP_DATA   0x1D
#define BMP581_REG_PRESS_DATA  0x20
#define BMP581_REG_OSR_CONFIG  0x36
#define BMP581_REG_ODR_CONFIG  0x37

/* BMM150 Registers */
#define BMM150_REG_CHIP_ID     0x40
#define BMM150_CHIP_ID         0x32
#define BMM150_REG_DATA_X_LSB  0x42
#define BMM150_REG_POWER       0x4B
#define BMM150_REG_OP_MODE     0x4C
#define BMM150_REG_REP_XY      0x51
#define BMM150_REG_REP_Z       0x52
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* LSM6DSO variables */
HAL_StatusTypeDef lsm6a_status;
HAL_StatusTypeDef lsm6b_status;
HAL_StatusTypeDef lsm_id_status;
uint8_t motors_armed = 0;
uint8_t lsm6dso_address = 0;
uint8_t lsm6dso_chip_id;

/* BMP581 variables */
HAL_StatusTypeDef bmp46_status;
HAL_StatusTypeDef bmp47_status;
HAL_StatusTypeDef bmp_id_status;
uint8_t bmp581_address;
uint8_t bmp581_chip_id;

/* BMM150 variables */
HAL_StatusTypeDef bmm10_status;
HAL_StatusTypeDef bmm11_status;
HAL_StatusTypeDef bmm12_status;
HAL_StatusTypeDef bmm13_status;
HAL_StatusTypeDef bmm_power_status;
HAL_StatusTypeDef bmm_id_status;
uint8_t bmm150_address;
uint8_t bmm150_chip_id;

/* Summary status */
uint8_t all_sensors_detected;

/* Live sensor measurement variables */
int16_t accel_raw_x, accel_raw_y, accel_raw_z;
int16_t gyro_raw_x, gyro_raw_y, gyro_raw_z;
int16_t mag_raw_x, mag_raw_y, mag_raw_z;

int32_t temp_raw;
uint32_t press_raw;
float press_hpa;
float temp_c;

/* UART Transmit buffer */
char uart_tx_buf[512];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
void Motor_SetPWM(float m1, float m2, float m3, float m4);
void Motor_Mixer(float throttle,
                 float roll_output,
                 float pitch_output,
                 float yaw_output);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* LSM6DSO registers */
#define LSM6DSO_WHO_AM_I      0x0F
#define LSM6DSO_CTRL1_XL      0x10
#define LSM6DSO_CTRL2_G       0x11
#define LSM6DSO_CTRL3_C       0x12

#define LSM6DSO_OUTX_L_G      0x22
#define LSM6DSO_OUTX_L_A      0x28

#define LSM6DSO_WHO_AM_I_VAL  0x6C

int16_t accel_x, accel_y, accel_z;
int16_t gyro_x, gyro_y, gyro_z;

float ax_g, ay_g, az_g;
float gx_dps, gy_dps, gz_dps;
float roll = 0.0f;
float pitch = 0.0f;

float accel_roll = 0.0f;
float accel_pitch = 0.0f;

uint32_t last_time = 0;
float dt = 0.0f;
PID_Controller roll_rate_pid =
{
    .Kp = 0.8f,
    .Ki = 0.0f,
    .Kd = 0.02f,
    .integral = 0.0f,
    .previous_error = 0.0f,
    .output = 0.0f,
    .integral_limit = 50.0f,
    .output_limit = 100.0f
};

PID_Controller pitch_rate_pid =
{
    .Kp = 0.8f,
    .Ki = 0.0f,
    .Kd = 0.02f,
    .integral = 0.0f,
    .previous_error = 0.0f,
    .output = 0.0f,
    .integral_limit = 50.0f,
    .output_limit = 100.0f
};

float desired_roll_rate = 0.0f;
float desired_pitch_rate = 0.0f;

float roll_rate_output = 0.0f;
float pitch_rate_output = 0.0f;
float throttle = 0.5f;

float yaw_output = 0.0f;

float motor1 = 0.0f;
float motor2 = 0.0f;
float motor3 = 0.0f;
float motor4 = 0.0f;

/* Find LSM6DSO */
uint8_t LSM6DSO_Detect(void)
{
    uint8_t who = 0;

    if (HAL_I2C_IsDeviceReady(&hi2c1, (0x6A << 1), 3, 100) == HAL_OK)
    {
        if (HAL_I2C_Mem_Read(&hi2c1,
                             (0x6A << 1),
                             LSM6DSO_WHO_AM_I,
                             I2C_MEMADD_SIZE_8BIT,
                             &who,
                             1,
                             100) == HAL_OK)
        {
            if (who == LSM6DSO_WHO_AM_I_VAL)
            {
                lsm6dso_address = 0x6A;
                return 1;
            }
        }
    }

    if (HAL_I2C_IsDeviceReady(&hi2c1, (0x6B << 1), 3, 100) == HAL_OK)
    {
        if (HAL_I2C_Mem_Read(&hi2c1,
                             (0x6B << 1),
                             LSM6DSO_WHO_AM_I,
                             I2C_MEMADD_SIZE_8BIT,
                             &who,
                             1,
                             100) == HAL_OK)
        {
            if (who == LSM6DSO_WHO_AM_I_VAL)
            {
                lsm6dso_address = 0x6B;
                return 1;
            }
        }
    }

    return 0;
}


/* Initialize accelerometer and gyro */
HAL_StatusTypeDef LSM6DSO_Init(void)
{
    uint8_t ctrl3 = 0x44;
    uint8_t ctrl1_xl = 0x40;
    uint8_t ctrl2_g = 0x40;

    HAL_StatusTypeDef status;

    /* CTRL3_C:
       BDU = 1
       IF_INC = 1
    */
    status = HAL_I2C_Mem_Write(&hi2c1,
                               (lsm6dso_address << 1),
                               LSM6DSO_CTRL3_C,
                               I2C_MEMADD_SIZE_8BIT,
                               &ctrl3,
                               1,
                               100);

    if (status != HAL_OK)
        return status;


    /* Accelerometer:
       ODR = 104 Hz
       Full scale = +/-2g
    */
    status = HAL_I2C_Mem_Write(&hi2c1,
                               (lsm6dso_address << 1),
                               LSM6DSO_CTRL1_XL,
                               I2C_MEMADD_SIZE_8BIT,
                               &ctrl1_xl,
                               1,
                               100);

    if (status != HAL_OK)
        return status;


    /* Gyroscope:
       ODR = 104 Hz
       Full scale = +/-250 dps
    */
    status = HAL_I2C_Mem_Write(&hi2c1,
                               (lsm6dso_address << 1),
                               LSM6DSO_CTRL2_G,
                               I2C_MEMADD_SIZE_8BIT,
                               &ctrl2_g,
                               1,
                               100);

    HAL_Delay(100);

    return status;
}


/* Read accelerometer */
HAL_StatusTypeDef LSM6DSO_ReadAccel(void)
{
    uint8_t data[6];

    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(&hi2c1,
                              (lsm6dso_address << 1),
                              LSM6DSO_OUTX_L_A,
                              I2C_MEMADD_SIZE_8BIT,
                              data,
                              6,
                              100);

    if (status != HAL_OK)
        return status;

    accel_x = (int16_t)((data[1] << 8) | data[0]);
    accel_y = (int16_t)((data[3] << 8) | data[2]);
    accel_z = (int16_t)((data[5] << 8) | data[4]);

    /* +/-2g = 0.061 mg/LSB */
    ax_g = accel_x * 0.000061f;
    ay_g = accel_y * 0.000061f;
    az_g = accel_z * 0.000061f;

    return HAL_OK;
}


/* Read gyroscope */
HAL_StatusTypeDef LSM6DSO_ReadGyro(void)
{
    uint8_t data[6];

    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(&hi2c1,
                              (lsm6dso_address << 1),
                              LSM6DSO_OUTX_L_G,
                              I2C_MEMADD_SIZE_8BIT,
                              data,
                              6,
                              100);

    if (status != HAL_OK)
        return status;

    gyro_x = (int16_t)((data[1] << 8) | data[0]);
    gyro_y = (int16_t)((data[3] << 8) | data[2]);
    gyro_z = (int16_t)((data[5] << 8) | data[4]);

    /* +/-250 dps = 8.75 mdps/LSB */
    gx_dps = gyro_x * 0.00875f;
    gy_dps = gyro_y * 0.00875f;
    gz_dps = gyro_z * 0.00875f;

    return HAL_OK;
}
void Calculate_Attitude(void)
{
    uint32_t current_time;

    /* Get elapsed time in seconds */
    current_time = HAL_GetTick();

    if (last_time == 0)
    {
        last_time = current_time;
        return;
    }

    dt = (current_time - last_time) / 1000.0f;
    last_time = current_time;

    /* Prevent abnormal dt */
    if (dt <= 0.0f || dt > 0.5f)
    {
        return;
    }

    /*
     * Accelerometer-based angles
     *
     * Roll  = rotation around X
     * Pitch = rotation around Y
     */
    accel_roll =
        atan2f(-ay_g, -az_g) * 57.2957795f;

    accel_pitch =
        atan2f(-ax_g,
               sqrtf(ay_g * ay_g + az_g * az_g))
        * 57.2957795f;

    /*
     * Gyroscope integration
     */
    roll += gx_dps * dt;
    pitch += gy_dps * dt;

    /*
     * Complementary filter
     *
     * Gyroscope:
     *   fast response, but drifts
     *
     * Accelerometer:
     *   stable long-term reference,
     *   but noisy during movement
     */
    roll =
        0.98f * roll +
        0.02f * accel_roll;

    pitch =
        0.98f * pitch +
        0.02f * accel_pitch;
}
PID_Controller roll_angle_pid =
{
    .Kp = 2.0f,
    .Ki = 0.0f,
    .Kd = 0.05f,
    .integral = 0.0f,
    .previous_error = 0.0f,
    .output = 0.0f,
    .integral_limit = 50.0f,
    .output_limit = 200.0f
};

PID_Controller pitch_angle_pid =
{
    .Kp = 2.0f,
    .Ki = 0.0f,
    .Kd = 0.05f,
    .integral = 0.0f,
    .previous_error = 0.0f,
    .output = 0.0f,
    .integral_limit = 50.0f,
    .output_limit = 200.0f
};

float PID_Update(PID_Controller *pid,
                 float target,
                 float actual,
                 float dt)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    /* Prevent division by zero or abnormal dt */
    if (dt <= 0.0001f)
    {
        return pid->output;
    }

    float error = target - actual;

    /* Integral with anti-windup clamping */
    pid->integral += error * dt;

    float int_limit = (pid->integral_limit > 0.0f) ? pid->integral_limit : 100.0f;
    if (pid->integral > int_limit)
    {
        pid->integral = int_limit;
    }
    else if (pid->integral < -int_limit)
    {
        pid->integral = -int_limit;
    }

    /* Derivative */
    float derivative = (error - pid->previous_error) / dt;
    pid->previous_error = error;

    /* PID output */
    pid->output =
        (pid->Kp * error) +
        (pid->Ki * pid->integral) +
        (pid->Kd * derivative);

    /* Limit output */
    float out_limit = (pid->output_limit > 0.0f) ? pid->output_limit : 100.0f;
    if (pid->output > out_limit)
    {
        pid->output = out_limit;
    }
    else if (pid->output < -out_limit)
    {
        pid->output = -out_limit;
    }

    return pid->output;
}

void PID_Reset(PID_Controller *pid)
{
    if (pid == NULL)
    {
        return;
    }
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->output = 0.0f;
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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000);
  char msg[128];
  int len;

  uint8_t lsm_id = 0;
  uint8_t bmp_id = 0;
  uint8_t bmm_id = 0;

  uint8_t lsm_addr = 0;
  uint8_t bmp_addr = 0;
  uint8_t bmm_addr = 0;

  HAL_Delay(100);

  /* ================================
   * LSM6DSO
   * ================================ */

  if (HAL_I2C_IsDeviceReady(&hi2c1, (0x6A << 1), 3, 100) == HAL_OK)
  {
      lsm_addr = 0x6A;
  }
  else if (HAL_I2C_IsDeviceReady(&hi2c1, (0x6B << 1), 3, 100) == HAL_OK)
  {
      lsm_addr = 0x6B;
  }

  if (lsm_addr != 0)
  {
      HAL_I2C_Mem_Read(&hi2c1,
                       (lsm_addr << 1),
                       0x0F,
                       I2C_MEMADD_SIZE_8BIT,
                       &lsm_id,
                       1,
                       100);
  }


  /* ================================
   * BMP581
   * ================================ */

  if (HAL_I2C_IsDeviceReady(&hi2c1, (0x46 << 1), 3, 100) == HAL_OK)
  {
      bmp_addr = 0x46;
  }
  else if (HAL_I2C_IsDeviceReady(&hi2c1, (0x47 << 1), 3, 100) == HAL_OK)
  {
      bmp_addr = 0x47;
  }

  if (bmp_addr != 0)
  {
      HAL_I2C_Mem_Read(&hi2c1,
                       (bmp_addr << 1),
                       0x01,
                       I2C_MEMADD_SIZE_8BIT,
                       &bmp_id,
                       1,
                       100);
  }


  /* ================================
   * BMM150
   * ================================ */

  if (HAL_I2C_IsDeviceReady(&hi2c1, (0x10 << 1), 3, 100) == HAL_OK)
  {
      bmm_addr = 0x10;
  }
  else if (HAL_I2C_IsDeviceReady(&hi2c1, (0x11 << 1), 3, 100) == HAL_OK)
  {
      bmm_addr = 0x11;
  }
  else if (HAL_I2C_IsDeviceReady(&hi2c1, (0x12 << 1), 3, 100) == HAL_OK)
  {
      bmm_addr = 0x12;
  }
  else if (HAL_I2C_IsDeviceReady(&hi2c1, (0x13 << 1), 3, 100) == HAL_OK)
  {
      bmm_addr = 0x13;
  }

  if (bmm_addr != 0)
  {
      uint8_t power = 0x01;

      HAL_I2C_Mem_Write(&hi2c1,
                        (bmm_addr << 1),
                        0x4B,
                        I2C_MEMADD_SIZE_8BIT,
                        &power,
                        1,
                        100);

      HAL_Delay(10);

      HAL_I2C_Mem_Read(&hi2c1,
                       (bmm_addr << 1),
                       0x40,
                       I2C_MEMADD_SIZE_8BIT,
                       &bmm_id,
                       1,
                       100);
  }
  lsm6dso_address = lsm_addr;
  bmp581_address = bmp_addr;
  bmm150_address = bmm_addr;

  /* Initialize LSM6DSO */
  if (lsm6dso_address != 0)
  {
      uint8_t ctrl3 = 0x44;   // BDU + IF_INC
      uint8_t ctrl1 = 0x40;   // Accelerometer: 104 Hz, ±2g
      uint8_t ctrl2 = 0x40;   // Gyroscope: 104 Hz, ±250 dps

      HAL_I2C_Mem_Write(&hi2c1,
                        (lsm6dso_address << 1),
                        LSM6DSO_REG_CTRL3_C,
                        I2C_MEMADD_SIZE_8BIT,
                        &ctrl3, 1, 100);

      HAL_I2C_Mem_Write(&hi2c1,
                        (lsm6dso_address << 1),
                        LSM6DSO_REG_CTRL1_XL,
                        I2C_MEMADD_SIZE_8BIT,
                        &ctrl1, 1, 100);

      HAL_I2C_Mem_Write(&hi2c1,
                        (lsm6dso_address << 1),
                        LSM6DSO_REG_CTRL2_G,
                        I2C_MEMADD_SIZE_8BIT,
                        &ctrl2, 1, 100);

      HAL_Delay(100);
  }
  /* ================================
   * PRINT RESULTS
   * ================================ */

  len = snprintf(msg, sizeof(msg),
                 "\r\nLSM6DSO: addr=0x%02X ID=0x%02X %s\r\n",
                 lsm_addr,
                 lsm_id,
                 (lsm_id == 0x6C) ? "OK" : "FAIL");

  HAL_UART_Transmit(&huart2,
                    (uint8_t *)msg,
                    len,
                    1000);

  len = snprintf(msg, sizeof(msg),
                 "BMP581 : addr=0x%02X ID=0x%02X %s\r\n",
                 bmp_addr,
                 bmp_id,
                 (bmp_id == 0x50) ? "OK" : "FAIL");

  HAL_UART_Transmit(&huart2,
                    (uint8_t *)msg,
                    len,
                    1000);

  len = snprintf(msg, sizeof(msg),
                 "BMM150 : addr=0x%02X ID=0x%02X %s\r\n",
                 bmm_addr,
                 bmm_id,
                 (bmm_id == 0x32) ? "OK" : "FAIL");

  HAL_UART_Transmit(&huart2,
                    (uint8_t *)msg,
                    len,
                    1000);

  len = snprintf(msg, sizeof(msg),
                 "-----------------------------\r\n");

  HAL_UART_Transmit(&huart2,
                    (uint8_t *)msg,
                    len,
                    1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      if (lsm6dso_address != 0)
      {
          LSM6DSO_ReadAccel();
          LSM6DSO_ReadGyro();

          Calculate_Attitude();
          float roll_target = 0.0f;
          float pitch_target = 0.0f;

          /* Cascade PID:
           * 1) Angle PID (outer loop): angle error -> desired angular rate
           * 2) Gyro Rate PID (inner loop): rate error -> PID rate correction
           */
          desired_roll_rate =
              PID_Update(&roll_angle_pid,
                         roll_target,
                         roll,
                         dt);

          desired_pitch_rate =
              PID_Update(&pitch_angle_pid,
                         pitch_target,
                         pitch,
                         dt);

          roll_rate_output =
              PID_Update(&roll_rate_pid,
                         desired_roll_rate,
                         gx_dps,
                         dt);

          pitch_rate_output =
              PID_Update(&pitch_rate_pid,
                         desired_pitch_rate,
                         gy_dps,
                         dt);
          Motor_Mixer(throttle,
                      roll_rate_output / 100.0f,
                      pitch_rate_output / 100.0f,
                      yaw_output);
          if (motors_armed)
          {
              Motor_SetPWM(motor1, motor2, motor3, motor4);
          }
          else
          {
              Motor_SetPWM(0.0f, 0.0f, 0.0f, 0.0f);
          }

          float roll_correction = roll_rate_output;
          float pitch_correction = pitch_rate_output;


          snprintf(uart_tx_buf, sizeof(uart_tx_buf),
                   "R:%.1f P:%.1f | "
                   "M1:%.3f M2:%.3f M3:%.3f M4:%.3f\r\n",
                   roll,
                   pitch,
                   motor1,
                   motor2,
                   motor3,
                   motor4);

          HAL_UART_Transmit(&huart2,
                            (uint8_t *)uart_tx_buf,
                            strlen(uart_tx_buf),
                            1000);
      }

      HAL_Delay(10);
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Motor_SetPWM(float m1, float m2, float m3, float m4)
{
    uint16_t pwm1 = 1000 + (uint16_t)(m1 * 1000.0f);
    uint16_t pwm2 = 1000 + (uint16_t)(m2 * 1000.0f);
    uint16_t pwm3 = 1000 + (uint16_t)(m3 * 1000.0f);
    uint16_t pwm4 = 1000 + (uint16_t)(m4 * 1000.0f);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm3);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm4);
}
void Motor_Mixer(float throttle,
                 float roll_output,
                 float pitch_output,
                 float yaw_output)
{
    motor1 = throttle + roll_output + pitch_output - yaw_output;
    motor2 = throttle - roll_output + pitch_output + yaw_output;
    motor3 = throttle - roll_output - pitch_output - yaw_output;
    motor4 = throttle + roll_output - pitch_output + yaw_output;

    if (motor1 > 1.0f) motor1 = 1.0f;
    if (motor1 < 0.0f) motor1 = 0.0f;

    if (motor2 > 1.0f) motor2 = 1.0f;
    if (motor2 < 0.0f) motor2 = 0.0f;

    if (motor3 > 1.0f) motor3 = 1.0f;
    if (motor3 < 0.0f) motor3 = 0.0f;

    if (motor4 > 1.0f) motor4 = 1.0f;
    if (motor4 < 0.0f) motor4 = 0.0f;
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
#ifdef USE_FULL_ASSERT
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
