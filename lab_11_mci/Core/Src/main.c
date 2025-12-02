/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f3xx_hal_spi.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
#define LSM_A_ADDR_W   (0x32)   // write address (8-bit as per your note)
#define LSM_A_ADDR_R   (0x33)   // read address (8-bit)
#define LSM_A_CTRL1    0x20
#define LSM_A_CTRL4    0x23
#define LSM_A_OUT_X_L  0x28
#define LSM_A_AUTO_INC 0x80
#define LSM_A_G_PER_LSB 0.0039f  // 3.9 mg/LSB

#define GYRO_REG_WHOAMI   0x0F
#define GYRO_REG_CTRL1    0x20
#define GYRO_REG_CTRL4    0x23
#define GYRO_REG_OUT_X_L  0x28
#define GYRO_SPI_READ     0x80
#define GYRO_SPI_AUTO_INC 0x40
#define GYRO_CS_LOW()   HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin, GPIO_PIN_RESET)
#define GYRO_CS_HIGH()  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin, GPIO_PIN_SET)

#define GYRO_CS_LOW()    HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin, GPIO_PIN_RESET)
#define GYRO_CS_HIGH()   HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin, GPIO_PIN_SET)

#define GYRO_REG_CTRL1   0x20
#define GYRO_REG_CTRL3   0x22
#define GYRO_REG_CTRL4   0x23
#define GYRO_REG_OUT_X_L 0x28
#define GYRO_SPI_READ     0x80
#define GYRO_SPI_AUTO_INC 0x40
#define GYRO_SENS_245DPS 0.00875f
/* ===== FILTER + PID GLOBALS ===== */
#define DT 0.01f       // 100 Hz loop
#define ALPHA 0.98f    // complementary filter coefficient

float angle_filtered = 0.0f;
float accel_angle = 0.0f;
float gyro_rate = 0.0f;

/* PID gains */
float Kp = 20.0f;
float Ki = 0.8f;
float Kd = 1.2f;

float pid_integral = 0.0f;
float last_error = 0.0f;

float setpoint = 0.5f;   // robot upright position
// Motor pins (your wiring)
#define MOTOR_IN1_GPIO_PORT   GPIOA   // D8 -> PA9
#define MOTOR_IN1_PIN         GPIO_PIN_9

#define MOTOR_IN2_GPIO_PORT   GPIOA   // D12 -> PA10
#define MOTOR_IN2_PIN         GPIO_PIN_10

#define MOTOR_EN_GPIO_PORT    GPIOC   // D10 -> PC6
#define MOTOR_EN_PIN          GPIO_PIN_6


#define MOTOR2_IN1_GPIO_PORT  GPIOB   // D7 -> PB4
#define MOTOR2_IN1_PIN        GPIO_PIN_4
#define MOTOR2_IN2_GPIO_PORT  GPIOB   // D6 -> PB5
#define MOTOR2_IN2_PIN        GPIO_PIN_5
#define MOTOR2_EN_GPIO_PORT   GPIOC   // D9 -> PC7 (TIM3_CH2)
#define MOTOR2_EN_PIN         GPIO_PIN_7


/* simple accel structure */
typedef struct { float ax, ay, az; } accel_t;

accel_t accel_data;
/* USER CODE BEGIN PV */
volatile uint8_t uart_flag = 0;   // Flag set by timer ISR
volatile uint8_t counter = 0;     // Counts timer interrupts
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef LSM_Accel_Init(void);
HAL_StatusTypeDef LSM_Accel_Read(accel_t *m);


// gyro prototypes
uint8_t gyro_read_u8(uint8_t reg);
void gyro_write_u8(uint8_t reg, uint8_t val);
void gyro_init_basic(void);
void motor_forward(void);
void motor_backward(void);
void motor_stop(void);


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
  // setpoint = angle_filtered;   // set current angle as zero


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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);   // Motor 1 EN (PC6)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);   // Motor 2 EN (PC7)

  // Fixed ~60% duty for both motors
  // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 600);
  // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 600);


  /* USER CODE BEGIN 2 */
if (HAL_I2C_IsDeviceReady(&hi2c1, LSM_A_ADDR_W, 3, 50) == HAL_OK) {
    LSM_Accel_Init();
    printf("Accel initialized\r\n");
} else {
    printf("Accel not present (I2C)\r\n");
}

gyro_init_basic();
printf("Gyro basic init done (WHOAMI=0x%02X)\r\n", gyro_read_u8(GYRO_REG_WHOAMI));

HAL_TIM_Base_Start_IT(&htim2);   // Start TIM2 interrupt at 100Hz
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4799;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//     if (htim->Instance == TIM2)
//     {
//         HAL_GPIO_TogglePin(GPIOE, LD6_Pin);  // toggles every 1/100 sec
//     }
// }
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//     if(htim->Instance == TIM2)
//     {
//         counter++;               // increment counter each ISR
//         if(counter >= 10)        // for 10 Hz if timer is 100 Hz
//         {
//             uart_flag = 1;       // set flag for main loop
//             counter = 0;         // reset counter
//         }
//     }
// }
/* USER CODE BEGIN 4 */
/* I2C helpers already available via HAL, so use HAL_I2C_Mem_Write/Read */

/* Initialize LSM303AGR accelerometer:
 * CTRL1_A = 0x67 (ODR 200 Hz + all axes on -> per your note)
 * CTRL4_A = 0x00 (±2g normal mode)
 */
HAL_StatusTypeDef LSM_Accel_Init(void)
{
    uint8_t v;
    HAL_StatusTypeDef st;

    v = 0x67; // CTRL1_A: 0x67 => ODR=200Hz, all axes on, normal mode
    st = HAL_I2C_Mem_Write(&hi2c1, LSM_A_ADDR_W, LSM_A_CTRL1, I2C_MEMADD_SIZE_8BIT, &v, 1, 100);
    if (st != HAL_OK) return st;

    v = 0x00; // CTRL4_A: ±2g normal mode, continuous update
    st = HAL_I2C_Mem_Write(&hi2c1, LSM_A_ADDR_W, LSM_A_CTRL4, I2C_MEMADD_SIZE_8BIT, &v, 1, 100);
    if (st != HAL_OK) return st;

    HAL_Delay(10);
    return HAL_OK;
}

/* Read accelerometer, convert to g.
 * Note: device returns left-justified 16-bit values for normal mode (10-bit useful data)
 * we right-shift by 6 to get 10-bit values, then multiply by 0.0039 g/LSB.
 */
HAL_StatusTypeDef LSM_Accel_Read(accel_t *m)
{
    uint8_t buf[6];
    HAL_StatusTypeDef st;

    st = HAL_I2C_Mem_Read(&hi2c1, LSM_A_ADDR_R, (LSM_A_OUT_X_L | LSM_A_AUTO_INC),
                          I2C_MEMADD_SIZE_8BIT, buf, 6, 100);
    if (st != HAL_OK) return st;

    int16_t rx = (int16_t)((buf[1] << 8) | buf[0]); // note order: low then high
    int16_t ry = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t rz = (int16_t)((buf[5] << 8) | buf[4]);

    // right shift by 6 to convert 16-bit left-justified to 10-bit value
    rx = rx >> 6;
    ry = ry >> 6;
    rz = rz >> 6;

    m->ax = ((float)rx) * LSM_A_G_PER_LSB;
    m->ay = ((float)ry) * LSM_A_G_PER_LSB;
    m->az = ((float)rz) * LSM_A_G_PER_LSB;

    return HAL_OK;
}
/* SPI gyro helpers for L3GD20 / I3G4250D style devices */
/* change these registers if your gyro uses different addresses */

// extern void GYRO_CS_LOW(void);  // if you have these macros (from earlier code)
// extern void GYRO_CS_HIGH(void);

 uint8_t gyro_read_u8(uint8_t reg)
{
    uint8_t header = (reg & 0x3F) | GYRO_SPI_READ;
    uint8_t v = 0;
    GYRO_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &header, 1, 100);
    HAL_SPI_Receive(&hspi1, &v, 1, 100);
    GYRO_CS_HIGH();
    return v;
}

 void gyro_write_u8(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg & 0x3F, val };
    GYRO_CS_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 2, 100);
    GYRO_CS_HIGH();
}

 void gyro_init_basic(void)
{
    uint8_t who = gyro_read_u8(GYRO_REG_WHOAMI);
    printf("WHOAMI = 0x%02X\r\n", who);

    // CTRL1: ODR = 400Hz, BW=110, enable XYZ, power on
    gyro_write_u8(GYRO_REG_CTRL1, 0x4F);

    // CTRL4: ±245 dps
    gyro_write_u8(GYRO_REG_CTRL4, 0x00);

    HAL_Delay(20);
}
void gyro_read_xyz(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t reg = GYRO_REG_OUT_X_L | GYRO_SPI_READ | GYRO_SPI_AUTO_INC;
    uint8_t buf[6];

    GYRO_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &reg, 1, 100);
    HAL_SPI_Receive(&hspi1, buf, 6, 100);
    GYRO_CS_HIGH();

    *gx = (int16_t)(buf[1] << 8 | buf[0]);
    *gy = (int16_t)(buf[3] << 8 | buf[2]);
    *gz = (int16_t)(buf[5] << 8 | buf[4]);
}
void motor_forward(void)
{
    // MOTOR 1: D8 HIGH, D12 LOW, EN HIGH
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_PORT, MOTOR_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(MOTOR_EN_GPIO_PORT, MOTOR_EN_PIN, GPIO_PIN_SET);

    // MOTOR 2: D7 HIGH, D6 LOW, EN HIGH
    HAL_GPIO_WritePin(MOTOR2_IN1_GPIO_PORT, MOTOR2_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR2_IN2_GPIO_PORT, MOTOR2_IN2_PIN, GPIO_PIN_RESET);
    // HAL_GPIO_WritePin(MOTOR2_EN_GPIO_PORT, MOTOR2_EN_PIN, GPIO_PIN_SET);
}

void motor_backward(void)
{
    // MOTOR 1: D8 LOW, D12 HIGH, EN HIGH
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_PORT, MOTOR_IN2_PIN, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(MOTOR_EN_GPIO_PORT, MOTOR_EN_PIN, GPIO_PIN_SET);

    // MOTOR 2: D7 LOW, D6 HIGH, EN HIGH
    HAL_GPIO_WritePin(MOTOR2_IN1_GPIO_PORT, MOTOR2_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR2_IN2_GPIO_PORT, MOTOR2_IN2_PIN, GPIO_PIN_SET);
    // HAL_GPIO_WritePin(MOTOR2_EN_GPIO_PORT, MOTOR2_EN_PIN, GPIO_PIN_SET);
}

void motor_stop(void)
{
    // MOTOR 1 off
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_EN_GPIO_PORT, MOTOR_EN_PIN, GPIO_PIN_RESET);

    // MOTOR 2 off
    HAL_GPIO_WritePin(MOTOR2_IN1_GPIO_PORT, MOTOR2_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR2_IN2_GPIO_PORT, MOTOR2_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR2_EN_GPIO_PORT, MOTOR2_EN_PIN, GPIO_PIN_RESET);
}


/* USER CODE BEGIN 4 */
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//     if (htim->Instance == TIM2)
//     {
//         // toggle LED so you can probe the pin at 100Hz
//         HAL_GPIO_TogglePin(GPIOE, LD6_Pin);

//         // read accel at 100 Hz
//         if (LSM_Accel_Read(&accel_data) == HAL_OK)
//         {
//             // compute tilt angle around X or Y axis. Example: pitch ≈ atan2(ax, az)
//             float angle_rad = atan2f(accel_data.ax, accel_data.az); // change axes as needed
//             float angle_deg = angle_rad * 180.0f / (float)M_PI;

//             // send via UART (quick print)
//             char buf[64];
//             int n = snprintf(buf, sizeof(buf), "ax=%.3fg ay=%.3fg az=%.3fg angle=%.2f\r\n",
//                              accel_data.ax, accel_data.ay, accel_data.az, angle_deg);
//             HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, HAL_MAX_DELAY);
//         }
//     }
// }

// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//     if(htim->Instance == TIM2)
//     {
//         counter++;
//         if(counter >= 10)  // 10Hz
//         {
//             counter = 0;

//             int16_t gx_raw, gy_raw, gz_raw;
//             gyro_read_xyz(&gx_raw, &gy_raw, &gz_raw);

//             float gx = gx_raw * GYRO_SENS_245DPS;
//             float gy = gy_raw * GYRO_SENS_245DPS;
//             float gz = gz_raw * GYRO_SENS_245DPS;

//             char buf[64];
//             int n = snprintf(buf, sizeof(buf),
//                              "Gx=%.2f dps  Gy=%.2f dps  Gz=%.2f dps\r\n",
//                              gx, gy, gz);
//             HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, HAL_MAX_DELAY);
//         }
//     }
// }
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        static uint8_t print_div = 0;   // for 10 Hz printing

        HAL_GPIO_TogglePin(GPIOE, LD6_Pin);   // 100 Hz LED toggle

        // ------------------ 1) ACCEL ------------------
        if (LSM_Accel_Read(&accel_data) == HAL_OK)
        {
            // YOU SAID BOTH ARE Y-AXIS BASED, so use ay
            accel_angle = atan2f(accel_data.ay, accel_data.az) * 180.0f / M_PI;
            static int first_init = 1;
            if (first_init) {
            angle_filtered = accel_angle;  // initialize filter with accel angle
              first_init = 0;
}

        }

        // ------------------ 2) GYRO ------------------
        int16_t gx_raw, gy_raw, gz_raw;
        gyro_read_xyz(&gx_raw, &gy_raw, &gz_raw);

        gyro_rate = gy_raw * GYRO_SENS_245DPS;   // you said Y axis


        // ------------------ 3) COMPLEMENTARY FILTER ------------------
        angle_filtered =
            ALPHA * (angle_filtered + gyro_rate * DT)
          + (1.0f - ALPHA) * accel_angle;


        // ------------------ 4) ANGLE ERROR ------------------
        // your upright is around 0.5° — so we set that as target
        float setpoint = -1.0f;   // you can adjust later
float error = setpoint - angle_filtered;


        // angle deadband for STOP condition
        float deadband_angle = 0.5f;

        // if angle is between 0° and 1° → STOP + reset PID
        if (fabsf(error) < deadband_angle)
        {
            motor_stop();
            pid_integral = 0.0f;
            last_error = 0.0f;
        }
        else
        {
            // clamp angle error to ±8 degrees (your request)
            float max_angle_error = 8.0f;
            if (error >  max_angle_error) error =  max_angle_error;
            if (error < -max_angle_error) error = -max_angle_error;

            // ---- PID ----
            pid_integral += error * DT;
            if (pid_integral > 20)  pid_integral = 20;
if (pid_integral < -20) pid_integral = -20;


            float derivative = (error - last_error) / DT;
            float u = (Kp*error + Ki*pid_integral + Kd*derivative) * 30.0f;
            last_error = error;

            // ------------------ 5) MOTOR DIR ------------------
            float u_abs = fabsf(u);
if (u_abs > 999) u_abs = 999;
uint16_t pwm = (uint16_t)u_abs;

// DIRECTION
if (u > 0)
{
    // forward
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_PORT, MOTOR_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_PORT, MOTOR_IN2_PIN, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(MOTOR2_IN1_GPIO_PORT, MOTOR2_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR2_IN2_GPIO_PORT, MOTOR2_IN2_PIN, GPIO_PIN_RESET);
}
else
{
    // backward
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_PORT, MOTOR_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_PORT, MOTOR_IN2_PIN, GPIO_PIN_SET);

    HAL_GPIO_WritePin(MOTOR2_IN1_GPIO_PORT, MOTOR2_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR2_IN2_GPIO_PORT, MOTOR2_IN2_PIN, GPIO_PIN_SET);
}

// SPEED (PWM)
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm);

        }


        // ------------------ 6) PRINT at 10 Hz ------------------
        print_div++;
        if (print_div >= 10)
        {
            print_div = 0;
            char buf[96];
            int n = snprintf(buf, sizeof(buf),
                "angF=%.2f  acc=%.2f  gyro=%.2f  err=%.2f\r\n",
                angle_filtered, accel_angle, gyro_rate, error);
            HAL_UART_Transmit(&huart1, (uint8_t*)buf, n, HAL_MAX_DELAY);
        }
    }
}



int _write(int file, char *data, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)data, len, HAL_MAX_DELAY);
    return len;
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