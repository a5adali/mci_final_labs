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
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Private define ------------------------------------------------------------*/
#define GYRO_CS_GPIO_Port   GPIOE
#define GYRO_CS_Pin         CS_I2C_SPI_Pin
#define GYRO_CS_LOW()       HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET)
#define GYRO_CS_HIGH()      HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET)

/* L3GD20 registers and constants */
#define GYRO_REG_WHOAMI   0x0F
#define GYRO_REG_CTRL1    0x20
#define GYRO_REG_CTRL4    0x23
#define GYRO_REG_OUT_X_L  0x28
#define GYRO_SPI_READ     0x80
#define GYRO_SPI_AUTO_INC 0x40
#define GYRO_SENS_250DPS  0.00875f

/* LSM303AGR accelerometer */
#define LSM_A_ADDR_8        (0x19u << 1)   // 0x32
#define LSM_A_CTRL1         0x20
#define LSM_A_CTRL4         0x23
#define LSM_A_OUT_X_L       0x28
#define LSM_A_AUTO_INC      0x80
#define LSM_A_G_PER_LSB     0.0040f

/* Complementary filter alpha */
#define CF_ALPHA 0.98f

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;
PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
/* Shared volatile state between ISR and main */
volatile float gyro_rate_x = 0.0f;    // latest gyro X rate (°/s) — updated in main
volatile float accel_angle_x = 0.0f;  // latest accel-derived pitch (deg) — updated in main
volatile float comp_angle_x = 0.0f;   // complementary filtered angle in deg — updated in ISR
volatile uint8_t flag_10hz_tasks = 0; // set by ISR every 10 ticks

/* Gyro bias (calibrated at startup) */
float gyro_bias_x = 0.0f;

/* sensor structs */
typedef struct { float ax, ay, az; float offx, offy, offz; } lsm_acc_t;
typedef struct { float gx, gy, gz; } gyro_t;

/* USER CODE END PV */


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* helper prototypes */
static void uart_print(const char *s);
static inline int16_t s16(uint8_t lo, uint8_t hi);
static void gyro_write_u8(uint8_t reg, uint8_t val);
static void gyro_read_n(uint8_t reg, uint8_t *buf, uint16_t n);
static uint8_t gyro_read_u8(uint8_t reg);
static void gyro_init(void);
static void gyro_read(gyro_t *g);
static float gyro_calibrate_bias_x(int samples, int delay_ms);

static HAL_StatusTypeDef i2c_write8(uint16_t dev8, uint8_t reg, uint8_t val);
static HAL_StatusTypeDef i2c_readn(uint16_t dev8, uint8_t reg, uint8_t *buf, uint16_t n);
static void LSM_Accel_Init(void);
static HAL_StatusTypeDef LSM_Accel_Read(lsm_acc_t *m);
static void LSM_Accel_Calibrate(lsm_acc_t *m, int N);

static void dps_to_text(char *dst, size_t n, float dps);
static void g_to_text(char *dst, size_t n, float g);
static void deg_to_text(char *dst, size_t n, float deg);
static void print_xyz_csv(float ax, float ay, float az);
static void print_gyro_csv(float gx, float gy, float gz);

static float accel_pitch_from_xyz(float ax, float ay, float az);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* UART print helper (non-blocking not used here) */
static void uart_print(const char *s) {
  HAL_UART_Transmit(&huart1, (uint8_t*)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}

/* combine bytes little-endian */
static inline int16_t s16(uint8_t lo, uint8_t hi) {
  return (int16_t)((hi << 8) | lo);
}

/* SPI helpers for gyro (L3GD20) */
static void gyro_write_u8(uint8_t reg, uint8_t val) {
  uint8_t tx[2] = { reg & 0x3F, val }; // write: bit7=0, bit6=0
  GYRO_CS_LOW();
  HAL_SPI_Transmit(&hspi1, tx, 2, 100);
  GYRO_CS_HIGH();
}

static void gyro_read_n(uint8_t reg, uint8_t *buf, uint16_t n) {
  uint8_t header = (reg & 0x3F) | GYRO_SPI_READ | (n>1 ? GYRO_SPI_AUTO_INC : 0);
  GYRO_CS_LOW();
  HAL_SPI_Transmit(&hspi1, &header, 1, 100);
  HAL_SPI_Receive(&hspi1, buf, n, 100);
  GYRO_CS_HIGH();
}

static uint8_t gyro_read_u8(uint8_t reg) {
  uint8_t v=0; gyro_read_n(reg, &v, 1); return v;
}

/* Bring gyro up: normal mode, 100 Hz ODR & ±250 dps */
static void gyro_init(void) {
  /* CTRL1: ODR=95/100Hz & enable X/Y/Z + power on -> 0x0F is common start */
  gyro_write_u8(GYRO_REG_CTRL1, 0x0F);
  /* CTRL4: scale = ±250 dps (FS bits = 00), continuous update -> 0x00 */
  gyro_write_u8(GYRO_REG_CTRL4, 0x00);
  HAL_Delay(10);
}

/* Read gyro X/Y/Z, convert to °/s */
static void gyro_read(gyro_t *g) {
  uint8_t b[6];
  gyro_read_n(GYRO_REG_OUT_X_L, b, 6);  // auto-increment reads X/Y/Z L/H
  int16_t x = s16(b[0], b[1]);
  int16_t y = s16(b[2], b[3]);
  int16_t z = s16(b[4], b[5]);
  g->gx = x * GYRO_SENS_250DPS;
  g->gy = y * GYRO_SENS_250DPS;
  g->gz = z * GYRO_SENS_250DPS;
}

/* Calibrate gyro X bias by averaging N samples (blocking) */
static float gyro_calibrate_bias_x(int samples, int delay_ms){
  gyro_t g; float sum = 0.0f;
  for(int i=0;i<samples;i++){
    gyro_read(&g);
    sum += g.gx;
    HAL_Delay(delay_ms);
  }
  return sum / samples;
}

/* I2C wrappers for accel */
static HAL_StatusTypeDef i2c_write8(uint16_t dev8, uint8_t reg, uint8_t val) {
  return HAL_I2C_Mem_Write(&hi2c1, dev8, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}
static HAL_StatusTypeDef i2c_readn(uint16_t dev8, uint8_t reg, uint8_t *buf, uint16_t n) {
  return HAL_I2C_Mem_Read(&hi2c1, dev8, reg, I2C_MEMADD_SIZE_8BIT, buf, n, 100);
}

/* Initialize accelerometer (100 Hz, ±2g normal mode) */
static void LSM_Accel_Init(void) {
  i2c_write8(LSM_A_ADDR_8, LSM_A_CTRL1, 0x67); // ODR=100Hz, all axes enabled
  i2c_write8(LSM_A_ADDR_8, LSM_A_CTRL4, 0x00); // ±2g, normal mode
  HAL_Delay(10);
}

/* Read accel raw, convert to g, subtract offsets */
static HAL_StatusTypeDef LSM_Accel_Read(lsm_acc_t *m) {
  uint8_t b[6];
  HAL_StatusTypeDef st = i2c_readn(LSM_A_ADDR_8, (LSM_A_OUT_X_L | LSM_A_AUTO_INC), b, 6);
  if (st != HAL_OK) return st;
  int16_t rx = s16(b[0], b[1]);
  int16_t ry = s16(b[2], b[3]);
  int16_t rz = s16(b[4], b[5]);
  m->ax = rx * LSM_A_G_PER_LSB - m->offx;
  m->ay = ry * LSM_A_G_PER_LSB - m->offy;
  m->az = rz * LSM_A_G_PER_LSB - m->offz;
  return HAL_OK;
}

/* Simple accel offset calibration: average N samples while still/flat */
static void LSM_Accel_Calibrate(lsm_acc_t *m, int N) {
  float sx=0, sy=0, sz=0;
  lsm_acc_t t = {0};
  for (int i=0; i<N; i++) {
    if (LSM_Accel_Read(&t) == HAL_OK) {
      sx += t.ax; sy += t.ay; sz += t.az;
    }
    HAL_Delay(10);
  }
  m->offx = sx / N;
  m->offy = sy / N;
  m->offz = sz / N;
}

/* formatting helpers (integer-only style) */
static void dps_to_text(char *dst, size_t n, float dps){
  int32_t mdps = (int32_t)((dps>=0)?(dps*1000.0f+0.5f):(dps*1000.0f-0.5f));
  int32_t s = (mdps<0); if(s) mdps=-mdps;
  int32_t i = mdps/1000, f = mdps%1000;
  if(s) snprintf(dst,n,"-%ld.%03ld",(long)i,(long)f);
  else  snprintf(dst,n, "%ld.%03ld",(long)i,(long)f);
}
static void g_to_text(char *dst, size_t n, float g) {
  int32_t mg = (int32_t)( (g >= 0.0f) ? (g*1000.0f + 0.5f) : (g*1000.0f - 0.5f) );
  int32_t s = (mg < 0);
  if (s) mg = -mg;
  int32_t i = mg / 1000;
  int32_t f = mg % 1000;
  if (s)
    snprintf(dst, n, "-%ld.%03ld", (long)i, (long)f);
  else
    snprintf(dst, n,  "%ld.%03ld", (long)i, (long)f);
}
static void deg_to_text(char *dst, size_t n, float deg){
  int32_t mdeg = (int32_t)((deg>=0)?(deg*1000.0f+0.5f):(deg*1000.0f-0.5f));
  int32_t s = (mdeg<0); if(s) mdeg=-mdeg;
  int32_t i = mdeg/1000, f = mdeg%1000;
  if(s) snprintf(dst,n,"-%ld.%03ld",(long)i,(long)f);
  else  snprintf(dst,n, "%ld.%03ld",(long)i,(long)f);
}

static void print_xyz_csv(float ax,float ay,float az){
  char a[16], b[16], c[16], line[64];
  g_to_text(a, sizeof(a), ax);
  g_to_text(b, sizeof(b), ay);
  g_to_text(c, sizeof(c), az);
  snprintf(line, sizeof(line), "%s, %s, %s\r\n", a, b, c);
  uart_print(line);
}
static void print_gyro_csv(float gx,float gy,float gz){
  char ax[16], ay[16], az[16], line[64];
  dps_to_text(ax,sizeof ax,gx);
  dps_to_text(ay,sizeof ay,gy);
  dps_to_text(az,sizeof az,gz);
  snprintf(line,sizeof line,"%s, %s, %s\r\n",ax,ay,az);
  uart_print(line);
}

/* Accel pitch estimate in degrees: pitch = atan2(-ax, sqrt(ay^2 + az^2)) */
static float accel_pitch_from_xyz(float ax, float ay, float az) {
  float denom = sqrtf(ay*ay + az*az);
  float pr = atan2f(-ax, denom);
  return pr * 57.295779513f;
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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */
  /* Ensure gyro CS idle HIGH (chip select inactive) */
  GYRO_CS_HIGH();
  LSM_Accel_Init();
  gyro_init();
  /* Calibrate accel offsets (optional) */
  lsm_acc_t acc_cal = {0};
  LSM_Accel_Calibrate(&acc_cal, 20); // blocking ~0.2s
  /* use calibrated offsets as baseline for reads */
  lsm_acc_t acc_store = {0};
  acc_store.offx = acc_cal.offx;
  acc_store.offy = acc_cal.offy;
  acc_store.offz = acc_cal.offz;

  /* Calibrate gyro bias (X axis) ~2 seconds */
  // uart_print("Calibrating gyro bias... keep board still (~2s)\r\n");
  // gyro_bias_x = gyro_calibrate_bias_x(200, 10);
  // {
  //   char buf[64]; int n = snprintf(buf, sizeof(buf), "Gyro bias X = %.4f dps\r\n", (double)gyro_bias_x);
  //   if (n>0) uart_print(buf);
  // }

  /* Read accel once to prime accel_angle_x and comp_angle_x */
  if (LSM_Accel_Read(&acc_store) == HAL_OK) {
    accel_angle_x = accel_pitch_from_xyz(acc_store.ax, acc_store.ay, acc_store.az);
    comp_angle_x = accel_angle_x;
  }

  /* Start TIM2 interrupt (100 Hz) */
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (flag_10hz_tasks) {
    flag_10hz_tasks = 0;

    /* Task 2: LED toggle */
    HAL_GPIO_TogglePin(GPIOE, LD4_Pin);

    /* Task 3: Read accelerometer via I2C */
    lsm_acc_t acc = acc_store; // copy offsets
    if (LSM_Accel_Read(&acc) == HAL_OK) {
        float ax = acc.ax;
        float ay = acc.ay;
        float az = acc.az;

        /* Send values over UART in CSV format */
        print_xyz_csv(ax, ay, az);
    } else {
        uart_print("Accel read ERR\r\n");
    }
} 
    }

    /* small sleep to reduce CPU burn (don't delay long) */
    HAL_Delay(1);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */


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
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;         // 8-bit
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;     // CPOL = 1
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;          // CPHA = 2EDGE
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; // moderate speed
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

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) {
    static uint8_t tick = 0;
    tick++;

    /* 1) Heartbeat at 100Hz */
    HAL_GPIO_TogglePin(GPIOE, LD3_Pin);

    /* 2) Complementary filter update (dt = 0.01s) */
    const float dt = 0.01f;
    float g_rate = gyro_rate_x;    // deg/s (volatile read)
    float a_angle = accel_angle_x; // deg (volatile read)
    float gyro_est = comp_angle_x + g_rate * dt;
    comp_angle_x = CF_ALPHA * gyro_est + (1.0f - CF_ALPHA) * a_angle;

    /* 3) every 10 ticks -> request 10Hz tasks (set by ISR) */
    if ((tick % 10) == 0) flag_10hz_tasks = 1;

    if (tick >= 100) tick = 0;
  }
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
