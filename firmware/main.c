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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c.h"
#include "../Usercode/Sensors/mpu6050.h"
#include "../UserCode/BLDC/bldc.h"
#include "../UserCode/PID/pid.h"
#include "../UserCode/user_define.h"
#include "../UserCode/Serial/serial.h"
#include "../UserCode/Serial/uart_proto.h"
#include "../UserCode/FS_iA6B/FS_iA6B.h"
#include "../UserCode/Sensors/BNO055.h"
#include "../UserCode/Sensors/BMP280.h"
//#include "../UserCode/Sensors/LPF.h"
#include "../UserCode/Sensors/kalman.h"
//#include "../UserCode/Sensors/EKF.h"
#include <stdio.h>
#include <math.h>
#include "../UserCode/Sensors/ekf_altitude.h"
#include "../UserCode/Sensors/ekf_imu.h"

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

extern I2C_HandleTypeDef hi2c1;

extern TIM_HandleTypeDef htim1;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart6_rx;

osThreadId TaskMpuHandle;
osThreadId TaskPidHandle;
osThreadId TaskTelemetryHandle;
osThreadId TaskRxDecodeHandle;
osThreadId TaskFilterHandle;
osThreadId TaskBarometerHandle;
osThreadId TaskEulerHandle;

osMessageQId myQueue01Handle;
osMessageQId myQueue02Handle;
osMessageQId myQueue03Handle;
osMessageQId myQueue04Handle;
osMessageQId myQueue05Handle;
osMessageQId myQueue06Handle;
osMessageQId myQueue07Handle;
/* USER CODE BEGIN PV */
extern PROCESS_t tprocess;

extern char scmd[4];
extern float fset_point;
extern float fset_point1;
extern float fset_point2;
extern float fkp;
extern float fki;
extern float fkd;

uint8_t pid_init_flag = 0;

uint8_t rxB2B;
uint8_t rxRawBuf[110] = { NULL };
uint8_t rxBuf[110] = { NULL };
uint8_t message[110] = { NULL };
uint16_t rxBufPtr[1] = { -3 };
uint16_t rxBufLen[1] = {0};
int frameState = 0;
uint8_t rxRawFrame[70] = { NULL };
uint8_t iBus_rxRawFrame[70] = {NULL};
uint8_t iBus_rxRawBuf[110] = {NULL};
uint8_t rxCutData[70] = { NULL };
volatile int8_t ptr;
int charPtr = 0;
uint8_t rawCommand[110];
char command[90];
char *token;
uint8_t rxMessLen = 0;
uint8_t rxRawBuf_len = 0;
uint32_t rxCounter = 0;
uint8_t fault_flag = 0;
uint8_t tune_flag = 0;

static uint8_t TaskTelemetry_isFailed = 0;
static uint8_t TaskRxDecode_isFailed = 0;
static uint8_t TaskMpu_isFailed = 0;
static uint8_t TaskBarometer_isFailed = 0;
static uint8_t TaskFilter_isFailed = 0;
static uint8_t TaskPid_isFailed = 0;
static uint8_t TaskEuler_isFailed = 0;


extern uint8_t ibus_rx_buf[32];
extern uint8_t ibus_rx_cplt_flag;

volatile static flag_valid_iBus = 0;
volatile static flag_valid_header = 0;

static uint8_t len = 0;

uint8_t iBus_command[32] = { NULL };

bno055_t bno;
bno055_calib_status_t *bno055_calibStat;

static bno055_euler_f_t txEuler_global;
static bno055_calib_status_t txBNO_calib_global;

uint16_t ccr1 = 0;
uint16_t ccr2 = 0;
uint16_t ccr3 = 0;
uint16_t ccr4 = 0;

BMP280_t bmp;

//BaroFilter_t baroFilter;
float altitude = 0;
float filtered_altitude = 0;
float LPF_altitude = 0;
float EKF_altitude = 0;

uint8_t flag_rxEuler = 0;
uint8_t flag_rxBaro = 0;
uint8_t flag_rxEuler2 = 0;

BMP280_f_t txAltitude_global;
bno055_vec3f_t txAccel_global;
float txAccel_global_filtered;

bno055_vec3f_t txGyro_global;
BMP280_f_t txBaro_global;
//AltitudeComplementary_t LowPassFilter;
//float LPF_Altitude = 0;

AltitudeKalman_t kalmanFilter;
float kalman_result = 0;

uint8_t flag_escCalibed = 0;
uint8_t motor_arming_flag = 0;

// Khai báo các cấu trúc dữ liệu toàn cục
//BaroFilter_t     BaroFilter_global;
AltitudeKalman_t AltKalman_global;

// Biến lưu trữ kết quả cuối cùng
float filtered_altitude_fused = 0.0f; // Kết quả cuối cùng sau Kalman

//	short gyro_x_offset = 1, gyro_y_offset = -23, gyro_z_offset = -3;
	unsigned short iBus_SwA_Prev = 0;
	float yaw_heading_reference;

uint32_t adc_raw = 0;
float adc_voltage = 0;
uint16_t adc_val[1];
uint8_t flag_lowBat = 0;

uint8_t sent = 0;
int uart_state = 0;
char test[6];
uint8_t txBufDMA[50] = { NULL };
float gx_offset, gy_offset, gz_offset = 0;

int16_t dyaw_difference;

float dyaw_proc;
float pre_dyaw;

//EKF_t ekf;
//
//EKF_t altitude_ekf;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
extern void MX_TIM1_Init(void);
extern void MX_I2C1_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
void StartTask01(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);
void StartTask05(void const * argument);
void StartTask06(void const * argument);
void StartTask07(void const * argument);

/* USER CODE BEGIN PFP */
float Read_ADC_Voltage(void)
{
    return (adc_val[0] * 3.3f / 4095.0f);
}

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
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_val, 1);
  /* Init BNO055 */
  // 1. Khởi tạo
  while (bno055_init(&bno, &hi2c1, (0x28 << 1)) != HAL_OK);
    bno055_set_axis_remap(&bno, 0x24, 0x00);

  // XOAY TRÁI 90° - heading giảm 90°
//  bno055_set_axis_remap(&bno, 0x21, 0x00);
  // Hoặc XOAY PHẢI 90° - heading tăng 90°
//   bno055_set_axis_remap(&bno, 0x21, 0x01);
// X→-X, Y→-Y, Z→Z
//   bno055_set_axis_remap(&bno, 0x24, 0x03);
// upside down
//   bno055_set_axis_remap(&bno, 0x24, 0x06);
   HAL_Delay(20);

  // 2. Đặt power mode NORMAL
  bno055_set_power_mode(&bno, BNO055_POWER_NORMAL);
  HAL_Delay(20);

  // 3. Vào CONFIG MODE để chỉnh cấu hình
  bno055_set_opr_mode(&bno, BNO055_OPR_MODE_CONFIG);
  HAL_Delay(25);

  // 4. Chọn PAGE 0 để ghi đơn vị
  bno055_set_page(&bno, BNO055_PAGE0);

  // 5. Ghi đơn vị đo
  bno055_config_units_default(&bno);
  HAL_Delay(20);

  // 6. Set lại axis map (nếu cần)
  // bno055_write8(&bno, BNO055_REG_AXIS_MAP_CONFIG, 0x24);
  // bno055_write8(&bno, BNO055_REG_AXIS_MAP_SIGN,   0x00);

  bno055_set_opr_mode(&bno, BNO055_OPR_MODE_NDOF);  // hoặc IMU / AMG
  HAL_Delay(20);

  // 8. Đọc thử lại xem UNIT_SEL đã được ghi chưa
  uint8_t unit;
  bno055_read8(&bno, BNO055_REG_UNIT_SEL, &unit);
  printf("UNIT_SEL = 0x%02X\r\n", unit);

  bno055_calib_status_t calib;
  while (1) {
      bno055_get_calibration_status(&bno, &calib);
//      printf("Sys:%d Gyr:%d Acc:%d Mag:%d\r\n",
//             calib.sys, calib.gyro, calib.accel, calib.mag);

	  serial_write_com(scmd, calib.sys, calib.accel, calib.gyro, calib.mag);

      if (calib.sys == 3) {
          printf("✓ Hoàn tất!\r\n");
          break;
      }
      HAL_Delay(10);
  }

  // Đọc và lưu calibration profile
  uint8_t calib_data[22];
  bno055_set_opr_mode(&bno, BNO055_OPR_MODE_CONFIG);
  HAL_Delay(25);

  HAL_I2C_Mem_Read(bno.hi2c, bno.dev_addr, 0x55,
                   I2C_MEMADD_SIZE_8BIT, calib_data, 22, HAL_MAX_DELAY);

  // In ra để lưu vào code
  printf("const uint8_t CALIB[22] = {");
  for (int i = 0; i < 22; i++) {
      printf("0x%02X%s", calib_data[i], i < 21 ? "," : "");
  }
  printf("};\r\n");

  bno055_set_opr_mode(&bno, BNO055_OPR_MODE_NDOF);

//
  while (BMP280_Init(&bmp, &hi2c1, BMP280_I2C_ADDR_LOW));
  AltKalman_Init(&kalmanFilter, 0.007);
  /*	Init DMA RX2Idle	*/
  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rxRawBuf, sizeof(rxRawBuf));
  __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
  // Phải có dòng này để bật DMA
//	serial_write_com(scmd, .001, .002, .003, 4);

  serial_init();

  pitch.in.kp = 1;
  pitch.in.ki = 0.7f;
  pitch.in.kd = 0;
  pitch.out.kp = 1;
  pitch.out.ki = .5f;
  pitch.out.kd = 0;

  roll.in.kp = 1;
  roll.in.ki = 0.7f;
  roll.in.kd = 0;
  roll.out.kp = 1.6f;
  roll.out.ki = .5f;
  roll.out.kd = 0;

  yaw_heading.kp = 1;
  yaw_heading.ki = .7f;
  yaw_heading.kd = 0;

  yaw_rate.kp = 1;
  yaw_rate.ki = .5f;
  yaw_rate.kd = 0;

  // Vòng lặp ngoài (Altitude Position)
  alt.out.kp = 0.5f;   // Ví dụ: KP cho Altitude
  alt.out.ki = 0.01f;  // Ví dụ: KI cho Altitude
  alt.out.kd = 0.0f;   // Thường là 0 cho D của vòng ngoài Altitude

  // Vòng lặp trong (Vertical Rate)
  alt.in.kp = 1.0f;    // Ví dụ: KP cho Vertical Rate
  alt.in.ki = 0.1f;   // Ví dụ: KI cho Vertical Rate
  alt.in.kd = 0.05f;   // Ví dụ: KD cho Vertical Rate

//  tprocess = VTUN;
  scmd[0] = 'V';
  scmd[1] = 'T';
  scmd[2] = 'U';
  scmd[3] = 'N';

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of myQueue01 */
  osMailQDef(myQueue01, 16, bno055_vec3f_t);	// max: 16
  myQueue01Handle = osMailCreate(osMailQ(myQueue01), NULL);

  osMailQDef(myQueue02, 12, bno055_euler_f_t);	// max: 16
  myQueue02Handle = osMailCreate(osMailQ(myQueue02), NULL);

  osMailQDef(myQueue03, 12, bno055_calib_status_t);	// max: 16
  myQueue03Handle = osMailCreate(osMailQ(myQueue03), NULL);

  osMailQDef(myQueue04, 12, BMP280_f_t);	// max: 16
  myQueue04Handle = osMailCreate(osMailQ(myQueue04), NULL);

  osMailQDef(myQueue05, 16, bno055_vec3f_t);	// max: 16
  myQueue05Handle = osMailCreate(osMailQ(myQueue05), NULL);

  osMailQDef(myQueue06, 12, BMP280_f_t);	// max: 16
  myQueue06Handle = osMailCreate(osMailQ(myQueue06), NULL);

  osMailQDef(myQueue07, 12, bno055_euler_f_t);	// max: 16
  myQueue07Handle = osMailCreate(osMailQ(myQueue07), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(myTask01, StartTask01, osPriorityAboveNormal, 0, 256);	// Đọc 3 DOF 1000Hz
  TaskMpuHandle = osThreadCreate(osThread(myTask01), NULL);

  osThreadDef(myTask02, StartTask02, osPriorityAboveNormal, 1, 256);		// PID 1000Hz
  TaskPidHandle = osThreadCreate(osThread(myTask02), NULL);

  osThreadDef(myTask03, StartTask03, osPriorityHigh, 1, 512);		// Telemetry
  TaskTelemetryHandle = osThreadCreate(osThread(myTask03), NULL);
//
  osThreadDef(myTask04, StartTask04, osPriorityHigh, 0, 128);		// RX Decode
  TaskRxDecodeHandle = osThreadCreate(osThread(myTask04), NULL);
//
  osThreadDef(myTask05, StartTask05, osPriorityNormal, 1, 256);		// Đọc Barometer 400Hz
  TaskFilterHandle = osThreadCreate(osThread(myTask05), NULL);
//
  osThreadDef(myTask06, StartTask06, osPriorityNormal, 2, 1024);		// Bộ lọc Complementary cho Barometer (400Hz)
  TaskBarometerHandle = osThreadCreate(osThread(myTask06), NULL);
//
  osThreadDef(myTask07, StartTask07, osPriorityBelowNormal, 0, 512);	// Đọc Euler (NDOF)	100Hz
  TaskEulerHandle = osThreadCreate(osThread(myTask07), NULL);


  if(TaskTelemetryHandle == NULL)
  {
	  TaskTelemetry_isFailed = 1;
  }
  if(TaskRxDecodeHandle == NULL)
  {
	  TaskRxDecode_isFailed = 1;
  }
  if(TaskMpuHandle == NULL)
  {
	  TaskMpu_isFailed = 1;
  }
  if(TaskBarometerHandle == NULL)
  {
	  TaskBarometer_isFailed = 1;
  }
  if(TaskFilterHandle == NULL)
  {
	  TaskFilter_isFailed = 1;
  }
  if(TaskPidHandle == NULL)
  {
	  TaskPid_isFailed = 1;
  }
  if(TaskEulerHandle == NULL)
  {
	  TaskEuler_isFailed = 1;
  }

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}



/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartTask01(void const * argument)		// IMU 1kHz		tx: Gyro_z + Euler_angle + Euler_Status
{
  /* USER CODE BEGIN 5 */

  /* Infinite loop */
  for(;;)
  {
	bno055_vec3f_t *txAccel = osMailAlloc(myQueue01Handle, sizeof(bno055_vec3f_t));
	bno055_vec3f_t *txGyro = osMailAlloc(myQueue05Handle, sizeof(bno055_vec3f_t));

    if (txAccel != NULL) {
		// Đọc Accel
    	bno055_vec3i16_t a_raw;
    	//    	bno055_set_opr_mode(&bno, BNO055_OPR_MODE_ACCONLY);
		bno055_read_acc_i16(&bno, &a_raw);
		bno055_read_acc_i16_to_f(&a_raw, txAccel);	// đọc trực tiếp vào txData

//        // LỌC ACCEL
//        Vec3f_t accel_input = {txAccel->x, txAccel->y, txAccel->z};
//        EKF_IMU_UpdateAccelFloat(&accel_input);
//
//        // LẤY KẾT QUẢ FILTERED
//        Vec3f_t accel_filtered;
//        EKF_IMU_GetAccel(&accel_filtered);

//        // GHI ĐÈ GIÁ TRỊ FILTERED
//        txAccel->x = accel_filtered.x;
//        txAccel->y = accel_filtered.y;
//        txAccel->z = accel_filtered.z;

		osMailPut(myQueue01Handle, txAccel);
    }

    if (txGyro != NULL) {
//    	bno055_set_opr_mode(&bno, BNO055_OPR_MODE_GYRONLY);
//    	bno055_read_gyr_i16(&bno, txGyro);
		// Đọc Gyro
    	bno055_vec3i16_t g_raw;
		bno055_read_gyr_i16(&bno, &g_raw);
		bno055_read_gyr_i16_to_f(&g_raw, txGyro);	// đọc trực tiếp vào txData
    	txGyro->y = - txGyro->y;
    	txGyro->z = - txGyro->z;

//		// lấy phủ định đối với i16
//    	txGyro->y = ~txGyro->y;
//    	txGyro->y = txGyro->y + 1;
//    	txGyro->z = ~txGyro->z;
//    	txGyro->z = txGyro->z + 1;

//        // LỌC GYRO (chỉ 2 dòng!)
//        Vec3f_t gyro_input = {txGyro->x, txGyro->y, txGyro->z};
//        EKF_IMU_UpdateGyroFloat(&gyro_input);
//
//        // LẤY KẾT QUẢ FILTERED
//        Vec3f_t gyro_filtered;
//        EKF_IMU_GetGyro(&gyro_filtered);
//
//        // GHI ĐÈ GIÁ TRỊ FILTERED
//        txGyro->x = gyro_filtered.x;
//        txGyro->y = gyro_filtered.y;
//        txGyro->z = gyro_filtered.z;


		osMailPut(myQueue05Handle, txGyro);
    }

    osDelay(1);
  }

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_TaskPid */
/**
* @brief Function implementing the TaskPid thread.
* @param argument: Not used
* @retval None
*/

bno055_euler_f_t *rxEuler;
bno055_vec3f_t *rxGyro;
BMP280_f_t *rxBaro;
/* USER CODE END Header_TaskPid */
void StartTask02(void const * argument)		//	PID		rx: Euler_angle + Gyro + Baro
{
  /* USER CODE BEGIN TaskPid */
	osEvent rxEuler_Event;
	osEvent rxGyro_Event;
	osEvent rxBaro_Event;

//    PID_InitAll();
	uint16_t time = 0;
	unsigned short current_LeftThrottle = 0;
  /* Infinite loop */
  for(;;)
  {
	  rxGyro_Event = osMailGet(myQueue05Handle, osWaitForever);
	  rxEuler_Event = osMailGet(myQueue02Handle, osWaitForever);
	  rxBaro_Event = osMailGet(myQueue06Handle, osWaitForever);

	  if (rxEuler_Event.status == osEventMail)
	  {
		  /*	Rx		*/
		    rxEuler = (bno055_euler_f_t*)rxEuler_Event.value.p;
			memcpy(&txEuler_global, rxEuler, sizeof(bno055_euler_f_t));

		    flag_rxEuler = 1;
	  }
	  if (rxBaro_Event.status == osEventMail)
	  {
		  /*	Rx		*/
		    rxBaro = (BMP280_f_t*)rxBaro_Event.value.p;
			memcpy(&txBaro_global, rxBaro, sizeof(BMP280_f_t));

		    flag_rxBaro = 1;
	  }
	  if (rxGyro_Event.status == osEventMail)
	  {
		  /*	Rx		*/
		  	rxGyro = (bno055_vec3f_t*)rxGyro_Event.value.p;
			memcpy(&txGyro_global, rxGyro, sizeof(bno055_vec3f_t));

			  /*	PID		*/
			float rc_range_half = (float)((MAX_I6X_THROTTLE - MIN_I6X_THROTTLE)/2);
			float pitch_normalized = (float)(iBus.RV - MEDIUM_I6X_THROTTLE) / rc_range_half;
			float target_pitch_angle = pitch_normalized * MAX_PITCH_ANGLE;
			float yaw_normalized = (float)(iBus.LH - MEDIUM_I6X_THROTTLE) / rc_range_half;
			float target_yaw_rate = yaw_normalized * MAX_YAW_RATE;

			Double_Roll_Pitch_PID_Calculation(&pitch, target_pitch_angle, rxEuler->pitch, rxGyro->x); // * 0.1 để giới hạn pitch_setpoint từ -50 độ -> 50 độ
			Double_Roll_Pitch_PID_Calculation(&roll, (iBus.RH - MEDIUM_I6X_THROTTLE) * 0.1f, rxEuler->roll, rxGyro->y);

			if(iBus.LV < 1030 || motor_arming_flag == 0)
			{
			  Reset_All_PID_Integrator();
			}

			if(iBus.LH < 1485 || iBus.LH > 1515)
			{
			  yaw_heading_reference = rxEuler->heading;

			  // tốc độ quay quanh trục Oz
			  Single_Yaw_Rate_PID_Calculation(&yaw_rate, target_yaw_rate, rxGyro->z);

			  ccr1 = MIN_50HZ_THROTTLE + 50 + (iBus.LV - MIN_I6X_THROTTLE) * 1 - pitch.in.pid_result + roll.in.pid_result - yaw_rate.pid_result;
			  ccr2 = MIN_50HZ_THROTTLE + 50 + (iBus.LV - MIN_I6X_THROTTLE) * 1 + pitch.in.pid_result + roll.in.pid_result + yaw_rate.pid_result;
			  ccr3 = MIN_50HZ_THROTTLE + 50 + (iBus.LV - MIN_I6X_THROTTLE) * 1 + pitch.in.pid_result - roll.in.pid_result - yaw_rate.pid_result;
			  ccr4 = MIN_50HZ_THROTTLE + 50 + (iBus.LV - MIN_I6X_THROTTLE) * 1 - pitch.in.pid_result - roll.in.pid_result + yaw_rate.pid_result;
			}
			else
			{
			  Single_Yaw_Heading_PID_Calculation(&yaw_heading, yaw_heading_reference, rxEuler->heading, rxGyro->z);

			  ccr1 = MIN_50HZ_THROTTLE + 50 + (iBus.LV - MIN_I6X_THROTTLE) * 1 - pitch.in.pid_result + roll.in.pid_result - yaw_heading.pid_result;
			  ccr2 = MIN_50HZ_THROTTLE + 50 + (iBus.LV - MIN_I6X_THROTTLE) * 1 + pitch.in.pid_result + roll.in.pid_result + yaw_heading.pid_result;
			  ccr3 = MIN_50HZ_THROTTLE + 50 + (iBus.LV - MIN_I6X_THROTTLE) * 1 + pitch.in.pid_result - roll.in.pid_result - yaw_heading.pid_result;
			  ccr4 = MIN_50HZ_THROTTLE + 50 + (iBus.LV - MIN_I6X_THROTTLE) * 1 - pitch.in.pid_result - roll.in.pid_result + yaw_heading.pid_result;
			}

			if (time > 100)
			{
				current_LeftThrottle = iBus.LV;
				if (abs(current_LeftThrottle - iBus.LV) > 5.0f) {
//				    int alt_set_point += (iBus.LV * 0.01f * 0.001f);	//0.001f: dt hay tần số của task101
//				    Double_Altitude_PID_Calculation(&alt, set_point_alt, filtered_altitude, gyro.)
				}
				time = 0;
			}

			ESC_Write(&htim1, 1, ccr1);
			ESC_Write(&htim1, 2, ccr2);
			ESC_Write(&htim1, 3, ccr3);
			ESC_Write(&htim1, 4, ccr4);

			if(iBus.SwA == 2000 && iBus_SwA_Prev != 2000)
			{
			  if(iBus.LV < 1010)
			  {
				  motor_arming_flag = 1;
				  yaw_heading_reference = rxEuler->heading;
			  }
			  else
			  {
				  while(iBus.LV >= 1010 || iBus.SwA == 2000)
				  {
					  // Khóa an toàn
					  // cảnh báo người dùng rằng việc mở khóa đã bị từ chối vì cần ga không ở mức MIN.
				  }
			  }
			}
			iBus_SwA_Prev = iBus.SwA;

			  /*	End PID		*/

			osMailFree(myQueue05Handle, rxGyro);
	  }

	  if (flag_rxEuler == 1)
	  {
		  osMailFree(myQueue02Handle, rxEuler);
	  }
	  if (flag_rxBaro == 1)
	  {
		  osMailFree(myQueue06Handle, rxBaro);
	  }
      osDelay(1);
  }
  /* USER CODE END TaskPid */
}

/* USER CODE BEGIN Header_TaskTelemetry */
/**
  * @brief  Function implementing the TaskTelemetry thread.
  * @param  argument: Not used
  * @retval None
  */
bno055_euler_f_t *rxEuler_toTelemetry;
bno055_calib_status_t *rxBNO_calib;
uint16_t time = 0;
void StartTask03(void const * argument)		// Task Telemetry rx: Euler + BNO_Status
{
	osEvent rxEuler_event;
	osEvent rxBNO_calib_event;

  /* Infinite loop */
	  for(;;)
	  {
		  rxEuler_event = osMailGet(myQueue07Handle, osWaitForever);

		  time++;
//		  if (time > 1000)	time = 0;

		  uint8_t flag_rxEuler_toTelemetry = 0;

		  if (rxEuler_event.status == osEventMail)
		  {
			  /*	Rx		*/
			rxEuler_toTelemetry = (bno055_euler_f_t*)rxEuler_event.value.p;
			memcpy(&txEuler_global, rxEuler_toTelemetry, sizeof(bno055_euler_f_t));
			
			int16_t dyaw_difference = txEuler_global.heading - pre_dyaw;

			if (dyaw_difference > 180)  dyaw_difference -= 360;
			if (dyaw_difference < -180) dyaw_difference += 360;

			dyaw_proc -= (float)dyaw_difference;

			pre_dyaw = txEuler_global.heading;

			if (sent == 1)
			{
//				if (time > 1000)
//				{
				serial_write_com_DMA(txBufDMA, scmd, txAccel_global.x, txAccel_global.y, rxEuler_toTelemetry->heading, rxEuler_toTelemetry->heading);

//				serial_write_com_DMA(txBufDMA, scmd, LPF_altitude, EKF_altitude, rxEuler_toTelemetry->heading, rxEuler_toTelemetry->heading);

//					serial_write_com_DMA(txBufDMA, scmd, txAccel_global.z, txAccel_global_filtered, rxEuler_toTelemetry->heading, rxEuler_toTelemetry->heading);

//					serial_write_com_DMA(txBufDMA, scmd, rxEuler_toTelemetry->roll, rxEuler_toTelemetry->pitch, rxEuler_toTelemetry->heading, rxEuler_toTelemetry->heading);
					sent = 0;
					time = 0;
//				}
			}

			flag_rxEuler_toTelemetry = 1;
		  }

		  if (tprocess == IMUCALIB)
		  {
			  rxBNO_calib_event = osMailGet(myQueue03Handle, osWaitForever);	// Hiển thị lên GUI

			  if (rxBNO_calib_event.status == osEventMail)
			  {
				  /*	Rx		*/
				rxBNO_calib = (bno055_calib_status_t*)rxBNO_calib_event.value.p;
				memcpy(&txBNO_calib_global, rxBNO_calib, sizeof(bno055_calib_status_t));
				serial_write_com(scmd, rxBNO_calib->sys, rxBNO_calib->accel, rxBNO_calib->gyro, rxBNO_calib->mag);

				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

				osMailFree(myQueue03Handle, rxBNO_calib);
			  }
		  }

		  if (flag_rxEuler_toTelemetry == 1)
		  {
			  osMailFree(myQueue07Handle, rxEuler_toTelemetry);
		  }
		  osDelay(3);		// ~ 333Hz
		  uart_state = __HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC);
	  }

  /* USER CODE END TaskTelemetry */
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

    if (huart->Instance == USART2) {
  	  HAL_UART_DMAStop(&huart2);

  	  sent = 1;
    }
}
/* USER CODE BEGIN Header_TaskRxDecode */
/**
  * @brief  Function implementing the TaskRxDecode thread.
  * @param  argument: Not used
  * @retval None
  */
void StartTask04(void const * argument)
{
    for(;;)
    {
		memcpy(&rxRawFrame[0], rxRawBuf, sizeof(rxRawFrame)); //void * memcpy (void * to , const void * from , size_t numBytes );

    	while ((rxRawFrame[ptr] != 0x20) && ((rxRawFrame[ptr + 1]) != 0x40))
    	{
    		ptr++;
    	}

		flag_valid_header = 1;

		len = ptr + 31;

		memcpy(&iBus_command, &rxRawFrame[ptr], 32); //void * memcpy (void * to , const void * from , size_t numBytes );

		// HÀM CHECKSUM
	    if (iBus_Check_CHKSUM(&iBus_command[0], 32) == 1)
		{
			// PARSE FRAME
			iBus_Parsing(&iBus_command[0], &iBus);

			// 		Xét SwC
			if (iBus.FailSafe == 1)	// if (iBus->failSafe == 1)
			{
				tprocess = IMUCALIB;
				ccr1 = ARMING_THRESHOLD_THROTTLE + 900;
				ccr2 = ARMING_THRESHOLD_THROTTLE + 900;
				ccr3 = ARMING_THRESHOLD_THROTTLE + 900;
				ccr4 = ARMING_THRESHOLD_THROTTLE + 900;

				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				flag_escCalibed = 0;
			}
			else if ((iBus.SwC == 2000) && (motor_arming_flag == 0))
			{
				tprocess = ESCCALIB;
				if (flag_escCalibed == 0)
				{
					osThreadSuspend(TaskMpuHandle);
					osThreadSuspend(TaskPidHandle);
					osThreadSuspend(TaskTelemetryHandle);
					osThreadSuspend(TaskFilterHandle);
					osThreadSuspend(TaskBarometerHandle);
					osThreadSuspend(TaskEulerHandle);

					ESC_Write_All(&htim1, 2000);

					osDelay(3000);

					ESC_Write_All(&htim1, 1000);

					osDelay(3000);
					// ESC Calib end

					// ESC Arming
				    for (int i=0; i<300; i++)
				    {
						ESC_Write(&htim1, 1, 1000);
						ESC_Write(&htim1, 2, 1000);
						ESC_Write(&htim1, 3, 1000);
						ESC_Write(&htim1, 4, 1000);

					    osDelay(10);
				    }
				    osDelay(500);
				    // ESC Arming end

					osThreadResume(TaskMpuHandle);
					osThreadResume(TaskPidHandle);
					osThreadResume(TaskTelemetryHandle);
					osThreadSuspend(TaskFilterHandle);
					osThreadSuspend(TaskBarometerHandle);
					osThreadSuspend(TaskEulerHandle);
				}
				flag_escCalibed = 1;
			}
			else
			{
				tprocess = NONE;
				flag_escCalibed = 0;
			}


			// 		Xét SwA
			if(iBus.SwA != 2000)
			{
				motor_arming_flag = 0; // DISARM!
			}
			else if ((iBus.SwA == 2000) && (iBus.FailSafe == 0))	// ARM!
			{
				motor_arming_flag = 1; // ARM!
				tprocess = ARMING;

				if(iBus.LV > 1030) // Ga lớn hơn Min Spin (1030)
				{
					ESC_Write(&htim1, 1, ccr1);
					ESC_Write(&htim1, 2, ccr2);
					ESC_Write(&htim1, 3, ccr3);
					ESC_Write(&htim1, 4, ccr4);
				}
				else // Ga ở mức 1010 < LV <= 1030
				{
					ESC_Write(&htim1, 1, ARMING_THRESHOLD_THROTTLE + MIN_50HZ_THROTTLE);
					ESC_Write(&htim1, 2, ARMING_THRESHOLD_THROTTLE + MIN_50HZ_THROTTLE);
					ESC_Write(&htim1, 3, ARMING_THRESHOLD_THROTTLE + MIN_50HZ_THROTTLE);
					ESC_Write(&htim1, 4, ARMING_THRESHOLD_THROTTLE + MIN_50HZ_THROTTLE);
				}
			}
			else
			{
				ESC_Write(&htim1, 1, MIN_50HZ_THROTTLE);
				ESC_Write(&htim1, 2, MIN_50HZ_THROTTLE);
				ESC_Write(&htim1, 3, MIN_50HZ_THROTTLE);
				ESC_Write(&htim1, 4, MIN_50HZ_THROTTLE);
			}


		}
		// reset mảng command[]
		for (int i = 0; i < 31; i++)
		{
			iBus_command[i] = '\0';
		}
		// reset flag_valid_iBus
		flag_valid_iBus = 0;
		// reset ptr
		ptr = 0;
		osDelay(7);		// Tx_iBus period = 7ms

    }
  /* USER CODE END TaskRxDecode */
}

void StartTask05(void const * argument)		// Read Barometer	tx: Altitude	Priority > Filter
{

	  /* Infinite loop */
	  for(;;)
	  {
		  BMP280_f_t *txAltitude = osMailAlloc(myQueue04Handle, sizeof(BMP280_f_t));

	    if (txAltitude != NULL) {

//	    	txAltitude->altitude = BMP280_ReadAltitude(&bmp, 1013.25f);
	    	// read without temperature
//	    	txAltitude->altitude = BMP280_ReadAltitude(&bmp, BMP280_ReadPressure(&bmp));
	    	// read with both temperature and pressure
	    	//		    	txAltitude->altitude = BMP280_ReadAltitude_2(&bmp, BMP280_ReadPressure(&bmp), BMP280_ReadTemperature(&bmp));

	    	if (BMP280_ReadAltitude_ToMeters(BMP280_ReadPressure(&bmp), BMP280_ReadTemperature(&bmp)) >= 0)
	    	{
		    	txAltitude->altitude = BMP280_ReadAltitude_ToMeters(BMP280_ReadPressure(&bmp), BMP280_ReadTemperature(&bmp));

	    	}

	        if (txAltitude->altitude >= 0)
			{
		        altitude = txAltitude->altitude;
		        // Low Pass Filter
		        LPF_altitude = LPF_altitude * 0.90f + altitude * (1.0f - 0.90f);
		        txAltitude->altitude = LPF_altitude;
			}
			osMailPut(myQueue04Handle, txAltitude);
	    }
	    osDelay(3);
	  }
}

bno055_vec3f_t *rxAccel;
BMP280_f_t *rxAltitude;
void StartTask06(void const * argument)		// Filter	rx: altitude + Accel_z, 	tx: Filtered Barometer		Priority < Baro, IMU
{
    TickType_t lastWake = xTaskGetTickCount();
	osEvent rxAccel_event;
	osEvent rxAltitude_event;
    EKF_Altitude_Init();  // Phải có!

	  /* Infinite loop */
	  for(;;)
	  {
		  rxAccel_event = osMailGet(myQueue01Handle, osWaitForever);
		  rxAltitude_event = osMailGet(myQueue04Handle, osWaitForever);

		  if ((rxAccel_event.status == osEventMail) && (rxAltitude_event.status == osEventMail))
		  {
			  /*	Rx		*/
			rxAccel = (bno055_vec3f_t*)rxAccel_event.value.p;
			memcpy(&txAccel_global, rxAccel, sizeof(bno055_vec3f_t));

			rxAltitude = (BMP280_f_t*)rxAltitude_event.value.p;
			txAltitude_global.altitude = rxAltitude->altitude;

            // ========== CẬP NHẬT EKF (chỉ 1 dòng) ==========
            EKF_Altitude_Update(rxAltitude->altitude, rxAccel->z);

			osMailFree(myQueue01Handle, rxAccel);
			osMailFree(myQueue04Handle, rxAltitude);
		  }

	        // ========== LẤY KẾT QUẢ FILTERED ==========
//		  filtered_altitude_fused = EKF_Altitude_Get();
		  EKF_altitude = EKF_Altitude_Get();
	        float vertical_vel = EKF_Altitude_GetVelocity();

	        // ========== LẤY KẾT QUẢ FILTERED ==========
		  BMP280_f_t *txFiltAltitude = osMailAlloc(myQueue06Handle, sizeof(BMP280_f_t));
		  if (txFiltAltitude != NULL) {
			txFiltAltitude->altitude = EKF_altitude;
			osMailPut(myQueue06Handle, txFiltAltitude);
		  }

	      osDelayUntil(&lastWake, 3); // 333 Hz
	  }
}
/* USER CODE BEGIN Header_TaskEuler */
/**
  * @brief  Function implementing the TaskEuler thread.
  * @param  argument: Not used
  * @retval None
  */
void StartTask07(void const * argument)		// tx: Euler_f + Euler_status
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	bno055_euler_f_t *txEulerToPid = osMailAlloc(myQueue02Handle, sizeof(bno055_euler_f_t));
	bno055_euler_f_t *txEulerToTelemetry = osMailAlloc(myQueue07Handle, sizeof(bno055_euler_f_t));

    if (txEulerToPid != NULL) {
		// Đọc Euler
		bno055_euler_i16_t e_raw;

		bno055_read_euler_i16(&bno, &e_raw);
		bno055_euler_i16_to_f(&e_raw, txEulerToPid);	// đọc trực tiếp vào txData
//		txEulerToTelemetry->roll = - txEulerToTelemetry->roll;
//		txEulerToTelemetry->pitch = - txEulerToTelemetry->pitch;
		osMailPut(myQueue02Handle, txEulerToPid);
    }
    if (txEulerToTelemetry != NULL) {
		// Đọc Euler
		bno055_euler_i16_t e_raw;

		bno055_read_euler_i16(&bno, &e_raw);
		bno055_euler_i16_to_f(&e_raw, txEulerToTelemetry);	// đọc trực tiếp vào txData
//		txEulerToTelemetry->roll = - txEulerToTelemetry->roll;
//		txEulerToTelemetry->pitch = - txEulerToTelemetry->pitch;
		osMailPut(myQueue07Handle, txEulerToTelemetry);
    }
	if (tprocess == IMUCALIB)	// mode BNO055 NDOF Calib
	{
		bno055_calib_status_t *txBNO_calib = osMailAlloc(myQueue03Handle, sizeof(bno055_calib_status_t));

		if (txBNO_calib != NULL)	// tạo Mail thành công
		{
			bno055_get_calibration_status(&bno, txBNO_calib);
			osMailPut(myQueue03Handle, txBNO_calib);
		}
	}
	osDelay(10);
  }
  /* USER CODE END 5 */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart2.Instance)
	{
		rxRawBuf[ptr] = rxB2B;
		ptr++;

    	if (rxRawBuf[ptr] == '\0' && rxRawBuf[ptr - 1] == 0x03)
		{
    		rxMessLen = (uint8_t)rxRawBuf[ptr - 1 - 3];
		}
		for (int i = 0; i < rxMessLen + 5; i++)
		{
			rxRawFrame[i] = rxRawBuf[ptr - (rxMessLen + 5) + i];
		}
		frameState = UART_get_data(&rxRawFrame, ptr, &rxCutData, &rxBufLen, &rxBufPtr, rxMessLen);

		switch (frameState) {
			case right:
				for (int i = 0; i < rxMessLen; i++)
				{
					message[ptr - rxMessLen + i] = rxCutData[i];
				}
		    	memset(rxCutData, '\0', sizeof(rxCutData));

			    frameState = 99;

			    while (message[charPtr] == '\0')
			    {
			    	charPtr++;
			    }
				for (int i = 0; i < sizeof(message); i++)
				{
					rawCommand[i] = message[charPtr + i];
				}
				for (int i = 0; i < sizeof(rawCommand); i++)
				{
					if (rawCommand[i] == '\0' && i < ptr)
					{
						rawCommand[i] = 0x20;
					}

					if (rawCommand[i] != 0x20 && i >= ptr)
					{
						rawCommand[i] = 0x20;
					}
				}
				token = strtok(&rawCommand, "~");
				int dem = 0;
				while(token != NULL)
				{
					++dem;
					for (int i = 0; i < strlen(token); i++)
					{
						command[i] = token[i];
					}

					command[rxMessLen - 1] = '\0';		// minus '~'

					char str[100];
					snprintf(str, sizeof(str), "%s", command);
					sscanf(str, "%s %f %f %f %f", scmd, &fset_point, &fkp, &fki, &fkd);

					token = strtok(NULL, "~");	// token = "^SPID 30 1 0" -> token overwrite len" SPID 30 1 0.523 0.3 \026\0 -> ^SPID 30 1 0523 0.3 \026\0
				}								// dau '^' nam trong rawCommand[98]

				if (StrCompare(scmd, (uint8_t*)"SPID", 4))
				{
				  tprocess = SPID;
				}
				else if (StrCompare(scmd, (uint8_t*)"VTUN", 4))
				{
					time = HAL_GetTick() / 1000.0f;

					tune_flag = 1;
				  tprocess = VTUN;
				}
//				else if (StrCompare(scmd, (uint8_t*)"PTUN", 4))
//				{
//					tune_flag = 1;
//				  tprocess = PTUN;
//				}
				else if (StrCompare(scmd, (uint8_t*)"STOP", 4))
				{
				  tprocess = STOP;
				}
				else if (StrCompare(scmd, (uint8_t*)"RSET", 4))
				{
				  HAL_NVIC_SystemReset();

				  tprocess = RSET;
				}
				else
				{
				  tprocess = NONE;
				}
		    	memset(command, '\0', sizeof(command));

			    // Avoid log overflow
			    rxRawBuf_len = strlen(rxRawBuf);
			    if (rxRawBuf_len > 70)
			    {
			    	memset(message, '\0', sizeof(message));
			    	memset(rxRawBuf, '\0', sizeof(rxRawBuf));
			    	memset(rawCommand, '\0', sizeof(rawCommand));
			    	ptr = 0;
			    	charPtr = 0;
			    }

				break;
			case no_valid:
		    	memset(rxCutData, '\0', sizeof(rxCutData));
			    frameState = 88;

			    rxRawBuf_len = strlen(rxRawBuf);
				break;
			case false_CRC:
		    	memset(rxCutData, '\0', sizeof(rxCutData));
			    frameState = 77;

			    // Avoid log overflow
			    rxRawBuf_len = strlen(rxRawBuf);
			    if (rxRawBuf_len > 70)
			    {
			    	memset(message, '\0', sizeof(message));
			    	memset(rxRawBuf, '\0', sizeof(rxRawBuf));
			    	memset(rawCommand, '\0', sizeof(rawCommand));
			    	ptr = 0;
			    	charPtr = 0;
			    }
				break;
			case false_lenght_data:
		    	memset(rxCutData, '\0', sizeof(rxCutData));

			    frameState = 66;

			    // Avoid log overflow
			    rxRawBuf_len = strlen(rxRawBuf);
			    if (rxRawBuf_len > 70)
			    {
			    	memset(message, '\0', sizeof(message));
			    	memset(rxRawBuf, '\0', sizeof(rxRawBuf));
			    	memset(rawCommand, '\0', sizeof(rawCommand));
			    	ptr = 0;
			    	charPtr = 0;
			    }
				break;
		}

		HAL_UART_Receive_DMA(&huart2, &rxB2B, 1);
	}
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11)
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
