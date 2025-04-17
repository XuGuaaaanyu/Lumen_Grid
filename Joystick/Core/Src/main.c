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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ssd1306.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef SSD1306_INCLUDE_FONT_11x18
extern const SSD1306_Font_t Font_11x18;
#endif

#ifdef SSD1306_INCLUDE_FONT_7x10
extern const SSD1306_Font_t Font_7x10;
#endif

#ifdef SSD1306_INCLUDE_FONT_6x8
extern const SSD1306_Font_t Font_6x8;
#endif

#define IMU_CTRL_REG1_A 	0x20 /*IMU control register address*/
#define IMU_OUT_X_L_A		0x28 /*IMU X-axis acceleration data*/
#define IMU_I2C_ADDR_R 		0x33 /*8 bit IMU device I2C read address*/
#define IMU_I2C_ADDR_W		0x32 /*8 bit IMU device I2C write address*/
#define IMU_I2C_ADDR		0x19 /*7 bit IMU device I2C address*/

#define	PI					3.14
#define FILTER_SIZE			5
#define RX_BUFFER_SIZE		6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

HAL_StatusTypeDef ret;
uint8_t Rx_Buffer[RX_BUFFER_SIZE];

typedef struct {
	uint8_t acc_raw[6];
	int16_t ax_raw;
	int16_t ay_raw;
	int16_t az_raw;
	float ax;
	float ay;
	float az;
} IMU_data;

typedef struct {
	float roll;
	float pitch;
} Orientation;

typedef struct {
	float buffer[FILTER_SIZE];
	uint8_t index;
	float sum;
} MovingAverage;

int vol = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void IMU_Init();
void read_IMU(IMU_data *data);
void read_FSR(uint32_t *data);
void BT_Transmit(char *str);
Orientation get_Orientation(IMU_data data);
uint8_t get_Stop(uint32_t data);
void Display_screen(int fpitch, int froll, uint8_t stop);
void MovingAverage_Init(MovingAverage *filter);
int MovingAverage_Update(MovingAverage *filter, float new_value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// Callback function of UART interrupt
// MODIFIES: pitch, roll, state
// BT message format: P<ppp>,R<rrr>,S<s>
// where each p, r, and s is one digit in ASCII
// the first digit of pitch and roll indicates plus or minus, '1' for negative values
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	//HAL_UART_DMAPause(huart);
	if (huart->Instance == USART1) {
		//Find the starting character: 'S'
		int index = 0;
		for (int i = 0; i < RX_BUFFER_SIZE; i++)
		{
			if (Rx_Buffer[i] == 'S')
				index = i;
		}
		vol = (Rx_Buffer[(2 + index) % RX_BUFFER_SIZE] - '0') * 1000
			+ (Rx_Buffer[(3 + index) % RX_BUFFER_SIZE] - '0') * 100
			+ (Rx_Buffer[(4 + index) % RX_BUFFER_SIZE] - '0') * 10
			+ (Rx_Buffer[(5 + index) % RX_BUFFER_SIZE] - '0');
	}
	//HAL_UART_DMAResume(huart);
	//memset(Rx_Buffer, 0, RX_BUFFER_SIZE);
	HAL_UART_Receive_DMA(&huart1, Rx_Buffer, RX_BUFFER_SIZE);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	IMU_data imu_data;
	uint32_t FSR_data;
  	char bt_str[20] = { 0 }; /*Bluetooth transmission buffer*/
	Orientation current_ore; /*Current pitch and roll*/
	uint8_t stop = 0; /*Immediate stop*/
	//uint16_t duty_cycle = 0; /*Duty cycle of vibration motor PWM*/
	MovingAverage pitch_filter;
	MovingAverage roll_filter;
	char Pa = '\0', Pb = '\0', Pc = '\0';
	char Ra = '\0', Rb = '\0', Rc = '\0';
	//int loop_count = 0;
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	ssd1306_Init();
	ssd1306_Fill(Black);
	IMU_Init();
	MovingAverage_Init(&pitch_filter);
	MovingAverage_Init(&roll_filter);
	HAL_UART_Receive_DMA(&huart1, Rx_Buffer, RX_BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//loop_count = (loop_count + 1) % 3;
		read_IMU(&imu_data);
		current_ore = get_Orientation(imu_data);
		int Fpitch = MovingAverage_Update(&pitch_filter, current_ore.pitch);
		int Froll = MovingAverage_Update(&roll_filter, current_ore.roll);
		read_FSR(&FSR_data);
		stop = get_Stop(FSR_data);

		Pb = (abs(Fpitch) / 10) + '0';
		Pc = (abs(Fpitch) % 10) + '0';
		if (Fpitch >= 0)
			Pa = '0';
		else
			Pa = '1';

		Rb = (abs(Froll) / 10) + '0';
		Rc = (abs(Froll) % 10) + '0';
		if (Froll >= 0)
			Ra = '0';
		else
			Ra = '1';

		//printf("%d,%d,%d\r\n", Fpitch, Froll, stop);
		//if (loop_count == 0)
		//{
		//printf("P%c%c%c,R%c%c%c,S%d\n\r", Pa, Pb, Pc, Ra, Rb, Rc, stop);

		sprintf(bt_str, "P%c%c%c,R%c%c%c,S%d", Pa, Pb, Pc, Ra, Rb, Rc, stop);
		BT_Transmit(bt_str);
		//}

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, vol);
		//duty_cycle += 100;
		//if (duty_cycle == 8000)
		//	duty_cycle = 0;

		Display_screen(Fpitch, Froll, stop);
		HAL_Delay(10);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x00B07CB4;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);
	return ch;
}

void IMU_Init() {
	printf("Connecting to IMU ...\r \n");

	uint8_t config[2];
	config[0] = IMU_CTRL_REG1_A;
	config[1] = 0x97;

	ret = HAL_I2C_Master_Transmit(&hi2c1,
	IMU_I2C_ADDR_W, config, 2,
	HAL_MAX_DELAY);

	if (ret != HAL_OK) {
		printf("Failure to connect to IMU! Please check wiring. \r \n");
	}
	printf("IMU connected!\r \n");

}

//REQUIRES: None
//MODIFIES: data
//EFFECTS: Read the accelerometer data from the GY-511 module, and stores the results in data.
void read_IMU(IMU_data *data) {
	//Read from IMU register OUT_X_L_A with auto increment for 6 bytes of data, stored in acc_raw buffer
	ret = HAL_I2C_Mem_Read(&hi2c1,
						IMU_I2C_ADDR << 1,
						IMU_OUT_X_L_A | 0x80,
						I2C_MEMADD_SIZE_8BIT, data->acc_raw, 6,
						HAL_MAX_DELAY);
/*	if (ret != HAL_OK)
	{
		while(1);
	}*/

	//Update the ax_raw, ay_raw, and az_raw
	data->ax_raw = (int16_t) (data->acc_raw[0] | data->acc_raw[1] << 8);
	data->ay_raw = (int16_t) (data->acc_raw[2] | data->acc_raw[3] << 8);
	data->az_raw = (int16_t) (data->acc_raw[4] | data->acc_raw[5] << 8);

	//Convert it to be in unit g
	data->ax = data->ax_raw * 2.0 / 32767.0;
	data->ay = data->ay_raw * 2.0 / 32767.0;
	data->az = data->az_raw * 2.0 / 32767.0;
}

//REQUIRES: None
//MODIFIES: None
//EFFECTS: 	From the given IMU_data, compute the roll and pitch of the sensor.
Orientation get_Orientation(IMU_data data) {
	Orientation ore;
	float AccX = data.ax * 9.81;
	float AccY = data.ay * 9.81;
	float AccZ = data.az * 9.81;
	ore.pitch = (180 * atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) / PI);
	ore.roll = (180 * atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) / PI);
	return ore;
}

uint8_t get_Stop(uint32_t data) {
	if (data > 3000)
		return 1;
	else
		return 0;
}

void read_FSR(uint32_t *data) {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 0xFFFFFFFF);
	*data = HAL_ADC_GetValue(&hadc1);
}

void BT_Transmit(char *str) {
	HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str), HAL_MAX_DELAY);
}

void Display_screen(int fpitch, int froll, uint8_t stop) {
	uint8_t y = 0;
	uint8_t x = 0;
	char pitch[10] = { '\0' };
	char roll[10] = { '\0' };
	char state[10] = { '\0' };
	sprintf(pitch, "P=%d", fpitch);
	sprintf(roll, "R=%d", froll);
	sprintf(state, "S=%d", stop);

	//display pitch on the first line
	ssd1306_SetCursor(2, y);
	ssd1306_WriteString((char*) pitch, Font_11x18, White);
	//clear the remaining line
	x = strlen(pitch) * 11 + 2;
	ssd1306_SetCursor(x, y);
	ssd1306_WriteString(" ", Font_11x18, White);

	//display roll on the second line
	y += 18;
	ssd1306_SetCursor(2, y);
	ssd1306_WriteString((char*) roll, Font_11x18, White);
	//clear the remaining line
	x = strlen(roll) * 11 + 2;
	ssd1306_SetCursor(x, y);
	ssd1306_WriteString(" ", Font_11x18, White);

	//display state on the third line
	y += 18;
	ssd1306_SetCursor(2, y);
	ssd1306_WriteString((char*) state, Font_11x18, White);
	ssd1306_UpdateScreen();
}

void MovingAverage_Init(MovingAverage *filter) {
	for (int i = 0; i < FILTER_SIZE; i++)
		filter->buffer[i] = 0.0;
	filter->index = 0;
	filter->sum = 0.0;
}

int MovingAverage_Update(MovingAverage *filter, float new_value) {
	filter->sum = 0;
	filter->buffer[filter->index] = new_value;
	filter->index = (filter->index + 1) % FILTER_SIZE;
	for (int i = 0; i < FILTER_SIZE; i++)
		filter->sum += filter->buffer[i];
	return (int) (filter->sum / FILTER_SIZE);
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
	while (1) {
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
