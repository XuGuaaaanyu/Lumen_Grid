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
#include <string.h>
#include <stdio.h>
#include "math.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUF_SIZE 		12
#define TX_BUF_SIZE			6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
float target_gz = 0.0f;       // Bluetooth command: G{value}
float target_a = 0.0f;        // Bluetooth command: A{value}
int forward_pwm = 300;        // Bluetooth command: P{value}
int mode = 0;                 // 0: PWM, 1: Turn, 2: Acceleration

float vol = 0.0; // Current speed (initially zero)

// Global variables fir pitch, roll, and state
int pitch = 0;
int roll = 0;
int state = 0;
char ca, cb, cc, cd, ce, c;
int count = 0;

uint8_t Data[RX_BUF_SIZE];
uint8_t *Trash;
char bt_str[TX_BUF_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void BT_Receive(char *str);
void motor_forward(void);
void motor_stop(void);
void dyna_forward(int val);
void dyna_backward(int val);
void dyna_right_spin(int val);
void dyna_left_spin(int val);
void gyro_turn_pid(float target_angle_deg, int spin_direction);
HAL_StatusTypeDef check_I2C_device(uint8_t device_address);
void test_I2C_devices(void);
void read_accelerometer(float *ax, float *ay, float *az);
void read_gyroscope(float *gx, float *gy, float *gz);
void read_magnetometer(float *mx, float *my, float *mz);
void init_magnetometer(void);
void init_accelerometer(void);
void init_gyroscope(void);
void set_pwm(float left_pwm, float right_pwm);
void BT_Transmit(char *str);
void curve_control(int target_gz, float signed_speed);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void i2c_scan() {
	printf("Scanning I2C bus...\r\n");
	for (uint8_t addr = 1; addr < 127; addr++) {
		if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 10) == HAL_OK) {
			printf("I2C device found at 0x%02X\r\n", addr);
		} else {
			printf("I2C device NOT found at 0x%02X\r\n", addr);
		}
	}
}

// Callback function of UART interrupt
// MODIFIES: pitch, roll, state
// BT message format: P<ppp>,R<rrr>,S<s>
// where each p, r, and s is one digit in ASCII
// the first digit of pitch and roll indicates plus or minus, '1' for negative values
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	//HAL_UART_DMAPause(huart);
	if (huart->Instance == USART1) {
		//HAL_UART_Transmit(&huart2, Data, 12, 1000);

		//Find the starting character: 'P'
		int index = 0;
		for (int i = 0; i < RX_BUF_SIZE; i++) {
			if (Data[i] == 'P')
				index = i;
		}

		pitch = (Data[(2 + index) % RX_BUF_SIZE] - '0') * 10
				+ (Data[(3 + index) % RX_BUF_SIZE] - '0');
		roll = (Data[(7 + index) % RX_BUF_SIZE] - '0') * 10
				+ (Data[(8 + index) % RX_BUF_SIZE] - '0');
		state = (Data[(11 + index) % RX_BUF_SIZE] - '0');

		if (Data[(1 + index) % RX_BUF_SIZE] == '1')
			pitch *= (-1);
		if (Data[(6 + index) % RX_BUF_SIZE] == '1')
			roll *= (-1);
	}
	//HAL_UART_DMAResume(huart);
	HAL_UART_Receive_DMA(&huart1, Data, RX_BUF_SIZE);
}

void i2c_bus_reset() {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	// 1. Disable I2C peripheral
	__HAL_RCC_I2C1_FORCE_RESET();
	__HAL_RCC_I2C1_RELEASE_RESET();

	// 2. Configure SCL and SDA as GPIO
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9; // Assuming PB8=SCL, PB9=SDA
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// 3. Pulse SCL 9 times
	for (int i = 0; i < 9; i++) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET); // SCL High
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // SCL Low
		HAL_Delay(1);
	}

	// 4. Reconfigure SDA/SCL for I2C
	MX_I2C1_Init();
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	//i2c_scan();
	init_accelerometer();
	init_gyroscope();
	init_magnetometer();
	//HAL_UARTEx_ReceiveToIdle_IT(&huart1, Data, MAX_BUFFER_SIZE);
	//	while((*Trash) != 'P')
	//	{
	//		HAL_UART_Receive(&huart1, Trash, 1, HAL_MAX_DELAY);
	//	}
	HAL_UART_Receive_DMA(&huart1, Data, RX_BUF_SIZE);

	if (HAL_I2C_IsDeviceReady(&hi2c1, 0x6B << 1, 1, HAL_MAX_DELAY) == HAL_OK) {
		printf("L3GD20H detected at 0x6B\r\n");
	} else {
		printf("L3GD20H NOT detected!\r\n");
	}
	if (HAL_I2C_IsDeviceReady(&hi2c1, 0x1D << 1, 1, HAL_MAX_DELAY) == HAL_OK) {
		printf("LSM303D detected at 0x1D\r\n");
	} else {
		printf("LSM303D NOT detected!\r\n");
	}
	//  if (huart2.RxState == HAL_UART_STATE_READY) {
	//        printf("Bluetooth detected\r\n");
	//    } else {
	//        printf("Bluetooth NOT detected!\r\n");
	//        printf("State %d\r\n",huart2.RxState);
	//    }
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		int num1 = (pitch < 10 && pitch > -10) ? 0 : pitch;
		int num2 = (roll < 10 && roll > -10) ? 0 : roll;
		int num3 = state;
		// Emergency stop logic
		if (num3 == 1) {
			vol = 0.0;
			num1 = 0;
			num2 = 0;

		} else {
			//			if (abs(num1) >= abs(num2)) {
			//				num2 = 0;
			//			} else {
			//				num1 = 0;
			//			}

			// Adjust the speed according to pitch (acceleration/deceleration)
			//printf("pitch:%d\r\n",num1);
			vol = num1 * 18;
			//printf("velocity:%.2f\r\n",vol);
			//	  			set_pwm(vol,vol);

			// Clamp vol to allowable speed range [-1000, 1000]
		}

		// Compute turning difference based on roll
		// Higher roll gives greater difference between wheels for sharper turns
		//printf("Roll:%d\r\n",num2);
		curve_control((int)((float)num2 * 1.80f), vol);
		//	  		double turn_delta = (vol * num2) / 90.0;
		//	  		if (vol<100&&vol>-100){
		//	  			int direction = (num2>0) ? 1 : (num2<0) ? -1 : 0;
		//	  			turn_delta = 100*direction;
		//	  		}
		//	  		// Calculate raw PWM values before mapping
		//	  		double left_pwm_raw = vol - turn_delta;
		//	  		double right_pwm_raw = vol + turn_delta;
		//
		//	  		// Map absolute speed to PWM (0->150, max->1000), preserving direction
		//	  		int left_pwm, right_pwm;
		//
		//	  		if (left_pwm_raw != 0) {
		//	  			int left_sign = (left_pwm_raw > 0) ? 1 : -1;
		//	  			left_pwm = left_sign
		//	  					* (150 + (abs(left_pwm_raw) * (1000 - 150)) / 1000);
		//	  		} else {
		//	  			left_pwm = 0;
		//	  		}
		//
		//	  		if (right_pwm_raw != 0) {
		//	  			int right_sign = (right_pwm_raw > 0) ? 1 : -1;
		//	  			right_pwm = right_sign
		//	  					* (150 + (abs(right_pwm_raw) * (1000 - 150)) / 1000);
		//	  		} else {
		//	  			right_pwm = 0;
		//	  		}
		//
		//	  		// Set PWM values to motors using existing motor control function
		//	  		set_pwm(left_pwm, right_pwm);
		//if(count == 10){
		c = 'S';
		if (vol < 0)
			ca = '1';
		else
			ca = '0';

		int vol_int = floor(abs(vol));
		int cbb = (vol_int / 1000);
		cb = cbb + '0';
		int ccc = ((vol_int - cbb * 1000) / 100);
		cc = ccc + '0';
		int cdd = ((vol_int - cbb * 1000 - ccc * 100) / 10);
		cd = cdd + '0';
		int cee = (vol_int - cbb * 1000 - ccc * 100 - 10 * cdd);
		ce = cee + '0';

		sprintf(bt_str, "%c%c%c%c%c%c", c, ca, cb, cc, cd, ce);
		//printf("%c%c%c%c%c%c\r\n",c,ca,cb,cc,cd,ce);

		count = 0;
		BT_Transmit(bt_str);
		//}
		//count++;
		// Delay to maintain loop responsiveness (e.g., 10 ms per loop)
		HAL_Delay(10);  // Keep loop responsive
		if (num3 == 1)
			HAL_Delay(1000);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
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
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 84;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1000;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 84;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 1000;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
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
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin PA8 PA9 */
	GPIO_InitStruct.Pin = LD2_Pin | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	// TIM3_CH2 -> PC7
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	// TIM4_CH1 -> PB6
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void motor_forward(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0); // 			RIGHT DIR 	PA8 	D7	Pin 7	0:For	1:Back
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0); // 			LEFT DIR 	PA9 	D8	Pin 8	0:For	1:Back
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 500); //	RIGHT VAL	PC7		D9	Pin 9
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 500); // 	LEFT VAL	PB6		D10	Pin10
}

void dyna_forward(int val) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0); // 			RIGHT DIR 	PA8 	D7	Pin 7	0:For	1:Back
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0); // 			LEFT DIR 	PA9 	D8	Pin 8	0:For	1:Back
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, val); //	RIGHT VAL	PC7		D9	Pin 9
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, val); // 	LEFT VAL	PB6		D10	Pin10
}

void dyna_backward(int val) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1); // 			RIGHT DIR 	PA8 	D7	Pin 7	0:For	1:Back
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1); // 			LEFT DIR 	PA9 	D8	Pin 8	0:For	1:Back
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, val); //	RIGHT VAL	PC7		D9	Pin 9
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, val); // 	LEFT VAL	PB6		D10	Pin10
}

void dyna_right_spin(int val) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1); // 			RIGHT DIR 	PA8 	D7	Pin 7	0:For	1:Back
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0); // 			LEFT DIR 	PA9 	D8	Pin 8	0:For	1:Back
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, val); //	RIGHT VAL	PC7		D9	Pin 9
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, val); // 	LEFT VAL	PB6		D10	Pin10

}

void dyna_left_spin(int val) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0); // 			RIGHT DIR 	PA8 	D7	Pin 7	0:For	1:Back
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1); // 			LEFT DIR 	PA9 	D8	Pin 8	0:For	1:Back
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, val); //	RIGHT VAL	PC7		D9	Pin 9
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, val); // 	LEFT VAL	PB6		D10	Pin10

}

void motor_stop(void) {
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0); //		RIGHT VAL	PC7		D9	Pin 9
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0); // 		LEFT VAL	PB6		D10	Pin10
}

void curve_control(int target_gz, float signed_speed) {
	static float integral = 0.0f;
	static float previous_error = 0.0f;
	static float gz_filtered = 0.0f;
	static uint32_t last_time = 0;

	float gx, gy, gz;
	read_gyroscope(&gx, &gy, &gz);

	gz_filtered = 0.9f * gz_filtered + 0.1f * gz;

	uint32_t now = HAL_GetTick();
	float dt = (now - last_time) / 1000.0f;
	if (dt < 0.01f)
		dt = 0.01f;
	last_time = now;

	float error = target_gz - gz_filtered;
	integral += error * dt;
	float derivative = (error - previous_error) / dt;
	previous_error = error;

	float Kp = 9.0f, Ki = 0.0f, Kd = 0.05f;
	float control = Kp * error + Ki * integral + Kd * derivative;

	float delta_pwm = control;
	//printf("delta_pwm %d \r\n",delta_pwm);
	if (delta_pwm > 1000.0)
		delta_pwm = 1000.0;
	if (delta_pwm < -1000.0)
		delta_pwm = -1000.0;
	if (target_gz == 0)
		delta_pwm = 0;
	float left_pwm = signed_speed - delta_pwm;
	float right_pwm = signed_speed + delta_pwm;

	set_pwm(left_pwm, right_pwm);

	printf("[CurveCtrl] gz=%.2f, err=%.2f, dt=%.3f, Δ=%.2f, L=%.2f, R=%.2f\r\n",
			gz_filtered, error, dt, delta_pwm, left_pwm, right_pwm);
}

HAL_StatusTypeDef check_I2C_device(uint8_t device_address) {
	return HAL_I2C_IsDeviceReady(&hi2c1, device_address, 1, HAL_MAX_DELAY);
}

void test_I2C_devices() {
	if (check_I2C_device(0x1D) == HAL_OK) {
		//printf("LSM303D detected at 0x1D\n");
	} else {
		//printf("LSM303D NOT detected!\n");
	}

	if (check_I2C_device(0x6B) == HAL_OK) {
		//printf("L3GD20H detected at 0x6B\n");
	} else {
		//printf("L3GD20H NOT detected!\n");
	}
}

// FIX THESE
void read_accelerometer(float *ax, float *ay, float *az) {
	uint8_t reg = 0x28 | 0x80; // Auto-increment
	uint8_t data[6] = { 0 };

	if (HAL_I2C_Mem_Read(&hi2c1, 0x1D << 1, reg, I2C_MEMADD_SIZE_8BIT, data, 6,
	HAL_MAX_DELAY) != HAL_OK) {
		//printf("Accel read failed\r\n");
		*ax = *ay = *az = 0.0f;
		return;
	}

	int16_t raw_ax = (int16_t) (data[1] << 8 | data[0]);
	int16_t raw_ay = (int16_t) (data[3] << 8 | data[2]);
	int16_t raw_az = (int16_t) (data[5] << 8 | data[4]);

	*ax = raw_ax / 16384.0f; // assuming ±2g range
	*ay = raw_ay / 16384.0f;
	*az = raw_az / 16384.0f;
}

void read_gyroscope(float *gx, float *gy, float *gz) {
	uint8_t reg = 0x28 | 0x80;
	uint8_t data[6] = { 0 };

	if (HAL_I2C_Mem_Read(&hi2c1, 0x6B << 1, reg, I2C_MEMADD_SIZE_8BIT, data, 6,
	HAL_MAX_DELAY) != HAL_OK) {
		//printf("Gyro read failed\r\n");
		*gx = *gy = *gz = 0.0f;
		return;
	}

	int16_t raw_gx = (int16_t) (data[1] << 8 | data[0]);
	int16_t raw_gy = (int16_t) (data[3] << 8 | data[2]);
	int16_t raw_gz = (int16_t) (data[5] << 8 | data[4]);

	*gx = raw_gx / 131.0f; // assuming ±250 dps range
	*gy = raw_gy / 131.0f;
	*gz = raw_gz / 131.0f;
}

void read_magnetometer(float *mx, float *my, float *mz) {
	uint8_t reg = 0x08 | 0x80;
	uint8_t data[6] = { 0 };

	if (HAL_I2C_Mem_Read(&hi2c1, 0x1D << 1, reg, I2C_MEMADD_SIZE_8BIT, data, 6,
	HAL_MAX_DELAY) != HAL_OK) {
		//printf("Magnetometer read failed\r\n");
		*mx = *my = *mz = 0.0f;
		return;
	}

	int16_t raw_mx = (int16_t) (data[1] << 8 | data[0]);
	int16_t raw_my = (int16_t) (data[3] << 8 | data[2]);
	int16_t raw_mz = (int16_t) (data[5] << 8 | data[4]);

	*mx = raw_mx * 0.080f; // assuming ±4 Gauss (80 mG/LSB)
	*my = raw_my * 0.080f;
	*mz = raw_mz * 0.080f;
}

void init_magnetometer(void) {
	// CTRL5 (0x24): Enable temperature, set resolution, 6.25Hz output rate
	uint8_t ctrl5 = 0x94;  // 1001 0100
	HAL_I2C_Mem_Write(&hi2c1, 0x1D << 1, 0x24, 1, &ctrl5, 1, HAL_MAX_DELAY);

	// CTRL6 (0x25): ±4 gauss range
	uint8_t ctrl6 = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, 0x1D << 1, 0x25, 1, &ctrl6, 1, HAL_MAX_DELAY);

	// CTRL7 (0x26): Continuous conversion mode
	uint8_t ctrl7 = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, 0x1D << 1, 0x26, 1, &ctrl7, 1, HAL_MAX_DELAY);
}

void init_accelerometer(void) {
	// CTRL1 (0x20): 0x57 = 0b01010111 => 50Hz, all axes enabled
	uint8_t ctrl1 = 0x57;
	HAL_I2C_Mem_Write(&hi2c1, 0x1D << 1, 0x20, 1, &ctrl1, 1, HAL_MAX_DELAY);

	// CTRL2 (0x21): 0x00 = ±2g, default anti-alias filter
	uint8_t ctrl2 = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, 0x1D << 1, 0x21, 1, &ctrl2, 1, HAL_MAX_DELAY);
}

void init_gyroscope(void) {
	// CTRL1 (0x20): 0x0F = Normal mode, all axes enabled, 95 Hz ODR
	uint8_t ctrl1 = 0x0F;
	HAL_I2C_Mem_Write(&hi2c1, 0x6B << 1, 0x20, 1, &ctrl1, 1, HAL_MAX_DELAY);

	// CTRL4 (0x23): 0x00 = 250 dps (default)
	uint8_t ctrl4 = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, 0x6B << 1, 0x23, 1, &ctrl4, 1, HAL_MAX_DELAY);
}

void set_pwm(float left_pwm, float right_pwm) {
	// LEFT motor
	if (left_pwm >= 0) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);  // Forward
	} else {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);    // Backward
		left_pwm = -left_pwm;
	}

	// RIGHT motor
	if (right_pwm >= 0) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);  // Forward
	} else {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);    // Backward
		right_pwm = -right_pwm;
	}

	// Clamp values
	if (left_pwm > 1000)
		left_pwm = 1000;
	if (right_pwm > 1000)
		right_pwm = 1000;

	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, left_pwm / 1); // LEFT (TIM4_CH1)
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, right_pwm / 1); // RIGHT (TIM3_CH2)
}

void BT_Transmit(char *str) {
	HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str), HAL_MAX_DELAY);
}

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
