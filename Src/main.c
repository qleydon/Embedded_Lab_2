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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Servo_enums.h"
#include "SERVO.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOV 		(0b00100000)
#define WAIT 		(0b01000000)
#define LOOP 		(0b10000000)
#define END_LOOP 	(0b10100000)
#define RECIPE_END 	(0b00000000)
#define INVALID_COMMAND	(0b11000000)

#define PRINT_INDEX	  (0b11100000)
#define PRINT_OPTIONS (0b01100000)

#define SERVO_MOTOR1	(0)
#define SERVO_MOTOR2	(1)
#define SERVO1_PIN		(56)
#define SERVO2_PIN 		(38)

#define DATA_SIZE		(200)
#define CMD_BUFFER_SIZE (10)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t loop_flag = 0;
uint8_t Rx_data = 0;
uint8_t Rx_command[10] = {0};
uint8_t Rx_idx = 0;

// define recipes
unsigned char recipe_test_default[] = {
	MOV+0, // There must NOT be intervening instructions in this group to allow
	MOV+5, // verification of default time delay.
	MOV+0,
	MOV+3,
	LOOP+0, //Test the default loop behavior.
	MOV+1,
	MOV+4,
	END_LOOP,
	MOV+0,
	MOV+2,
	WAIT+0,
	MOV+3, // Move to an adjacent position to verify
	WAIT+0,
	MOV+2,
	MOV+3, // Measure the timing precision of the 9.3 second delay with an external
	WAIT+31, // timer.
	WAIT+31,
	WAIT+31,
	MOV+4,
	RECIPE_END
};
unsigned char recipe_test_all_possible_positions[] = {
    MOV+0,
	WAIT+10,
	MOV+1,
	WAIT+10,
	MOV+2,
	WAIT+10,
	MOV+3,
	WAIT+10,
	MOV+4,
	WAIT+10,
	MOV+5,
	RECIPE_END
};

unsigned char recipe_test_command_error[] = {
    INVALID_COMMAND,
	RECIPE_END
};

unsigned char recipe_test_command_error2[] = {
    MOV+17,
	RECIPE_END
};

unsigned char recipe_test_nested_loop_error[] = {
    LOOP+0,
	LOOP+0,
	END_LOOP,
	END_LOOP,
	RECIPE_END
};

unsigned char recipe_test_verify_continue_override[] = {
	MOV+0,
	MOV+5,
	MOV+0,
	MOV+3,
	MOV+0,
	WAIT+0,
	MOV+5,
	WAIT+0,
	MOV+2,
	MOV+0,
	RECIPE_END,
	MOV+5, // mov after recipe end
};

unsigned char recipe_test_deliberate_error_verify_mov[] = {
	MOV+0,
	MOV+5,
	MOV+2,
	MOV+0,
	MOV+5,
	MOV+12, // deliberate error
	MOV+0,
	RECIPE_END,
};
unsigned char recipe_test_graduate_opcodes[] = {
	MOV+0,
	MOV+5,
	PRINT_INDEX,
	WAIT+10,
	MOV+0,
	MOV+5,
	PRINT_OPTIONS,
	RECIPE_END,
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */

// standardized clock period for while loop
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		loop_flag = 1;
	}
}

// called to retrieve character input
// simply stores data and repeats character
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	HAL_UART_Receive_IT(&huart2, &Rx_data, 1);
	if(Rx_data == 'x' || Rx_data == 'X'){
		Rx_idx = 0;
		memset(Rx_command, 0, sizeof(Rx_command[0]) * CMD_BUFFER_SIZE); // wipe Rx_command
		uint8_t message[DATA_SIZE] = "\r\n >";
		HAL_UART_Transmit(&huart2, message, DATA_SIZE, HAL_MAX_DELAY);
		return;
	}
	else{
		Rx_command[Rx_idx] = Rx_data;
		Rx_idx++;
		HAL_UART_Transmit(&huart2, &Rx_data, 1, HAL_MAX_DELAY);
		return;
	}
}

// reads and validates the user input commands
void read_user_command(SERVO_Cfg* SERVO_1, SERVO_Cfg* SERVO_2){
	if(Rx_idx < 3){ // all valid commands are "char,char,\r"
		return;
	}
	else if(Rx_idx > 3 || Rx_command[2] != '\r'){
		Rx_idx = 0;
		memset(Rx_command, 0, sizeof(Rx_command[0]) * CMD_BUFFER_SIZE); // wipe Rx_command
		uint8_t message[DATA_SIZE] = " - invalid entry, please try again. Use command 'O' or 'o' for more command options\r\n >"; // right now just exits
		HAL_UART_Transmit(&huart2, message, DATA_SIZE, HAL_MAX_DELAY);
		return;
	}
	// 2 char and '\r', exactly 3
	// Rx_command could be overwritten from interrupt, so we store
	uint8_t command_1 = Rx_command[0];
	uint8_t command_2 = Rx_command[1];

	Rx_idx = 0;
	memset(Rx_command, 0, sizeof(Rx_command[0]) * CMD_BUFFER_SIZE); // wipe Rx_command

	uint8_t message[DATA_SIZE] = "\r\n >";
	HAL_UART_Transmit(&huart2, message, DATA_SIZE, HAL_MAX_DELAY);

	process_user_command(command_1, SERVO_1);
	process_user_command(command_2, SERVO_2);
	return;

}

// Code to start the move (adjust PWM) and start the timing delay based on the
// current position.
static void start_move(SERVO_Cfg* SERVO,  unsigned char position )
{
	// position out of valid range
	if(position > 5 || position < 0){
		SERVO->SERVO_STATUS = status_command_error;
		process_status_led(status_command_error, SERVO);
		return;
	}
	SERVO->SERVO_POSITION_GOAL = position;
	int8_t difference = abs(SERVO->SERVO_POSITION_GOAL - SERVO->SERVO_POSITION);
	switch(difference){
	case(0): // no change in position
			SERVO->SERVO_COUNT_MOVE = 0; // may have been in motion when this occurred
			SERVO->SERVO_STATE = state_at_position;
      if(SERVO->SERVO_STATUS != status_paused) {
        SERVO->INDEX++; // look at next move
      }
			return;
			break;
	case(1):
		SERVO->SERVO_COUNT_MOVE = 1; //54 ms
		break;
	case(2):
		SERVO->SERVO_COUNT_MOVE = 2; //108 ms
		break;
	case(3):
		SERVO->SERVO_COUNT_MOVE = 2; //160 ms
		break;
	case(4):
		SERVO->SERVO_COUNT_MOVE = 3; //216 ms
		break;
	case(5):
		SERVO->SERVO_COUNT_MOVE = 3; //270 ms
		break;
	}

	SERVO->SERVO_STATE = state_moving;
	__HAL_TIM_SET_COMPARE(&htim3, SERVO->PWM_TIM_CH, 110+position*18); //out of 2000, 1 = 0.01 ms
}

static void start_loop(SERVO_Cfg* SERVO,  unsigned char loop){
	if(SERVO->LOOP_FLAG != 0){
		SERVO->SERVO_STATUS = status_nested_error; // nested error
		process_status_led(status_nested_error, SERVO);
		return;
	}
	SERVO->LOOP_FLAG = 1;// in a loop
	SERVO->LOOP_START = SERVO->INDEX;
	SERVO->SERVO_COUNT_LOOP = loop; // 5 LSB
	SERVO->INDEX++;
	SERVO->SERVO_STATE = state_at_position; // I should not need this
	return;
}

static void start_end_loop(SERVO_Cfg* SERVO){
	if(SERVO->SERVO_COUNT_LOOP !=0){
		SERVO->SERVO_COUNT_LOOP--;
		SERVO->INDEX = SERVO->LOOP_START +1; // one past loop start
	}
	else{
		SERVO->INDEX++;
		SERVO->LOOP_FLAG = 0; // no longer in loop
	}
}

static void start_wait(SERVO_Cfg* SERVO,  unsigned char wait){
	SERVO->SERVO_COUNT_WAIT = wait+1; // a wait of 0 is a wait of 1
	SERVO->SERVO_STATE = state_waiting;
}

static void end_recipe(SERVO_Cfg* SERVO){
	SERVO->SERVO_STATE = state_recipe_ended;
	SERVO->RECIPE_INDEX++; // GO TO NEXT RECIPE
	SERVO->SERVO_STATUS = status_paused;
	SERVO->INDEX = 0;
	process_status_led(status_paused, SERVO);
}

static void print_index(SERVO_Cfg* SERVO) {
	uint8_t rx_data[DATA_SIZE] = "";
	const char* which_servo = "";
	if(SERVO->SERVO_PIN == SERVO1_PIN) {
		which_servo = "servo1";
	} else if(SERVO->SERVO_PIN == SERVO2_PIN) {
		which_servo = "servo2";
	} else {
		which_servo = "unknown";
	}
	snprintf(rx_data, DATA_SIZE,"%s index: %u \r\n >", which_servo, SERVO->INDEX);
	HAL_UART_Transmit(&huart2, rx_data, DATA_SIZE, HAL_MAX_DELAY);
}

static void print_options(void) {
	uint8_t rx_data[DATA_SIZE] = "";
	snprintf(rx_data, DATA_SIZE,"P:Pause, C:Continue, R:Right, L:Left, N:NoOp, B:Begin, I:Index, O:Options \r\n >");
	HAL_UART_Transmit(&huart2, rx_data, DATA_SIZE, HAL_MAX_DELAY);
}

// execute functionality based on opcodes
void process_recipe(SERVO_Cfg* SERVO, unsigned char recipe[]){
	if(SERVO->SERVO_STATUS == status_running){
		switch(recipe[SERVO->INDEX]>>5){
			case(LOOP>>5): //4
				start_loop(SERVO, recipe[SERVO->INDEX] & 0b00011111);
				break;

			case(END_LOOP>>5): //5
				start_end_loop(SERVO);
				break;

			case(MOV>>5): //1
				start_move(SERVO, recipe[SERVO->INDEX] & 0b00011111); // 5 LSB
				break;

			case(WAIT>>5)://2
				start_wait(SERVO, recipe[SERVO->INDEX] & 0b00011111); // 5 LSB
				break;

			case(RECIPE_END>>5)://0
				end_recipe(SERVO);
				break;

			// GRAD OPCODES
			case(PRINT_INDEX>>5):
				print_index(SERVO);
				SERVO->INDEX++;
				break;

			case(PRINT_OPTIONS>>5):
				print_options();
				SERVO->INDEX++;
				break;

			default:
				SERVO->SERVO_STATUS = status_command_error;
				process_status_led(status_command_error, SERVO);
				break;
		}
	}
}

// decide what to do based on current servo state and status
void process_timestep(SERVO_Cfg* SERVO, unsigned char recipe[]){
	switch(SERVO->SERVO_STATE){
		case(state_moving):
			SERVO->SERVO_COUNT_MOVE--; // decrement the moving counter
			if(SERVO->SERVO_POSITION_GOAL > SERVO->SERVO_POSITION){ 
				SERVO->SERVO_POSITION++;
			}
			else if(SERVO->SERVO_POSITION_GOAL < SERVO->SERVO_POSITION){
				SERVO->SERVO_POSITION--;
			}
			if(SERVO->SERVO_COUNT_MOVE == 0){
				SERVO->SERVO_STATE = state_at_position; // Finally at position.
        if(SERVO->SERVO_STATUS != status_paused) {
          SERVO->INDEX++;
        }
				process_recipe(SERVO, recipe);
			}
			break;

		case(state_at_position):
			process_recipe(SERVO, recipe); // go to next recipe state
			break;

		case(state_waiting):
			if(SERVO->SERVO_STATUS == status_paused){
				break;
			}
			SERVO->SERVO_COUNT_WAIT--; // decrement waiting counter
			if(SERVO->SERVO_COUNT_WAIT == 0){
				SERVO->SERVO_STATE = state_at_position; // at position, done waiting.
				SERVO->INDEX++; // go to next recipe
				process_recipe(SERVO, recipe); // process next recipe state
			}
			break;


		case(state_unknown):
			process_recipe(SERVO, recipe);
			break;

		case(state_recipe_ended):
			SERVO->SERVO_STATUS = status_paused;
			SERVO->SERVO_STATE = state_recipe_ended;
			SERVO->INDEX = 0;
			break;
	}
	process_status_led(SERVO->SERVO_STATUS, SERVO);
}

void process_status_led(enum status one_event, SERVO_Cfg* SERVO){
	if(SERVO->SERVO_PIN != SERVO1_PIN) {
		return;
	}
	switch(one_event){
		case(status_running):
			//
			HAL_GPIO_WritePin(SHLD_D13_GPIO_Port, SHLD_D13_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SHLD_D12_GPIO_Port, SHLD_D12_Pin, GPIO_PIN_SET);
			break;
		case(status_paused):
			//
			HAL_GPIO_WritePin(SHLD_D13_GPIO_Port, SHLD_D13_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SHLD_D12_GPIO_Port, SHLD_D12_Pin, GPIO_PIN_SET);
			break;
		case(status_command_error):
			//
			HAL_GPIO_WritePin(SHLD_D13_GPIO_Port, SHLD_D13_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(SHLD_D12_GPIO_Port, SHLD_D12_Pin, GPIO_PIN_RESET);
			break;
		case(status_nested_error):
			//
			HAL_GPIO_WritePin(SHLD_D13_GPIO_Port, SHLD_D13_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SHLD_D12_GPIO_Port, SHLD_D12_Pin, GPIO_PIN_RESET);
			break;
	}
}

void process_user_command(uint8_t command, SERVO_Cfg* SERVO){
	if((command == 'P'|| command == 'p')&&
			(SERVO->SERVO_STATUS != status_command_error)&&
			(SERVO->SERVO_STATUS !=status_nested_error)&&
			(SERVO->SERVO_STATE != state_recipe_ended)){
		SERVO->SERVO_STATUS = status_paused;
		process_status_led(status_paused, SERVO);
	}
	else if((command == 'C'|| command == 'c')&&
			(SERVO->SERVO_STATUS != status_command_error)&&
			(SERVO->SERVO_STATUS !=status_nested_error)&&
			(SERVO->SERVO_STATE != state_recipe_ended)){
		SERVO->SERVO_STATUS = status_running;
		process_status_led(status_running, SERVO);
		}
	else if((command == 'L'|| command == 'l')&&
			(SERVO->SERVO_STATUS == status_paused)&&
			(SERVO->SERVO_POSITION != 5)){
		start_move(SERVO, SERVO->SERVO_POSITION+1);
	}
	else if((command == 'R'|| command == 'r')&&
			(SERVO->SERVO_STATUS == status_paused)&&
			(SERVO->SERVO_POSITION != 0)){
		start_move(SERVO, SERVO->SERVO_POSITION-1);
	}
	else if((command == 'B'|| command == 'b')){
		SERVO->SERVO_STATUS = status_running;
		SERVO->SERVO_STATE = state_unknown;
		process_status_led(status_running, SERVO);
		SERVO->INDEX = 0;

	}
	else if((command == 'I'|| command == 'i')){
		print_index(SERVO);
	}
	else if((command == 'O'|| command == 'o')){
		print_options();
	}

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2); // Start timer.
  HAL_UART_Receive_IT (&huart2, &Rx_data, 1);

  SERVO_Cfg* servo1_p = &SERVO_Cfg_1;
  SERVO_Cfg* servo2_p = &SERVO_Cfg_2;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // start in paused state
  process_status_led(status_paused, servo1_p);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);   // Output PWM Generation
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);   // Output PWM Generation

  uint8_t rx_data[DATA_SIZE] = "\r\n > ";
  HAL_UART_Transmit(&huart2, rx_data, DATA_SIZE, HAL_MAX_DELAY);

  // 				demo tests
  // -----------------------------------------
  // recipe_test_default
  // recipe_test_all_possible_positions
  // recipe_test_command_error
  // recipe_test_command_error2
  // recipe_test_nested_loop_error
  // recipe_test_verify_continue_override
  // recipe_test_deliberate_error_verify_mov
  // recipe_test_graduate_opcodes

  while (1)
  {
	  loop_flag = 0;
	  process_timestep(servo1_p, recipe_test_graduate_opcodes);
	  process_timestep(servo2_p, recipe_test_default);
    read_user_command(servo1_p, servo2_p); // read user input
	  while(loop_flag == 0);

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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  htim2.Init.Prescaler = 15999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 159;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000;
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
  sConfigOC.Pulse = 110;
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
  HAL_GPIO_WritePin(GPIOA, SHLD_D13_Pin|SHLD_D12_Pin|SHLD_D11_Pin|SHLD_D7_SEG7_Clock_Pin
                          |SHLD_D8_SEG7_Data_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SHLD_D4_SEG7_Latch_Pin|SHLD_D10_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SHLD_A5_Pin SHLD_A4_Pin */
  GPIO_InitStruct.Pin = SHLD_A5_Pin|SHLD_A4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SHLD_A0_Pin SHLD_D2_Pin */
  GPIO_InitStruct.Pin = SHLD_A0_Pin|SHLD_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SHLD_A1_Pin SHLD_A2_Pin */
  GPIO_InitStruct.Pin = SHLD_A1_Pin|SHLD_A2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SHLD_D13_Pin SHLD_D12_Pin SHLD_D11_Pin SHLD_D7_SEG7_Clock_Pin */
  GPIO_InitStruct.Pin = SHLD_D13_Pin|SHLD_D12_Pin|SHLD_D11_Pin|SHLD_D7_SEG7_Clock_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SHLD_A3_Pin */
  GPIO_InitStruct.Pin = SHLD_A3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SHLD_A3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SHLD_D6_Pin */
  GPIO_InitStruct.Pin = SHLD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SHLD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SHLD_D8_SEG7_Data_Pin */
  GPIO_InitStruct.Pin = SHLD_D8_SEG7_Data_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SHLD_D8_SEG7_Data_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SHLD_D4_SEG7_Latch_Pin */
  GPIO_InitStruct.Pin = SHLD_D4_SEG7_Latch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SHLD_D4_SEG7_Latch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SHLD_D10_Pin */
  GPIO_InitStruct.Pin = SHLD_D10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SHLD_D10_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SHLD_D15_Pin SHLD_D14_Pin */
  GPIO_InitStruct.Pin = SHLD_D15_Pin|SHLD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
