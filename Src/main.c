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

#define PRINT_RECIPE  (0b11000000)
#define PRINT_INDEX	  (0b11100000)
#define PRINT_OPTIONS (0b01100000)

#define SERVO_MOTOR1	(0)
#define SERVO_MOTOR2	(1)

#define data_size		(200)
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
uint8_t Rx_data=0;
uint8_t Rx_command[10] = {0};
uint8_t Rx_idx = 0;

// define recipes
unsigned char recipe1[] = { MOV + 3, MOV | 5, RECIPE_END } ;
unsigned char recipe2[] = { MOV | 5, MOV | 2, RECIPE_END } ;

unsigned char recipe_test[] = {
    MOV + 0,
	MOV + 3,
	WAIT + 31,
    MOV + 5,
    MOV + 0,
    MOV + 3,
    LOOP + 0,
    MOV + 1,
    MOV + 4,
    END_LOOP,
    MOV + 0,
    MOV + 2,
    WAIT + 0,
    MOV + 3,
    WAIT + 0,
    MOV + 2,
    MOV + 3,
    WAIT + 31,
    WAIT + 31,
    MOV + 4,
	RECIPE_END
};
// If you set up an array like this then you can easily switch recipes
// using an additional user input command.
unsigned char *recipes[] = { recipe1, recipe2, NULL } ; // recipes
unsigned char *recipes_test[] = { recipe_test, recipe_test, NULL } ; // recipes
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		loop_flag = 1;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){// simply stores data and repeats character
	if(Rx_idx <10){
		HAL_UART_Receive_IT(&huart2, &Rx_data, 1);
		if(Rx_data == 'x' || Rx_data == 'X'){
			Rx_idx = 0;
			memset(Rx_command, 0, sizeof(Rx_command[0])); // wipe Rx_command
			uint8_t message[data_size] = "\r\n >";
			HAL_UART_Transmit(&huart2, message, data_size, HAL_MAX_DELAY);
			return;
		}
		else{
			Rx_command[Rx_idx] = Rx_data;
			Rx_idx++;
			HAL_UART_Transmit(&huart2, &Rx_data, 1, HAL_MAX_DELAY);
			return;
		}
	}
}

void read_command(SERVO_Cfg* SERVO_1, SERVO_Cfg* SERVO_2){
	//first check the buffer
	if(Rx_idx < 3){ // all validcommands are "char,char,\r"
		/*if(Rx_command[Rx_idx] == '\r'){
			Rx_idx = 0;
			memset(Rx_command, 0, sizeof(Rx_command[0])); // wipe Rx_command
			uint8_t message[data_size] = " - invalid entry, please try again.\r\n >"; // right now just exits
			HAL_UART_Transmit(&huart2, message, data_size, HAL_MAX_DELAY);
		}*/
		return;
	}
	else if(Rx_idx > 3 || Rx_command[2] != '\r'){
		Rx_idx = 0;
		memset(Rx_command, 0, sizeof(Rx_command[0])); // wipe Rx_command
		uint8_t message[data_size] = " - invalid entry, please try again.\r\n >"; // right now just exits
		HAL_UART_Transmit(&huart2, message, data_size, HAL_MAX_DELAY);
		return;
	}
	else{ // 2 char and '\r'
		uint8_t command_1 = Rx_command[0]; // Rx_command could be overwritten while action is occurring
		uint8_t command_2 = Rx_command[1];
		Rx_idx = 0;
		memset(Rx_command, 0, sizeof(Rx_command[0])); // wipe Rx_command
		uint8_t message[data_size] = "\r\n >";
		HAL_UART_Transmit(&huart2, message, data_size, HAL_MAX_DELAY);

		process_command(command_1, SERVO_1);
		process_command(command_2, SERVO_2);
		return;
	}
}

// Code to start the move (adjust PWM) and start the timing delay based on the
// current position.
static void start_move(SERVO_Cfg* SERVO,  unsigned char position )
{
	/*
	 * TO DO
	 * set CCRx to value
	 */
	if(position > 5 || position < 0){
		SERVO->SERVO_STATUS = status_command_error;
		process_status(status_command_error);
		return;
	}
	SERVO->SERVO_POSITION_GOAL = position;
	int8_t difference = abs(SERVO->SERVO_POSITION_GOAL - SERVO->SERVO_POSITION);
	switch(difference){
	case(0):
	// no change in position
			SERVO->SERVO_COUNT_MOVE = 0; // may have been in motion when this occurred
			SERVO->SERVO_STATE = state_at_position;
			SERVO->INDEX++; // look at next move
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
	// __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 110+position*18); //out of 2000, 1 = 0.01 ms
	__HAL_TIM_SET_COMPARE(&htim3, SERVO->PWM_TIM_CH, 110+position*18); //out of 2000, 1 = 0.01 ms
	// set CCRx to value
}
static void start_loop(SERVO_Cfg* SERVO,  unsigned char loop){
	if(SERVO->LOOP_FLAG != 0){
		SERVO->SERVO_STATUS = status_nested_error; // nested error
		process_status(SERVO);
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
		SERVO->SERVO_COUNT_LOOP--; //decrement
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
	process_status(status_paused);
}
void process_recipe(SERVO_Cfg* SERVO, unsigned char recipe[]){ // the servo struct and the recipe for the servo.
	if(SERVO->SERVO_STATUS == status_running){
		/*
		 if(SERVO->INDEX > sizeof(recipe[])/sizeof(recipe[0])){
			// end of recipe
			end_recipe(SERVO);
		}
		 */

		uint8_t rx_data[data_size] = "";
		//snprintf(rx_data, data_size,"index: %u, command: %u \r\n",SERVO->INDEX, recipe[SERVO->INDEX]>>5);
		//HAL_UART_Transmit(&huart2, rx_data, data_size, HAL_MAX_DELAY);
		unsigned char instruction = recipe[SERVO->INDEX]; // for debugging
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
			// added
			case(PRINT_INDEX>>5):
				snprintf(rx_data, data_size,"index: %u \r\n >",SERVO->INDEX);
				HAL_UART_Transmit(&huart2, rx_data, data_size, HAL_MAX_DELAY);
				break;
			case(PRINT_RECIPE>>5):
				snprintf(rx_data, data_size,"Recipe index: %u \r\n >",SERVO->RECIPE_INDEX);
				HAL_UART_Transmit(&huart2, rx_data, data_size, HAL_MAX_DELAY);
				break;
			case(PRINT_OPTIONS>>5):
				snprintf(rx_data, data_size,"P:Pause, C:Continue, R:Right, L:Left, N:NoOp; B:Begin \r\n >");
				HAL_UART_Transmit(&huart2, rx_data, data_size, HAL_MAX_DELAY);
				break;
		}
	}
}
void process_Timestep(SERVO_Cfg* SERVO, unsigned char recipe[]){ // the servo struct and the recipe for the servo.
	//first step is to update the timestep from the previous setup

	uint8_t rx_data[data_size] = "";
	switch(SERVO->SERVO_STATE){
		case(state_moving):
			SERVO->SERVO_COUNT_MOVE--; // decrement the moving counter
			if(SERVO->SERVO_POSITION_GOAL > SERVO->SERVO_POSITION){ // this is important. this depends on the servo moving one position every 100ms. THIS NEEDS TO BE VALIDATED
				SERVO->SERVO_POSITION++;
			}
			else if(SERVO->SERVO_POSITION_GOAL < SERVO->SERVO_POSITION){
				SERVO->SERVO_POSITION--;
			}
			//snprintf(rx_data, data_size,"goal position: %d, current position: %d, count_move: %d \r\n", SERVO->SERVO_POSITION_GOAL, SERVO->SERVO_POSITION, SERVO->SERVO_COUNT_MOVE);
			//HAL_UART_Transmit(&huart2, rx_data, data_size, HAL_MAX_DELAY);

			if(SERVO->SERVO_COUNT_MOVE == 0){
				SERVO->SERVO_STATE = state_at_position; // Finally at position.
				SERVO->INDEX++; // go to next recipe
				process_recipe(SERVO, recipe); // go to next recipe state
			}
			break;

		case(state_at_position):
			//snprintf(rx_data, data_size,"goal position: %d, current position: %d, count: %d \r\n", SERVO->SERVO_POSITION_GOAL, SERVO->SERVO_POSITION, SERVO->SERVO_COUNT_MOVE);
			//HAL_UART_Transmit(&huart2, rx_data, data_size, HAL_MAX_DELAY);

			//SERVO->INDEX++; // go to next recipe
			process_recipe(SERVO, recipe); // go to next recipe state
			break;

		case(state_waiting):
			if(SERVO->SERVO_STATUS == status_paused){
				break;
			}
			SERVO->SERVO_COUNT_WAIT--; // decrement waiting counter
			//snprintf(rx_data, data_size,"waiting: %d, current position: %d \r\n", SERVO->SERVO_COUNT_WAIT, SERVO->SERVO_POSITION);
			//HAL_UART_Transmit(&huart2, rx_data, data_size, HAL_MAX_DELAY);

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
			//snprintf(rx_data, data_size,"That's all folks! \r\n");
			//HAL_UART_Transmit(&huart2, rx_data, data_size, HAL_MAX_DELAY);
			SERVO->SERVO_STATUS = status_paused;
			SERVO->SERVO_STATE = state_recipe_ended;
			SERVO->RECIPE_INDEX++;
			SERVO->INDEX = 0; // go ab
			break;
	}
	process_status(SERVO->SERVO_STATUS);
}

void process_status(enum status one_event){
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

void process_command(uint8_t command, SERVO_Cfg* SERVO){
	if((command == 'P'|| command == 'p')&&
			(SERVO->SERVO_STATUS != status_command_error)&&
			(SERVO->SERVO_STATUS !=status_nested_error)&&
			(SERVO->SERVO_STATE != state_recipe_ended)){
		SERVO->SERVO_STATUS = status_paused;
		process_status(status_paused);
	}
	else if((command == 'C'|| command == 'c')&&
			(SERVO->SERVO_STATUS != status_command_error)&&
			(SERVO->SERVO_STATUS !=status_nested_error)&&
			(SERVO->SERVO_STATE != state_recipe_ended)){
		SERVO->SERVO_STATUS = status_running;
		process_status(status_running);
		}
	else if((command == 'L'|| command == 'l')&&
			(SERVO->SERVO_STATUS == status_paused)&&
			(SERVO->SERVO_POSITION != 5)){
		//will override any current move.
		start_move(SERVO, SERVO->SERVO_POSITION+1);
	}
	else if((command == 'R'|| command == 'r')&&
			(SERVO->SERVO_STATE != state_recipe_ended)&&
			(SERVO->SERVO_POSITION != 0)){
		//will override any current move.
		start_move(SERVO, SERVO->SERVO_POSITION-1);
	}
	else if((command == 'B'|| command == 'b')){
		SERVO->SERVO_STATUS = status_running;
		SERVO->SERVO_STATE = state_unknown;
		process_status(status_running);
		SERVO->INDEX = 0;

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

  int i = 0;

  // define structs for servo1 and servo2
  //SERVO_Cfg servo1 = SERVO_Cfg_1;
  //SERVO_Cfg servo2 = SERVO_Cfg_2;
  struct SERVO_Cfg* servo1_p;
  servo1_p= &SERVO_Cfg_1;
  struct SERVO_Cfg* servo2_p;
  servo2_p= &SERVO_Cfg_2;

  // put servo1 into start status
  //servo1_p->SERVO_STATUS = status_running;
  //servo2_p->SERVO_STATE = state_unknown;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  process_status(status_running);
  SERVO_Cfg_1.SERVO_STATUS = status_paused;
  SERVO_Cfg_1.SERVO_STATE = state_waiting;
  SERVO_Cfg_2.SERVO_STATUS = status_paused;
  SERVO_Cfg_2.SERVO_STATE = state_waiting;

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);   // Output PWM Generation
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);   // Output PWM Generation
  /*
  uint32_t newDutyCycle = 100;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, newDutyCycle);

  HAL_Delay(5000);
  newDutyCycle = 170;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, newDutyCycle);
  HAL_Delay(5000);
  newDutyCycle = 150;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, newDutyCycle);
  HAL_Delay(5000);
  newDutyCycle = 100;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, newDutyCycle);
   */
  uint8_t rx_data[data_size] = "\r\n > ";
  HAL_UART_Transmit(&huart2, rx_data, data_size, HAL_MAX_DELAY);
  while (1)
  {
	  loop_flag = 0;
	  process_Timestep(servo1_p, recipe_test); // recipe for motor 1
	  process_Timestep(servo2_p, recipe_test); // recipe for motor 1
	  read_command(servo1_p, servo2_p);
	  while(loop_flag == 0){
		  ;//HAL_Delay(1); // wait for next tick
	  }

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
