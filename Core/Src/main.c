/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "lcd.h"
#include "stdlib.h"
#include "string.h"
#include <stdbool.h>
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
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const uint8_t key_code [8][2] =
{
		{1,2},
		{5,6},
		{9,10},
		{13,14},
		{3,4},
		{7,8},
		{11,12},
		{15,16},
};
uint8_t key_current=0;
uint8_t key_prev=0;

void selectRow(uint8_t row)
{
	switch (row){
	case 0:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET);
	break;
	case 1:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, SET);
		break;
	case 3:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, SET);
		break;
	case 4:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET);
		break;
	case 5:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET);
		break;
	case 6:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, SET);
		break;
	case 7:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, SET);
		break;
	}
};

uint8_t row;
uint8_t col;
uint8_t Keypad_Getkey()
{
	for (row=0 ; row<8 ; row ++)
	{
		selectRow(row);
		HAL_Delay(2);
			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == 0)
			{
				HAL_Delay(50);
				if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == 0){
					return key_code[row][0];
				}
			}
			else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 0)
			{
				HAL_Delay(50);
				if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == 0){
					return key_code[row][1];
				}
			}
	}
	return 0;
}

const char keypad_layout[2][16]= {"789+456-123xs0=:", "g<>+def-abcxSc=:"};
bool splash = false;

int64_t Result(char * string){
	int64_t temp_var[10] = {0,0,0,0,0,0,0,0,0,0};
	uint8_t num = 0;
	uint8_t tmp[19] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint8_t num_cnt = 0;
	uint8_t equaltion[9] = {0,0,0,0,0,0,0,0,0};
	for(uint8_t i = 0; i < strlen(string); i++){
		int tempk = string[i]-'0';
		if (tempk < 10 && tempk >= 0){
			tmp[num_cnt] = tempk;
			num_cnt ++;
		}
		else {
			if (string[i] == '+'){
				equaltion[num] = 1;
			}
			else if (string[i] == '-') {
				equaltion[num] = 2;
			}
			else if (string[i] == 'x') {
				equaltion[num] = 3;
			}
			else if (string[i] == ':') {
				equaltion[num] = 4;
			}
			for(uint8_t j = 0; j < num_cnt; j++){
				uint64_t tempkk = tmp[j];
				for(uint8_t k =0; k < num_cnt-j-1; k++){
					tempkk = tempkk*10;
				}
				temp_var[num] = temp_var[num] + tempkk;
				tmp[j] = 0;
			}
			num_cnt = 0;
			num ++;
		}
	}
	for(uint8_t i = 0; i < 10; i++){
		if (equaltion[i] == 3) {
			temp_var[i+1] = temp_var[i] * temp_var[i+1];
			temp_var[i] = 0;
			if (i>0){
				equaltion[i] = equaltion[i-1];
			}
			else equaltion[i] = 1;
		}
		else if (equaltion[i] == 4) {
			temp_var[i+1] = temp_var[i] / temp_var[i+1];
			temp_var[i] = 0;
			if (i>0){
				equaltion[i] = equaltion[i-1];
			}
			else equaltion[i] = 1;
		}
	}
	for(uint8_t i = 0; i < 10; i++){
		if (equaltion[i] == 1) {
			temp_var[i+1] = temp_var[i] + temp_var[i+1];
		}
		else if (equaltion[i] == 2) {
			temp_var[i+1] = temp_var[i] - temp_var[i+1];
		}
		else if (equaltion[i] == 0) {
			return temp_var[i];
		}
	}
}

// A utility function to reverse a string
void reverse(char str[], int length)
{
    int start = 0;
    int end = length - 1;
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        end--;
        start++;
    }
}
// Implementation of citoa()
char* citoa(int64_t num, char* str, int base)
{
    int i = 0;
    bool isNegative = false;

    /* Handle 0 explicitly, otherwise empty string is
     * printed for 0 */
    if (num == 0) {
        str[i++] = '0';
        str[i] = '\0';
        return str;
    }

    // In standard itoa(), negative numbers are handled
    // only with base 10. Otherwise numbers are
    // considered unsigned.
    if (num < 0 && base == 10) {
        isNegative = true;
        num = -num;
    }

    // Process individual digits
    while (num != 0) {
        int rem = num % base;
        str[i++] = (rem > 9) ? (rem - 10) + 'a' : rem + '0';
        num = num / base;
    }

    // If number is negative, append '-'
    if (isNegative)
        str[i++] = '-';

    str[i] = '\0'; // Append string terminator

    // Reverse the string
    reverse(str, i);

    return str;
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  Lcd_PortType ports[] = { GPIOA, GPIOA, GPIOA, GPIOA };
  Lcd_PinType pins[] = {LCD_D4_Pin, LCD_D5_Pin, LCD_D6_Pin, LCD_D7_Pin};
  Lcd_HandleTypeDef lcd;
  lcd = Lcd_create(ports, pins, GPIOA, LCD_RS_Pin, GPIOA, LCD_E_Pin, LCD_4_BIT_MODE);

  Lcd_cursor(&lcd, 0, 0);
  Lcd_string(&lcd, "BTL ESD NHOM 18");
  HAL_Delay(1500);
  Lcd_clear(&lcd);

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t cursor_cnt = 0;
  uint8_t splash_cnt = 0;
  uint8_t cursor_max = 0;
  uint8_t lcd_zero = 0;
  uint8_t current_layout = 0;
  char s;
  char *lcd_buffer=(char*) malloc(200+1);
  memset(lcd_buffer,0,strlen(lcd_buffer));
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  key_current = Keypad_Getkey();
	  if(key_current != 0 && key_current != key_prev) {
		  s = keypad_layout[current_layout][key_current-1];

		  if (s == keypad_layout[0][12]) {
			  current_layout = 1;
		  }
		  else if (s == keypad_layout[1][12]) {
			  current_layout = 0;
		  }
		  else if (s == keypad_layout[1][13]) {
			  Lcd_clear(&lcd);
			  cursor_cnt = 0;
			  memset(lcd_buffer,0,strlen(lcd_buffer));
		  }
		  else if (s == keypad_layout[0][14]) {
			  char snum[19];
			  lcd_buffer[cursor_max] = s;
			  citoa(Result(lcd_buffer), snum, 10);
			  Lcd_cursor(&lcd, 1, 0);
			  Lcd_string(&lcd, snum);
		  }
		  else if (s == '<') {
			  cursor_cnt --;
		  }
		  else if (s == '>') {
			  if (cursor_cnt < cursor_max) {
				  cursor_cnt ++;
			  }
		  }
		  else {
			  lcd_buffer[cursor_cnt] = s;

			  cursor_cnt ++;
			  cursor_max = cursor_cnt;
		  }
		  // print lcd
		  if (cursor_cnt>15){
			  lcd_zero = cursor_cnt-15;
		  }
		  else {
			  lcd_zero = 0;
		  }
		  char subbuff[16];
		  memcpy( subbuff, &lcd_buffer[lcd_zero], 15 );
		  subbuff[15] = '\0';
		  Lcd_cursor(&lcd, 0, 0);
		  Lcd_string(&lcd, subbuff);

		  if (cursor_cnt>15){
			  	  splash_cnt = 15;
			 }
		  else splash_cnt = cursor_cnt;

	  }
	  if (splash){
		  Lcd_cursor(&lcd, 0, splash_cnt);
		  Lcd_string(&lcd, " ");
	  }
	  else {
		  Lcd_cursor(&lcd, 0, splash_cnt);
		  Lcd_string(&lcd, "_");

	  }
	  key_prev = key_current;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 6399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 5000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RS_Pin|LCD_E_Pin|LCD_D4_Pin|LCD_D5_Pin
                          |LCD_D6_Pin|LCD_D7_Pin|R2_Pin|R1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(R0_GPIO_Port, R0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_RS_Pin LCD_E_Pin LCD_D4_Pin LCD_D5_Pin
                           LCD_D6_Pin LCD_D7_Pin R2_Pin R1_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_E_Pin|LCD_D4_Pin|LCD_D5_Pin
                          |LCD_D6_Pin|LCD_D7_Pin|R2_Pin|R1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : R0_Pin */
  GPIO_InitStruct.Pin = R0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(R0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : C2_Pin C1_Pin */
  GPIO_InitStruct.Pin = C2_Pin|C1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if (htim == &htim3){
		splash = !splash;
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
