/* USER CODE BEGIN Header */
/**
 ******************************************************************************
   @file           : main.c
   @brief          : Main program body
 ******************************************************************************
   @attention

   <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
   All rights reserved.</center></h2>

   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause

 ******************************************************************************
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "gcode_sample_200.h"
#include "gcode_sample_480_string.h"
#include<stdlib.h>
#include<string.h>
#include<math.h>
#include<stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define X_EN_HIGH()      HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2, GPIO_PIN_SET);
#define X_DIR_HIGH()  HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1, GPIO_PIN_SET);
#define X_STEP_HIGH() HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_SET);

#define X_EN_LOW()      HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2, GPIO_PIN_RESET);
#define X_DIR_LOW()   HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1, GPIO_PIN_RESET);
#define X_STEP_LOW()  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9, GPIO_PIN_RESET);

#define Y_EN_HIGH()   HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7, GPIO_PIN_SET);
#define Y_DIR_HIGH()  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8, GPIO_PIN_SET);
#define Y_STEP_HIGH() HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_SET);

#define Y_EN_LOW()    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7, GPIO_PIN_RESET);
#define Y_DIR_LOW()   HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8, GPIO_PIN_RESET);
#define Y_STEP_LOW()  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11, GPIO_PIN_RESET);

#define E_EN_HIGH()   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3, GPIO_PIN_SET);
#define E_DIR_HIGH()  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0, GPIO_PIN_SET);
#define E_STEP_HIGH() HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14, GPIO_PIN_SET);

#define E_EN_LOW()    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3, GPIO_PIN_RESET);
#define E_DIR_LOW()   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0, GPIO_PIN_RESET);
#define E_STEP_LOW()  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14, GPIO_PIN_RESET);

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

//------------------------------------
volatile uint32_t time_tick = 0;

unsigned int curr_millis = 0;
unsigned int prev_millis_x = 0;
unsigned int prev_millis_y = 0;
unsigned int pre_millis = 0;

unsigned int curr_micros = 0;

unsigned int prev_micros_x = 0;
unsigned int prev_micros_y = 0;
unsigned int prev_micros_e = 0;
//-------------------------------------
volatile char toggle_x = 0;
volatile char toggle_y = 0;
int step_count_x = 0;
int step_count_y = 0;

volatile int motor_move_x = 0;
volatile int motor_move_y = 0;

int gcode_index = 0;
int state = 0;

double prev_x = 66;
double prev_y = 82;

/*double prev_x = 0;
  double prev_y = 0;*/

double diff_x = 0;
double diff_y = 0;

unsigned int count_motor_x = 0;
unsigned int count_motor_y = 0;

//Extruder make
char extruder_action_enable = 0;
char toggle_e = 0;

FATFS FatFs;   //Fatfs handle
FIL fil;    //File handle
FRESULT fres; //Result after operations
UINT rc;

unsigned char read_buf[100];
unsigned char gcode_buf[100][50];
unsigned char buf_gcode_str[4][50];
unsigned int file_pos = 0;
int sd_gcode_index = 0;
int buf_index = 0;

#define QUEUE_SIZE 1001
#define SD_READ_SIZE 100
unsigned char BufferQueue[QUEUE_SIZE];
unsigned int Head = 0;
unsigned int Tail = 0;

unsigned char gcode_one_line[50];

int tail_gcode = 0;
int head_gcode = 0;

unsigned char gcode_one_line_exist = 0;

#define SIZE_BUF 486
float buf_gcode[SIZE_BUF][4];
int index_buf = 0;
char which_buf = 0;
char index_char[4] = {'X', 'Y', 'E', 'F'};

enum {
  LEFT = 0x01, RIGHT = 0x02, UP = 0x04, DOWN = 0x08
};

/*int rect_pos[4][2] = {{50, 0}, {50, 50}, {0, 50}, {0, 0}};
  int tri_pos[3][2] = {{50, 0}, {50, 30}, {0, 0}};*/
int speed_x = 0;
int speed_y = 0;

char array_string[4][10];
char string_index[4] = {'X', 'Y', 'E', 'F'};

int which_array = 0;
int index_array = 0;

enum {
  ARRAY_X = 0,
  ARRAY_Y,
  ARRAY_E,
  ARRAY_F
};

uint16_t adc_buf[2];
//float xy_pos_float[200][4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void motor_direction(char dir) {
  if (dir == LEFT) {
    X_DIR_LOW();
  }
  if (dir == RIGHT) {
    X_DIR_HIGH();
  }
  if (dir == UP) {
    Y_DIR_LOW();
  }
  if (dir == DOWN) {
    Y_DIR_HIGH();
  }
}


//======================================================================================


void motor_move_1(double x, double y, int speed)
{
  diff_x = x - prev_x;
  diff_y = y - prev_y;
  if (diff_x < 0)
  {
    motor_direction(LEFT);

  }
  else if (diff_x >= 0)
  {
    motor_direction(RIGHT);
  }

  if (diff_y > 0)
  {
    motor_direction(UP);
  }
  else if (diff_y <= 0)
  {
    motor_direction(DOWN);
  }

  //--------------------------------------------
  // motor count
  if (diff_x >= 0)
  {
    count_motor_x = (int)(diff_x * 80);
  }
  else if (diff_x < 0)
  {
    count_motor_x = (int)(diff_x * (-80));
  }

  if (diff_y >= 0)
  {
    count_motor_y = (int)(diff_y * 80);
  }
  else if (diff_y < 0)
  {
    count_motor_y = (int)(diff_y * (-80));
  }

  if (count_motor_x > 0) motor_move_x = 1;
  if (count_motor_y > 0) motor_move_y = 1;


  prev_x = x;
  prev_y = y;
  //------------------------------------------------------
  // speed

  float speed_per_second = (float)((float)speed / 60.0);

  int half_clock = (int)(12500 / speed_per_second / 2.0);

  float theta = atan2(count_motor_x, count_motor_y);
  if (count_motor_x != 0 && count_motor_y != 0)
  {
    speed_x = (int)(half_clock / sin(theta));
    speed_y = (int)(half_clock / cos(theta));
  }
  else if (count_motor_x == 0 && count_motor_y == 0)
  {
    speed_x = 200;
    speed_y = 200;
  }
  else if (count_motor_x == 0)
  {
    speed_x = 200;
    speed_y = half_clock;
  }
  else if (count_motor_y == 0)
  {
    speed_x = half_clock;
    speed_y = 200;
  }

  TIM2->CNT = 0;
  TIM3->CNT = 0;

  TIM2->ARR = speed_x - 1;
  TIM3->ARR = speed_y - 1;

  TIM2->CR1 = 1;
  TIM3->CR1 = 1;

  curr_micros = prev_micros_x = prev_micros_y = micros();
}



//=====================================================================================



/*void xy_pos_parsing_1()
  {

  for (int i = 0; i < SIZE_BUF; i++)
  {

    for (int j = 0; j < 50; j++)
    {
      for (int k = 0; k < 4; k++)
      {
        buf_gcode_string[i][k][j] = 0;
      }
    }

    int str_len = strlen(xy_pos_string[i]);

    if (xy_pos_string[i][0] == 'G' && xy_pos_string[i][1] == '1')
    {
      for (int j = 2; j < str_len; j++)
      {
        for (int k = 0; k < 4; k++)
        {
          if (xy_pos_string[i][j] == index_char[k])
          {
            index_buf = 0;
            which_buf = k;
          }
        }
        if (xy_pos_string[i][j] != ' ')
          buf_gcode_string[i][which_buf][index_buf++] = xy_pos_string[i][j];
      }
    }
    for (int j = 0; j < 4; j++)
    {
      buf_gcode[i][j] = atof((buf_gcode_string[i][j] + 1));
      if (buf_gcode[i][j] == 0 && i != 0)
      {
        buf_gcode[i][j] = buf_gcode[i - 1][j];
      }

    }
  }


  for (int i = 0; i < SIZE_BUF; i++)
  {
    printf("[%d] ", i);
    printf(" %f", buf_gcode[i][0]);
    printf(" %f", buf_gcode[i][1]);
    printf(" %f", buf_gcode[i][2]);
    printf(" %f\r\n", buf_gcode[i][3]);
  }

  }*/
void xy_pos_parsing_2(unsigned char *gcode_str)
{

  int str_len = strlen(gcode_str);

  if (gcode_str[0] == 'G' && gcode_str[1] == '1')
  {
    for (int j = 2; j < str_len; j++)
    {
      for (int k = 0; k < 4; k++)
      {
        if (gcode_str[j] == index_char[k])
        {
          index_buf = 0;
          which_buf = k;
        }
      }
      if (gcode_str[j] != ' ')
        buf_gcode_str[which_buf][index_buf++] = gcode_str[j];
    }
    tail_gcode = (tail_gcode+1)%SIZE_BUF;
    for (int j = 0; j < 4; j++)
    {
      buf_gcode[tail_gcode][j] = atof((buf_gcode_str[j] + 1));
      if (buf_gcode[tail_gcode][j] == 0 && tail_gcode != 0)
      {
        buf_gcode[tail_gcode][j] = buf_gcode[tail_gcode - 1][j];
      }

    }
//
//    printf("[%d] ", tail_gcode);
//    printf(" %f", buf_gcode[tail_gcode][0]);
//    printf(" %f", buf_gcode[tail_gcode][1]);
//    printf(" %f", buf_gcode[tail_gcode][2]);
//    printf(" %f\r\n", buf_gcode[tail_gcode][3]);
  }
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
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_DMA_Init();
  MX_ADC3_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);

  printf("haha\r\n");

  sd_buf_init();
  sd_init();

  //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);

  //Bed on
  //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  /* xy_pos_parsing_1();*/

  X_EN_LOW();
  Y_EN_LOW();
  E_EN_LOW();
  E_DIR_HIGH();

  //  while (1) {
  //    curr_millis = millis();
  //    curr_micros = micros();
  //    printf("adc_buf[0] = %d , adc_buf[1] = %d\r\n", adc_buf[0] , adc_buf[1]);
  //
  //    if (curr_millis - pre_millis > 100)
  //    {
  //      pre_millis = curr_millis;
  //      if (adc_buf[1] < 350) {
  //        extruder_action_enable = 1;
  //        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  //      }
  //      else if (adc_buf[1] > 370)
  //      {
  //        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  //      }
  //      if (adc_buf[0] < 2950)
  //      {
  //        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
  //      }
  //      else if (adc_buf[0] > 2960)
  //      {
  //        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
  //      }
  //    }
  //    if (curr_micros - prev_micros_e > 3000)
  //    {
  //      prev_micros_e = curr_micros;
  //      if (extruder_action_enable == 1)
  //      {
  //        if (toggle_e == 0)
  //        {
  //          toggle_e = 1;
  //          E_STEP_HIGH();
  //        }
  //        else if (toggle_e == 1)
  //        {
  //          toggle_e = 0;
  //          E_STEP_LOW();
  //        }
  //      }
  //    }
  //  }

  //motor_direction(LEFT);
  //motor_move_x = 1;
  //motor_move_y = 0;


  TIM2->CR1 = 0x01;
  NVIC->ISER[0] |= (0x01 << 28);
  TIM3->CR1 = 0x01;
  NVIC->ISER[0] |= (0x01 << 29);

  while (1) {
	  if((tail_gcode + 1) % SIZE_BUF != head_gcode) //full?´ ?•„?‹ ?•Œ
	  {
		  sd_data_enqueue();
		  get_one_line();
	  }
    if (gcode_one_line_exist == 1)
    {
    	if((tail_gcode + 1) % SIZE_BUF != head_gcode) //full?´ ?•„?‹ ?•Œ
    		{
    	      xy_pos_parsing_2(gcode_one_line);
    	      gcode_one_line_exist = 0;
    		}
    }
    if(tail_gcode != head_gcode)
    {
    	 if (motor_move_x == 0 && motor_move_y == 0)
    	    {
    		 head_gcode = (head_gcode + 1) % SIZE_BUF;
    	      motor_move_1(((double)buf_gcode[head_gcode][0]), (double)(buf_gcode[head_gcode][1]), (int)(buf_gcode[head_gcode][3]));
    	    }
    }
    //===================================================================
    curr_micros = micros();

    /*   if (curr_micros - prev_micros_x >= speed_x) {
         prev_micros_x = curr_micros;


       }
       if (curr_micros - prev_micros_y >= speed_y) {
         prev_micros_y = curr_micros;


       }*/
  }
  sd_close();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 2;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */
  HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc_buf, 2);
  /* USER CODE END ADC3_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
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
  htim2.Init.Prescaler = 8400;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
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


  RCC->APB1ENR |= 0x01;
  TIM2 -> CNT = 0;
  TIM2 -> DIER = 0x01;
  TIM2 -> PSC = 84 - 1;
  TIM2 -> ARR = 200 - 1;

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8400;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
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

  RCC->APB1ENR |= 0x02;
  TIM3 -> CNT = 0;
  TIM3 -> DIER = 0x01;
  TIM3 -> PSC = 84 - 1;
  TIM3 -> ARR = 200 - 1;

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PF1 PF2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC3 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA4 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PE8 PE9 PE11 PE14 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//-------------------------------------------------------
//------------------------
int __io_putchar(int ch) {
  HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, 0xFFFF);

  return ch;
}
//------------------------
void Time_Update(void) {
  time_tick++;
  if (time_tick == 0x400000UL)
    time_tick = 0;

}

unsigned int GetTick() {
  return time_tick;
}

unsigned int millis() {
  return time_tick;
}

#define SYS_CLOCK    168
#define SYSTICK_LOAD (168000 - 1)

unsigned int micros() {
  return (time_tick & 0x3FFFFF) * 1000
         + (SYSTICK_LOAD - SysTick->VAL) / SYS_CLOCK;
}

void delay_ms(unsigned int ms) {
  unsigned int tickstart = GetTick();

  while ((GetTick() - tickstart) < ms)
    ;
}

void delay_us(unsigned int us) {
  unsigned int temp = micros();
  unsigned int comp = temp + us;
  char flag = 0;
  while (comp > temp) {
    if (((time_tick & 0x3FFFFF) == 0) && (flag == 0)) {
      flag = 1;
    }
    if (flag)
      temp = micros() + 0x400000UL * 1000;
    else
      temp = micros();
  }
}

void timer2_isr()
{
  if (motor_move_x == 1)
  {
    if (toggle_x == 1) {
      toggle_x = 0;
      X_STEP_HIGH();
    }
    else if (toggle_x == 0) {
      toggle_x = 1;
      X_STEP_LOW();

      step_count_x++;
      if (step_count_x >= count_motor_x) {
        step_count_x = 0;
        motor_move_x = 0;

        TIM2->CR1 = 0;
      }

    }
  }
}
void timer3_isr()
{
  if (motor_move_y == 1) {

    if (toggle_y == 1) {
      toggle_y = 0;
      Y_STEP_HIGH();
    }
    else if (toggle_y == 0) {
      toggle_y = 1;
      Y_STEP_LOW();

      step_count_y++;
      if (step_count_y >= count_motor_y) {
        step_count_y = 0;
        motor_move_y = 0;

        TIM3->CR1 = 0;
      }


    }
  }
}
//-------------------------------------------------------
void sd_test_code()
{

  FATFS FatFs;  //Fatfs handle
  FIL fil;    //File handle
  FRESULT fres; //Result after operations


  fres = f_mount(&FatFs, "", 1); //1=mount now
  if (fres != FR_OK) {
    printf("f_mount error (%i)\r\n", (int)fres);
    while (1);
  }

  DWORD free_clusters, free_sectors, total_sectors;

  FATFS* getFreeFs;

  fres = f_getfree("", &free_clusters, &getFreeFs);
  if (fres != FR_OK) {
    printf("f_getfree error (%i)\r\n", (int)fres);
    while (1);
  }

  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
  free_sectors = free_clusters * getFreeFs->csize;

  printf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);

  fres = f_open(&fil, "test.txt", FA_READ);
  if (fres != FR_OK) {
    printf("f_open error (%i)\r\n");
    while (1);
  }
  printf("I was able to open 'test.txt' for reading!\r\n");

  BYTE readBuf[30];

  TCHAR* rres = f_gets((TCHAR*)readBuf, 30, &fil);
  if (rres != 0) {
    printf("Read string from 'test.txt' contents: %s\r\n", readBuf);
  } else {
    printf("f_gets error (%i)\r\n", fres);
  }

  f_close(&fil);

  fres = f_open(&fil, "write.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
  if (fres == FR_OK) {
    printf("I was able to open 'write.txt' for writing\r\n");
  } else {
    printf("f_open error (%i)\r\n", fres);
  }

  strncpy((char*)readBuf, "a new file is made!", 19);
  UINT bytesWrote;
  fres = f_write(&fil, readBuf, 19, &bytesWrote);
  if (fres == FR_OK) {
    printf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
  } else {
    printf("f_write error (%i)\r\n");
  }

  f_close(&fil);

  f_mount(NULL, "", 0);

}
void sd_buf_init()
{
  for (int j = 0; j < 50; j++)
    for (int i = 0; i < 50; i++) gcode_buf[j][i] = 0;
}
void sd_init()
{
  fres = f_mount(&FatFs, "", 1); //1=mount now
  if (fres != FR_OK) {
    printf("f_mount error (%i)\r\n", (int)fres);
    while (1);
  }

  fres = f_open(&fil, "gcode.gco", FA_READ);
  if (fres != FR_OK) {
    printf("f_open error (%i)\r\n");
    while (1);
  }
  printf("File Open Succeed 1!\r\n");
}

void sd_read()
{
  //  f_lseek(&fil,file_pos);
  f_read(&fil, read_buf, SD_READ_SIZE, &rc);

  //  for (int i = 0; i < 100; i++)
  //  {
  //    gcode_buf[sd_gcode_index][buf_index++] = read_buf[i];
  //    if (read_buf[i] == '\n')
  //    {
  //      //printf("%s",gcode_buf[gcode_index]);
  //      for (int j = 0; j < 100; j++)
  //      {
  //        printf("%c", gcode_buf[sd_gcode_index][j]);
  //      }
  //
  //      file_pos += buf_index;
  //      buf_index = 0;
  //      sd_gcode_index++;
  //    }
  //  }

  //  while(1)
  //  {
  //      f_read(&fil, &r_data, 1, &rc);
  //      gcode_buf[gcode_index][buf_index++] = r_data;
  //
  //       if(r_data == '\n')
  //       {
  //         printf("%s",gcode_buf[gcode_index]);
  //
  //         buf_index = 0;
  //         gcode_index++;
  //         if(gcode_index == 50)
  //         {
  //           gcode_index = 0;
  //           break;
  //         }
  //       }
  //
  //      HAL_Delay(10);
  //  }
}

void sd_close()
{
  f_close(&fil);

  f_mount(NULL, "", 0);
}

void clear_buffer()
{
  unsigned int i;

  for (i = 0; i < QUEUE_SIZE; i++)
  {
    BufferQueue[i] = 0x00;
  }

}

int IsEmpty(void)
{
  if (Head == Tail)
    return 1;
  else
    return 0;
}

int IsFull(void)
{
  if (((Tail + 1) % QUEUE_SIZE) == Head)
  {
    printf("Buffer Full\n");
    return 1;
  }
  else
    return 0;
}

void EnQueue(unsigned char *dat, int length)
{
  uint32_t i;

  for (i = 0; i < length; i++)
  {
    Tail = (Tail + 1) % QUEUE_SIZE;
    BufferQueue[Tail] = dat[i];
  }
  //printf("Tail : %d\r\n", Tail);
}

unsigned char dequeue_one_char()
{
  Head = (Head + 1) % QUEUE_SIZE;
  unsigned char return_value = BufferQueue[Head];
  return return_value;
}

unsigned int num_of_free_buff()
{
  int free_buff = 0;
  int count_of_buff;
  if (Tail >= Head)
  {
    count_of_buff = Tail - Head;
    free_buff = QUEUE_SIZE - count_of_buff;
  }
  else if (Tail < Head)
  {
    count_of_buff = Tail - Head + QUEUE_SIZE;
    free_buff = QUEUE_SIZE - count_of_buff;
  }

  return free_buff - 1;
}

void sd_data_enqueue()
{
  int remaind_buff = num_of_free_buff();
  //printf("num_of_free_buff %d\r\n", remaind_buff);
  if (remaind_buff >= SD_READ_SIZE)
  {
    sd_read();
    EnQueue(read_buf, SD_READ_SIZE);
  }
}

void get_one_line()
{
  int index_one_line = 0;
  for (int i = 0; i < 50; i++)
  {
    gcode_one_line[i] = 0;
  }
  while (1)
  {
    unsigned char dequeue_data = dequeue_one_char();
    if (dequeue_data == '\n')
    {
      gcode_one_line_exist = 1;
      break;
    }
    else {
      gcode_one_line[index_one_line++] = dequeue_data;
    }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
