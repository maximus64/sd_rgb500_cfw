/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <inttypes.h>
#include "main.h"
#include "usb_device.h"
#include "usbd_laser.h"
#include "stm32f1xx_hal_dac_ex.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LASER_RED_PIN     GPIO_PIN_12
#define LASER_GREEN_PIN   GPIO_PIN_13
#define LASER_BLUE_PIN    GPIO_PIN_14

#define LASER_RED_PORT    GPIOB
#define LASER_GREEN_PORT  GPIOB
#define LASER_BLUE_PORT   GPIOB

#define LASER_ON_THRESSHOLD 128

#define OUTPUT_TIMER_PERIOD (1500 - 1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;

UART_HandleTypeDef huart1;

enum buffer_event {
  BUFFER_NORMAL,
  BUFFER_UNDERFLOW,
  BUFFER_OVERFLOW,
};

enum output_speed_fsm_state {
  OUTPUT_FSM_NORMAL = 0,
  OUTPUT_FSM_FAST,
  OUTPUT_FSM_SLOW,
  OUTPUT_FSM_WAIT_NORMAL,
};

enum timer_speed {
  TIMER_SPEED_NORMAL = 0,
  TIMER_SPEED_SLOW,
  TIMER_SPEED_FAST,
};

static enum output_speed_fsm_state _output_fsm;

/* USER CODE BEGIN PV */
TIM_HandleTypeDef htim_TIM6;

extern USBD_LASER_HandleTypeDef laserd_handle;
static USBD_LASER_HandleTypeDef *hlaser = &laserd_handle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void TIMER_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint16_t get_ring_capacity(void)
{
    uint16_t ring_cap =  (hlaser->wr_ptr >= hlaser->rd_ptr)
                ? (uint16_t) (hlaser->wr_ptr - hlaser->rd_ptr)
                : LASER_RING_BUF_SIZE - hlaser->rd_ptr + hlaser->wr_ptr;
    return ring_cap;
}

static void set_timer_speed(enum timer_speed speed)
{
  switch (speed)
  {
    case TIMER_SPEED_NORMAL:
      printf("Switch output timer to normal speed.\r\n");
      htim_TIM6.Instance->ARR = (uint32_t)(OUTPUT_TIMER_PERIOD);
      break;
    case TIMER_SPEED_SLOW:
      printf("Switch output timer to SLOW speed.\r\n");
      htim_TIM6.Instance->ARR = (uint32_t)(OUTPUT_TIMER_PERIOD + 1);
      break;
    case TIMER_SPEED_FAST:
      printf("Switch output timer to FAST speed.\r\n");
      htim_TIM6.Instance->ARR = (uint32_t)(OUTPUT_TIMER_PERIOD - 1);
      break;
    default:
      Error_Handler();
  }
}

static void output_speed_fsm(enum buffer_event ev)
{
  switch (_output_fsm)
  {
  case OUTPUT_FSM_NORMAL:
    if (ev == BUFFER_UNDERFLOW) {
      set_timer_speed(TIMER_SPEED_SLOW);
      _output_fsm = OUTPUT_FSM_SLOW;
    }
    else if (ev == BUFFER_OVERFLOW) {
      set_timer_speed(TIMER_SPEED_FAST);
      _output_fsm = OUTPUT_FSM_FAST;
    }
    break;
  case OUTPUT_FSM_SLOW:
    if (ev == BUFFER_OVERFLOW) {
      set_timer_speed(TIMER_SPEED_NORMAL);
      _output_fsm = OUTPUT_FSM_WAIT_NORMAL;
    }
    break;
  case OUTPUT_FSM_FAST:
    if (ev == BUFFER_UNDERFLOW) {
      set_timer_speed(TIMER_SPEED_NORMAL);
      _output_fsm = OUTPUT_FSM_WAIT_NORMAL;
    }
    break;
  case OUTPUT_FSM_WAIT_NORMAL:
    if (ev == BUFFER_NORMAL) {
      _output_fsm = OUTPUT_FSM_NORMAL;
    }
    break;
  default:
    Error_Handler();
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
  MX_DAC_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  TIMER_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
  if (HAL_DAC_Start(&hdac, DAC_CHANNEL_2) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
  printf("Hello World\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint16_t ring_cap = get_ring_capacity();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    printf("buffer read enable=%d ring_capacity=%u underflow=%u overerflow=%u\r\n",
      hlaser->rd_enable, ring_cap, hlaser->underflow_cnt, hlaser->overflow_cnt);

    if (hlaser->rd_enable)
    {
      if (ring_cap < 8)
      {
        output_speed_fsm(BUFFER_UNDERFLOW);
      }
      else if (ring_cap > 24)
      {
        output_speed_fsm(BUFFER_OVERFLOW);
      }
      else
      {
        output_speed_fsm(BUFFER_NORMAL);
      }
    }

    HAL_Delay(200U);
  }
  /* USER CODE END 3 */
}

static void set_laser_rgb(uint8_t r, uint8_t g, uint8_t b)
{
  if (r > LASER_ON_THRESSHOLD)
    HAL_GPIO_WritePin(LASER_RED_PORT, LASER_RED_PIN, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(LASER_RED_PORT, LASER_RED_PIN, GPIO_PIN_RESET);

  if (g > LASER_ON_THRESSHOLD)
    HAL_GPIO_WritePin(LASER_GREEN_PORT, LASER_GREEN_PIN, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(LASER_GREEN_PORT, LASER_GREEN_PIN, GPIO_PIN_RESET);

  if (b > LASER_ON_THRESSHOLD)
    HAL_GPIO_WritePin(LASER_BLUE_PORT, LASER_BLUE_PIN, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(LASER_BLUE_PORT, LASER_BLUE_PIN, GPIO_PIN_RESET);
}

static void disable_laser_output(void)
{
  set_laser_rgb(0,0,0);
}

static void underflow_handler(void)
{
  /* Disable laser output */
  disable_laser_output();

  /* reset output FSM and output timer speed */
  _output_fsm = OUTPUT_FSM_NORMAL;
  set_timer_speed(TIMER_SPEED_NORMAL);

  /* Disable sample read and wait for resync */
  hlaser->rd_enable = 0;
}

static void pop_sample(void)
{
  static unsigned sample_ptr;
  static int last_underflow;
  sample_pkg_t *sample;

  if (!hlaser->rd_enable)
    return;

  // check if ring buffer empty
  if (hlaser->rd_ptr == hlaser->wr_ptr)
  {
    if (!last_underflow)
    {
      hlaser->underflow_cnt++;
      last_underflow = 1;

      //let resync
      underflow_handler();
    }
    return;
  }
  else
  {
    last_underflow = 0;
  }

  sample = (sample_pkg_t*) hlaser->buffer[hlaser->rd_ptr];

  /* Set XY DAC output */
  if (HAL_DACEx_DualSetValue(&hdac,
                        DAC_ALIGN_12B_L,
                        sample[sample_ptr].y,
                        (uint16_t)(~sample[sample_ptr].x)) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  /* Set RGB output */
  set_laser_rgb(sample[sample_ptr].r,
                sample[sample_ptr].g,
                sample[sample_ptr].b);

  sample_ptr += 1;
  if (sample_ptr == NUM_SAMPLES_PER_RING)
  {
    sample_ptr = 0;
    hlaser->rd_ptr = (hlaser->rd_ptr + 1) % LASER_RING_BUF_SIZE;
  }
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config 
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void TIMER_Init(void)
{
  /* Set TIMx instance */
  htim_TIM6.Instance = TIM6;

  htim_TIM6.Init.Period            = OUTPUT_TIMER_PERIOD;
  htim_TIM6.Init.Prescaler         = 0;
  htim_TIM6.Init.ClockDivision     = 0;
  htim_TIM6.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim_TIM6.Init.RepetitionCounter = 0;
  htim_TIM6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_Base_Init(&htim_TIM6) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if (HAL_TIM_Base_Start_IT(&htim_TIM6) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  (void) htim;
  pop_sample();
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* Disable Laser output */
  disable_laser_output();

  printf("Error Handler called\r\n");
  while (1)
  {
    //TODO: fix disable laser logic
    disable_laser_output();
    continue;
  }
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
