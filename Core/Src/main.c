/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "main.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#define DWT_CTRL				(*(volatile uint32_t*) 0xE0001000)
#define N_BUTTONS 			(4)

TaskHandle_t button_task_handle = NULL;
TaskHandle_t control_led_task_handle = NULL;

QueueHandle_t event_q_handle;

Button b1;

Button button[N_BUTTONS];

Button_Event button_event = {0};

void read_button_task(void *params);
void control_led_task(void *params);

extern void SEGGER_UART_init(uint32_t);

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
  /* USER CODE BEGIN 2 */

	DWT_CTRL |= (1 << 0);

	SEGGER_UART_init(500000);
	SEGGER_SYSVIEW_Conf();

  BaseType_t status;
  status = xTaskCreate(read_button_task, "Button Task", 200, NULL, 2, &button_task_handle);
  configASSERT(status == pdPASS);

  status = xTaskCreate(control_led_task, "Led control task", 200, NULL, 1, &control_led_task_handle);
  configASSERT(status == pdPASS);

  event_q_handle = xQueueCreate(10, sizeof(Button_Event));
  configASSERT(event_q_handle != 0);

  button_init(&b1, GPIOB, GPIO_PIN_3);
//  for (uint8_t i = 0; i < N_BUTTONS; ++i) {
//  	button_init(&button[i], GPIOB, GPIO_PIN_0);
//  }
  button_init(&button[0], GPIOB, GPIO_PIN_3);
  button_init(&button[1], GPIOB, GPIO_PIN_4);
  button_init(&button[2], GPIOB, GPIO_PIN_5);
  button_init(&button[3], GPIOB, GPIO_PIN_6);

  button_event_init(&button_event, 0, BUTTON_STATE_IDLE);

  vTaskStartScheduler();
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

uint16_t static read_gpio_port(GPIO_TypeDef* GPIOx) {
	return GPIOx->IDR & 0xFFFF;
}

void static decode_buttons(Button button[], uint8_t n_buttons, uint16_t port_val) {
	uint8_t i = 0;
	uint8_t btn_port_val = (uint8_t) (port_val & 0x78);
	while (i < n_buttons) {
		button_set_state(&button[i], (btn_port_val >> (i + 3)) & 1);				// extract the value of bits which are associated with buttons
		if (button_is_state_changed(&button[i]) == true) {
			button_event.id = i;
			button_event.state = button[i].state;
			xQueueSend(event_q_handle, &button_event, 0);
		}
		i++;
	}
}

void read_button_task(void *params) {
	TickType_t last_wake_time;
	last_wake_time = xTaskGetTickCount();

	while (1) {
		decode_buttons(button, N_BUTTONS, read_gpio_port(GPIOB));
//		button_set_state(&b1, HAL_GPIO_ReadPin(b1.port, b1.pin));
//		if (button_is_released(&b1)) {
//			xTaskNotify(control_led_task_handle, 0, eNoAction);
//		}
		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5));
	}
}

void control_led_task(void *params) {
//	uint16_t led_arr[] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};
//	uint8_t cnt = 0;
	Button_Event received_event;
	while (1) {
//		if (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) == pdPASS) {
//			HAL_GPIO_TogglePin(GPIOB, led_arr[cnt % 4]);
//			cnt++;
//		}
		if (xQueueReceive(event_q_handle, &received_event, portMAX_DELAY) == pdTRUE) {
			switch (received_event.id) {
				case 0:
					if (received_event.state == BUTTON_STATE_RELEASED) {
						HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
					}
//					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
					break;
				case 1:
					if (received_event.state == BUTTON_STATE_RELEASED) {
						HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
					}
//					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
					break;
				case 2:
					if (received_event.state == BUTTON_STATE_RELEASED) {
						HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
					}
//					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
					break;
				case 3:
					if (received_event.state == BUTTON_STATE_RELEASED) {
						HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
					}
//					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
					break;
				default:
					break;
			}
		}
	}
}
/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
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
