/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART1_RX_FIFO_SIZE 256U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
static uint8_t uart1_rx_irq_byte;
static volatile uint8_t uart1_rx_fifo[UART1_RX_FIFO_SIZE];
static volatile uint16_t uart1_rx_head;
static volatile uint16_t uart1_rx_tail;
static volatile uint8_t uart1_rx_fault;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Send one USART HMI instruction followed by three binary FF bytes. */
static HAL_StatusTypeDef HMI_SendCmd(const char *cmd)
{
  static const uint8_t end[3] = {0xFF, 0xFF, 0xFF};
  size_t length = strlen(cmd);

  if ((length == 0U) || (length > 65535U))
  {
    return HAL_ERROR;
  }
  HAL_StatusTypeDef status = HAL_UART_Transmit(&huart3, (uint8_t *)cmd,
                                               (uint16_t)length, 100U);
  if (status != HAL_OK)
  {
    return status;
  }
  return HAL_UART_Transmit(&huart3, (uint8_t *)end, sizeof(end), 100U);
}

static HAL_StatusTypeDef HMI_SetCount(uint32_t count)
{
  char cmd[32];
  int length = snprintf(cmd, sizeof(cmd), "n0.val=%lu",
                        (unsigned long)count);
  if ((length <= 0) || (length >= (int)sizeof(cmd)))
  {
    return HAL_ERROR;
  }
  return HMI_SendCmd(cmd);
}

/* The interrupt stores bytes; only the main loop parses commands and writes UARTs. */
static uint8_t UART1_Dequeue(uint8_t *byte)
{
  uint16_t tail = uart1_rx_tail;
  if (tail == uart1_rx_head)
  {
    return 0U;
  }
  *byte = uart1_rx_fifo[tail];
  uart1_rx_tail = (uint16_t)((tail + 1U) % UART1_RX_FIFO_SIZE);
  return 1U;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uint16_t next = (uint16_t)((uart1_rx_head + 1U) % UART1_RX_FIFO_SIZE);
    if (next != uart1_rx_tail)
    {
      uart1_rx_fifo[uart1_rx_head] = uart1_rx_irq_byte;
      uart1_rx_head = next;
    }
    else
    {
      uart1_rx_fault = 1U;
    }
    if (HAL_UART_Receive_IT(huart, &uart1_rx_irq_byte, 1U) != HAL_OK)
    {
      uart1_rx_fault = 1U;
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uart1_rx_fault = 1U;
    if (huart->RxState == HAL_UART_STATE_READY)
    {
      __HAL_UART_CLEAR_OREFLAG(huart);
      (void)HAL_UART_Receive_IT(huart, &uart1_rx_irq_byte, 1U);
    }
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
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  uint32_t last_blink_ms = HAL_GetTick();
  uint32_t blink_count = 0;
  uint8_t rx_byte;
  char tx_buf[64];
  static const uint8_t boot_message[] = "F407 bringup ready\r\n";

  HAL_UART_Transmit(&huart1, (uint8_t *)boot_message,
                    sizeof(boot_message) - 1U, 100U);
  GPIO_PinState pc1_last_raw = HAL_GPIO_ReadPin(BOARD_KEY_GPIO_Port, BOARD_KEY_Pin);
  GPIO_PinState pc1_stable = pc1_last_raw;
  uint32_t pc1_changed_ms = HAL_GetTick();
  uint32_t pc1_count = 0;
  char pc1_msg[64];
  char hmi_console_cmd[96];
  uint8_t hmi_console_length = 0;
  uint8_t hmi_discard_until_newline = 0U;
  char rx_log[sizeof(hmi_console_cmd) + 8U];

  /* Give the independently powered screen time to enter its first page. */
  HAL_Delay(1000U);
  HAL_StatusTypeDef hmi_status = HMI_SendCmd("t0.txt=\"STM32 OK\"");
  if (hmi_status == HAL_OK)
  {
    hmi_status = HMI_SetCount(0U);
  }
  static const uint8_t hmi_ready[] =
      "HMI USART3 ready. Type an HMI command and press Enter.\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t *)hmi_ready,
                    sizeof(hmi_ready) - 1U, 100U);
  if (hmi_status != HAL_OK)
  {
    static const uint8_t hmi_error[] = "HMI UART transmit error\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)hmi_error,
                      sizeof(hmi_error) - 1U, 100U);
  }
  if (HAL_UART_Receive_IT(&huart1, &uart1_rx_irq_byte, 1U) != HAL_OK)
  {
    static const uint8_t rx_start_error[] = "USART1 receive start failed\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)rx_start_error,
                      sizeof(rx_start_error) - 1U, 100U);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* PC0 is the board LED: low level means on. */
    if ((HAL_GetTick() - last_blink_ms) >= 500U)
    {
      last_blink_ms = HAL_GetTick();
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

      int length = snprintf(tx_buf, sizeof(tx_buf),
                            "LED toggle %lu\r\n", (unsigned long)++blink_count);
      if ((length > 0) && (length < (int)sizeof(tx_buf)))
      {
        HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf,
                          (uint16_t)length, 100U);
      }
    }

    /* An RX error invalidates the current command; wait for a fresh line. */
    if (uart1_rx_fault != 0U)
    {
      __disable_irq();
      uart1_rx_tail = uart1_rx_head;
      uart1_rx_fault = 0U;
      __enable_irq();
      hmi_console_length = 0U;
      hmi_discard_until_newline = 1U;
      static const uint8_t rx_error[] = "RX overrun/error; command discarded\r\n";
      HAL_UART_Transmit(&huart1, (uint8_t *)rx_error,
                        sizeof(rx_error) - 1U, 100U);
    }

    /* PC -> USART1 -> MCU -> USART3 -> screen. Enter ends one command. */
    while (UART1_Dequeue(&rx_byte) != 0U)
    {
      if (hmi_discard_until_newline != 0U)
      {
        if ((rx_byte == '\r') || (rx_byte == '\n'))
        {
          hmi_discard_until_newline = 0U;
        }
        continue;
      }

      if ((rx_byte == '\r') || (rx_byte == '\n'))
      {
        if (hmi_console_length > 0U)
        {
          hmi_console_cmd[hmi_console_length] = '\0';
          int rx_length = snprintf(rx_log, sizeof(rx_log),
                                   "RX: %s\r\n", hmi_console_cmd);
          if ((rx_length > 0) && (rx_length < (int)sizeof(rx_log)))
          {
            HAL_UART_Transmit(&huart1, (uint8_t *)rx_log,
                              (uint16_t)rx_length, 100U);
          }
          HAL_StatusTypeDef status = HMI_SendCmd(hmi_console_cmd);
          static const uint8_t sent[] = "HMI command sent\r\n";
          static const uint8_t failed[] = "HMI UART transmit error\r\n";
          const uint8_t *message = (status == HAL_OK) ? sent : failed;
          uint16_t message_length = (status == HAL_OK)
                                  ? (uint16_t)(sizeof(sent) - 1U)
                                  : (uint16_t)(sizeof(failed) - 1U);
          HAL_UART_Transmit(&huart1, (uint8_t *)message,
                            message_length, 100U);
          hmi_console_length = 0U;
        }
      }
      else if (hmi_console_length < (sizeof(hmi_console_cmd) - 1U))
      {
        hmi_console_cmd[hmi_console_length++] = (char)rx_byte;
      }
      else
      {
        hmi_console_length = 0U;
        hmi_discard_until_newline = 1U;
        static const uint8_t too_long[] = "HMI command too long; discarded\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t *)too_long,
                          sizeof(too_long) - 1U, 100U);
      }
    }

    /* Debounce PC1 and count one event per physical press. */
    GPIO_PinState raw = HAL_GPIO_ReadPin(BOARD_KEY_GPIO_Port, BOARD_KEY_Pin);
    uint32_t now = HAL_GetTick();

    if (raw != pc1_last_raw)
    {
      pc1_last_raw = raw;
      pc1_changed_ms = now;
    }

    if ((raw != pc1_stable) && ((now - pc1_changed_ms) >= 30U))
    {
      pc1_stable = raw;
      int length;

      if (raw == GPIO_PIN_RESET)
      {
        length = snprintf(pc1_msg, sizeof(pc1_msg),
                          "PC1 pressed: LOW, count=%lu\r\n",
                          (unsigned long)++pc1_count);
        if (HMI_SetCount(pc1_count) != HAL_OK)
        {
          static const uint8_t hmi_error[] = "HMI UART transmit error\r\n";
          HAL_UART_Transmit(&huart1, (uint8_t *)hmi_error,
                            sizeof(hmi_error) - 1U, 100U);
        }
      }
      else
      {
        length = snprintf(pc1_msg, sizeof(pc1_msg),
                          "PC1 released: HIGH\r\n");
      }

      if ((length > 0) && (length < (int)sizeof(pc1_msg)))
      {
        HAL_UART_Transmit(&huart1, (uint8_t *)pc1_msg,
                          (uint16_t)length, 100U);
      }
    }
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOARD_KEY_Pin */
  GPIO_InitStruct.Pin = BOARD_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BOARD_KEY_GPIO_Port, &GPIO_InitStruct);

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
