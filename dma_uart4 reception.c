/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Receives data on UART4 via DMA and prints it to a debug port.
  ******************************************************************************
  * @attention
  *
  * This program uses a dedicated RTOS task to listen for incoming UART data.
  * Reception is handled by DMA with an idle-line interrupt, making it suitable
  * for messages of variable length. Received data is then printed to USART2
  * using a thread-safe logging task.
  *
  * This version includes a larger buffer and a HAL_UART_ErrorCallback to
  * gracefully handle UART overrun errors and automatically restart reception,
  * making the system more robust.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpdma.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h> // For variadic functions
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART4_RX_BUFFER_SIZE 256 // Increased buffer size

// --- Debug Logging Configuration ---
#define DEBUG_ENABLED 1
#define DEBUG_LOG_QUEUE_LEN 10
#define DEBUG_LOG_MAX_MSG_LEN 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#if DEBUG_ENABLED
#define DEBUG_PRINTF(...) log_message(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
// Externally defined in their respective .c files
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
// --- RTOS Handles ---
QueueHandle_t g_debug_log_queue;
SemaphoreHandle_t g_uart4_rx_sem;

// --- Buffers and state variables ---
uint8_t g_uart4_rx_buffer[UART4_RX_BUFFER_SIZE];
volatile uint16_t g_uart4_bytes_received = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void vDebugLogTask(void *pvParameters);
void vUart4RxTask(void *pvParameters);
void log_message(const char* format, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#if DEBUG_ENABLED
void log_message(const char* format, ...) {
    char log_buffer[DEBUG_LOG_MAX_MSG_LEN];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(log_buffer, sizeof(log_buffer), format, args);
    va_end(args);

    if (len > 0 && g_debug_log_queue != NULL) {
        char* msg_to_send = pvPortMalloc(len + 1);
        if (msg_to_send != NULL) {
            strcpy(msg_to_send, log_buffer);
            if (xQueueSend(g_debug_log_queue, &msg_to_send, (TickType_t)0) != pdPASS) {
                vPortFree(msg_to_send);
            }
        }
    }
}

void vDebugLogTask(void *pvParameters) {
    char* msg_ptr;
    log_message("--- RTOS Logging Task Initialized ---\r\n");
    for (;;) {
        if (xQueueReceive(g_debug_log_queue, &msg_ptr, portMAX_DELAY) == pdPASS) {
            if (msg_ptr != NULL) {
                HAL_UART_Transmit(&huart2, (uint8_t*)msg_ptr, strlen(msg_ptr), HAL_MAX_DELAY);
                vPortFree(msg_ptr);
            }
        }
    }
}
#endif // DEBUG_ENABLED

/**
  * @brief  Reception Event Callback (triggers on idle line or full buffer).
  * @param  huart: UART handle
  * @param  Size: Number of data received
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART4)
    {
        g_uart4_bytes_received = Size;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(g_uart4_rx_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
  * @brief  UART error callback.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
        uint32_t error_code = HAL_UART_GetError(huart);
        DEBUG_PRINTF("!!! UART4 Error, code: 0x%lX. Restarting DMA reception. !!!\r\n", error_code);

        // Abort any ongoing transfers and clear flags, then restart reception.
        // This is a robust way to recover from errors like Overrun.
        HAL_UART_AbortReceive(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart4_rx_buffer, UART4_RX_BUFFER_SIZE);
    }
}


/**
 * @brief Task to handle received data from UART4
 */
void vUart4RxTask(void *pvParameters)
{
    DEBUG_PRINTF("--- UART4 RX Task Started. Listening for data... ---\r\n");

    if(HAL_UARTEx_ReceiveToIdle_DMA(&huart4, g_uart4_rx_buffer, UART4_RX_BUFFER_SIZE) != HAL_OK)
    {
        DEBUG_PRINTF("ERROR: Failed to start UART4 DMA reception.\r\n");
        Error_Handler();
    }

    while(1)
    {
        if (xSemaphoreTake(g_uart4_rx_sem, portMAX_DELAY) == pdTRUE)
        {
            uint8_t local_buffer[UART4_RX_BUFFER_SIZE + 1]; // +1 for null terminator
            uint16_t local_bytes_received;

            taskENTER_CRITICAL();
            local_bytes_received = g_uart4_bytes_received;
            memcpy(local_buffer, g_uart4_rx_buffer, local_bytes_received);
            taskEXIT_CRITICAL();

            // IMPORTANT: Restart the DMA reception to be able to receive again
            if(HAL_UARTEx_ReceiveToIdle_DMA(&huart4, g_uart4_rx_buffer, UART4_RX_BUFFER_SIZE) != HAL_OK)
            {
                 DEBUG_PRINTF("ERROR: Failed to restart UART4 DMA reception after normal receive.\r\n");
            }

            if (local_bytes_received > 0)
            {
                // Null-terminate and print the local copy
                local_buffer[local_bytes_received] = '\0';
                DEBUG_PRINTF("UART4 RX (%u bytes): %s\r\n", local_bytes_received, (char*)local_buffer);
                BSP_LED_Toggle(LED_GREEN);
            }
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
  MX_GPDMA2_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);

  // --- Create RTOS Objects and Tasks ---
  g_debug_log_queue = xQueueCreate(DEBUG_LOG_QUEUE_LEN, sizeof(char*));
  g_uart4_rx_sem = xSemaphoreCreateBinary();

  if (g_debug_log_queue == NULL || g_uart4_rx_sem == NULL) {
      Error_Handler();
  }

  xTaskCreate(vDebugLogTask, "DebugLog", 512, NULL, 1, NULL);
  xTaskCreate(vUart4RxTask, "Uart4Rx", 512, NULL, 2, NULL);

  DEBUG_PRINTF("\r\n--- System Initialized. Starting scheduler... ---\r\n");

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 250;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
      BSP_LED_On(LED_RED); // Turn on Red LED for permanent error
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
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

