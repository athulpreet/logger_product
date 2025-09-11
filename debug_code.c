/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Program to test Debug UART (USART2) using FreeRTOS
  ******************************************************************************
  * @attention
  *
  * This program uses a thread-safe, queue-based logging task to handle
  * all debug messages sent over USART2. This is the recommended approach for
  * logging in a multi-threaded FreeRTOS application.
  *
  * To run this test:
  * 1. In STM32CubeMX, ensure FreeRTOS is enabled.
  * 2. Ensure USART2 is enabled in Asynchronous mode.
  * 3. Ensure GPIOs for LED_GREEN are configured.
  * 4. Connect to the ST-Link COM port with a serial terminal (PuTTY, etc.)
  * at 115200 baud.
  * 5. You should see the green LED blinking and messages appearing in your
  * terminal every second.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h> // For variadic functions
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- Debug Logging Configuration ---
#define DEBUG_ENABLED 1
#define DEBUG_LOG_QUEUE_LEN 10
#define DEBUG_LOG_MAX_MSG_LEN 100
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
// Externally defined in usart.c
extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// --- RTOS Handles ---
QueueHandle_t g_debug_log_queue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void vDebugLogTask(void *pvParameters);
void vMainTestTask(void *pvParameters);
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

void vMainTestTask(void *pvParameters)
{
    DEBUG_PRINTF("\r\n--- Main Test Task Initialized ---\r\n");
    DEBUG_PRINTF("You should see this message and a blinking green LED.\r\n");
    DEBUG_PRINTF("If you see this, your RTOS UART logging is correct.\r\n");

    while(1)
    {
        DEBUG_PRINTF("Debug message loop...\r\n");
        BSP_LED_Toggle(LED_GREEN);
        vTaskDelay(pdMS_TO_TICKS(1000));
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  BSP_LED_Init(LED_GREEN);

  // --- Create RTOS Objects and Tasks ---
  g_debug_log_queue = xQueueCreate(DEBUG_LOG_QUEUE_LEN, sizeof(char*));
  if (g_debug_log_queue == NULL) {
      Error_Handler();
  }

  xTaskCreate(vDebugLogTask, "DebugLog", 256, NULL, 1, NULL);
  xTaskCreate(vMainTestTask, "MainTest", 256, NULL, 2, NULL);

  // The first message is sent here before the scheduler starts to ensure
  // it gets queued up.
  DEBUG_PRINTF("\r\n--- System Initialized. Starting scheduler... ---\r\n");

  vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // This part of the code should not be reached when the scheduler is running.
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

