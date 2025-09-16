/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Connects to MQTT and publishes a message every second.
  ******************************************************************************
  * @attention
  *
  * This program uses a dedicated RTOS task to handle all modem communication.
  * It connects to a cellular network, establishes an MQTT connection, and then
  * periodically publishes a message.
  *
  * UART reception is handled efficiently by DMA with an idle-line interrupt,
  * and a queue is used to pass received data from the ISR to the processing task.
  *
  * This version includes robust error handling that will power-cycle the modem
  * on connection failure and attempt to reconnect automatically.
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
#include <stdbool.h> // For bool type
#include <stdarg.h> // For variadic functions
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART4_RX_BUFFER_SIZE 256

// --- Modem Power Cycle Configuration ---
#define MODEM_PWR_RST_GPIO_Port GPIOC
#define MODEM_PWR_RST_Pin GPIO_PIN_6

// --- Debug Logging Configuration ---
#define DEBUG_ENABLED 1
#define DEBUG_LOG_QUEUE_LEN 10
#define DEBUG_LOG_MAX_MSG_LEN 256
#define UART4_RX_QUEUE_LEN 5
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
// --- MQTT Configuration ---
char g_mqtt_broker_ip[40] = "3.109.116.92";
char g_mqtt_broker_port[6] = "1883";
char g_mqtt_client_id[32] = "spring-client";
char g_mqtt_username[32] = "Thinture";
char g_mqtt_password[32] = "Thinture24";
char g_mqtt_topic[32] = "Test1";

// --- RTOS Handles ---
QueueHandle_t g_debug_log_queue;
QueueHandle_t g_uart4_rx_queue;

// --- Buffers and state variables ---
uint8_t g_uart4_dma_rx_buffer[UART4_RX_BUFFER_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void vDebugLogTask(void *pvParameters);
void vMqttTask(void *pvParameters);
void log_message(const char* format, ...);
bool send_and_wait_for_response(const char* cmd, const char* expected_response, uint32_t timeout_ms);
void perform_modem_power_cycle(void);
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
 * @brief Performs a hardware power cycle on the cellular modem.
 */
void perform_modem_power_cycle(void)
{
    DEBUG_PRINTF("--- Performing Modem Power Cycle ---\r\n");

    // Note: If a GPS is running on another UART, you might want to temporarily
    // halt its DMA reception here, e.g., HAL_UART_AbortReceive(&huart3);

    DEBUG_PRINTF("Powering modem OFF...\r\n");
    // Assuming RESET (logic low) turns the modem power off
    HAL_GPIO_WritePin(MODEM_PWR_RST_GPIO_Port, MODEM_PWR_RST_Pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(2000));

    DEBUG_PRINTF("Powering modem ON...\r\n");
    // Assuming SET (logic high) turns the modem power on
    HAL_GPIO_WritePin(MODEM_PWR_RST_GPIO_Port, MODEM_PWR_RST_Pin, GPIO_PIN_SET);

    DEBUG_PRINTF("--- Modem Power Cycle Complete. Waiting for boot... ---\r\n");
    vTaskDelay(pdMS_TO_TICKS(12000)); // Wait for modem to boot up fully

    // Note: Restart GPS DMA reception here if it was stopped earlier
    // e.g., HAL_UARTEx_ReceiveToIdle_DMA(&huart3, g_gps_dma_rx_buffer, ...);
}


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
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        uint8_t received_data[UART4_RX_BUFFER_SIZE + 1] = {0};
        memcpy(received_data, g_uart4_dma_rx_buffer, Size);

        xQueueSendFromISR(g_uart4_rx_queue, &received_data, &xHigherPriorityTaskWoken);

        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, g_uart4_dma_rx_buffer, UART4_RX_BUFFER_SIZE);

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
        HAL_UART_AbortReceive(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart4_dma_rx_buffer, UART4_RX_BUFFER_SIZE);
    }
}

/**
 * @brief Sends a command and waits for a specific response from the modem.
 * @param cmd The AT command to send.
 * @param expected_response The string to look for in the reply.
 * @param timeout_ms Maximum time to wait for the response.
 * @return true if the expected response was received, false otherwise.
 */
bool send_and_wait_for_response(const char* cmd, const char* expected_response, uint32_t timeout_ms)
{
    uint8_t rx_buffer[UART4_RX_BUFFER_SIZE + 1];
    xQueueReset(g_uart4_rx_queue);

    DEBUG_PRINTF("CMD >> %s", cmd);
    HAL_UART_Transmit(&huart4, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);

    TickType_t start_ticks = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start_ticks) < pdMS_TO_TICKS(timeout_ms))
    {
        if (xQueueReceive(g_uart4_rx_queue, &rx_buffer, pdMS_TO_TICKS(100)) == pdPASS)
        {
            DEBUG_PRINTF("RSP << %s\r\n", (char*)rx_buffer);
            if (strstr((char*)rx_buffer, expected_response) != NULL)
            {
                DEBUG_PRINTF("SUCCESS: Found '%s'\r\n", expected_response);
                return true;
            }
        }
    }

    DEBUG_PRINTF("ERROR: Timeout waiting for '%s'\r\n", expected_response);
    return false;
}

/**
 * @brief Main task to handle MQTT connection and publishing.
 */
void vMqttTask(void *pvParameters)
{
    DEBUG_PRINTF("--- MQTT Task Started ---\r\n");
    char command_buffer[256];
    uint8_t connection_retries = 0;
    const uint8_t max_retries = 3;

    // Start listening on UART4 for modem responses
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, g_uart4_dma_rx_buffer, UART4_RX_BUFFER_SIZE);
    vTaskDelay(pdMS_TO_TICKS(2000)); // Give modem time to boot initially

    // Main loop for the task
    while(1)
    {
        bool is_connected = false;

        // --- Connection Loop ---
        while(!is_connected)
        {
            DEBUG_PRINTF("--- MQTT Connection Attempt #%d ---\r\n", connection_retries + 1);
            bool setup_ok = false;
            do {
                if (!send_and_wait_for_response("AT\r\n", "OK", 1000)) break;
                vTaskDelay(pdMS_TO_TICKS(500));

                if (!send_and_wait_for_response("AT+CGATT=1\r\n", "OK", 8000)) break;
                vTaskDelay(pdMS_TO_TICKS(2000)); // Increased delay for network registration

                // Deactivate any old context to ensure a clean slate.
                // We don't check the return value because it's okay if it fails.
                send_and_wait_for_response("AT+CGACT=0,1\r\n", "OK", 8000);
                vTaskDelay(pdMS_TO_TICKS(500));

                snprintf(command_buffer, sizeof(command_buffer), "AT+CGDCONT=1,\"IP\",\"%s\"\r\n", "internet");
                if (!send_and_wait_for_response(command_buffer, "OK", 5000)) break;
                vTaskDelay(pdMS_TO_TICKS(500));

                if (!send_and_wait_for_response("AT+CGACT=1,1\r\n", "OK", 10000)) break;
                vTaskDelay(pdMS_TO_TICKS(1000));

                snprintf(command_buffer, sizeof(command_buffer), "AT+QMTOPEN=0,\"%s\",%s\r\n", g_mqtt_broker_ip, g_mqtt_broker_port);
                if (!send_and_wait_for_response(command_buffer, "+QMTOPEN: 0,0", 20000)) break;
                vTaskDelay(pdMS_TO_TICKS(1000));

                snprintf(command_buffer, sizeof(command_buffer), "AT+QMTCONN=0,\"%s\",\"%s\",\"%s\"\r\n", g_mqtt_client_id, g_mqtt_username, g_mqtt_password);
                if (!send_and_wait_for_response(command_buffer, "+QMTCONN: 0,0,0", 20000)) break;

                setup_ok = true;
            } while(0);

            if (setup_ok) {
                is_connected = true;
                connection_retries = 0; // Reset counter on success
                DEBUG_PRINTF("\r\n--- MQTT Connection Successful ---\r\n\r\n");
            } else {
                DEBUG_PRINTF("\r\n--- MQTT Connection Failed ---\r\n");
                connection_retries++;
                if (connection_retries >= max_retries) {
                    DEBUG_PRINTF("!!! Max connection retries reached. Halting. !!!\r\n");
                    Error_Handler();
                }
                perform_modem_power_cycle();
            }
        } // --- End of Connection Loop ---

        // --- Publishing Loop ---
        while(is_connected)
        {
            char payload[] = "hello";
            snprintf(command_buffer, sizeof(command_buffer), "AT+QMTPUBEX=0,0,0,0,\"%s\",%d\r\n", g_mqtt_topic, (int)strlen(payload));

            if (send_and_wait_for_response(command_buffer, ">", 5000))
            {
                if (send_and_wait_for_response(payload, "+QMTPUBEX: 0,0,0", 10000)) {
                    DEBUG_PRINTF("Publish OK\r\n");
                    BSP_LED_Toggle(LED_GREEN);
                } else {
                    DEBUG_PRINTF("Publish payload failed. Connection likely lost.\r\n");
                    is_connected = false; // This will break the loop and trigger reconnection
                }
            } else {
                 DEBUG_PRINTF("Publish command failed. Connection likely lost.\r\n");
                 is_connected = false; // This will break the loop and trigger reconnection
            }

            if(is_connected) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        } // --- End of Publishing Loop ---
    } // --- End of Main Task Loop ---
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
  g_uart4_rx_queue = xQueueCreate(UART4_RX_QUEUE_LEN, UART4_RX_BUFFER_SIZE + 1);

  if (g_debug_log_queue == NULL || g_uart4_rx_queue == NULL) {
      Error_Handler();
  }

  // Create the two tasks for our application
  xTaskCreate(vDebugLogTask, "DebugLog", 512, NULL, 1, NULL);
  xTaskCreate(vMqttTask, "MqttTask", 1024, NULL, 2, NULL);

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

