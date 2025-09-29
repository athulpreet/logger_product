/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program for GPS Tracker with MQTT reporting to dual servers.
  ******************************************************************************
  * @attention
  *
  * This firmware integrates two main functionalities into a FreeRTOS application:
  * 1.  A GPS Task that listens on UART5 for NMEA sentences, parses them to
  * extract location, speed, date, and time, and stores the data securely.
  * 2.  An MQTT Task that manages a cellular modem on UART4 to connect to the
  * internet and publish the GPS data to a primary and an optional secondary
  * remote server every second.
  *
  * The system uses DMA with idle-line interrupts for efficient, non-blocking
  * UART communication. It is designed to be robust, with automatic reconnection
  * and modem power-cycling logic to handle network errors gracefully.
  * The secondary server functionality can be enabled or disabled at compile time.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpdma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h> // For isdigit

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float latitude;
    float longitude;
    float altitude;
    float speed_knots;
    float course_degrees;
    char time[12];
    char date[8]; // ADDED: To store date "DDMMYY"
    bool is_valid;
} gps_data_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- Buffer Sizes ---
#define UART4_RX_BUFFER_SIZE 256 // For Modem
#define GPS_DMA_RX_BUFFER_SIZE 512 // For GPS
#define MAX_NMEA_SENTENCE_LEN 100

// --- Modem Power Cycle Configuration ---
#define MODEM_PWR_RST_GPIO_Port GPIOC
#define MODEM_PWR_RST_Pin GPIO_PIN_6

// --- Debug Logging Configuration ---
#define DEBUG_ENABLED 1
#define DEBUG_LOG_QUEUE_LEN 15
#define DEBUG_LOG_MAX_MSG_LEN 300 // Increased for larger JSON payloads
#define UART4_RX_QUEUE_LEN 5

// --- NEW: Secondary Server Configuration ---
#define USE_SECONDARY_SERVER 1 // Set to 1 to enable, 0 to disable
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

COM_InitTypeDef BspCOMInit;

/* USER CODE BEGIN PV */


#define IDLE_SPEED_THRESHOLD_KNOTS 2.0f

static gps_data_t g_last_valid_gps_data = { .is_valid = false };
#define MAX_SPEED_METERS_PER_SECOND 85.0 // Max plausible speed (~306 km/h)


// --- MQTT Configuration ---
// --- Primary Server (Socket 0) ---
char g_mqtt_broker_ip[40] = "3.109.116.92";
char g_mqtt_broker_port[6] = "1883";
char g_mqtt_client_id[32] = "spring-client";
char g_mqtt_username[32] = "Thinture";
char g_mqtt_password[32] = "Thinture24";
char g_mqtt_topic[32] = "Test";

// --- NEW: Secondary Server (Socket 1) ---
#if USE_SECONDARY_SERVER
char g_secondary_mqtt_broker_ip[40] = "43.205.58.131";
char g_secondary_mqtt_broker_port[6] = "1884";
char g_secondary_mqtt_client_id[32] = "spring-client";
char g_secondary_mqtt_username[32] = "Thinture";
char g_secondary_mqtt_password[32] = "Thinture";
char g_secondary_mqtt_topic[32] = "Test";
#endif


// --- RTOS Handles ---
QueueHandle_t g_debug_log_queue;
QueueHandle_t g_uart4_rx_queue;
SemaphoreHandle_t g_gps_data_ready_sem;
SemaphoreHandle_t g_gps_data_mutex;

// --- Global Data Structures ---
gps_data_t g_gps_data = {0};

// --- Buffers and State Variables ---
uint8_t g_uart4_dma_rx_buffer[UART4_RX_BUFFER_SIZE];
uint8_t g_gps_dma_rx_buffer[GPS_DMA_RX_BUFFER_SIZE];
volatile uint16_t g_gps_bytes_received = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void vDebugLogTask(void *pvParameters);


void vMqttTask(void *pvParameters);


void vGpsTask(void *pvParameters);


void log_message(const char* format, ...);


bool send_and_wait_for_response(const char* cmd, const char* expected_response, uint32_t timeout_ms);


void perform_modem_power_cycle(void);


void parse_gnrmc(char* gnrmc_sentence);


void parse_gngga(char* gngga_sentence);


float nmea_to_decimal(float nmea_coord, char direction);


void process_gps_buffer(uint8_t* buffer, uint16_t size);


double atof_custom(const char *s);


void format_coord(char* buffer, size_t buffer_size, float coord, char positive_dir, char negative_dir);


void convert_utc_datetime_to_ist(const char* utc_date_str, const char* utc_time_str, char* ist_buffer, size_t buffer_size);


void configure_gps_module(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief Configures the L89 GPS module for optimal fleet tracking in India.
  * @note  This version enables GPS, GLONASS, Galileo, and IRNSS (NavIC).
  */
void configure_gps_module(void) {
    const char* cmd;
    DEBUG_PRINTF("\r\n--- Configuring L89 GPS Module (with IRNSS) ---\r\n");

    // 1. Set the update rate to 1Hz.
    cmd = "$PMTK220,1000*1F\r\n";
    DEBUG_PRINTF("GPS CMD >> %s", cmd);
    HAL_UART_Transmit(&huart5, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(150));

    // 2. Configure NMEA output for only GNRMC and GNGGA.
    cmd = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
    DEBUG_PRINTF("GPS CMD >> %s", cmd);
    HAL_UART_Transmit(&huart5, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(150));

    // 3. Configure constellations: Enable GPS, GLONASS, Galileo, and IRNSS.
    // Format: <gps>,<glonass>,<galileo>,<qzss>,<beidou>,<irnss>
    // 2 = Tracked and used in positioning.
    cmd = "$PSTMCFGCONST,2,2,2,0,0,2\r\n"; // <-- THIS LINE IS UPDATED
    DEBUG_PRINTF("GPS CMD >> %s", cmd);
    HAL_UART_Transmit(&huart5, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(150));

    // 4. (For L89H variant) Enable Dead Reckoning.
    cmd = "$PQCFGDR,1,1,0,100,0*3E\r\n";
    DEBUG_PRINTF("GPS CMD >> %s", cmd);
    HAL_UART_Transmit(&huart5, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(150));

    // 5. Save all settings to the module's flash memory.
    cmd = "$PSTMSAVEPAR\r\n";
    DEBUG_PRINTF("GPS CMD >> %s", cmd);
    HAL_UART_Transmit(&huart5, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(150));

    DEBUG_PRINTF("--- L89 GPS Module Configuration Sent ---\r\n\r\n");
}

/**
  * @brief Calculates the distance between two GPS coordinates in meters.
  * @param lat1 Latitude of the first point in decimal degrees.
  * @param lon1 Longitude of the first point in decimal degrees.
  * @param lat2 Latitude of the second point in decimal degrees.
  * @param lon2 Longitude of the second point in decimal degrees.
  * @return The distance in meters.
  */
double haversine_distance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0; // Earth's radius in meters
    const double PI_DIV_180 = M_PI / 180.0;

    double lat1_rad = lat1 * PI_DIV_180;
    double lon1_rad = lon1 * PI_DIV_180;
    double lat2_rad = lat2 * PI_DIV_180;
    double lon2_rad = lon2 * PI_DIV_180;

    double dLat = lat2_rad - lat1_rad;
    double dLon = lon2_rad - lon1_rad;

    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(lat1_rad) * cos(lat2_rad) *
               sin(dLon / 2) * sin(dLon / 2);

    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return R * c;
}

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
            strncpy(msg_to_send, log_buffer, len);
            msg_to_send[len] = '\0';
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
  * @brief [FIXED] Converts a UTC date and time to an IST date and time string.
  * @param utc_date_str   Input UTC date string in "DDMMYY" format.
  * @param utc_time_str   Input UTC time string in "HHMMSS.sss" format.
  * @param ist_buffer     Output buffer for the formatted IST string.
  * @param buffer_size    Size of the output buffer.
  * @note  The output format is "YYYY-MM-DD HH:MM:SS".
  */
void convert_utc_datetime_to_ist(const char* utc_date_str, const char* utc_time_str, char* ist_buffer, size_t buffer_size) {
    // --- Input Validation ---
    if (utc_date_str == NULL || strlen(utc_date_str) < 6 ||
        utc_time_str == NULL || strlen(utc_time_str) < 6) {
        snprintf(ist_buffer, buffer_size, "2025-01-01 00:00:00");
        return;
    }

    // --- Parse Date and Time from UTC strings ---
    int day   = (utc_date_str[0] - '0') * 10 + (utc_date_str[1] - '0');
    int month = (utc_date_str[2] - '0') * 10 + (utc_date_str[3] - '0');
    int year  = 2000 + (utc_date_str[4] - '0') * 10 + (utc_date_str[5] - '0'); // NMEA is YY format

    int hour   = (utc_time_str[0] - '0') * 10 + (utc_time_str[1] - '0');
    int minute = (utc_time_str[2] - '0') * 10 + (utc_time_str[3] - '0');
    int second = (utc_time_str[4] - '0') * 10 + (utc_time_str[5] - '0');

    // --- Apply IST Offset (+5 hours, +30 minutes) ---
    hour += 5;
    minute += 30;

    // --- Handle Time Rollovers ---
    if (minute >= 60) {
        minute -= 60;
        hour++;
    }

    bool date_changed = false;
    if (hour >= 24) {
        hour -= 24;
        date_changed = true;
    }

    // --- Handle Date Rollover (if time crossed midnight) ---
    if (date_changed) {
        day++;
        const int days_in_month[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
        int month_days = days_in_month[month];

        // Check for leap year
        if (month == 2 && ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))) {
            month_days = 29;
        }

        if (day > month_days) {
            day = 1;
            month++;
            if (month > 12) {
                month = 1;
                year++;
            }
        }
    }

    // --- Format the final IST date and time string ---
    snprintf(ist_buffer, buffer_size, "%04d-%02d-%02d %02d:%02d:%02d", year, month, day, hour, minute, second);
}


/**
  * @brief Formats a float coordinate into the required string format DDD.DDDDDDD[C].
  */
void format_coord(char* buffer, size_t buffer_size, float coord, char positive_dir, char negative_dir) {
    char dir = (coord >= 0) ? positive_dir : negative_dir;
    float coord_abs = fabsf(coord);
    int degrees = (int)coord_abs;
    unsigned long long frac_ll = (unsigned long long)((coord_abs - degrees) * 10000000.0);
    if (frac_ll > 9999999) frac_ll = 9999999;
    unsigned long frac = (unsigned long)frac_ll;
    snprintf(buffer, buffer_size, "%03d.%07lu%c", degrees, frac, dir);
}


/**
  * @brief Custom string-to-float conversion function.
  */
double atof_custom(const char *s)
{
    double a = 0.0;
    int e = 0;
    int c;
    if (!s) return 0.0;
    while ((c = *s++) != '\0' && (c >= '0' && c <= '9')) {
        a = a*10.0 + (c - '0');
    }
    if (c == '.') {
        while ((c = *s++) != '\0' && (c >= '0' && c <= '9')) {
            a = a*10.0 + (c - '0');
            e = e-1;
        }
    }
    while (e < 0) {
        a *= 0.1;
        e++;
    }
    return a;
}


/**
  * @brief Performs a hardware power cycle on the cellular modem.
  */
void perform_modem_power_cycle(void) {
    DEBUG_PRINTF("--- Performing Modem Power Cycle ---\r\n");
    DEBUG_PRINTF("Powering modem OFF...\r\n");
    HAL_GPIO_WritePin(MODEM_PWR_RST_GPIO_Port, MODEM_PWR_RST_Pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(2000));
    DEBUG_PRINTF("Powering modem ON...\r\n");
    HAL_GPIO_WritePin(MODEM_PWR_RST_GPIO_Port, MODEM_PWR_RST_Pin, GPIO_PIN_SET);
    DEBUG_PRINTF("--- Modem Power Cycle Complete. Waiting for boot... ---\r\n");
    vTaskDelay(pdMS_TO_TICKS(12000));
}

/**
  * @brief Converts NMEA coordinate format to decimal degrees.
  */
float nmea_to_decimal(float nmea_coord, char direction) {
    if (nmea_coord == 0.0f) {
        return 0.0f;
    }
    int degrees = (int)(nmea_coord / 100.0f);
    double minutes = nmea_coord - (degrees * 100.0f);
    double decimal_degrees = degrees + (minutes / 60.0f);
    if (direction == 'S' || direction == 'W') {
        decimal_degrees = -decimal_degrees;
    }
    return (float)decimal_degrees;
}

/**
  * @brief Parses a GNGGA NMEA sentence to extract altitude.
  */
void parse_gngga(char* gngga_sentence) {
    char temp_sentence[MAX_NMEA_SENTENCE_LEN];
    strncpy(temp_sentence, gngga_sentence, sizeof(temp_sentence) - 1);
    temp_sentence[sizeof(temp_sentence) - 1] = '\0';

    char* fields[15] = {NULL};
    int field_count = 0;
    char *saveptr; // For strtok_r
    char* token = strtok_r(temp_sentence, ",*", &saveptr);
    while(token != NULL && field_count < 15) {
        fields[field_count++] = token;
        token = strtok_r(NULL, ",*", &saveptr);
    }

    if (field_count >= 10 && fields[9] != NULL && strlen(fields[9]) > 0) {
        float altitude = atof_custom(fields[9]);
        if (xSemaphoreTake(g_gps_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            g_gps_data.altitude = altitude;
            xSemaphoreGive(g_gps_data_mutex);
        }
    }
}

/**
  * @brief Parses a GNRMC NMEA sentence to update GPS data with plausibility check.
  */
void parse_gnrmc(char* gnrmc_sentence) {
    char temp_sentence[MAX_NMEA_SENTENCE_LEN];
    gps_data_t new_data = {0};

    strncpy(temp_sentence, gnrmc_sentence, sizeof(temp_sentence) - 1);
    temp_sentence[sizeof(temp_sentence) - 1] = '\0';

    char* fields[15] = {NULL};
    int field_count = 0;
    char *saveptr;
    char* token = strtok_r(temp_sentence, ",*", &saveptr);
    while(token != NULL && field_count < 15) {
        fields[field_count++] = token;
        token = strtok_r(NULL, ",*", &saveptr);
    }

    if (field_count >= 10 && fields[2] != NULL && fields[2][0] == 'A' &&
        fields[3] != NULL && strlen(fields[3]) > 4 &&
        fields[5] != NULL && strlen(fields[5]) > 5)
    {
        new_data.is_valid = true;
        if (fields[1] != NULL && strlen(fields[1]) < sizeof(new_data.time)) strncpy(new_data.time, fields[1], sizeof(new_data.time) - 1);
        if (fields[9] != NULL && strlen(fields[9]) < sizeof(new_data.date)) strncpy(new_data.date, fields[9], sizeof(new_data.date) - 1);
        new_data.latitude = nmea_to_decimal(atof_custom(fields[3]), fields[4][0]);
        new_data.longitude = nmea_to_decimal(atof_custom(fields[5]), fields[6][0]);
        if (fields[7] != NULL) new_data.speed_knots = atof_custom(fields[7]);
        if (field_count >= 9 && fields[8] != NULL) new_data.course_degrees = atof_custom(fields[8]);

        // --- NEW: MOVEMENT PLAUSIBILITY CHECK ---
        // Check only if we have a previously stored valid point to compare against.
        if (g_last_valid_gps_data.is_valid) {
            double distance_moved = haversine_distance(
                g_last_valid_gps_data.latitude, g_last_valid_gps_data.longitude,
                new_data.latitude, new_data.longitude
            );

            // If the distance moved in 1 second is greater than our max speed, it's a glitch.
            if (distance_moved > MAX_SPEED_METERS_PER_SECOND) {
                new_data.is_valid = false; // Invalidate this new data point.
                DEBUG_PRINTF("!!! Plausibility Check Failed: Impossible jump of %.1f meters detected. Rejecting point.\r\n", distance_moved);
            }
        }
        // --- END OF PLAUSIBILITY CHECK ---

    } else {
        new_data.is_valid = false;
    }

    if (xSemaphoreTake(g_gps_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (new_data.is_valid) {
            new_data.altitude = g_gps_data.altitude; // Preserve altitude
            g_gps_data = new_data; // Update the main global data
            g_last_valid_gps_data = new_data; // ALSO UPDATE the last known good position
        } else {
            g_gps_data.is_valid = false;
        }
        xSemaphoreGive(g_gps_data_mutex);
    }
}

/**
  * @brief Processes the raw GPS DMA buffer to find and parse NMEA sentences.
  */
void process_gps_buffer(uint8_t* buffer, uint16_t size) {
    static char nmea_sentence[MAX_NMEA_SENTENCE_LEN];
    static uint16_t sentence_index = 0;
    if (buffer == NULL || size == 0) return;

    for (uint16_t i = 0; i < size; i++) {
        char ch = (char)buffer[i];
        if (ch == '$') sentence_index = 0;

        if (sentence_index < (sizeof(nmea_sentence) - 1)) {
            nmea_sentence[sentence_index++] = ch;
        }

        if (ch == '\n') {
            nmea_sentence[sentence_index] = '\0';
            if (strncmp(nmea_sentence, "$GNRMC", 6) == 0) parse_gnrmc(nmea_sentence);
            else if (strncmp(nmea_sentence, "$GNGGA", 6) == 0) parse_gngga(nmea_sentence);
            sentence_index = 0;
        }
    }
}

/**
  * @brief RTOS task for handling all GPS-related processing.
  */
void vGpsTask(void *pvParameters) {
    DEBUG_PRINTF("--- GPS Task Started. Listening on UART5... ---\r\n");

    configure_gps_module();

    __HAL_UART_CLEAR_FLAG(&huart5, UART_CLEAR_OREF);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, g_gps_dma_rx_buffer, GPS_DMA_RX_BUFFER_SIZE);

    while(1) {
        if (xSemaphoreTake(g_gps_data_ready_sem, portMAX_DELAY) == pdTRUE) {
            if (g_gps_bytes_received > 0) {
                process_gps_buffer(g_gps_dma_rx_buffer, g_gps_bytes_received);
            }
            memset(g_gps_dma_rx_buffer, 0, GPS_DMA_RX_BUFFER_SIZE);
            HAL_UARTEx_ReceiveToIdle_DMA(&huart5, g_gps_dma_rx_buffer, GPS_DMA_RX_BUFFER_SIZE);
        }
    }
}

/**
  * @brief  Reception Event Callback for all UARTs.
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == UART4) { // Modem
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        uint8_t received_data[UART4_RX_BUFFER_SIZE + 1] = {0};
        memcpy(received_data, g_uart4_dma_rx_buffer, Size);
        xQueueSendFromISR(g_uart4_rx_queue, &received_data, &xHigherPriorityTaskWoken);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, g_uart4_dma_rx_buffer, UART4_RX_BUFFER_SIZE);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    } else if (huart->Instance == UART5) { // GPS
        g_gps_bytes_received = Size;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(g_gps_data_ready_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
  * @brief  UART error callback for all UARTs.
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART4) { // Modem
        uint32_t error_code = HAL_UART_GetError(huart);
        DEBUG_PRINTF("!!! UART4 Error, code: 0x%lX. Restarting DMA. !!!\r\n", error_code);
        HAL_UART_AbortReceive(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart4_dma_rx_buffer, UART4_RX_BUFFER_SIZE);
    } else if (huart->Instance == UART5) { // GPS
        uint32_t error_code = HAL_UART_GetError(huart);
        DEBUG_PRINTF("!!! UART5 GPS Error, code: 0x%lX. Restarting DMA. !!!\r\n", error_code);
        HAL_UART_AbortReceive(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, g_gps_dma_rx_buffer, GPS_DMA_RX_BUFFER_SIZE);
    }
}

/**
  * @brief Sends a command and waits for a specific response from the modem.
  */
bool send_and_wait_for_response(const char* cmd, const char* expected_response, uint32_t timeout_ms) {
    uint8_t rx_buffer[UART4_RX_BUFFER_SIZE + 1];
    xQueueReset(g_uart4_rx_queue);
    DEBUG_PRINTF("CMD >> %s\r\n", cmd);
    HAL_UART_Transmit(&huart4, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);

    TickType_t start_ticks = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start_ticks) < pdMS_TO_TICKS(timeout_ms)) {
        if (xQueueReceive(g_uart4_rx_queue, &rx_buffer, pdMS_TO_TICKS(100)) == pdPASS) {
            // Don't print the whole buffer if it's just the payload
            if (strlen((char*)rx_buffer) < 50) {
                 DEBUG_PRINTF("RSP << %s\r\n", (char*)rx_buffer);
            }
            if (strstr((char*)rx_buffer, expected_response) != NULL) {
                return true;
            }
        }
    }
    DEBUG_PRINTF("ERROR: Timeout waiting for '%s'\r\n", expected_response);
    return false;
}

/**
  * @brief Main task to handle MQTT connection and publishing with idle drift filtering.
  * @note This version is modified to handle a primary and a secondary MQTT server.
  */
/**
 * @brief MQTT Task to manage connections and data publishing to two servers simultaneously.
 *
 * This task handles independent connection management for a primary and a secondary MQTT server.
 * It uses robust response checking to ensure connections are correctly identified and cleans up
 * failed connection attempts to ensure stability.
 */
/**
 * @brief MQTT Task to manage connections and data publishing to two servers simultaneously.
 *
 * This task performs a one-time network setup to activate a PDP context. It then
 * handles independent, simultaneous connection management for a primary and a secondary
 * MQTT server. It uses robust response checking and error handling to ensure stability.
 */
// --- Add this global variable declaration at the top of your file ---
// --- IMPORTANT: Replace "internet" with your SIM card's actual APN ---
char g_apn[32] = "internet"; // Example: "airtelgprs.com", "jionet", "www"

/**
 * @brief MQTT Task based on the user's proven single-server logic, now correctly
 * adapted for two simultaneous servers.
 */
void vMqttTask(void *pvParameters) {
    DEBUG_PRINTF("--- MQTT Task Started ---\r\n");
    char command_buffer[256];
    char mqtt_payload[512];
    uint8_t primary_connection_retries = 0;
    const uint8_t max_retries_before_halt = 5;

    // --- Connection state for each server ---
    bool is_primary_connected = false;
    #if USE_SECONDARY_SERVER
    bool is_secondary_connected = false;
    #endif

    // Static variable to store the last reported position for drift filtering.
    static gps_data_t last_reported_gps_data = { .is_valid = false };

    // Initial delay and setup
    vTaskDelay(pdMS_TO_TICKS(5000));
    __HAL_UART_CLEAR_FLAG(&huart4, UART_CLEAR_OREF);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, g_uart4_dma_rx_buffer, UART4_RX_BUFFER_SIZE);
    perform_modem_power_cycle();

    DEBUG_PRINTF("Waiting for modem to respond...\r\n");
    while(!send_and_wait_for_response("AT\r\n", "OK", 1000)) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    send_and_wait_for_response("ATE0\r\n", "OK", 1000);
    DEBUG_PRINTF("Modem is responsive.\r\n");

    // --- ONE-TIME PDP CONTEXT SETUP ---
    // This block now runs once at the start to establish the main data connection.
    DEBUG_PRINTF("--- Configuring Network Data Connection (PDP Context) ---\r\n");
    bool pdp_ready = false;
    while (!pdp_ready) {
        // This reliable sequence is from your working single-server code.
        if (send_and_wait_for_response("AT+CGATT=1\r\n", "OK", 8000)) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            // Deactivate context 1 first to prevent "Operation not allowed" error. Ignore failure.
            send_and_wait_for_response("AT+CGACT=0,1\r\n", "OK", 8000);
            vTaskDelay(pdMS_TO_TICKS(1000));

            snprintf(command_buffer, sizeof(command_buffer), "AT+CGDCONT=1,\"IP\",\"%s\"\r\n", g_apn);
            if (send_and_wait_for_response(command_buffer, "OK", 5000)) {
                if (send_and_wait_for_response("AT+CGACT=1,1\r\n", "OK", 60000)) {
                    DEBUG_PRINTF("--- Network Data Connection is Active ---\r\n");
                    pdp_ready = true;
                }
            }
        }
        if (!pdp_ready) {
            DEBUG_PRINTF("!!! Network Data Connection Failed. Retrying... !!!\r\n");
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }

    // --- Main Task Loop ---
    while(1) {
        // --- Primary Server Connection Management ---
        if (!is_primary_connected) {
            DEBUG_PRINTF("--- MQTT Primary Connection Attempt ---\r\n");
            bool setup_ok = false;
            send_and_wait_for_response("AT+QMTCLOSE=0\r\n", "OK", 5000); // Close just in case
            vTaskDelay(pdMS_TO_TICKS(500));
            do {
                snprintf(command_buffer, sizeof(command_buffer), "AT+QMTOPEN=0,\"%s\",%s\r\n", g_mqtt_broker_ip, g_mqtt_broker_port);
                if (!send_and_wait_for_response(command_buffer, "+QMTOPEN: 0,0", 20000)) break;
                vTaskDelay(pdMS_TO_TICKS(1000));

                snprintf(command_buffer, sizeof(command_buffer), "AT+QMTCONN=0,\"%s\",\"%s\",\"%s\"\r\n", g_mqtt_client_id, g_mqtt_username, g_mqtt_password);
                if (!send_and_wait_for_response(command_buffer, "+QMTCONN: 0,0,0", 20000)) break;

                setup_ok = true;
            } while(0);

            if (setup_ok) {
                is_primary_connected = true;
                primary_connection_retries = 0;
                DEBUG_PRINTF("--- MQTT Primary Connection Successful ---\r\n");
            } else {
                DEBUG_PRINTF("--- MQTT Primary Connection Failed ---\r\n");
                primary_connection_retries++;
                if (primary_connection_retries >= max_retries_before_halt) {
                    DEBUG_PRINTF("!!! Max retries for primary. Rebooting modem. !!!\r\n");
                    perform_modem_power_cycle();
                    // After reboot, PDP needs to be re-established
                    pdp_ready = false;
                    // Break from inner loops and let the main loop restart the PDP process
                    vTaskDelay(pdMS_TO_TICKS(5000));
                    continue;
                }
            }
        }

        // --- Secondary Server Connection Management ---
        #if USE_SECONDARY_SERVER
        if (!is_secondary_connected) {
            DEBUG_PRINTF("--- MQTT Secondary Connection Attempt ---\r\n");
            bool setup_ok = false;
            send_and_wait_for_response("AT+QMTCLOSE=1\r\n", "OK", 5000); // Close just in case
            vTaskDelay(pdMS_TO_TICKS(500));
            do {
                snprintf(command_buffer, sizeof(command_buffer), "AT+QMTOPEN=1,\"%s\",%s\r\n", g_secondary_mqtt_broker_ip, g_secondary_mqtt_broker_port);
                if (!send_and_wait_for_response(command_buffer, "+QMTOPEN: 1,0", 20000)) break;
                vTaskDelay(pdMS_TO_TICKS(1000));

                if (strlen(g_secondary_mqtt_username) > 0) {
                     snprintf(command_buffer, sizeof(command_buffer), "AT+QMTCONN=1,\"%s\",\"%s\",\"%s\"\r\n", g_secondary_mqtt_client_id, g_secondary_mqtt_username, g_secondary_mqtt_password);
                } else {
                     snprintf(command_buffer, sizeof(command_buffer), "AT+QMTCONN=1,\"%s\"\r\n", g_secondary_mqtt_client_id);
                }

                // FIX: Check for the exact success code. A response like "+QMTCONN: 1,0,5" will now correctly be treated as a failure.
                if (!send_and_wait_for_response(command_buffer, "+QMTCONN: 1,0,0", 20000)) break;

                setup_ok = true;
            } while(0);

            if (setup_ok) {
                is_secondary_connected = true;
                DEBUG_PRINTF("--- MQTT Secondary Connection Successful ---\r\n");
            } else {
                DEBUG_PRINTF("--- MQTT Secondary Connection Failed. Check credentials or broker status. Will retry next cycle. ---\r\n");
            }
        }
        #endif

        // --- Data Acquisition & Payload Creation (Your complete logic) ---
        gps_data_t current_gps_data = {0};
        if (xSemaphoreTake(g_gps_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            current_gps_data = g_gps_data;
            xSemaphoreGive(g_gps_data_mutex);
        }

        if (current_gps_data.is_valid) {
            if (!last_reported_gps_data.is_valid || current_gps_data.speed_knots > IDLE_SPEED_THRESHOLD_KNOTS) {
                last_reported_gps_data = current_gps_data;
            }
        } else {
            last_reported_gps_data.is_valid = false;
        }

        if (last_reported_gps_data.is_valid) {
            char lat_str[25], lon_str[25], speed_str[4], course_str[4];
            char timestamp_str[20];
            const char* vehicle_status = (current_gps_data.speed_knots < IDLE_SPEED_THRESHOLD_KNOTS) ? "IDLE" : "RUNNING";

            convert_utc_datetime_to_ist(current_gps_data.date, current_gps_data.time, timestamp_str, sizeof(timestamp_str));
            format_coord(lat_str, sizeof(lat_str), last_reported_gps_data.latitude, 'N', 'S');
            format_coord(lon_str, sizeof(lon_str), last_reported_gps_data.longitude, 'E', 'W');
            snprintf(course_str, sizeof(course_str), "%03d", (int)roundf(last_reported_gps_data.course_degrees));

            if (strcmp(vehicle_status, "IDLE") == 0) {
                snprintf(speed_str, sizeof(speed_str), "000");
            } else {
                float speed_kmh = last_reported_gps_data.speed_knots * 1.852;
                snprintf(speed_str, sizeof(speed_str), "%03d", (int)roundf(speed_kmh));
            }

            int offset = 0;
            offset += snprintf(mqtt_payload + offset, sizeof(mqtt_payload) - offset, "{\"deviceID\":\"THIN0011\",\"IMEI\":\"864501070030500\",\"timestamp\":\"%s\"", timestamp_str);
            offset += snprintf(mqtt_payload + offset, sizeof(mqtt_payload) - offset, ",\"dataValidity\":\"Valid\",\"status\":\"N1\",\"latitude\":\"%s\"", lat_str);
            offset += snprintf(mqtt_payload + offset, sizeof(mqtt_payload) - offset, ",\"longitude\":\"%s\",\"speed\":\"%s\",\"course\":\"%s\"", lon_str, speed_str, course_str);
            offset += snprintf(mqtt_payload + offset, sizeof(mqtt_payload) - offset, ",\"ignition\":\"IGon\",\"vehicleStatus\":\"%s\"", vehicle_status);
            offset += snprintf(mqtt_payload + offset, sizeof(mqtt_payload) - offset, ",\"additionalData\":\"0000000000000\",\"timeIntervals\":\"002,010,002\",");
            offset += snprintf(mqtt_payload + offset, sizeof(mqtt_payload) - offset, "\"angleInterval\":\"015\",\"distanceInterval\":\"100\",\"gsmStrength\":\"073\",\"sequenceNumber\":\"Q263\"}");
        } else {
            char timestamp_str[20];
            snprintf(timestamp_str, sizeof(timestamp_str), "2025-09-29 12:13:00");
            snprintf(mqtt_payload, sizeof(mqtt_payload),
                     "NO GPS FIX");
        }

        // --- Publishing Loop ---
        if (is_primary_connected) {
            snprintf(command_buffer, sizeof(command_buffer), "AT+QMTPUBEX=0,0,0,0,\"%s\",%d\r\n", g_mqtt_topic, (int)strlen(mqtt_payload));
            if (send_and_wait_for_response(command_buffer, ">", 5000)) {
                if (send_and_wait_for_response(mqtt_payload, "+QMTPUBEX: 0,0,0", 10000)) {
                    DEBUG_PRINTF("Publish OK (Primary)\r\n");
                    BSP_LED_Toggle(LED_GREEN);
                } else {
                    DEBUG_PRINTF("Publish FAILED (Primary). Marking for reconnect.\r\n");
                    is_primary_connected = false;
                }
            } else {
                 DEBUG_PRINTF("Publish command FAILED (Primary). Marking for reconnect.\r\n");
                is_primary_connected = false;
            }
        }

        #if USE_SECONDARY_SERVER
        if (is_secondary_connected) {
            vTaskDelay(pdMS_TO_TICKS(200));
            snprintf(command_buffer, sizeof(command_buffer), "AT+QMTPUBEX=1,0,0,0,\"%s\",%d\r\n", g_secondary_mqtt_topic, (int)strlen(mqtt_payload));
             if (send_and_wait_for_response(command_buffer, ">", 5000)) {
                if (send_and_wait_for_response(mqtt_payload, "+QMTPUBEX: 1,0,0", 10000)) {
                    DEBUG_PRINTF("Publish OK (Secondary)\r\n");
                } else {
                    DEBUG_PRINTF("Publish FAILED (Secondary). Marking for reconnect.\r\n");
                    is_secondary_connected = false;
                }
            } else {
                 DEBUG_PRINTF("Publish FAILED (Secondary). Marking for reconnect.\r\n");
                is_secondary_connected = false;
            }
        }
        #endif

        vTaskDelay(pdMS_TO_TICKS(5000));
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
  MX_GPDMA2_Init();
  MX_GPIO_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);

  // --- Create RTOS Objects ---
  g_debug_log_queue = xQueueCreate(DEBUG_LOG_QUEUE_LEN, sizeof(char*));
  g_uart4_rx_queue = xQueueCreate(UART4_RX_QUEUE_LEN, UART4_RX_BUFFER_SIZE + 1);
  g_gps_data_ready_sem = xSemaphoreCreateBinary();
  g_gps_data_mutex = xSemaphoreCreateMutex();

  if (!g_debug_log_queue || !g_uart4_rx_queue || !g_gps_data_ready_sem || !g_gps_data_mutex) {
      Error_Handler();
  }

  // --- Create RTOS Tasks ---
  xTaskCreate(vDebugLogTask, "DebugLog", 512, NULL, 1, NULL);
  // MODIFIED: Swapped task priorities. GPS data handling is more time-critical
  // than MQTT communication and should have higher priority to prevent UART overruns.
  xTaskCreate(vGpsTask, "GpsTask", 2048, NULL, 3, NULL); // Priority changed from 2 to 3
  xTaskCreate(vMqttTask, "MqttTask", 2048, NULL, 2, NULL); // Priority changed from 3 to 2

  DEBUG_PRINTF("\r\n--- System Initialized. Starting scheduler... ---\r\n");

  vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_1);
}

/* USER CODE BEGIN 4 */
// The contents of your usart.c file (MX_UART_Init functions and MSP functions)
// would typically go here or in their own file. For simplicity in this single-file
// view, they are assumed to be correctly generated by CubeMX.
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
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
      BSP_LED_On(LED_RED); // Turn on Red LED for permanent error
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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

