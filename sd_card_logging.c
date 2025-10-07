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
#include "i2c.h"
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

#include "fatfs.h"


#include<stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define MAX_LOG_ENTRY_SIZE 100
#define DATALOF_FILE "datalog.txt"
#define HEADER_FILE "HEADER.txt"
#define LOG_INTERVAL_MS 1000   // Log one entry every 1000 milliseconds











#define LOG_INTERVAL_SECONDS 5
#define HOURS_TO_STORE 24
#define TOTAL_LOG_ENTRIES ((uint32_t)((HOURS_TO_STORE * 3600UL) / LOG_INTERVAL_SECONDS))  // 51,840 entries
//#define TOTAL_LOG_ENTRIES 100

#define METADATA_FILE "logmeta.txt"
#define DATALOG_FILE "datalog.txt"










typedef struct {
    float latitude;
    float longitude;
    float altitude;
    float speed_knots;
    float course_degrees;
    char time[12];
    char date[8];
    bool is_valid;
} gps_data_t;


#define MAX_LINES 10
char GPS_log[256];


typedef struct {
    uint32_t current_position;
    uint32_t total_entries_written;
    uint8_t buffer_full;
} LogMetadata;

typedef struct {
    char line[512];
} LogEntry;


LogEntry circularBuffer[MAX_LINES];
uint32_t writeIndex = 0;
uint32_t sequence_num = 1;



// CIRCULAR BUFFER GLOBALS - ADD THESE
FIL circularLogFile;
LogMetadata logMeta;







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
#define DEBUG_LOG_MAX_MSG_LEN 300
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
#define MAX_SPEED_METERS_PER_SECOND 85.0 //  speed (306 km/h)


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

void vSD_CARD_LOG_Task(void *pvParameters) ;


bool mount_sd_card_with_retry(FATFS* fatfs, uint8_t max_attempts, uint32_t retry_delay_ms);


void vDebugLogTask(void *pvParameters);


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






FATFS* fatfs;
FRESULT fres;

FATFS FatFs;
FIL headerFile;


char test_arr[150];


UINT bytesWritten;







char *header_details =
		"  THINTURE TECHNOLOGIES PVT LTD\n"
		"OWNER'S NAME\n"
		"OWNERS ID\n"
		"+9199959XXXXX\n"
		"KA 01 78XX\n"
		"CH123456789\n"
		"VEHICLE MAKE\n"
		"SPEED LIMITER CERT NO:\n"
		"SPEED LIMITER TYPE\n"
		"SER NO:\n"
		"DATE OF FITTING\n"
		"\n";







void vSD_CARD_LOG_Task(void *pvParameters) {


	 MX_FATFS_Init();
	 vTaskDelay(pdMS_TO_TICKS(1000));


	 mount_sd_card_with_retry(&FatFs, 3, 2000);

	 vTaskDelay(pdMS_TO_TICKS(2500));
	 fres = f_open(&headerFile, HEADER_FILE, FA_WRITE | FA_CREATE_ALWAYS);

	 	fres = f_write(&headerFile, header_details, strlen(header_details), &bytesWritten);

	 	f_sync(&headerFile);
	 	f_close(&headerFile);

	 	vTaskDelay(pdMS_TO_TICKS(1000));

	 	//printf




	 	FIL file;
	 		    if (f_open(&file, "test.txt", FA_WRITE | FA_CREATE_ALWAYS) == FR_OK) {
	 		        const char *text = "Hello SD!";
	 		        UINT bw;
	 		        f_write(&file, text, strlen(text), &bw);
	 		        f_close(&file);
	 		        sprintf(test_arr,"File written successfully!\n");
	 		        HAL_UART_Transmit(&huart2,(uint8_t*)test_arr,(uint16_t)(strlen((char*)test_arr)), 1000);
	 		    } else {
	 		    	sprintf(test_arr,"f_open failed\n");
	 		        HAL_UART_Transmit(&huart2,(uint8_t*)test_arr,(uint16_t)(strlen((char*)test_arr)), 1000);
	 		    }

	while(1){


		vTaskDelay(pdMS_TO_TICKS(1000));



	}
}












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


//RTC ********************************************************************************

//RTC - CIRCULAR BUFFER IMPLEMENTATION ****************************************************

//RTC - CIRCULAR BUFFER IMPLEMENTATION ****************************************************

// =================================================================================
// == DS3231 RTC and Circular Buffer SD Logging Task (72 Hours of Data)
// =================================================================================

// -- Part 1: DS3231 Definitions and Basic Functions (unchanged) --





#define DS3231_I2C_ADDR  0x68 << 1
#define DS3231_REG_SECONDS   0x00
#define DS3231_REG_CONTROL   0x0E
#define DS3231_REG_STATUS    0x0F


// -- PART2 : CIRCULAR BUFFER CONFIGURATION





typedef struct{




	uint8_t seconds;
	uint8_t minutes;
	uint8_t hour;
	uint8_t day_of_week;
	uint8_t day_of_month;
	uint8_t month;
	uint8_t year;

}RTC_Time;



static uint8_t decToBcd(int val){




	return (uint8_t)((val/10*16)+(val%10));
}

static int bcdToDec(uint8_t val){




	return (int)((val / 16*10)+ (val%16));

}

void DS3231_SetTime(RTC_Time *time) {
    uint8_t buf[7];
    buf[0] = decToBcd(time->seconds);
    buf[1] = decToBcd(time->minutes);
    buf[2] = decToBcd(time->hour);
    buf[3] = decToBcd(time->day_of_week);
    buf[4] = decToBcd(time->day_of_month);
    buf[5] = decToBcd(time->month);
    buf[6] = decToBcd(time->year);
    HAL_I2C_Mem_Write(&hi2c4, DS3231_I2C_ADDR, DS3231_REG_SECONDS, 1, buf, 7, HAL_MAX_DELAY);
}




void DS3231_GetTime(RTC_Time *time) {
    uint8_t buf[7];
    HAL_I2C_Mem_Read(&hi2c4, DS3231_I2C_ADDR, DS3231_REG_SECONDS, 1, buf, 7, HAL_MAX_DELAY);
    time->seconds = bcdToDec(buf[0]);
    time->minutes = bcdToDec(buf[1]);
    time->hour = bcdToDec(buf[2]);
    time->day_of_week = bcdToDec(buf[3]);
    time->day_of_month = bcdToDec(buf[4]);
    time->month = bcdToDec(buf[5]);
    time->year = bcdToDec(buf[6]);
}


bool load_log_metadata(LogMetadata *metadata) {
    FIL metaFile;
    FRESULT fres;
    UINT bytesRead;

    fres = f_open(&metaFile, METADATA_FILE, FA_READ);
    if (fres != FR_OK) {
        // File doesn't exist, initialize with defaults
        metadata->current_position = 0;
        metadata->total_entries_written = 0;
        metadata->buffer_full = 0;
        DEBUG_PRINTF("LOG_META: No metadata file found, starting fresh\r\n");
        return false;
    }

    fres = f_read(&metaFile, metadata, sizeof(LogMetadata), &bytesRead);
    f_close(&metaFile);

    if (fres == FR_OK && bytesRead == sizeof(LogMetadata)) {
        DEBUG_PRINTF("LOG_META: Loaded - Pos:%lu, Total:%lu, Full:%d\r\n",
                    metadata->current_position, metadata->total_entries_written, metadata->buffer_full);
        return true;
    } else {
        // Corrupted metadata, reset
        metadata->current_position = 0;
        metadata->total_entries_written = 0;
        metadata->buffer_full = 0;
        DEBUG_PRINTF("LOG_META: Corrupted metadata, resetting\r\n");
        return false;
    }
}






bool save_log_metadata(LogMetadata *metadata) {
    FIL metaFile;
    FRESULT fres;
    UINT bytesWritten;

    DEBUG_PRINTF("LOG_META: Opening metadata file for writing...\r\n");
    fres = f_open(&metaFile, METADATA_FILE, FA_WRITE | FA_CREATE_ALWAYS);
    if (fres != FR_OK) {
        DEBUG_PRINTF("LOG_META: Failed to open metadata file for writing, error: %d\r\n", fres);
        return false;
    }

    DEBUG_PRINTF("LOG_META: Writing metadata (size: %d bytes)...\r\n", sizeof(LogMetadata));
    fres = f_write(&metaFile, metadata, sizeof(LogMetadata), &bytesWritten);
    if (fres != FR_OK) {
        DEBUG_PRINTF("LOG_META: Write failed, error: %d\r\n", fres);
        f_close(&metaFile);
        return false;
    }

    DEBUG_PRINTF("LOG_META: Wrote %d bytes, syncing to SD card...\r\n", bytesWritten);
    f_sync(&metaFile);  // Ensure data is written to SD card
    f_close(&metaFile);

    if (bytesWritten == sizeof(LogMetadata)) {
        DEBUG_PRINTF("LOG_META: Metadata saved successfully (%d bytes)\r\n", bytesWritten);
        return true;
    } else {
        DEBUG_PRINTF("LOG_META: Size mismatch - expected: %d, written: %d\r\n",
                    sizeof(LogMetadata), bytesWritten);
        return false;
    }
}










bool init_circular_log_file(FIL *logFile, LogMetadata *metadata) {
    FRESULT fres;

    // First, try to open the file assuming it already exists.
    fres = f_open(logFile, DATALOG_FILE, FA_WRITE | FA_READ);

    if (fres == FR_OK) { // The file already exists.
        sprintf(test_arr, "LOG_INIT: Opened existing log file.\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)test_arr, (uint16_t)strlen((char*)test_arr), 1000);

        // Good practice: check if the file size is correct.
        FSIZE_t file_size = f_size(logFile);
        FSIZE_t expected_size = (FSIZE_t)TOTAL_LOG_ENTRIES * MAX_LOG_ENTRY_SIZE;

        if (file_size != expected_size) {
            sprintf(test_arr, "LOG_INIT: File size mismatch! Recreating file.\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t*)test_arr, (uint16_t)strlen((char*)test_arr), 1000);
            f_close(logFile);
            f_unlink(DATALOG_FILE); // Delete the corrupted file.
            fres = FR_NO_FILE; // Set status to force recreation below.
        } else {
            // File exists and is the correct size, we're ready to go.
            return true;
        }
    }

    // This block runs if the file does not exist (fres != FR_OK) or was just deleted.
    if (fres != FR_OK) {
        // STEP 1: Open the file with the create flag.
        sprintf(test_arr, "LOG_INIT: Creating new log file...\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)test_arr, (uint16_t)strlen((char*)test_arr), 1000);
        fres = f_open(logFile, DATALOG_FILE, FA_WRITE | FA_CREATE_NEW);
        if (fres != FR_OK) {
            sprintf(test_arr, "LOG_INIT: FAILED to create log file! Error: %d\r\n", fres);
            HAL_UART_Transmit(&huart2, (uint8_t*)test_arr, (uint16_t)strlen((char*)test_arr), 1000);
            return false;
        }

        // STEP 2: Pre-allocate space for the entire circular buffer.
        char dummy_entry[MAX_LOG_ENTRY_SIZE];
        memset(dummy_entry, ' ', MAX_LOG_ENTRY_SIZE - 2);
        dummy_entry[MAX_LOG_ENTRY_SIZE - 2] = '\r';
        dummy_entry[MAX_LOG_ENTRY_SIZE - 1] = '\n';

        sprintf(test_arr, "LOG_INIT: Pre-allocating space for %lu entries...\r\n", (unsigned long)TOTAL_LOG_ENTRIES);
        HAL_UART_Transmit(&huart2, (uint8_t*)test_arr, (uint16_t)strlen((char*)test_arr), 1000);

        for (uint32_t i = 0; i < TOTAL_LOG_ENTRIES; i++) {
            f_write(logFile, dummy_entry, MAX_LOG_ENTRY_SIZE, NULL);
            // This loop takes a long time. A small delay prevents it from blocking
            // the entire system and allows other tasks (like the logger) to run.
            if (i % 1000 == 0) {
                sprintf(test_arr, "LOG_INIT: Allocated %lu/%lu entries...\r\n", (unsigned long)i, (unsigned long)TOTAL_LOG_ENTRIES);
                HAL_UART_Transmit(&huart2, (uint8_t*)test_arr, (uint16_t)strlen((char*)test_arr), 1000);
                vTaskDelay(pdMS_TO_TICKS(5)); // Give scheduler time
            }
        }

        // STEP 3: CRITICAL! Close the file to finalize its creation and size on the SD card.
        sprintf(test_arr, "LOG_INIT: Allocation complete. Finalizing file on SD card...\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)test_arr, (uint16_t)strlen((char*)test_arr), 1000);

        fres = f_close(logFile);
        if (fres != FR_OK) {
            sprintf(test_arr, "LOG_INIT: FAILED to finalize (close) new file! Error: %d\r\n", fres);
            HAL_UART_Transmit(&huart2, (uint8_t*)test_arr, (uint16_t)strlen((char*)test_arr), 1000);
            return false;
        }

        // STEP 4: Re-open the file so it's ready for writing log entries.
        fres = f_open(logFile, DATALOG_FILE, FA_READ | FA_WRITE);
        if (fres != FR_OK) {
            sprintf(test_arr, "LOG_INIT: FAILED to re-open file for logging! Error: %d\r\n", fres);
            HAL_UART_Transmit(&huart2, (uint8_t*)test_arr, (uint16_t)strlen((char*)test_arr), 1000);
            return false;
        }

        sprintf(test_arr, "LOG_INIT: New log file created and ready for use.\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)test_arr, (uint16_t)strlen((char*)test_arr), 1000);

        // Reset metadata since we created a new file.
        metadata->current_position = 0;
        metadata->total_entries_written = 0;
        metadata->buffer_full = 0;
    }

    return true;
}




bool write_circular_log_entry(FIL *logFile, LogMetadata *metadata, const char *log_entry) {

	sprintf(test_arr,"hIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII\r\n");
	        HAL_UART_Transmit(&huart2,(uint8_t*)test_arr,(uint16_t)(strlen((char*)test_arr)), 1000);


    FRESULT fres;
    UINT bytesWritten;
    char padded_entry[MAX_LOG_ENTRY_SIZE];

    // Calculate file position for this entry
    FSIZE_t file_position = metadata->current_position * MAX_LOG_ENTRY_SIZE;

    // Prepare padded entry (pad with spaces and ensure it ends with \r\n)
    memset(padded_entry, ' ', MAX_LOG_ENTRY_SIZE);
    size_t entry_len = strlen(log_entry);
    if (entry_len > MAX_LOG_ENTRY_SIZE - 2) {
        entry_len = MAX_LOG_ENTRY_SIZE - 2;  // Leave space for \r\n
    }

    memcpy(padded_entry, log_entry, entry_len);
    padded_entry[MAX_LOG_ENTRY_SIZE - 2] = '\r';
    padded_entry[MAX_LOG_ENTRY_SIZE - 1] = '\n';

    // Seek to the correct position
    fres = f_lseek(logFile, file_position);
    if (fres != FR_OK) {
       // DEBUG_PRINTF("LOG_WRITE: Seek failed at position %lu\r\n", (uint32_t)file_position);


        sprintf(test_arr,"LOG_WRITE: Seek failed at position %lu\r\n", (uint32_t)file_position);
        HAL_UART_Transmit(&huart2,(uint8_t*)test_arr,(uint16_t)(strlen((char*)test_arr)), 1000);
        return false;
    }

    // Write the entry
    fres = f_write(logFile, padded_entry, MAX_LOG_ENTRY_SIZE, &bytesWritten);
    if (fres != FR_OK || bytesWritten != MAX_LOG_ENTRY_SIZE) {
        //DEBUG_PRINTF("LOG_WRITE: Write failed! Error: %d, Bytes: %d\r\n", fres, bytesWritten);
        sprintf(test_arr,"LOG_WRITE: Write failed! Error: %d, Bytes: %d\r\n", fres, bytesWritten);
        HAL_UART_Transmit(&huart2,(uint8_t*)test_arr,(uint16_t)(strlen((char*)test_arr)), 1000);
        return false;
    }

    // Force write to SD card
    f_sync(logFile);

    // Update metadata
    metadata->total_entries_written++;
    metadata->current_position++;

    // Handle circular buffer wraparound
    if (metadata->current_position >= TOTAL_LOG_ENTRIES) {
        metadata->current_position = 0;
        metadata->buffer_full = 1;
       // DEBUG_PRINTF("LOG_CIRCULAR: Buffer wrapped around - starting overwrite cycle\r\n");

        sprintf(test_arr,"LOG_CIRCULAR: Buffer wrapped around - starting overwrite cycle\r\n");
                HAL_UART_Transmit(&huart2,(uint8_t*)test_arr,(uint16_t)(strlen((char*)test_arr)), 1000);

    }

    return true;
}







bool mount_sd_card_with_retry(FATFS* fatfs, uint8_t max_attempts, uint32_t retry_delay_ms) {
    FRESULT fres;
    uint8_t attempt = 0;

    while (attempt < max_attempts) {
        attempt++;

        DEBUG_PRINTF("SD_MOUNT: Attempt %d/%d...\r\n", attempt, max_attempts);

        // Try to mount the SD card
        fres = f_mount(fatfs, "", 1);

        if (fres == FR_OK) {
            DEBUG_PRINTF("SD_MOUNT: Success on attempt %d\r\n", attempt);
            return true;
        }

        // Log the specific error
        DEBUG_PRINTF("SD_MOUNT: Failed on attempt %d - Error: %d\r\n", attempt, fres);

        // Print human-readable error message
        switch(fres) {
            case FR_NO_FILESYSTEM:
                DEBUG_PRINTF("SD_MOUNT: No valid FAT filesystem found\r\n");
                break;
            case FR_DISK_ERR:
                DEBUG_PRINTF("SD_MOUNT: Physical disk error\r\n");
                break;
            case FR_NOT_READY:
                DEBUG_PRINTF("SD_MOUNT: Drive not ready\r\n");
                break;
            case FR_NO_FILE:
                DEBUG_PRINTF("SD_MOUNT: Could not find the file\r\n");
                break;
            default:
                DEBUG_PRINTF("SD_MOUNT: Unknown error code: %d\r\n", fres);
                break;
        }

        // Don't delay on the last failed attempt
        if (attempt < max_attempts) {
            DEBUG_PRINTF("SD_MOUNT: Retrying in %lu ms...\r\n", retry_delay_ms);
            vTaskDelay(pdMS_TO_TICKS(retry_delay_ms));
        }
    }

    DEBUG_PRINTF("SD_MOUNT: All %d attempts failed!\r\n", max_attempts);
    return false;
}


void vCircular_Log_Task(void *pvParameters){





	  FATFS FatFs;
	  FIL circularLogFile;


	 LogMetadata logMeta;

	char logBuffer[MAX_LOG_ENTRY_SIZE];


	static uint32_t log_counter=0;





	//sd_card_state_tracking



	bool sd_card_avilable = false;
	uint32_t last_mount_attempt=0;
	const uint32_t mount_retry_interval=30000;
	uint32_t lastMetadataSave=0;
	const uint32_t metadata_save_interval=120000;









	vTaskDelay(pdMS_TO_TICKS(1000));

	//DEBUG_PRINTF("\r\n  CIRCULAR SD CARD LOGGING TASK STARTED--------\r\n");

	  sprintf(test_arr,"\r\n  CIRCULAR SD CARD LOGGING TASK STARTED--------\r\n");
	  HAL_UART_Transmit(&huart2,(uint8_t*)test_arr,(uint16_t)(strlen((char*)test_arr)), 1000);

	//MX_FATFS_init();


	sd_card_avilable= mount_sd_card_with_retry(&FatFs,3,2000);



	if(sd_card_avilable){


		//DEBUG_PRINTF("\r\n SD CARD MOUNTED SUCCESFULLY***************************** \r\n");
		 sprintf(test_arr,"\r\n SD CARD MOUNTED SUCCESFULLY***************************** \r\n");
	     HAL_UART_Transmit(&huart2,(uint8_t*)test_arr,(uint16_t)(strlen((char*)test_arr)), 1000);

		load_log_metadata(&logMeta);

		//init_circular_log_file(&circularLogFile, &logMeta);

		if(!init_circular_log_file(&circularLogFile, &logMeta)){

			//DEBUG_PRINTF("\r\n failed to initialize circular log file \r\n");
			sprintf(test_arr,"\r\n failed to initialize circular log file \r\n");
				     HAL_UART_Transmit(&huart2,(uint8_t*)test_arr,(uint16_t)(strlen((char*)test_arr)), 1000);

			sd_card_avilable=false;
		}



		else{



			log_counter= logMeta.total_entries_written;


			//DEBUG_PRINTF("LOG_TASK: Circular log initialize. Total entries so far: %lu \r\n", log_counter);
			sprintf(test_arr,"LOG_TASK: Circular log initialize. Total entries so far: %lu \r\n", log_counter);
							     HAL_UART_Transmit(&huart2,(uint8_t*)test_arr,(uint16_t)(strlen((char*)test_arr)), 1000);

			save_log_metadata(&logMeta);
		}




	}
	else{

		DEBUG_PRINTF("\r\n mounting error*****************************");
		sprintf(test_arr,"\r\n mounting error*****************************");
		HAL_UART_Transmit(&huart2,(uint8_t*)test_arr,(uint16_t)(strlen((char*)test_arr)), 1000);
	}

while(1){



	if(sd_card_avilable){



		snprintf(logBuffer,sizeof(logBuffer), "Log entry number: %lu", (unsigned long)log_counter++);

		if(write_circular_log_entry(&circularLogFile,&logMeta,logBuffer)){

			sprintf(test_arr,"LOG_SUCCESS: Entry %lu written.\r\n", logMeta.total_entries_written);
					HAL_UART_Transmit(&huart2,(uint8_t*)test_arr,(uint16_t)(strlen((char*)test_arr)), 1000);




		}





		else{

			sd_card_avilable=false;

			f_close(&circularLogFile);

		}


		//periodically save the metadata

		if((HAL_GetTick()- lastMetadataSave) > metadata_save_interval){



			if(save_log_metadata(&logMeta)){


				sprintf(test_arr,"LOG_META: Metadata saved successfully.\r\n");
		       HAL_UART_Transmit(&huart2,(uint8_t*)test_arr,(uint16_t)(strlen((char*)test_arr)), 1000);

			}


			else{



				sprintf(test_arr,"LOG_META: Failed to save metadata!\r\n");
			    HAL_UART_Transmit(&huart2,(uint8_t*)test_arr,(uint16_t)(strlen((char*)test_arr)), 1000);


			}


			lastMetadataSave =HAL_GetTick();
			}
	}






	vTaskDelay(pdMS_TO_TICKS(LOG_INTERVAL_MS));


}


}




//********************************************************************************************************************************************************************


/*void RTC_SD_Log_Task_Circular(void *pvParameters) {




    RTC_Time currentTime;
    gps_data_t localGpsData;
    char logBuffer[MAX_LOG_ENTRY_SIZE];
    FATFS FatFs;

    // Variables to prevent duplicate logging
    static uint8_t lastLoggedSecond = 255;
    static uint32_t lastLoggedMinute = 0;
    static uint32_t lastMetadataSave = 0;

    // SD card state tracking - NEW VARIABLES
    bool sd_card_available = false;
    uint32_t last_mount_attempt = 0;
    uint32_t mount_retry_interval = 30000; // Retry every 30 seconds

    vTaskDelay(pdMS_TO_TICKS(1000));
    DEBUG_PRINTF("\r\n--- Enhanced Circular Buffer RTC+GPS Logging Task Started ---\r\n");
    DEBUG_PRINTF("LOG_CONFIG: 24 hours storage, %lu total entries, %d bytes per entry\r\n",
                (unsigned long)TOTAL_LOG_ENTRIES, MAX_LOG_ENTRY_SIZE);

    // Initial SD card mount attempt - MODIFIED
    sd_card_available = mount_sd_card_with_retry(&FatFs, 3, 2000);
//1,10,000
    if (sd_card_available) {
        DEBUG_PRINTF("LOG_TASK: SD card mounted successfully\r\n");

        // Load metadata
        load_log_metadata(&logMeta);

        // Initialize circular log file
        if (!init_circular_log_file(&circularLogFile, &logMeta)) {
            DEBUG_PRINTF("LOG_TASK: Failed to initialize circular log file!\r\n");
            sd_card_available = false;
        } else {
            // Save initial metadata immediately after initialization
            if (save_log_metadata(&logMeta)) {
                DEBUG_PRINTF("LOG_META: Initial metadata saved successfully\r\n");
            } else {
                DEBUG_PRINTF("LOG_META: Failed to save initial metadata\r\n");
            }
        }
    } else {
        DEBUG_PRINTF("LOG_TASK: SD card mount failed - will continue without logging to SD\r\n");
    }

    // Set the time once when first programming the board
    RTC_Time initial_time = {
        .seconds = 00, .minutes = 25, .hour = 17,
        .day_of_week = 4, .day_of_month = 14, .month = 8, .year = 25
    };
    DEBUG_PRINTF("RTC time set.\r\n");

    DEBUG_PRINTF("LOG_TASK: Initialization complete. SD Available: %s\r\n",
                sd_card_available ? "YES" : "NO");

    if (sd_card_available) {
        DEBUG_PRINTF("LOG_STATUS: Position: %lu/%lu, Total written: %lu, Buffer full: %s\r\n",
                    logMeta.current_position, TOTAL_LOG_ENTRIES, logMeta.total_entries_written,
                    logMeta.buffer_full ? "YES" : "NO");
    }

    while (1) {
        // NEW: Periodic retry of SD card mount if not available
        if (!sd_card_available &&
            (HAL_GetTick() - last_mount_attempt) > mount_retry_interval) {

            DEBUG_PRINTF("LOG_TASK: Attempting SD card recovery...\r\n");
            last_mount_attempt = HAL_GetTick();

            sd_card_available = mount_sd_card_with_retry(&FatFs, 2, 1000);

            if (sd_card_available) {
                DEBUG_PRINTF("LOG_TASK: SD card recovery successful!\r\n");

                // Reinitialize logging structures
                load_log_metadata(&logMeta);

                if (!init_circular_log_file(&circularLogFile, &logMeta)) {
                    DEBUG_PRINTF("LOG_TASK: Failed to reinitialize log file!\r\n");
                    sd_card_available = false;
                }
            } else {
                DEBUG_PRINTF("LOG_TASK: SD card still unavailable, will retry in %lu seconds\r\n",
                           mount_retry_interval/1000);
            }
        }

        // Read current time from RTC
        DS3231_GetTime(&currentTime);

        // Check if we need to log (every 5 seconds)
        bool shouldLog = (currentTime.seconds % LOG_INTERVAL_SECONDS == 0);
        bool isDuplicate = (currentTime.seconds == lastLoggedSecond) &&
                          (currentTime.minutes == lastLoggedMinute);

        if (shouldLog && !isDuplicate) {
            // MODIFIED: Always show logging attempt, even if SD card unavailable
            DEBUG_PRINTF("LOG: %02d:%02d:%02d [SD: %s]\r\n",
                        currentTime.hour, currentTime.minutes, currentTime.seconds,
                        sd_card_available ? "OK" : "UNAVAILABLE");

            // Get GPS data with minimal mutex hold time
            bool gpsDataValid = false;
            if (xSemaphoreTake(g_gps_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                localGpsData = g_gps_data;
                gpsDataValid = true;
                xSemaphoreGive(g_gps_data_mutex);
            } else {
                DEBUG_PRINTF("LOG: Failed to get GPS mutex\r\n");
                localGpsData.is_valid = false;
                gpsDataValid = true;  // Continue with invalid GPS data
            }

            // Format log entry
            if (gpsDataValid && localGpsData.is_valid) {
                char lat_dir = (localGpsData.latitude >= 0) ? 'N' : 'S';
                float lat_val = (localGpsData.latitude >= 0) ? localGpsData.latitude : -localGpsData.latitude;
                char lon_dir = (localGpsData.longitude >= 0) ? 'E' : 'W';
                float lon_val = (localGpsData.longitude >= 0) ? localGpsData.longitude : -localGpsData.longitude;

                snprintf(logBuffer, sizeof(logBuffer),
                         "20%02d/%02d/%02d\t%02d:%02d:%02d\t%.1f\t%.4f° %c\t%.4f° %c",
                         currentTime.year, currentTime.month, currentTime.day_of_month,
                         currentTime.hour, currentTime.minutes, currentTime.seconds,
                         localGpsData.speed_knots, lat_val, lat_dir, lon_val, lon_dir);
            } else {
                snprintf(logBuffer, sizeof(logBuffer),
                         "20%02d/%02d/%02d\t%02d:%02d:%02d\tGPS NOT FIXED",
                         currentTime.year, currentTime.month, currentTime.day_of_month,
                         currentTime.hour, currentTime.minutes, currentTime.seconds);
            }

            // MODIFIED: Write to SD card only if available
            if (sd_card_available) {
                if (write_circular_log_entry(&circularLogFile, &logMeta, logBuffer)) {
                    DEBUG_PRINTF("LOG_SUCCESS: Entry %lu written\r\n", logMeta.total_entries_written);
                } else {
                    DEBUG_PRINTF("LOG_ERROR: Write failed - SD card may be corrupted\r\n");
                    sd_card_available = false; // Mark as unavailable for retry
                    f_close(&circularLogFile); // Close file handle
                    last_mount_attempt = HAL_GetTick(); // Reset retry timer
                }
            } else {
                // NEW: Log to console when SD unavailable (for debugging)
                DEBUG_PRINTF("LOG_CONSOLE: %s\r\n", logBuffer);
            }

            // Update last logged time
            lastLoggedSecond = currentTime.seconds;
            lastLoggedMinute = currentTime.minutes;

            // MODIFIED: Save metadata periodically (only if SD card available)
            if (sd_card_available && (HAL_GetTick() - lastMetadataSave) > 120000) {  // 2 minutes for testing
                DEBUG_PRINTF("LOG_META: Attempting to save metadata...\r\n");
                if (save_log_metadata(&logMeta)) {
                    DEBUG_PRINTF("LOG_META: Metadata saved successfully\r\n");
                } else {
                    DEBUG_PRINTF("LOG_META: Failed to save metadata\r\n");
                }
                lastMetadataSave = HAL_GetTick();
            }

            // Status report every hour
            if (sd_card_available && logMeta.total_entries_written % 720 == 0) {  // 720 entries = 1 hour
                float hours_stored = logMeta.buffer_full ? HOURS_TO_STORE :
                                   (logMeta.total_entries_written * LOG_INTERVAL_SECONDS) / 3600.0f;
                DEBUG_PRINTF("LOG_STATUS: %.1f hours of data stored, Position: %lu/%lu\r\n",
                            hours_stored, logMeta.current_position, TOTAL_LOG_ENTRIES);
            }
        }

        // Sleep for 500ms to check twice per second
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}*/







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
  MX_SPI5_Init();
  MX_I2C4_Init();


  MX_FATFS_Init();
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

 // xTaskCreate(vGpsTask, "GpsTask", 2048, NULL, 3, NULL);
  //xTaskCreate(vMqttTask, "MqttTask", 2048, NULL, 2, NULL);


 //xTaskCreate(vSD_CARD_LOG_Task, "sd_card_logg", 1024,NULL,2,NULL);
  xTaskCreate(vCircular_Log_Task, "vCircular_Log_Task", 2048, NULL, 2, NULL);
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
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV2;

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
