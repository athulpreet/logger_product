/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "fatfs.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <ctype.h>

#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_hid.h"
#include "usbd_desc.h"
#include "usbd_composite_builder.h"

/*
 * for MSLD
 */
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
#include "global_declarations.h"
#include "handheld_programing.h"
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
    char date[8];
    bool is_valid;
} gps_data_t;

// --- Structures for RTC ---
typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hour;
    uint8_t day_of_week;
    uint8_t day_of_month;
    uint8_t month;
    uint8_t year;
} RTC_Time;

// --- Structures for SD Card Logging ---
typedef struct {
    uint32_t current_position;
    uint32_t total_entries_written;
    uint8_t buffer_full;
} LogMetadata;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

uint8_t CDC_EpAdd_Inst[3] = {CDC_IN_EP, CDC_OUT_EP, CDC_CMD_EP}; 	/* CDC Endpoint Addresses array */
uint8_t HID_EpAdd_Inst = HID_EPIN_ADDR;								/* HID Endpoint Address array */
USBD_HandleTypeDef hUsbDeviceFS;
uint8_t hid_report_buffer[4];
uint8_t HID_InstID = 0, CDC_InstID = 0;

// --- Definitions for the Logging Task ---
#define MAX_LOG_MSG_LEN         256
#define LOG_QUEUE_SIZE          20

// --- Definitions for Modem and MQTT ---
#define USE_SECONDARY_SERVER            1       // Set to 1 to enable, 0 to disable
#define MODEM_UART_RX_BUFFER_SIZE       256
#define MODEM_PWR_RST_GPIO_Port         GPIOB
#define MODEM_PWR_RST_Pin               GPIO_PIN_9
#define IDLE_SPEED_THRESHOLD_KNOTS      2.0f
#define MODEM_UART_RX_QUEUE_LEN         5

// --- Definitions for RTC (DS3231) ---
#define DS3231_I2C_ADDR  (0x68 << 1)
#define DS3231_REG_SECONDS   0x00

// --- Definitions for SD Card Circular Log ---
#define LOG_INTERVAL_SECONDS 5
#define HOURS_TO_STORE 12
#define TOTAL_LOG_ENTRIES ((uint32_t)((HOURS_TO_STORE * 3600UL) / LOG_INTERVAL_SECONDS))
#define MAX_LOG_ENTRY_SIZE 100
#define METADATA_FILE "logmeta.txt"
#define DATALOG_FILE "datalog.txt"



#define AUTHORIZED_SENDER "+919110470625"


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c4;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart12;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_NodeTypeDef Node_GPDMA2_Channel7;
DMA_QListTypeDef List_GPDMA2_Channel7;
DMA_HandleTypeDef handle_GPDMA2_Channel7;
DMA_HandleTypeDef handle_GPDMA2_Channel6;
DMA_NodeTypeDef Node_GPDMA2_Channel5;
DMA_QListTypeDef List_GPDMA2_Channel5;
DMA_HandleTypeDef handle_GPDMA2_Channel5;

PCD_HandleTypeDef hpcd_USB_DRD_FS;

/* USER CODE BEGIN PV */
/*
 * for MSLD
 */
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

QueueHandle_t handheld_uart_queue;
TaskHandle_t xTaskHandler = NULL;
TaskHandle_t xTaskHandler1 = NULL;
TaskHandle_t xTaskHandler2 = NULL;
TaskHandle_t xTaskHandler3= NULL;
TaskHandle_t xTaskHandler4 = NULL;
TaskHandle_t xTaskHandler5 = NULL;

SemaphoreHandle_t handheld_data_ready_sem;
uint8_t uart2_rx_size=0;

int gps_eeprom_status=0;
int buzz_off_flag=0;
int pedal_exit_flag=0;


struct timer_struct
{
	int8_t st_CFlg;
	int16_t st_counter;
	int8_t st_SFlg;
};
struct timer_struct serial_speed_receive;
struct timer_struct stack_size_tester;
struct timer_struct control_buzz_onoff;
struct timer_struct pid_pedal_exit;
struct timer_struct signal_disconnection;
struct timer_struct dma_hand_held;

uint16_t timer_counter_ivms=50;


float output_pedal_value=0.0;

float pid_result=0.0;

uint8_t rx_high_set_speed_flag=0;


uint16_t dma_delay_serial=5;

uint8_t serial_data_true=0;

float pid_steady_val=0.0;

float previous_speed=0.0;

uint16_t PRINT_DEBUG_TIME=0;

volatile float ppr_value=0;
float spd=0;
int spd_exit=0;

float error_value=0;
float I_=0;
float I_Clamping=0;
float pre_error_value=0;

uint8_t pid_loop_condition=0;

int8_t cf,buzz;//cruise mode flag
int8_t set_speed_flag=0;

unsigned char ser_speed;
int8_t spd_change=0;

volatile float speed_rng=0;
float error_percentage=0.0;
float steady_value=0;
uint8_t ch_ok;
float pid_return=0;
uint8_t speed_exit_flag;
uint8_t set_speed_rx_flg=0;


float speed=0.0;
uint8_t can_speedx=0;

uint16_t counter_send_return=0;
int rx_uart3_flag=0;
int ivms_uart3_flag=0;

int Is_First_Captured=0;
float frequency = 0;

int frequency_flag=0;
uint8_t ch_ok;
uint32_t Timer_count=0;
uint8_t power_input=0;
uint8_t ignition_input=0;

char power_status[8];
char ignitionStatus[8];
int get_gps_fast=0;
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

// --- GPS Task objects ---
SemaphoreHandle_t g_gps_data_ready_sem;
SemaphoreHandle_t g_gps_data_mutex;
gps_data_t g_gps_data = {0};

// --- Buffers and constants for GPS ---
#define GPS_DMA_RX_BUFFER_SIZE      512
#define MAX_NMEA_SENTENCE_LEN       100
#define MAX_SPEED_METERS_PER_SECOND 85.0 // (306 km/h)
uint8_t g_gps_dma_rx_buffer[GPS_DMA_RX_BUFFER_SIZE];

// Pointers for managing the circular DMA buffer
static uint16_t read_pos = 0;
volatile uint16_t write_pos = 0;
static gps_data_t g_last_valid_gps_data = { .is_valid = false };

// --- RTOS Objects for the Logging System ---
QueueHandle_t g_log_queue;
char g_log_buffer[MAX_LOG_MSG_LEN];

// --- MQTT Task objects ---
QueueHandle_t g_modem_uart_rx_queue;
QueueHandle_t g_data_snapshot_queue; // GPS task sends snapshots to MQTT task here
uint8_t g_modem_dma_rx_buffer[MODEM_UART_RX_BUFFER_SIZE];
char g_apn[32] = "internet"; // IMPORTANT: Replace with your SIM card's actual APN

// --- MQTT Configuration ---
// Primary Server (Socket 0)
char g_mqtt_broker_ip[40] = "3.109.116.92";
char g_mqtt_broker_port[6] = "1883";
char g_mqtt_client_id[32] = "spring-client";
char g_mqtt_username[32] = "Thinture";
char g_mqtt_password[32] = "Thinture24";
char g_mqtt_topic[32] = "Test";

// Secondary Server (Socket 1)
#if USE_SECONDARY_SERVER
char g_secondary_mqtt_broker_ip[40] = "43.205.58.131";
char g_secondary_mqtt_broker_port[6] = "1884";
char g_secondary_mqtt_client_id[32] = "spring-client-secondary"; // Use a different client ID
char g_secondary_mqtt_username[32] = "Thinture";
char g_secondary_mqtt_password[32] = "Thinture";
char g_secondary_mqtt_topic[32] = "Test";
#endif

// --- SD Card and RTC Objects ---
FATFS g_fatfs;
FIL g_circular_log_file;
LogMetadata g_log_meta;

// --- Global flag for GPS fix status ---
volatile bool g_gps_fix_acquired = false;
volatile bool g_gsm_connection_acquired = false;





// --- SMS/MQTT MODEM MUTEX ---
SemaphoreHandle_t g_modem_mutex;
volatile uint32_t g_successful_publishes = 0; // For the STATUS SMS command
volatile uint32_t g_session_start_time = 0;   // For the STATUS SMS command


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPDMA2_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C4_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_UART12_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_I2C1_Init(void);
void MX_USB_PCD_Init(void);
static void MX_TIM2_Init(void);
static void MX_ICACHE_Init(void);
/* USER CODE BEGIN PFP */






// --- RTOS Task Prototypes ---
void vStatusLedTask(void *pvParameters);
void vGpsTask(void *pvParameters);
void vLoggingTask(void *pvParameters);
void vMqttTask(void *pvParameters);
void vCircularTask(void *pvParameters);



// --- SMS Task and Helpers ---
void vSmsTask(void *pvParameters);
bool send_sms(const char* recipient, const char* message);
void flush_modem_uart_buffer(void);

///////////////////////////////////////////
/*
 * RTOS TASK for MSLD
 */
void dev_set( void *pvParameters );
void timer_jobs(void *pvParameters);
void vHandTask( void *pvParameters );
void iHandTask( void *pvParameters );
void SYHandTask( void *pvParameters );
void time_jobs( void *pvParameters );
void HH_dma( void *pvParameters );
void process_uart2_buffer(uint8_t* buffer, uint16_t size);
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////


// --- Helper Function Prototypes ---
bool send_and_wait_for_response(const char* cmd, const char* expected_response, uint32_t timeout_ms);
void perform_modem_power_cycle(void);
void format_coord(char* buffer, size_t buffer_size, float coord, char positive_dir, char negative_dir);
void convert_utc_datetime_to_ist(const char* utc_date_str, const char* utc_time_str, char* ist_buffer, size_t buffer_size);

// --- RTC Function Prototypes ---
void DS3231_SetTime(RTC_Time *time);
void DS3231_GetTime(RTC_Time *time);

// --- SD Card Function Prototypes ---
bool mount_sd_card_with_retry(FATFS* fatfs, uint8_t max_attempts, uint32_t retry_delay_ms);
bool load_log_metadata(LogMetadata *metadata);
bool save_log_metadata(LogMetadata *metadata);
bool init_circular_log_file(FIL *logFile, LogMetadata *metadata);
bool write_circular_log_entry(FIL *logFile, LogMetadata *metadata, const char *log_entry);



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * free rtos setting to run all task
 */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    taskENTER_CRITICAL();
    for( ;; );
}

void vApplicationMallocFailedHook(void)
{
    taskENTER_CRITICAL();
    for( ;; );
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helper function to send messages to the logging queue safely
void log_message(const char* message) {
    if (g_log_queue != NULL) {
        // Use a small timeout to prevent this function from blocking if the queue is full
        xQueueSend(g_log_queue, message, pdMS_TO_TICKS(10));
    }
}

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

void configure_gps_module_lc86g(void) {
    const char* cmd;
    const uint32_t cmd_delay = 150; // ms

    log_message("\r\n--- Configuring LC86G GNSS Module ---\r\n");

    // Perform a factory reset to start from a clean state.
    log_message("--- Performing factory reset... ---\r\n");
    cmd = "$PAIR007*3D\r\n"; // FULL_COLD_START (clears all settings)
    HAL_UART_Transmit(&huart12, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Set GNSS constellation search mode. This command causes another reboot.
    log_message("--- Setting GNSS constellations (causes reboot) ---\r\n");
    cmd = "$PAIR066,1,1,1,1,1,0*3B\r\n"; // Enable GPS+GLONASS+Galileo+BDS+QZSS
    HAL_UART_Transmit(&huart12, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Send all other desired configuration commands after reboot
    log_message("--- Sending remaining configuration... ---\r\n");
    cmd = "$PAIR050,1000*12\r\n"; // Set fix rate to 1Hz
    HAL_UART_Transmit(&huart12, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(cmd_delay));

    // Disable all unnecessary NMEA sentences to reduce CPU load
    const char* disable_cmds[] = {
        "$PAIR062,1,0*3A\r\n", // Disable GLL
        "$PAIR062,2,0*39\r\n", // Disable GSA
        "$PAIR062,3,0*38\r\n", // Disable GSV
        "$PAIR062,5,0*3F\r\n", // Disable VTG
        NULL
    };
    for(int i=0; disable_cmds[i] != NULL; ++i) {
        HAL_UART_Transmit(&huart12, (uint8_t*)disable_cmds[i], strlen(disable_cmds[i]), HAL_MAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(cmd_delay));
    }

    // Enable only the sentences we need: RMC and GGA
    cmd = "$PAIR062,4,1*3D\r\n"; // Enable RMC
    HAL_UART_Transmit(&huart12, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(cmd_delay));

    cmd = "$PAIR062,0,1*3B\r\n"; // Enable GGA
    HAL_UART_Transmit(&huart12, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(cmd_delay));

    // Save settings permanently
    log_message("--- Saving configuration to GNSS module flash ---\r\n");
    cmd = "$PAIR513*3D\r\n"; // Save settings to NVRAM
    HAL_UART_Transmit(&huart12, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(500)); // Allow extra time for flash write operation

    log_message("--- LC86G GNSS Module Configuration Sent and Saved ---\r\n\r\n");
}

int if_altitude_true=0;
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
        if (!g_gps_fix_acquired) {
            g_gps_fix_acquired = true;
            log_message("******** GPS FIX ACQUIRED ********\r\n");
        }
		if (fields[1] != NULL) strncpy(new_data.time, fields[1], sizeof(new_data.time) - 1);
		if (fields[9] != NULL) strncpy(new_data.date, fields[9], sizeof(new_data.date) - 1);
		new_data.latitude = nmea_to_decimal(atof_custom(fields[3]), fields[4][0]);
		new_data.longitude = nmea_to_decimal(atof_custom(fields[5]), fields[6][0]);
		if (fields[7] != NULL) new_data.speed_knots = atof_custom(fields[7]);
		if (fields[8] != NULL) new_data.course_degrees = atof_custom(fields[8]);

        // MOVEMENT PLAUSIBILITY CHECK
        if (g_last_valid_gps_data.is_valid) {
            double distance_moved = haversine_distance(
                g_last_valid_gps_data.latitude, g_last_valid_gps_data.longitude,
                new_data.latitude, new_data.longitude
            );
            if (distance_moved > MAX_SPEED_METERS_PER_SECOND) {
                new_data.is_valid = false;
                log_message("!!! Plausibility Check Failed: Impossible jump detected. Rejecting point.\r\n");
            }
        }

    } else {
        new_data.is_valid = false;
        if (g_gps_fix_acquired) {
             g_gps_fix_acquired = false;
             log_message("******** GPS FIX LOST ********\r\n");
        }
    }

    if (xSemaphoreTake(g_gps_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (new_data.is_valid) {
            new_data.altitude = g_gps_data.altitude; // Preserve altitude from GGA
            g_gps_data = new_data;
            g_last_valid_gps_data = new_data; // Update last known good position
        } else {
            g_gps_data.is_valid = false;
        }
        xSemaphoreGive(g_gps_data_mutex);
    }
}

void parse_gngga(char* gngga_sentence) {
    char temp_sentence[MAX_NMEA_SENTENCE_LEN];
    strncpy(temp_sentence, gngga_sentence, sizeof(temp_sentence) - 1);
    temp_sentence[sizeof(temp_sentence) - 1] = '\0';

    char* fields[15] = {NULL};
    int field_count = 0;
    char *saveptr;
    char* token = strtok_r(temp_sentence, ",*", &saveptr);
    while(token != NULL && field_count < 15) {
        fields[field_count++] = token;
        token = strtok_r(NULL, ",*", &saveptr);
    }

    if (field_count >= 10 && fields[9] != NULL && strlen(fields[9]) > 0) {
        float altitude = atof_custom(fields[9]);
        if (xSemaphoreTake(g_gps_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        	if_altitude_true=9;
            g_gps_data.altitude = altitude;
            xSemaphoreGive(g_gps_data_mutex);
            //log_message("\n\naltitude##########\n\n");
        }
    }
}

void process_gps_buffer(uint8_t* buffer, uint16_t size) {
    static char nmea_sentence[MAX_NMEA_SENTENCE_LEN];
    static uint16_t sentence_index = 0;
    if (buffer == NULL || size == 0) return;

    for (uint16_t i = 0; i < size; i++) {
        char ch = (char)buffer[i];
        if (ch == '$') {
            sentence_index = 0;
        }

        if (sentence_index < (sizeof(nmea_sentence) - 1)) {
            nmea_sentence[sentence_index++] = ch;
        }

        if (ch == '\n') {
            nmea_sentence[sentence_index] = '\0';
            if (strncmp(nmea_sentence, "$GNRMC", 6) == 0) {
                parse_gnrmc(nmea_sentence);
            } else if (strncmp(nmea_sentence, "$GNGGA", 6) == 0) {
                parse_gngga(nmea_sentence);
            }
            sentence_index = 0;
        }
    }
}

// --- DS3231 RTC HELPER FUNCTIONS ---
static uint8_t decToBcd(int val) {
    return (uint8_t)((val / 10 * 16) + (val % 10));
}

static int bcdToDec(uint8_t val) {
    return (int)((val / 16 * 10) + (val % 16));
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

// --- SD CARD AND FATFS HELPER FUNCTIONS ---
bool mount_sd_card_with_retry(FATFS* fatfs, uint8_t max_attempts, uint32_t retry_delay_ms) {
    FRESULT fres;
    for (uint8_t attempt = 1; attempt <= max_attempts; ++attempt) {
        snprintf(g_log_buffer, MAX_LOG_MSG_LEN, "SD_MOUNT: Attempt %d/%d...\r\n", attempt, max_attempts);
        log_message(g_log_buffer);
        fres = f_mount(fatfs, "", 1);
        if (fres == FR_OK) {
            log_message("SD_MOUNT: Success!\r\n");
            return true;
        }
        snprintf(g_log_buffer, MAX_LOG_MSG_LEN, "SD_MOUNT: Failed with error code %d\r\n", fres);
        log_message(g_log_buffer);
        if (attempt < max_attempts) vTaskDelay(pdMS_TO_TICKS(retry_delay_ms));
    }
    return false;
}

bool load_log_metadata(LogMetadata *metadata) {
    FIL metaFile;
    UINT bytesRead;
    if (f_open(&metaFile, METADATA_FILE, FA_READ) != FR_OK) {
        metadata->current_position = 0;
        metadata->total_entries_written = 0;
        metadata->buffer_full = 0;
        log_message("LOG_META: No metadata file found, starting fresh.\r\n");
        return false;
    }
    f_read(&metaFile, metadata, sizeof(LogMetadata), &bytesRead);
    f_close(&metaFile);
    if (bytesRead == sizeof(LogMetadata)) {
        log_message("LOG_META: Metadata loaded successfully.\r\n");
        return true;
    }
    // Corrupted metadata, reset
    metadata->current_position = 0;
    metadata->total_entries_written = 0;
    metadata->buffer_full = 0;
    log_message("LOG_META: Corrupted metadata file, resetting.\r\n");
    return false;
}

bool save_log_metadata(LogMetadata *metadata) {
    FIL metaFile;
    UINT bytesWritten;
    if (f_open(&metaFile, METADATA_FILE, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) {
        log_message("LOG_META: Failed to open metadata file for writing!\r\n");
        return false;
    }
    f_write(&metaFile, metadata, sizeof(LogMetadata), &bytesWritten);
    f_close(&metaFile); // Use f_close which also syncs
    if (bytesWritten == sizeof(LogMetadata)) {
        return true;
    }
    return false;
}

bool init_circular_log_file(FIL *logFile, LogMetadata *metadata) {
    if (f_open(logFile, DATALOG_FILE, FA_WRITE | FA_READ) == FR_OK) {
        if (f_size(logFile) == (FSIZE_t)TOTAL_LOG_ENTRIES * MAX_LOG_ENTRY_SIZE) {
            log_message("LOG_INIT: Existing log file is valid.\r\n");
            return true;
        }
        f_close(logFile); // Close corrupted file before deleting
        f_unlink(DATALOG_FILE);
        log_message("LOG_INIT: File size mismatch! Recreating file.\r\n");
    }

    if (f_open(logFile, DATALOG_FILE, FA_WRITE | FA_CREATE_NEW) != FR_OK) {
        log_message("LOG_INIT: FAILED to create new log file!\r\n");
        return false;
    }
    // Pre-allocate space
    char dummy_entry[MAX_LOG_ENTRY_SIZE];
    memset(dummy_entry, 0, MAX_LOG_ENTRY_SIZE);
    snprintf(dummy_entry, MAX_LOG_ENTRY_SIZE, "\r\n");

    log_message("LOG_INIT: Pre-allocating space... This may take some time.\r\n");
    for (uint32_t i = 0; i < TOTAL_LOG_ENTRIES; i++) {
        UINT bw;
        f_write(logFile, dummy_entry, MAX_LOG_ENTRY_SIZE, &bw);
        if (i % 1000 == 0) vTaskDelay(pdMS_TO_TICKS(5));
    }
    f_close(logFile);

    // Re-open for normal operations
    if (f_open(logFile, DATALOG_FILE, FA_READ | FA_WRITE) != FR_OK) {
        log_message("LOG_INIT: FAILED to re-open file after allocation!\r\n");
        return false;
    }
    log_message("LOG_INIT: New log file created successfully.\r\n");
    metadata->current_position = 0;
    metadata->total_entries_written = 0;
    metadata->buffer_full = 0;
    return true;
}

bool write_circular_log_entry(FIL *logFile, LogMetadata *metadata, const char *log_entry) {
    char padded_entry[MAX_LOG_ENTRY_SIZE];
    UINT bytesWritten;

    // Prepare padded entry
    memset(padded_entry, ' ', MAX_LOG_ENTRY_SIZE);
    size_t entry_len = strlen(log_entry);
    if (entry_len > MAX_LOG_ENTRY_SIZE - 2) entry_len = MAX_LOG_ENTRY_SIZE - 2;
    memcpy(padded_entry, log_entry, entry_len);
    padded_entry[MAX_LOG_ENTRY_SIZE - 2] = '\r';
    padded_entry[MAX_LOG_ENTRY_SIZE - 1] = '\n';

    // Seek and write
    if (f_lseek(logFile, metadata->current_position * MAX_LOG_ENTRY_SIZE) != FR_OK) return false;
    if (f_write(logFile, padded_entry, MAX_LOG_ENTRY_SIZE, &bytesWritten) != FR_OK) return false;
    if (bytesWritten != MAX_LOG_ENTRY_SIZE) return false;

    // Update metadata
    metadata->current_position++;
    metadata->total_entries_written++;
    if (metadata->current_position >= TOTAL_LOG_ENTRIES) {
        metadata->current_position = 0;
        metadata->buffer_full = 1;
    }
    return true;
}


// --- RTOS TASKS ---

void vLoggingTask(void *pvParameters) {
    char rx_buffer[MAX_LOG_MSG_LEN];
    log_message("--- RTOS Logging Task Initialized ---\r\n");
    for (;;) {
        if (xQueueReceive(g_log_queue, &rx_buffer, pdMS_TO_TICKS(350)) == pdPASS) {
            HAL_UART_Transmit(&huart6, (uint8_t*)rx_buffer, strlen(rx_buffer), HAL_MAX_DELAY);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

char lat_str[25], lon_str[25];
char alt_str[25];

void vGpsTask(void *pvParameters) {
    log_message("--- GPS Task Started. Listening on UART12... ---\r\n");

    vTaskDelay(pdMS_TO_TICKS(500));
    configure_gps_module_lc86g();

    HAL_UARTEx_ReceiveToIdle_DMA(&huart12, g_gps_dma_rx_buffer, GPS_DMA_RX_BUFFER_SIZE);

    for(;;) {
        // This task's only job is to process incoming data and update the global struct.
        // It waits indefinitely for the DMA interrupt to give the semaphore.
        if (xSemaphoreTake(g_gps_data_ready_sem, pdMS_TO_TICKS(350)) == pdTRUE) {
            uint16_t write_pos_local = write_pos;
            if (write_pos_local != read_pos) {
                if (write_pos_local > read_pos) {
                    process_gps_buffer(&g_gps_dma_rx_buffer[read_pos], write_pos_local - read_pos);
                } else {
                    process_gps_buffer(&g_gps_dma_rx_buffer[read_pos], GPS_DMA_RX_BUFFER_SIZE - read_pos);
                    process_gps_buffer(&g_gps_dma_rx_buffer[0], write_pos_local);
                }
            }
            read_pos = write_pos_local;
        }
        if(get_gps_fast==9)
        {
        	get_gps_fast=0;
        	//HAL_UART_Transmit(&huart12,(uint8_t*)flash_read_buffer,(uint16_t)(strlen((char*)flash_read_buffer)), 1000);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
  * @brief  vCircularTask: Manages RTC timekeeping and circular logging to SD card.
  * @param  pvParameters: Not used
  * @retval None
  * @note   This version includes the fix to prevent SD card corruption by calling
  * f_sync() after every successful write.
  *
  * @note   UPDATED to log "NO GPS FIX" and handle blank fields as requested.
  */
void vCircularTask(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for other tasks to initialize
    log_message("--- RTC & SD Card Logging Task Started ---\r\n");

    bool sd_card_available = false;
    char log_buffer[MAX_LOG_ENTRY_SIZE];
    RTC_Time current_time;
    static uint8_t last_log_second = 99; // Prevents duplicate logs

    // --- SD CARD INITIALIZATION ---
    sd_card_available = mount_sd_card_with_retry(&g_fatfs, 3, 2000);
    if (sd_card_available) {
        load_log_metadata(&g_log_meta);
        if (!init_circular_log_file(&g_circular_log_file, &g_log_meta)) {
            log_message("LOG_TASK: Failed to initialize log file!\r\n");
            sd_card_available = false;
        } else {
             save_log_metadata(&g_log_meta); // Save fresh metadata if file was created
        }
    }

    for(;;) {
        DS3231_GetTime(&current_time);

        // Check if it's a 5-second interval and not a duplicate second
        if ((current_time.seconds % LOG_INTERVAL_SECONDS == 0) && (current_time.seconds != last_log_second)) {
        	log_message("circular task\r\n");
            last_log_second = current_time.seconds;

            snprintf(g_log_buffer, MAX_LOG_MSG_LEN, "Power=%d-->%d\r\n", power_input, ignition_input);
            log_message(g_log_buffer);

            // 1. Create a snapshot of the current GPS data
            gps_data_t snapshot_data = { .is_valid = false };
            if (xSemaphoreTake(g_gps_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                snapshot_data = g_gps_data;
                xSemaphoreGive(g_gps_data_mutex);
            }

            // --- Get MSLD speed snapshot ---
            float msld_speed_snapshot = speed_rng; // Read the volatile variable directly

            if (snapshot_data.is_valid) {
                 snprintf(g_log_buffer, MAX_LOG_MSG_LEN, "RTC Trigger: Fix: Lat=%.5f, Lon=%.5f\r\n", snapshot_data.latitude, snapshot_data.longitude);
                 log_message(g_log_buffer);
            } else {
                 log_message("RTC Trigger: No valid GPS fix.\r\n");
            }

            // 2. Log the snapshot to the SD card
            if (sd_card_available) {

                // --- NEW UPDATED LOGIC ---
                char log_payload_str[90]; // String for the data part
                char msld_part_str[25];

                // Part 1: Format MSLD Data (blank if not available)
                if (msld_speed_snapshot > 0.0f) {
                     snprintf(msld_part_str, sizeof(msld_part_str), "MsldSpd_kmh:%.1f", msld_speed_snapshot);
                } else {
                     snprintf(msld_part_str, sizeof(msld_part_str), "MsldSpd_kmh:"); // Blank field
                }

                // Part 2: Format GPS Data and combine with MSLD
                if (snapshot_data.is_valid) {
                    // GPS is valid, log all GPS data
                    snprintf(log_payload_str, sizeof(log_payload_str),
                             "Lat:%.5f,Lon:%.5f,GpsSpd_kn:%.1f,%s",
                             snapshot_data.latitude, snapshot_data.longitude, snapshot_data.speed_knots,
                             msld_part_str);
                } else {
                    // No GPS fix, log "NO GPS FIX", blank GPS speed, and MSLD data
                    snprintf(log_payload_str, sizeof(log_payload_str),
                             "NO GPS FIX,GpsSpd_kn:,%s", // "NO GPS FIX" and blank GPS speed
                             msld_part_str);
                }

                // Part 3: Build the final log entry with timestamp
                snprintf(log_buffer, sizeof(log_buffer),
                         "20%02d/%02d/%02d,%02d:%02d:%02d,%s",
                         current_time.year, current_time.month, current_time.day_of_month,
                         current_time.hour, current_time.minutes, current_time.seconds,
                         log_payload_str);
                // --- END OF NEW UPDATED LOGIC ---


                if (!write_circular_log_entry(&g_circular_log_file, &g_log_meta, log_buffer)) {
                    log_message("LOG_ERROR: Write to SD failed!\r\n");
                    // Could add logic here to try and remount the SD card
                    sd_card_available = false;
                    f_close(&g_circular_log_file);
                } else {
                    // **************************************************
                    // ** CRITICAL FIX APPLIED **
                    // Sync data and save metadata immediately after every successful write.
                    // **************************************************
                    FRESULT fres = f_sync(&g_circular_log_file);
                    if (fres == FR_OK) {
                        // Only save metadata if the data file sync was successful
                        if (!save_log_metadata(&g_log_meta)) {
                             log_message("LOG_ERROR: Failed to save metadata after sync!\r\n");
                        }
                    } else {
                        // If sync fails, we must assume the card is disconnected or faulty
                        snprintf(g_log_buffer, MAX_LOG_MSG_LEN, "LOG_ERROR: f_sync failed! Code %d\r\n", fres);
                        log_message(g_log_buffer);
                        sd_card_available = false;
                        f_close(&g_circular_log_file);
                    }
                }


            }

            // 3. Send the EXACT same snapshot to the MQTT task
            xQueueOverwrite(g_data_snapshot_queue, &snapshot_data);
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Poll RTC at ~10Hz
    }
}



uint8_t history_lat[19];
uint8_t history_lon[19];
uint8_t history_alt[19];

uint8_t firstPart[25];
char *hashPos;

uint8_t trigger_gps_fix[100];

int parse(char data[25])
{
    // Find position of '#'
    hashPos = strchr(data, '#');
    if (hashPos != NULL)
    {
        // Calculate length before '#'
        size_t len = hashPos - data;
        // Copy that part into another string
        strncpy((char*)firstPart, data, len);
        firstPart[len] = '\0';   // null terminate
        return 1;
    }
    else
        return 0;

}

void write_gps_value_to_eeprom(uint16_t address, float value)
{
	int i=0;
	uint8_t buffer_data[19];

    sprintf((char*)buffer_data, "%f#", value);
    for(i=0;i<strlen((char*)buffer_data);i++)
    {
    	eeprom_WRITEBYTE((address+i),buffer_data[i]);
    }
    buffer_data[i]='\0';
	log_message("{");
	log_message((char*)buffer_data);
	log_message("}");
    vTaskDelay(pdMS_TO_TICKS(2));
}

void read_gps_value_from_eeprom(uint16_t address, uint8_t *output)
{
    //eeprom_READ24FC(address, 19);
    for(int k=0;k<19;k++)
    {
    	output[k]=eeprom_READBYTE(address+k);
    	if(output[k]=='#')
    	{
    		output[k]='\0';
    		k=99;
    	}
    }
    //sprintf(output, "%s", firstPart);
    log_message("{");
    log_message((char*)output);
    log_message("}");
    //sprintf(output, "%s", eeprom_TRANSFER);

//    int parse_return = parse(eeprom_TRANSFER);
//    if (parse_return == 1)
//    {
//        sprintf(output, "%s", firstPart);
//        log_message("{");
//        log_message(output);
//        log_message("}");
//    }
}

unsigned int calculate_checksum( char *sentence)
{
    unsigned int checksum = 0;
    // Start after the '$' character
    if (*sentence == '$') {
        sentence++;
    }
    // XOR until '*' or end of string
    while (*sentence && *sentence != '*') {
        checksum ^= (unsigned int)(*sentence);
        sentence++;
    }

    return checksum;
}

void vStatusLedTask(void *pvParameters)
{
	int parse_return=0;
	char buffer_data[30];
	uint16_t first_gps_fix=300;
	uint16_t gps_store_eeprom=0;
	int loop_counter=0;
	int serial_set_speed_flag=0;
	//char test_var[15];
	uint16_t serial_speed_counter=0;
	int gps_ever_disconnected=0;
	uint16_t gps_string=0;
	int test_flg=0;

	vTaskDelay(pdMS_TO_TICKS(700));
//	read_gps_value_from_eeprom(190, history_lat);
//	read_gps_value_from_eeprom(210, history_lon);
//	read_gps_value_from_eeprom(230, history_alt);

	//HAL_NVIC_EnableIRQ(USB_DRD_FS_IRQn);

	for(;;)
	{
		loop_counter=loop_counter+1;
		//gps_store_eeprom++;
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9); // Heartbeat LED
		if(loop_counter>=5)
		{
			loop_counter=0;
			if (g_gps_fix_acquired)
			{
				test_flg=20;
				if(gps_string==0)
					gps_string=9;
				HAL_GPIO_WritePin(GPS_INDICATION_GPIO_Port, GPS_INDICATION_Pin, GPIO_PIN_SET);  // Fix LED ON
			}
			else
			{
				test_flg=40;
				gps_string=0;
				if_altitude_true=0;
				gps_ever_disconnected=9;
				HAL_GPIO_TogglePin(GPS_INDICATION_GPIO_Port, GPS_INDICATION_Pin);                 // Searching LED blinks
			}
			if (g_gsm_connection_acquired)
			{
				HAL_GPIO_WritePin(GSM_INDICATION_GPIO_Port, GSM_INDICATION_Pin, GPIO_PIN_SET);  // Fix LED ON
			}
			else
			{
				HAL_GPIO_TogglePin(GSM_INDICATION_GPIO_Port, GSM_INDICATION_Pin);                 // Searching LED blinks
			}
		}

		timer_counter_ivms++;
		if(timer_counter_ivms > 2000)
		{
			timer_counter_ivms = 50;
		}

		if(serial_speed_receive.st_CFlg==1)
		{
			serial_speed_receive.st_counter++;
			if(serial_speed_receive.st_counter > NO_SERIAL_DATA_TIME)//NO_SERIAL_DATA_TIME = desired time in ms / 100
			{
				serial_speed_receive.st_SFlg=1;
				serial_speed_receive.st_CFlg=0;
				serial_speed_receive.st_counter=0;
				serial_data_true=0;
			}
		}
		else
		{
			serial_speed_receive.st_SFlg=0;
			serial_speed_receive.st_counter=0;
		}

		if(stack_size_tester.st_CFlg==1)
		{
			stack_size_tester.st_counter++;
			if(stack_size_tester.st_counter> 20)//NO_SERIAL_DATA_TIME = desired time in ms / 100
			{
				stack_size_tester.st_SFlg=1;
				stack_size_tester.st_CFlg=0;
				stack_size_tester.st_counter=0;
			}
		}
		else
		{
			stack_size_tester.st_SFlg=0;
			stack_size_tester.st_counter=0;
		}

		if(control_buzz_onoff.st_CFlg==1)
		{
			control_buzz_onoff.st_counter++;
			if(control_buzz_onoff.st_counter> 1)//NO_SERIAL_DATA_TIME = desired time in ms / 100
			{
				control_buzz_onoff.st_SFlg=1;
				//control_buzz_onoff.st_CFlg=0;
				control_buzz_onoff.st_counter=0;
			}
		}
		else
		{
			control_buzz_onoff.st_SFlg=0;
			control_buzz_onoff.st_counter=0;
		}

		if(pid_pedal_exit.st_CFlg==1)
		{
			pid_pedal_exit.st_counter++;
			//if(pid_pedal_exit.st_counter> 10)//PEDAL PID EXIT COUNTER = desired time in ms / 100
			if(pid_pedal_exit.st_counter > 4)////it was 8 earlier
			{
				pid_pedal_exit.st_SFlg=1;
				//pid_pedal_exit.st_CFlg=0;
				pid_pedal_exit.st_counter=0;
			}
		}
		else
		{
			pid_pedal_exit.st_SFlg=0;
			pid_pedal_exit.st_counter=0;
		}

		if(signal_disconnection.st_CFlg==1)
		{
			signal_disconnection.st_counter++;
			//if(pid_pedal_exit.st_counter> 10)//PEDAL PID EXIT COUNTER = desired time in ms / 100
			if(signal_disconnection.st_counter> 25)
			{
				signal_disconnection.st_SFlg=1;
				//pid_pedal_exit.st_CFlg=0;
				signal_disconnection.st_counter=0;
			}
		}
		else
		{
			signal_disconnection.st_SFlg=0;
			signal_disconnection.st_counter=0;
		}
		power_input 	= HAL_GPIO_ReadPin(POWERCUT_IN_GPIO_Port, POWERCUT_IN_Pin);
		ignition_input	= HAL_GPIO_ReadPin(IGNITION_IN_GPIO_Port, IGNITION_IN_Pin);


//		if((gps_store_eeprom>300)||(gps_string==9))
//		{
//			if((g_gps_fix_acquired==1)&&(if_altitude_true==9))
//			{
//				gps_string=99;
//				first_gps_fix=300;
//				gps_store_eeprom=0;
//				write_gps_value_to_eeprom(190, g_gps_data.latitude);
//				write_gps_value_to_eeprom(210, g_gps_data.longitude);
//				write_gps_value_to_eeprom(230, g_gps_data.altitude);
//				log_message("gps_stored");
//				sprintf(debug_test_varaible,"%f-%f-%f\n",g_gps_data.latitude,g_gps_data.longitude,g_gps_data.altitude);
//				log_message(debug_test_varaible);
//			}
//			else if(g_gps_fix_acquired==0)
//			{
//				first_gps_fix=300;
//				gps_ever_disconnected=0;
//				gps_store_eeprom=0;
//				read_gps_value_from_eeprom(190, history_lat);
//				read_gps_value_from_eeprom(210, history_lon);
//				read_gps_value_from_eeprom(230, history_alt);
//				sprintf(debug_test_varaible,"%s-%s-%s\n",history_lat,history_lon,history_alt);
//				log_message(debug_test_varaible);
//				sprintf(trigger_gps_fix,"$PAIR600,%s,%s,%s,50.0,50.0,0.0,100.0","13.077972","77.559517","317.0");
//				unsigned int cs = calculate_checksum(trigger_gps_fix);
//				sprintf(flash_read_buffer,"%s*%02x\r\n",trigger_gps_fix,cs);
//				log_message("trigger_gps_fix");
//				log_message(flash_read_buffer);
//				//get_gps_fast=9;
//			}
//		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

// --- MQTT HELPER FUNCTIONS ---

// --- THIS IS THE NEW, MUTEX-AWARE VERSION ---
void perform_modem_power_cycle(void) {

    // --- MUTEX TAKE ---
    if (xSemaphoreTake(g_modem_mutex, pdMS_TO_TICKS(15000)) != pdTRUE) {
        log_message("ERROR: Modem Mutex Timeout on Power Cycle\r\n");
        return;
    }

    char msg[80];
    snprintf(msg, sizeof(msg), "--- Performing Modem Power Cycle on %s Pin %d ---\r\n", "GPIOB", 9);
    log_message(msg);

    log_message("Powering modem OFF...\r\n");
    HAL_GPIO_WritePin(MODEM_PWR_RST_GPIO_Port, MODEM_PWR_RST_Pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(2000));
    log_message("Powering modem ON...\r\n");
    HAL_GPIO_WritePin(MODEM_PWR_RST_GPIO_Port, MODEM_PWR_RST_Pin, GPIO_PIN_SET);
    log_message("--- Modem Power Cycle Complete. Waiting for boot... ---\r\n");
    vTaskDelay(pdMS_TO_TICKS(12000));

    xSemaphoreGive(g_modem_mutex); // --- MUTEX GIVE ---
}

// --- THIS IS THE NEW, MUTEX-AWARE VERSION ---
bool send_and_wait_for_response(const char* cmd, const char* expected_response, uint32_t timeout_ms) {

    // --- MUTEX TAKE ---
    // Wait up to 15 seconds to get control of the modem
    if (xSemaphoreTake(g_modem_mutex, pdMS_TO_TICKS(15000)) != pdTRUE) {
        log_message("ERROR: Modem Mutex Timeout\r\n");
        return false;
    }

    uint8_t rx_buffer[MODEM_UART_RX_BUFFER_SIZE + 1];
    char log_buf[300];

    xQueueReset(g_modem_uart_rx_queue);
    snprintf(log_buf, sizeof(log_buf), "CMD >> %s\r\n", cmd);
    log_message(log_buf);
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);

    TickType_t start_ticks = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start_ticks) < pdMS_TO_TICKS(timeout_ms)) {
        if (xQueueReceive(g_modem_uart_rx_queue, &rx_buffer, pdMS_TO_TICKS(100)) == pdPASS) {
            if (strlen((char*)rx_buffer) < 250) { // Avoid logging large payloads
                 snprintf(log_buf, sizeof(log_buf), "RSP << %s\r\n", (char*)rx_buffer);
                 log_message(log_buf);
            }
            if (strstr((char*)rx_buffer, expected_response) != NULL) {
                xSemaphoreGive(g_modem_mutex); // --- MUTEX GIVE (on success) ---
                return true;
            }
        }
    }
    snprintf(log_buf, sizeof(log_buf), "ERROR: Timeout waiting for '%s'\r\n", expected_response);
    log_message(log_buf);
    xSemaphoreGive(g_modem_mutex); // --- MUTEX GIVE (on failure) ---
    return false;
}

void convert_utc_datetime_to_ist(const char* utc_date_str, const char* utc_time_str, char* ist_buffer, size_t buffer_size) {
    if (utc_date_str == NULL || strlen(utc_date_str) < 6 || utc_time_str == NULL || strlen(utc_time_str) < 6) {
        snprintf(ist_buffer, buffer_size, "2025-01-01 00:00:00");
        return;
    }
    int day = (utc_date_str[0] - '0') * 10 + (utc_date_str[1] - '0');
    int month = (utc_date_str[2] - '0') * 10 + (utc_date_str[3] - '0');
    int year = 2000 + (utc_date_str[4] - '0') * 10 + (utc_date_str[5] - '0');
    int hour = (utc_time_str[0] - '0') * 10 + (utc_time_str[1] - '0');
    int minute = (utc_time_str[2] - '0') * 10 + (utc_time_str[3] - '0');
    int second = (utc_time_str[4] - '0') * 10 + (utc_time_str[5] - '0');

    hour += 5;
    minute += 30;

    if (minute >= 60) { minute -= 60; hour++; }
    bool date_changed = false;
    if (hour >= 24) { hour -= 24; date_changed = true; }

    if (date_changed) {
        day++;
        const int days_in_month[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
        int month_days = days_in_month[month];
        if (month == 2 && ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))) { month_days = 29; }
        if (day > month_days) { day = 1; month++; if (month > 12) { month = 1; year++; } }
    }
    snprintf(ist_buffer, buffer_size, "%04d-%02d-%02d %02d:%02d:%02d", year, month, day, hour, minute, second);
}

void format_coord(char* buffer, size_t buffer_size, float coord, char positive_dir, char negative_dir) {
    char dir = (coord >= 0) ? positive_dir : negative_dir;
    float coord_abs = fabsf(coord);
    int degrees = (int)coord_abs;
    unsigned long long frac_ll = (unsigned long long)((coord_abs - degrees) * 10000000.0);
    if (frac_ll > 9999999) frac_ll = 9999999;
    unsigned long frac = (unsigned long)frac_ll;
    snprintf(buffer, buffer_size, "%03d.%07lu%c", degrees, frac, dir);
}



// --- THIS IS THE UPDATED vMqttTask ---
void vMqttTask(void *pvParameters) {
    log_message("--- MQTT Task Started ---\r\n");
    char command_buffer[256];
    char mqtt_payload[512];
    uint8_t primary_connection_retries = 0;
    const uint8_t max_retries_before_halt = 5;

    bool is_primary_connected = false;
    #if USE_SECONDARY_SERVER
    bool is_secondary_connected = false;
    #endif

    // --- ADDED THIS LINE (for STATUS SMS) ---
    g_session_start_time = HAL_GetTick();

    vTaskDelay(pdMS_TO_TICKS(5000));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_modem_dma_rx_buffer, MODEM_UART_RX_BUFFER_SIZE);
    perform_modem_power_cycle();

    log_message("Waiting for modem to respond...\r\n");
    while(!send_and_wait_for_response("AT\r\n", "OK", 1000)) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    send_and_wait_for_response("ATE0\r\n", "OK", 1000);
    log_message("Modem is responsive.\r\n");

    // ONE-TIME PDP CONTEXT SETUP
    log_message("--- Configuring Network Data Connection (PDP Context) ---\r\n");
    bool pdp_ready = false;
    while (!pdp_ready) {
        if (send_and_wait_for_response("AT+CGATT=1\r\n", "OK", 8000)) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            send_and_wait_for_response("AT+CGACT=0,1\r\n", "OK", 8000);
            vTaskDelay(pdMS_TO_TICKS(1000));
            snprintf(command_buffer, sizeof(command_buffer), "AT+CGDCONT=1,\"IP\",\"%s\"\r\n", g_apn);
            if (send_and_wait_for_response(command_buffer, "OK", 5000)) {
                if (send_and_wait_for_response("AT+CGACT=1,1\r\n", "OK", 60000)) {
                    log_message("--- Network Data Connection is Active ---\r\n");
                    pdp_ready = true;
                }
            }
        }
        if (!pdp_ready) {
            log_message("!!! Network Data Connection Failed. Retrying... !!!\r\n");
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }

    while(1) {
        // Primary Server Connection Management
        if (!is_primary_connected) {
            log_message("--- MQTT Primary Connection Attempt ---\r\n");
            bool setup_ok = false;
            send_and_wait_for_response("AT+QMTCLOSE=0\r\n", "OK", 5000);
            vTaskDelay(pdMS_TO_TICKS(500));
            snprintf(command_buffer, sizeof(command_buffer), "AT+QMTOPEN=0,\"%s\",%s\r\n", g_mqtt_broker_ip, g_mqtt_broker_port);
            if (send_and_wait_for_response(command_buffer, "+QMTOPEN: 0,0", 20000)) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                snprintf(command_buffer, sizeof(command_buffer), "AT+QMTCONN=0,\"%s\",\"%s\",\"%s\"\r\n", g_mqtt_client_id, g_mqtt_username, g_mqtt_password);
                if (send_and_wait_for_response(command_buffer, "+QMTCONN: 0,0,0", 20000)) {
                    setup_ok = true;
                }
            }

            if (setup_ok) {
                is_primary_connected = true;
                primary_connection_retries = 0;
                log_message("--- MQTT Primary Connection Successful ---\r\n");
            } else {
                log_message("--- MQTT Primary Connection Failed ---\r\n");
                if (++primary_connection_retries >= max_retries_before_halt) {
                    perform_modem_power_cycle();
                    pdp_ready = false; continue;
                }
            }
        }
        g_gsm_connection_acquired = is_primary_connected;

        // Secondary Server Connection Management
        #if USE_SECONDARY_SERVER
        if (!is_secondary_connected) {
            log_message("--- MQTT Secondary Connection Attempt ---\r\n");
            send_and_wait_for_response("AT+QMTCLOSE=1\r\n", "OK", 5000);
            vTaskDelay(pdMS_TO_TICKS(500));
            snprintf(command_buffer, sizeof(command_buffer), "AT+QMTOPEN=1,\"%s\",%s\r\n", g_secondary_mqtt_broker_ip, g_secondary_mqtt_broker_port);
            if (send_and_wait_for_response(command_buffer, "+QMTOPEN: 1,0", 20000)) {
                vTaskDelay(pdMS_TO_TICKS(1000));
                snprintf(command_buffer, sizeof(command_buffer), "AT+QMTCONN=1,\"%s\",\"%s\",\"%s\"\r\n", g_secondary_mqtt_client_id, g_secondary_mqtt_username, g_secondary_mqtt_password);
                if (send_and_wait_for_response(command_buffer, "+QMTCONN: 1,0,0", 20000)) {
                    is_secondary_connected = true;
                    log_message("--- MQTT Secondary Connection Successful ---\r\n");
                } else { log_message("--- MQTT Secondary Connection Failed. ---\r\n"); }
            } else { log_message("--- MQTT Secondary Open Failed. ---\r\n"); }
        }
        #endif

        // DATA ACQUISITION AND PUBLISHING
        gps_data_t data_to_send;
        if (xQueueReceive(g_data_snapshot_queue, &data_to_send, portMAX_DELAY) == pdPASS)
        {
            // --- ADDED: Get MSLD speed snapshot ---
            float msld_speed_snapshot = speed_rng; // Read the volatile variable directly

            // --- REPLACEMENT LOGIC ---
            char ts[22];
            char lat_str[25];
            char lon_str[25];
            char speed_str[6]; // GPS speed km/h
            char course_str[6];
            char data_validity_str[10];
            char status_str[10];
            char msld_speed_kmh_str[10]; // For the new field

            if (data_to_send.is_valid) {
                // --- Valid GPS Fix ---
                convert_utc_datetime_to_ist(data_to_send.date, data_to_send.time, ts, sizeof(ts));
                format_coord(lat_str, sizeof(lat_str), data_to_send.latitude, 'N', 'S');
                format_coord(lon_str, sizeof(lon_str), data_to_send.longitude, 'E', 'W');
                snprintf(course_str, sizeof(course_str), "%03d", (int)roundf(data_to_send.course_degrees));

                if(data_to_send.speed_knots < IDLE_SPEED_THRESHOLD_KNOTS) {
                    strncpy(status_str, "IDLE", sizeof(status_str));
                } else {
                    strncpy(status_str, "RUNNING", sizeof(status_str));
                }

                float speed_kmh = data_to_send.speed_knots * 1.852;
                snprintf(speed_str, sizeof(speed_str), "%03d", (strcmp(status_str, "IDLE") == 0) ? 0 : (int)roundf(speed_kmh));
                strncpy(data_validity_str, "Valid", sizeof(data_validity_str));
            } else {
                // --- No Valid GPS Fix ---
                // Get time from RTC instead
                RTC_Time current_time;
                DS3231_GetTime(&current_time);
                snprintf(ts, sizeof(ts), "%04d-%02d-%02d %02d:%02d:%02d", 2000 + current_time.year, current_time.month, current_time.day_of_month, current_time.hour, current_time.minutes, current_time.seconds);

                // Per your request, send blank/default values for GPS data
                snprintf(lat_str, sizeof(lat_str), ""); // Blank
                snprintf(lon_str, sizeof(lon_str), ""); // Blank
                snprintf(course_str, sizeof(course_str), "000");
                snprintf(speed_str, sizeof(speed_str), "000");
                strncpy(status_str, "IDLE", sizeof(status_str));
                strncpy(data_validity_str, "Invalid", sizeof(data_validity_str));
            }

            // --- Format MSLD Speed (common to both cases) ---
            // This will be inserted as a JSON number (e.g., 85.5 or 0.0)
            snprintf(msld_speed_kmh_str, sizeof(msld_speed_kmh_str), "%.1f", msld_speed_snapshot);

            // --- Build the final JSON Payload ---
            // Note: latitude and longitude are now JSON strings. They will be "" if no fix.
            snprintf(mqtt_payload, sizeof(mqtt_payload),
                     "{\"deviceID\":\"THIN0011\",\"IMEI\":\"864501070030500\",\"timestamp\":\"%s\","
                     "\"dataValidity\":\"%s\",\"status\":\"N1\",\"latitude\":\"%s\","
                     "\"longitude\":\"%s\",\"speed\":\"%s\",\"course\":\"%s\","
                     "\"ignition\":\"%s\",\"vehicleStatus\":\"%s\","
                     "\"additionalData\":\"0000000000000\",\"timeIntervals\":\"002,010,002\","
                     "\"angleInterval\":\"015\",\"distanceInterval\":\"100\","
                     "\"gsmStrength\":\"073\",\"sequenceNumber\":\"Q263\","
                     "\"msldSpeed_kmh\":%s}", // <-- New field added here as a number
                     ts, data_validity_str, lat_str, lon_str, speed_str, course_str,
                     ignitionStatus, status_str, msld_speed_kmh_str); // <-- New variable added here

            // --- END OF REPLACEMENT LOGIC ---

            // Publish to Primary
            if (is_primary_connected) {
                snprintf(command_buffer, sizeof(command_buffer), "AT+QMTPUBEX=0,0,0,0,\"%s\",%d\r\n", g_mqtt_topic, (int)strlen(mqtt_payload));
                if (send_and_wait_for_response(command_buffer, ">", 5000)) {
                    if (send_and_wait_for_response(mqtt_payload, "+QMTPUBEX: 0,0,0", 10000)) {
                        log_message("Publish OK (Primary)\r\n");

                        // --- ADDED THIS LINE (for STATUS SMS) ---
                        g_successful_publishes++;

                    } else { is_primary_connected = false; }
                } else { is_primary_connected = false; }

            }

            #if USE_SECONDARY_SERVER
            if (is_secondary_connected) {
                // ... (your secondary publish logic) ...
            }
            #endif
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// --- UART CALLBACKS ---

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
   // GPS Module on UART12
   if (huart->Instance == UART12) {
        write_pos = Size;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(g_gps_data_ready_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
   }
   // Cellular Modem on USART1
   else if (huart->Instance == USART1) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        uint8_t received_data[MODEM_UART_RX_BUFFER_SIZE + 1] = {0};
        memcpy(received_data, g_modem_dma_rx_buffer, Size);

        xQueueSendFromISR(g_modem_uart_rx_queue, &received_data, &xHigherPriorityTaskWoken);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_modem_dma_rx_buffer, MODEM_UART_RX_BUFFER_SIZE);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
   }
   else if(huart->Instance == USART2)
	{
		//HAL_UART_Transmit(&huart1,(uint8_t*)"G\n" , 2, HAL_MAX_DELAY);
		uart2_rx_size = Size;
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		//xSemaphoreGiveFromISR(handheld_data_ready_sem, &xHigherPriorityTaskWoken);
		//portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    char error_msg[80];
    if (huart->Instance == UART12) {
        if (HAL_UART_GetError(huart) & HAL_UART_ERROR_ORE) {
           snprintf(error_msg, sizeof(error_msg), "!! UART12 (GPS) Overrun Error. Restarting DMA. !!\r\n");
           log_message(error_msg);
            HAL_UART_AbortReceive(huart);
            HAL_UARTEx_ReceiveToIdle_DMA(&huart12, g_gps_dma_rx_buffer, GPS_DMA_RX_BUFFER_SIZE);
        }
    } else if (huart->Instance == USART1) {
         uint32_t error_code = HAL_UART_GetError(huart);
         snprintf(error_msg, sizeof(error_msg), "!! UART1 (Modem) Error, code: 0x%lX. Restarting DMA. !!\r\n", error_code);
         log_message(error_msg);
         HAL_UART_AbortReceive(huart);
         HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_modem_dma_rx_buffer, MODEM_UART_RX_BUFFER_SIZE);
    }
}

/*
 * MSLD CODE and TASK
 * Variables on the top and in global_declaration.s
 */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int _write(int file, char *ptr, int len)
{
	int DataIdx;
	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		//__io_putchar(*ptr++);
		ITM_SendChar(*ptr++);
	}
	return len;
}

float getSpeed()
{
	float filtered_speed=0.0;
	int test_var[15];
	float frequency_rate=0.0;
	int input_switch_fz_lock=0;

	float alpha = 0.3;

	ppr_value=program_setting_data[pgmr_pointer][6];
	//ppr_value=program_setting_data[dp][6];
	//speed=ppr_value;//
	speed=ppr_value*frequency;//ppr*freq
	//speed=ppr_value*freq;//ppr*freq

	//printf("ppr=%1.3f\n",ppr_value);
	//printf("speed=%2.2f\n",speed);


	/*
	sprintf(test_var,"Speed=%3.3f\n",speed);
	HAL_UART_Transmit(&huart1,(uint8_t*)test_var,(uint16_t)(strlen((char*)test_var)), 1000);
	vTaskDelay(1/portTICK_PERIOD_MS);
	*/

	if((speed<(spd+10)) && (speed>(spd-10)))//if((speed<(spd+8)) && (speed>(spd-8)) )
	{
		speed=speed;
		spd=speed;
		spd_exit=0;
	}
	else
	{
		spd_exit++;
		if(spd_exit>50)
		{
			spd_exit=0;
			spd=speed;
		}
		if(frequency<=0)
		{
			speed=0;
			spd=speed;
		}
		else
		{
			speed=spd;
		}
	}
	return speed;
	//filtered_speed = alpha * speed + (1 - alpha) * previous_speed;
	//previous_speed = speed;
	//return speed;
	//return filtered_speed;
}

float pid_controller(float Kp,float Ki,float Kd,float set_speed,float current_speed,float pedal_voltage_at_rated_speed)
{
	float increase_pedal_percentage=0.0;
	//!Kp-proportional gain
	//!Ki-integral gain
	//!Kd-defferential gain
	//!set_speed-Rated speed
	//!Current_speed-Running speed
	//!pedal_voltage_at_rated_speed-Pedal value at which speed cross the set speed
	//!P=proportional variable
	//!I=Integral variable
	//!D=differential variable
	//!pedal_volatge=Variable to store value of pedal at limiting area.
	//!//
	float P=0;
	float D=0;
	float pedal_voltage=0;
	float temp_ki=0.0;
	float temp_kp=0.0;
	//Get error value form the set speed and current running speed.
	//Simply take difference between set speed and current speed of the vehicle
	error_value=set_speed-current_speed;
	//store the current error value in proportional variable P.
	P=error_value;
	//Get the integral term by simply accumulating the errors generated by the PID controller

	temp_ki = Ki;
	temp_kp = Kp;



	if(I_Clamping==0)
	{
		I_=I_+error_value;
	}

	//Differential varable D can be find out by simply taking the difference between current and previous errors
	D=error_value-pre_error_value;
	//Store the current error values to the previous error variable for getting D term

	//To get the pid controller final action take the product sum of all three terms
	//with the corresponding gain values.And by adding the pedal value at the set speed and pid factor ,
	//to get the pedal voltage to maintain the speed.

	////Edited by Renjith
	/*
	if(pgmr_pointer==0)
	{
		I_ += error_value * 0.01;
		D 	= (error_value-pre_error_value)/0.01;
	}
	*/

	error_percentage = (float)(error_value * 0.833);//Percentage 100 / max speed = 120Km/Hr

	///////////Below line for izusu
	//temp_ki = Ki;
	//error_percentage = (float)(error_value * 0.833);

	if(error_percentage>0.9)//early 0.5 and +0.004
	{
		//I_=I_-(error_value/2);
		//temp_ki = ki;
		temp_ki = Ki+0.003;
	}
	///earlier multiple of 4 now multiple of .35
	increase_pedal_percentage = pedal_voltage_at_rated_speed;
	if(set_speed>=120)
	{
		increase_pedal_percentage = pedal_voltage_at_rated_speed-1.2;//1.34 was better//1.6 not that much//1.2not tested
	}
	else if(set_speed>=110)
	{
		increase_pedal_percentage = pedal_voltage_at_rated_speed-1.2;
	}
	else if(set_speed>=100)
	{
		increase_pedal_percentage = pedal_voltage_at_rated_speed-0.8;
	}
	else if(set_speed>=90)
	{
		increase_pedal_percentage = pedal_voltage_at_rated_speed-0.4;
	}
	else
	{
		increase_pedal_percentage = pedal_voltage_at_rated_speed;
	}

	pid_result = ((temp_kp*P)+(temp_ki*I_)+(Kd*D));
	pedal_voltage=pedal_voltage_at_rated_speed + pid_result;

	if(pedal_voltage<=program_setting_data[pgmr_pointer][0])
	{
		pedal_voltage=program_setting_data[pgmr_pointer][0];
		I_Clamping=1;
	}
	else if(pedal_voltage>=program_setting_data[pgmr_pointer][1])
	{
		pedal_voltage=program_setting_data[pgmr_pointer][1];
		I_Clamping=1;
	}
	else
	{
		I_Clamping=0;
	}
	pre_error_value=error_value;
	return(pedal_voltage);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	BaseType_t xReturned;
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C4_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_UART12_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_I2C1_Init();
  MX_USB_PCD_Init();
  MX_TIM2_Init();
  MX_ICACHE_Init();
  /* USER CODE BEGIN 2 */

  MX_FATFS_Init();


  HAL_GPIO_WritePin(MCP4922_LDAC_GPIO_Port, MCP4922_LDAC_Pin, GPIO_PIN_SET);

	ignition_input = HAL_GPIO_ReadPin(IGNITION_IN_GPIO_Port, IGNITION_IN_Pin);
	if(ignition_input==0)
	{
		sprintf(ignitionStatus,"IGon");
	}
	else
	{
		sprintf(ignitionStatus,"IGoff");
	}

	//MX_USB_PCD_Init();

	  if(USBD_Init(&hUsbDeviceFS, &Class_Desc, 0) != USBD_OK)
	  	Error_Handler();
	  /* Store HID Instance Class ID */
	 // HID_InstID = hUsbDeviceFS.classId;
	  /* Register the HID Class */
	//  if(USBD_RegisterClassComposite(&hUsbDeviceFS, USBD_HID_CLASS, CLASS_TYPE_HID, &HID_EpAdd_Inst) != USBD_OK)
	//  	Error_Handler();
	  /* Store the HID Class */
	//  CDC_InstID = hUsbDeviceFS.classId;
	  /* Register CDC Class First Instance */
	  if(USBD_RegisterClassComposite(&hUsbDeviceFS, USBD_CDC_CLASS, CLASS_TYPE_CDC, CDC_EpAdd_Inst) != USBD_OK)
	  	Error_Handler();
	  /* Add CDC Interface Class */
	  if (USBD_CMPSIT_SetClassID(&hUsbDeviceFS, CLASS_TYPE_CDC, 0) != 0xFF)
	  {
	  	USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_CDC_Template_fops);
	  }


	  HAL_GPIO_WritePin(POWER_LED_GPIO_Port, POWER_LED_Pin, GPIO_PIN_SET);


  // --- Create RTOS objects ---

  	handheld_data_ready_sem = xSemaphoreCreateBinary();
	g_gps_data_ready_sem = xSemaphoreCreateBinary();
	g_gps_data_mutex = xSemaphoreCreateMutex();
	g_log_queue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(char[MAX_LOG_MSG_LEN]));
	g_modem_uart_rx_queue = xQueueCreate(MODEM_UART_RX_QUEUE_LEN, MODEM_UART_RX_BUFFER_SIZE + 1);
	g_data_snapshot_queue = xQueueCreate(1, sizeof(gps_data_t)); // Length 1, acts like an overwriting mailbox


	g_modem_mutex = xSemaphoreCreateMutex(); // Create the new mutex


	// --- Create RTOS tasks ---
	xTaskCreate(vStatusLedTask, "StatusLedTask", 1024, NULL, tskIDLE_PRIORITY +4, NULL);
	xTaskCreate(vLoggingTask, "LoggingTask", 1024, NULL, tskIDLE_PRIORITY + 7, NULL);
	xTaskCreate(vMqttTask, "MqttTask", 4096, NULL, tskIDLE_PRIORITY + 6, NULL);
	xTaskCreate(vCircularTask, "CircularLogTask", 4096, NULL, tskIDLE_PRIORITY + 6, NULL);
	xTaskCreate(vGpsTask, "GpsTask", 2048, NULL, tskIDLE_PRIORITY + 7, NULL); // Highest priority

	xTaskCreate(vSmsTask, "SmsTask", 2048, NULL, tskIDLE_PRIORITY + 5, NULL); // Create the new SMS task


//	xReturned=xTaskCreate( timer_jobs, "Handler",512, NULL,tskIDLE_PRIORITY + 11, &xTaskHandler );//frequency
//	if( xReturned == pdPASS )
//	{
//	}

	xReturned=xTaskCreate( SYHandTask,"Indication",2048, NULL,7 , NULL );
	xTaskCreate( vHandTask, "Handler",512, NULL,7, &xTaskHandler2 );//frequency
	xTaskCreate( iHandTask, "pedal_read",1024,NULL, 7, NULL );//pedal volatge


	xTaskCreate( time_jobs, "Handler",512, NULL,tskIDLE_PRIORITY + 6, NULL );//frequency

	xTaskCreate( dev_set, "Handler",512, NULL,tskIDLE_PRIORITY + 4, NULL );//frequency

	//xReturned=xTaskCreate( iHandlerTask, "pedal_read",256, NULL,2, &xTaskHandler1 );//pedal volatge

	xReturned=xTaskCreate( HH_dma,"Indication",1024,NULL, 6, NULL );


	 HAL_UARTEx_ReceiveToIdle_DMA(&huart2,(uint8_t*) handheld_serial_data, DMA_BUFFER_SIZE);

	// Start the RTOS scheduler
	log_message("\r\n--- System Initialized. Starting scheduler... ---\r\n");

	HAL_TIM_Base_Start(&htim2);
	//HAL_NVIC_SetPriority(EXTI0_IRQn,15, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

   // HAL_NVIC_EnableIRQ(GPDMA2_Channel5_IRQn);

    HAL_NVIC_EnableIRQ(GPDMA2_Channel6_IRQn);

    HAL_NVIC_EnableIRQ(GPDMA2_Channel7_IRQn);
    HAL_NVIC_EnableIRQ(UART12_IRQn);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

   // HAL_NVIC_EnableIRQ(USB_DRD_FS_IRQn);

    USBD_Start(&hUsbDeviceFS);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_HSI48;

  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 150;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_0;
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

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_1);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief GPDMA2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA2_Init(void)
{

  /* USER CODE BEGIN GPDMA2_Init 0 */

  /* USER CODE END GPDMA2_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA2_CLK_ENABLE();

  /* GPDMA2 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA2_Channel5_IRQn, 6, 0);
  //  HAL_NVIC_EnableIRQ(GPDMA2_Channel5_IRQn);
    HAL_NVIC_SetPriority(GPDMA2_Channel6_IRQn, 6, 0);
   // HAL_NVIC_EnableIRQ(GPDMA2_Channel6_IRQn);
    HAL_NVIC_SetPriority(GPDMA2_Channel7_IRQn, 6, 0);
   // HAL_NVIC_EnableIRQ(GPDMA2_Channel7_IRQn);

  /* USER CODE BEGIN GPDMA2_Init 1 */

  /* USER CODE END GPDMA2_Init 1 */
  /* USER CODE BEGIN GPDMA2_Init 2 */

  /* USER CODE END GPDMA2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x109093DC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x109093DC;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache (default 2-ways set associative cache)
  */
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x7;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi2.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi2.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x7;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi3.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi3.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 98304;
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

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief UART12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART12_Init(void)
{

  /* USER CODE BEGIN UART12_Init 0 */

  /* USER CODE END UART12_Init 0 */

  /* USER CODE BEGIN UART12_Init 1 */

  /* USER CODE END UART12_Init 1 */
  huart12.Instance = UART12;
  huart12.Init.BaudRate = 115200;
  huart12.Init.WordLength = UART_WORDLENGTH_8B;
  huart12.Init.StopBits = UART_STOPBITS_1;
  huart12.Init.Parity = UART_PARITY_NONE;
  huart12.Init.Mode = UART_MODE_TX_RX;
  huart12.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart12.Init.OverSampling = UART_OVERSAMPLING_16;
  huart12.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart12.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart12.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart12) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart12, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart12, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart12) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART12_Init 2 */

  /* USER CODE END UART12_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart6, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart6, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_DRD_FS.Instance = USB_DRD_FS;
  hpcd_USB_DRD_FS.Init.dev_endpoints = 8;
  hpcd_USB_DRD_FS.Init.speed = USBD_FS_SPEED;
  hpcd_USB_DRD_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_DRD_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.bulk_doublebuffer_enable = DISABLE;
  hpcd_USB_DRD_FS.Init.iso_singlebuffer_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_DRD_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OUTPUT_BUZZER_Pin|VALVE_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GSM_INDICATION_GPIO_Port, GSM_INDICATION_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SDCARD_CS_Pin|GPS_RESET_Pin|CTRL_INDICATION_Pin|GSM_RESET_Pin
                          |GSM_POWERKEY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SGL_INDICATION_Pin|GPS_INDICATION_Pin|MCP4922_SS_Pin|MCP4922_LDAC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(POWER_LED_GPIO_Port, POWER_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OUTPUT_BUZZER_Pin VALVE_OUT_Pin */
  GPIO_InitStruct.Pin = OUTPUT_BUZZER_Pin|VALVE_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_IN_2_Pin SEATBELT_IN_Pin POWERCUT_IN_Pin IGNITION_IN_Pin
                           GSM_STATUS_Pin OPTO_OP_Pin */
  GPIO_InitStruct.Pin = SW_IN_2_Pin|SEATBELT_IN_Pin|POWERCUT_IN_Pin|IGNITION_IN_Pin
                          |GSM_STATUS_Pin|OPTO_OP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPEED_SGL_P_Pin */
  GPIO_InitStruct.Pin = SPEED_SGL_P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPEED_SGL_P_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GSM_INDICATION_Pin */
  GPIO_InitStruct.Pin = GSM_INDICATION_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GSM_INDICATION_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SDCARD_CS_Pin GPS_RESET_Pin CTRL_INDICATION_Pin GSM_RESET_Pin
                           GSM_POWERKEY_Pin */
  GPIO_InitStruct.Pin = SDCARD_CS_Pin|GPS_RESET_Pin|CTRL_INDICATION_Pin|GSM_RESET_Pin
                          |GSM_POWERKEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SGL_INDICATION_Pin GPS_INDICATION_Pin MCP4922_SS_Pin MCP4922_LDAC_Pin */
  GPIO_InitStruct.Pin = SGL_INDICATION_Pin|GPS_INDICATION_Pin|MCP4922_SS_Pin|MCP4922_LDAC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : POWER_LED_Pin */
  GPIO_InitStruct.Pin = POWER_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(POWER_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PGMR_CONE_Pin */
  GPIO_InitStruct.Pin = PGMR_CONE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PGMR_CONE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_IN_1_Pin */
  GPIO_InitStruct.Pin = SW_IN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_IN_1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 8, 0);
 // HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */





// --- HELPER FUNCTION FROM REFERENCE CODE (ADAPTED) ---
// This function must only be called when the modem mutex is already taken!
void flush_modem_uart_buffer(void)
{
    uint8_t flush_char;
    uint16_t flush_count = 0;
    char log_buf[60];

    while (HAL_UART_Receive(&huart1, &flush_char, 1, 10) == HAL_OK && flush_count < 100) {
        flush_count++;
    }

    if (flush_count > 0) {
        snprintf(log_buf, sizeof(log_buf), "Flushed %d bytes from modem buffer\r\n", flush_count);
        log_message(log_buf);
    }
}

// --- HELPER FUNCTION FROM REFERENCE CODE (ADAPTED) ---
bool send_sms(const char* recipient, const char* message)
{
    char cmd_buf[200];
    char ctrl_z = 26;
    char log_buf[80];

    snprintf(log_buf, sizeof(log_buf), "--- Sending SMS to %s ---\r\n", recipient);
    log_message(log_buf);

    // We don't need to take the mutex here, because send_and_wait_for_response
    // will handle the mutex for each command.
    if (!send_and_wait_for_response("AT+CMGF=1\r\n", "OK", 3000)) return false;

    snprintf(cmd_buf, sizeof(cmd_buf), "AT+CMGS=\"%s\"\r\n", recipient);
    if (!send_and_wait_for_response(cmd_buf, ">", 8000)) return false;

    vTaskDelay(pdMS_TO_TICKS(200));

    snprintf(cmd_buf, sizeof(cmd_buf), "%s%c", message, ctrl_z);
    if (!send_and_wait_for_response(cmd_buf, "OK", 15000)) return false;

    log_message("SMS sent successfully.\r\n");
    return true;
}


// --- NEW SMS TASK (ADAPTED FOR YOUR PROJECT) ---
void vSmsTask(void *pvParameters)
{
    char sms_buffer[512];
    char command_buffer[100];
    char temp_log[200]; // For formatting log messages

    vTaskDelay(pdMS_TO_TICKS(15000)); // Initial delay

    // --- Clear all SMS on startup ---
    log_message("=== STARTUP: Clearing all SMS messages ===\r\n");
    send_and_wait_for_response("AT+CMGF=1\r\n", "OK", 3000);
    send_and_wait_for_response("AT+CMGDA=\"DEL ALL\"\r\n", "OK", 5000);
    log_message("All SMS messages cleared on startup\r\n");


    while(1)
    {
        // Check for SMS every 20 seconds
        vTaskDelay(pdMS_TO_TICKS(20000));

        log_message("----------------------------------------- Checking for SMS Commands ------\r\n");

        if (!send_and_wait_for_response("AT+CMGF=1\r\n", "OK", 3000))
        {
            continue; // Will retry after 20 seconds
        }

        // --- THIS IS THE CORRECTED LOGIC ---
        // It now uses the g_modem_uart_rx_queue instead of HAL_UART_Receive

        if (xSemaphoreTake(g_modem_mutex, pdMS_TO_TICKS(15000)) != pdTRUE) {
             log_message("ERROR: SMS Task Mutex Timeout\r\n");
             continue;
        }

        memset(sms_buffer, 0, sizeof(sms_buffer));
        xQueueReset(g_modem_uart_rx_queue); // Clear the queue of any old data

        // Send the command to list all SMS
        HAL_UART_Transmit(&huart1, (uint8_t*)"AT+CMGL=\"ALL\"\r\n", strlen("AT+CMGL=\"ALL\"\r\n"), HAL_MAX_DELAY);

        uint16_t rx_index = 0;
        TickType_t start_tick = xTaskGetTickCount();

        // Loop to read from the *queue*
        while((xTaskGetTickCount() - start_tick) < pdMS_TO_TICKS(8000)) // 8-second timeout
        {
            uint8_t queue_rx_buffer[MODEM_UART_RX_BUFFER_SIZE + 1];

            // Wait up to 200ms for a message from the queue
            if (xQueueReceive(g_modem_uart_rx_queue, &queue_rx_buffer, pdMS_TO_TICKS(200)) == pdPASS)
            {
                // Append received data chunk to our main sms_buffer
                int new_len = strlen((char*)queue_rx_buffer);
                if (rx_index + new_len < sizeof(sms_buffer) - 1)
                {
                    strcat(sms_buffer, (char*)queue_rx_buffer);
                    rx_index += new_len;
                }

                // Check if we have received the final "OK" or "ERROR"
                if (strstr(sms_buffer, "\r\nOK\r\n") != NULL || strstr(sms_buffer, "\r\nERROR\r\n") != NULL)
                {
                    break; // Stop reading
                }
            }
        }

        // Log the complete response we built from the queue
        snprintf(temp_log, sizeof(temp_log), "SMS RSP << %s\r\n", sms_buffer);
        log_message(temp_log);

        // Give the mutex back *before* processing and calling send_sms
        xSemaphoreGive(g_modem_mutex);
        // --- END OF CORRECTED LOGIC ---


        char* current_msg_ptr = strstr(sms_buffer, "+CMGL:");
        while (current_msg_ptr != NULL)
        {
            int msg_index;
            char sender_number[20] = {0};
            char msg_content[161] = {0};

            // Parse the "+CMGL: <index>,"<status>","<sender>"..."
            sscanf(current_msg_ptr, "+CMGL: %d,\"%*[^\"]\",\"%[^\"]\"", &msg_index, sender_number);

            snprintf(temp_log, sizeof(temp_log), "Found SMS at index %d from %s\n", msg_index, sender_number);
            log_message(temp_log);

            // Find the start of the message content (which is on the next line)
            char* content_start = strchr(current_msg_ptr, '\n');
            if (content_start != NULL)
            {
                content_start++; // Move past the '\n'
                char* content_end = strchr(content_start, '\n'); // Find the end of the line
                if (content_end != NULL)
                {
                    size_t content_len = content_end - content_start;
                    if (content_len < sizeof(msg_content))
                    {
                        strncpy(msg_content, content_start, content_len);
                        // Clean up trailing \r
                        if (content_len > 0 && msg_content[content_len - 1] == '\r')
                        {
                            msg_content[content_len - 1] = '\0';
                        }
                    }
                }
            }

            if (strcmp(sender_number, AUTHORIZED_SENDER) == 0)
            {
                snprintf(temp_log, sizeof(temp_log), "Authorized message: '%s'\r\n", msg_content);
                log_message(temp_log);

                char ack_message[160] = "ACK: Unknown Command";
                char* value_ptr = strchr(msg_content, ':');
                if (value_ptr != NULL)
                {
                    *value_ptr = '\0'; // Split string at ':'
                    value_ptr++;       // value_ptr now points to the value
                }

                // --- Process commands ---
                if (strcmp(msg_content, "SET_MQTT_BROKER") == 0 && value_ptr != NULL) {
                    char port[6];
                    sscanf(value_ptr, "%[^,],%s", g_mqtt_broker_ip, port);
                    strcpy(g_mqtt_broker_port, port);
                    snprintf(ack_message, sizeof(ack_message), "ACK: Broker set to %s:%s", g_mqtt_broker_ip, g_mqtt_broker_port);
                }
                else if (strcmp(msg_content, "SET_MQTT_CLIENT") == 0 && value_ptr != NULL) {
                    strncpy(g_mqtt_client_id, value_ptr, sizeof(g_mqtt_client_id) - 1);
                    snprintf(ack_message, sizeof(ack_message), "ACK: Client ID set");
                }
                else if (strcmp(msg_content, "SET_MQTT_USER") == 0 && value_ptr != NULL) {
                    strncpy(g_mqtt_username, value_ptr, sizeof(g_mqtt_username) - 1);
                    snprintf(ack_message, sizeof(ack_message), "ACK: User set");
                }
                else if (strcmp(msg_content, "SET_MQTT_PASS") == 0 && value_ptr != NULL) {
                    strncpy(g_mqtt_password, value_ptr, sizeof(g_mqtt_password) - 1);
                    snprintf(ack_message, sizeof(ack_message), "ACK: Password set");
                }
                else if (strcmp(msg_content, "SET_MQTT_TOPIC") == 0 && value_ptr != NULL) {
                    strncpy(g_mqtt_topic, value_ptr, sizeof(g_mqtt_topic) - 1);
                    snprintf(ack_message, sizeof(ack_message), "ACK: Topic set");
                }
                else if (strcmp(msg_content, "GPS_STATUS") == 0) {
                    gps_data_t current_gps_data = {0};
                    if (xSemaphoreTake(g_gps_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        current_gps_data = g_gps_data;
                        xSemaphoreGive(g_gps_data_mutex);
                    }
                    if (current_gps_data.is_valid) {
                        snprintf(ack_message, sizeof(ack_message), "GPS: %.5f,%.5f Alt:%.1fm Spd:%.1fkn Msgs:%lu",
                                 current_gps_data.latitude, current_gps_data.longitude,
                                 current_gps_data.altitude, current_gps_data.speed_knots, g_successful_publishes);
                    } else {
                        snprintf(ack_message, sizeof(ack_message), "GPS: No Fix. Msgs:%lu Runtime:%lus",
                                 g_successful_publishes, (unsigned long)((HAL_GetTick() - g_session_start_time) / 1000));
                    }
                }
                else if (strcmp(msg_content, "STATUS") == 0) {
                    snprintf(ack_message, sizeof(ack_message), "Runtime:%lus Msgs:%lu SpdLim:%u",
                             (unsigned long)((HAL_GetTick() - g_session_start_time) / 1000), g_successful_publishes, set_speed_eep);
                }

                // --- THIS IS YOUR NEW CUSTOM COMMAND ---
                else if (strcmp(msg_content, "SET_SPEED_LIMIT") == 0 && value_ptr != NULL) {
                    uint8_t new_speed = (uint8_t)atoi(value_ptr);
                    if (new_speed >= 5 && new_speed <= 150) { // Safety check

                        set_speed_eep = new_speed; // Update the default speed limit from handheld_programing.h

                        // If not in serial override mode, also update the *current* set_speed
                        if (set_speed_flag == 0) {
                            set_speed = set_speed_eep;
                        }
                        snprintf(ack_message, sizeof(ack_message), "ACK: Speed limit set to %u", set_speed_eep);
                    } else {
                        snprintf(ack_message, sizeof(ack_message), "ACK: Invalid speed value (5-150)");
                    }
                }

                else if (strcmp(msg_content, "REBOOT") == 0)
                {
                    // We must delete the message *before* we reboot.
                    snprintf(command_buffer, sizeof(command_buffer), "AT+CMGD=%d\r\n", msg_index);
                    send_and_wait_for_response(command_buffer, "OK", 3000);

                    vTaskDelay(pdMS_TO_TICKS(500));

                    snprintf(ack_message, sizeof(ack_message), "ACK: Rebooting after %lu messages...", g_successful_publishes);
                    send_sms(AUTHORIZED_SENDER, ack_message);

                    vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for SMS to send

                    log_message("REBOOT command executed - resetting system\r\n");
                    HAL_NVIC_SystemReset();
                    // We will not get past this line
                }

                // Send the ACK message (unless we are rebooting)
                send_sms(AUTHORIZED_SENDER, ack_message);
            }
            else
            {
                snprintf(temp_log, sizeof(temp_log), "Ignoring message from unauthorized sender: %s\r\n", sender_number);
                log_message(temp_log);
            }

            // Delete this specific SMS (this call is mutex-safe)
            snprintf(command_buffer, sizeof(command_buffer), "AT+CMGD=%d\r\n", msg_index);
            send_and_wait_for_response(command_buffer, "OK", 3000);

            // Find the next message
            current_msg_ptr = strstr(current_msg_ptr + 1, "+CMGL:");
        }
    }
}






int a1,a2;
int a5=0;
int a=0;
char debug_print_var[100];
uint8_t counter_=0;
uint8_t data_len=0;


int inside_pid_loop_flag=0;

void dev_set( void *pvParameters )
{

	for(;;)
	{
		power_input = HAL_GPIO_ReadPin(POWERCUT_IN_GPIO_Port, POWERCUT_IN_Pin);
		if(power_input==0)
		{
			//power_limp=9;
			//limp mode
			//HAL_GPIO_WritePin(LIMP_ENABLE_GPIO_Port, LIMP_ENABLE_Pin, GPIO_PIN_RESET);
			sprintf(power_status,"PowD");
		}
		else
		{
			//power_limp=0;
			sprintf(power_status,"PowC");
//			if(gps_limp!=9)
//			{
//				//HAL_GPIO_WritePin(LIMP_ENABLE_GPIO_Port, LIMP_ENABLE_Pin, GPIO_PIN_SET);
//				//DEBUG_PRINTF("Limp\n");
//				//limp_flag1=9;
//			}
		}

		ignition_input = HAL_GPIO_ReadPin(IGNITION_IN_GPIO_Port, IGNITION_IN_Pin);
		if(ignition_input==0)
		{
			sprintf(ignitionStatus,"IGon");
			//sd_card_busy_printing=0;
		}
		else
		{
			sprintf(ignitionStatus,"IGoff");
//			printer_switch_status = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);
//			if(printer_switch_status==0)
//			{
//				sd_card_busy_printing=9;
//				DEBUG_PRINTF("Printer Data\n");
//				read_header_entry(80);
//				read_circular_log_entry(&circularLogFile,80,5);
//				sd_card_busy_printing=0;
//
//			}
		}
	}
}


//void timer_jobs(void *pvParameters)
//{
//	int serial_set_speed_flag=0;
//	//char test_var[15];
//	uint16_t serial_speed_counter=0;
//
//	for(;;)
//	{
//		timer_counter_ivms++;
//		if(timer_counter_ivms > 2000)
//		{
//			timer_counter_ivms = 50;
//		}
//
//		if(serial_speed_receive.st_CFlg==1)
//		{
//			serial_speed_receive.st_counter++;
//			if(serial_speed_receive.st_counter > NO_SERIAL_DATA_TIME)//NO_SERIAL_DATA_TIME = desired time in ms / 100
//			{
//				serial_speed_receive.st_SFlg=1;
//				serial_speed_receive.st_CFlg=0;
//				serial_speed_receive.st_counter=0;
//				serial_data_true=0;
//			}
//		}
//		else
//		{
//			serial_speed_receive.st_SFlg=0;
//			serial_speed_receive.st_counter=0;
//		}
//
//		if(stack_size_tester.st_CFlg==1)
//		{
//			stack_size_tester.st_counter++;
//			if(stack_size_tester.st_counter> 20)//NO_SERIAL_DATA_TIME = desired time in ms / 100
//			{
//				stack_size_tester.st_SFlg=1;
//				stack_size_tester.st_CFlg=0;
//				stack_size_tester.st_counter=0;
//			}
//		}
//		else
//		{
//			stack_size_tester.st_SFlg=0;
//			stack_size_tester.st_counter=0;
//		}
//
//		if(control_buzz_onoff.st_CFlg==1)
//		{
//			control_buzz_onoff.st_counter++;
//			if(control_buzz_onoff.st_counter> 2)//NO_SERIAL_DATA_TIME = desired time in ms / 100
//			{
//				control_buzz_onoff.st_SFlg=1;
//				//control_buzz_onoff.st_CFlg=0;
//				control_buzz_onoff.st_counter=0;
//			}
//		}
//		else
//		{
//			control_buzz_onoff.st_SFlg=0;
//			control_buzz_onoff.st_counter=0;
//		}
//
//		if(pid_pedal_exit.st_CFlg==1)
//		{
//			pid_pedal_exit.st_counter++;
//			//if(pid_pedal_exit.st_counter> 10)//PEDAL PID EXIT COUNTER = desired time in ms / 100
//			if(pid_pedal_exit.st_counter > 5)////it was 8 earlier
//			{
//				pid_pedal_exit.st_SFlg=1;
//				//pid_pedal_exit.st_CFlg=0;
//				pid_pedal_exit.st_counter=0;
//			}
//		}
//		else
//		{
//			pid_pedal_exit.st_SFlg=0;
//			pid_pedal_exit.st_counter=0;
//		}
//
//		if(signal_disconnection.st_CFlg==1)
//		{
//			signal_disconnection.st_counter++;
//			//if(pid_pedal_exit.st_counter> 10)//PEDAL PID EXIT COUNTER = desired time in ms / 100
//			if(signal_disconnection.st_counter> 25)
//			{
//				signal_disconnection.st_SFlg=1;
//				//pid_pedal_exit.st_CFlg=0;
//				signal_disconnection.st_counter=0;
//			}
//		}
//		else
//		{
//			signal_disconnection.st_SFlg=0;
//			signal_disconnection.st_counter=0;
//		}
//
//		vTaskDelay(pdMS_TO_TICKS(100));
//	}
//
//}

void time_jobs( void *pvParameters )
{
	char testvar[25];
	a1=a2=0;
	counter_=0;
	for(;;)
	{
		dma_hand_held.st_counter++;
		//a5++;
		a1++;
		a2++;
		if(a1>250) a1=0;
		if(a2>250) a2=0;
		if(a1>248)
		{
			a1=0;
			log_message("time_jobs\r\n");
		}
		if(dma_hand_held.st_counter>5)
		{
			dma_hand_held.st_counter=0;
			dma_hand_held.st_SFlg=9;
			if(dma_hand_held.st_SFlg==9)
			{
				counter_++;
				previous_COUNT = current_COUNT;
				dma_hand_held.st_counter=0;
				dma_hand_held.st_SFlg=0;
				//handheld_dma_rx=9;
				current_COUNT = __HAL_DMA_GET_COUNTER(huart2.hdmarx);
				//HAL_UART_Transmit(&huart1, (uint8_t*)"\nData=", 6, HAL_MAX_DELAY);
				//HAL_UART_Transmit(&huart1, (uint8_t*)handheld_serial_data, strlen(handheld_serial_data), HAL_MAX_DELAY);
				if(current_COUNT!=previous_COUNT)
				{
					handheld_dma_rx=0;
					//HAL_UART_Transmit(&huart1, (uint8_t*)"\nData=", 6, HAL_MAX_DELAY);
					//HAL_UART_Transmit(&huart1, (uint8_t*)handheld_serial_data, strlen(handheld_serial_data), HAL_MAX_DELAY);
					current_COUNT = __HAL_DMA_GET_COUNTER(huart2.hdmarx);
					data_len = parse_received_dma_data(parse_dma_rx_data);
					sprintf(testvar,"\nDLEN=%u\n",data_len);
					//HAL_UART_Transmit(&debug_UART, testvar, strlen(testvar), HAL_MAX_DELAY);
					log_message(testvar);
					if(data_len>0)
					{
						sprintf(testvar,"\nParsed=%u,%u\n",counter_,uart2_rx_size);
						log_message(testvar);
						log_message(parse_dma_rx_data);
						//HAL_UART_Transmit(&debug_UART, testvar, strlen(testvar), HAL_MAX_DELAY);
						//HAL_UART_Transmit(&debug_UART, (uint8_t*)parse_dma_rx_data, strlen(parse_dma_rx_data), HAL_MAX_DELAY);
						sprintf(testvar,"\nrem=%u\n",current_COUNT);
						//HAL_UART_Transmit(&debug_UART, (uint8_t*)testvar, strlen(testvar), HAL_MAX_DELAY);
						log_message(testvar);
						handheld_dma_rx=9;

					}
					else
					{
						sprintf(testvar,"\nNO DATA\n");
						log_message(testvar);
						//HAL_UART_Transmit(&debug_UART, testvar, strlen(testvar), HAL_MAX_DELAY);
					}
				}
			}
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}
uint16_t spi_data=0;
void vHandTask( void *pvParameters )
{
	float frequency_per_speed_=0.0;
	char testarray[20];
	uint32_t ulNotifyValue;
	//uint32_t Timer_count=0;
	uint16_t stacktestflag=0;
	//HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
	vTaskDelay(pdMS_TO_TICKS(400));
	Is_First_Captured=0;

	for(;;)
	{
		/* Wait to receive a notification sent directly to this task from the interrupt handler. */
		//ulNotifyValue = ulTaskNotifyTake(pdTRUE, 0xFFFFFFFF);
		ulNotifyValue = ulTaskNotifyTake(pdTRUE,1000);//last tested one
		//ulNotifyValue = ulTaskNotifyTake(pdTRUE,2000);
		if(ulNotifyValue != 0)
		{
			//HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
			if (Is_First_Captured==0) // if the first value is not captured
			{
				__HAL_TIM_SET_COUNTER(&htim2, 0);  // reset the counter
				//IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
				Is_First_Captured = 1;  // set the first captured as true
			}
			else
			{
				Timer_count = (uint32_t)__HAL_TIM_GET_COUNTER(&htim2);  // get the counter
				//float refClock = TIMCLOCK/(PRESCALAR); //72M/7200
				//frequency = refClock/Difference;
				//10,000hz
				//1 tick = 0.00001 seconds
				//frequency = 10000.0/(float)Timer_count;
				if(Timer_count>0)
				{
					//frequency = (float)(8888.8888/(float)Timer_count);
					//frequency = 5000.0f/(float)Timer_count;////5000 means 10000/2
					//frequency = 6666.6666f/(float)Timer_count;////6666.6666 means 13333.3333/2
					//frequency = 32000.0f/(float)Timer_count;////6666.6666 means 13333.3333/2 for 65535 period
					//frequency = 31372.549f/(float)Timer_count;
					//last used
					//frequency = 62745.098f/(float)Timer_count;
					//frequency = 213333.333f/(float)Timer_count;
					frequency = 91428.58f/(float)Timer_count;





					//frequency = 5990.0f/(float)Timer_count;
					//frequency = 5990.0f/(float)Timer_count;

					/////////////////////////////////////
//					if((can_speedx>=SET_PPR_1)&&((can_speedx<=(SET_PPR_1+5))))
//					{
//						if(ppr_completion_[0]==0)
//						{
//							ppr_completion_[0]=1;
//							frequency_per_speed_ = frequency;
//							ppr_value_array_1[0] = can_speedx / frequency_per_speed_;
//						}
//					}
//					if((can_speedx>=SET_PPR_2)&&((can_speedx<=(SET_PPR_2+5))))
//					{
//						if(ppr_completion_[1]==0)
//						{
//							ppr_completion_[1]=1;
//							frequency_per_speed_ = frequency;
//							ppr_value_array_1[1] = can_speedx / frequency_per_speed_;
//						}
//					}
//					if((can_speedx>=SET_PPR_3)&&((can_speedx<=(SET_PPR_3+5))))
//					{
//						if(ppr_completion_[2]==0)
//						{
//							ppr_completion_[2]=1;
//							frequency_per_speed_ = frequency;
//							ppr_value_array_1[2] = can_speedx / frequency_per_speed_;
//						}
//					}
					//////////////////////////////////////////
				}
				//frequency = frequency/2;
				Is_First_Captured = 0;
				ch_ok=1;
			}
			/*Your task job here*/
			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
			//BSP_LED_Toggle(LED1);
			frequency_flag = 1;
		}
		else
		{
			frequency=0;
			Is_First_Captured = 0;
			frequency_flag = 9;
		}
		/*
		stacktestflag++;
		if(stacktestflag>100)
		{
			stacktestflag=0;

			UBaseType_t watermark = uxTaskGetStackHighWaterMark(NULL);
			sprintf(testarray,"FHAND: %lu\n", (unsigned long)watermark);
			HAL_UART_Transmit(&huart1,(uint8_t*)testarray,(uint16_t)(strlen((char*)testarray)), 1000);
			vTaskDelay(2/portTICK_PERIOD_MS);
		}
		vTaskDelay(1/portTICK_PERIOD_MS);
		*/

	}
}


void iHandTask(void *pvParameters )
{
	char send_speed_data[70];

	int ret_flag;
	uint32_t serial_array_size=0;
	ret_flag=0;
	double pedal_avg;
	double temp_var_pedal;
	int i=0;
	float speed_diff=0.0;
	//counter_send_return=0;
	//set_speed_rx_flg=0;
	int speed_integer_val=0;

	for(;;)
	{
		//sprintf(send_speed_data,"C=%lu\n",counter_send_return);
		//HAL_UART_Transmit(&huart2,(uint8_t*)send_speed_data,(uint16_t)(strlen((char*)send_speed_data)), 1000);
		//vTaskDelay(200/portTICK_PERIOD_MS);
		pedal_avg=0.0;
		temp_var_pedal=0.0;
		for(i=0;i<10;i++)
		{
			temp_var_pedal = get_pedal1(hadc2);//Get_pedal_volt(hadc1);
			vTaskDelay(pdMS_TO_TICKS(2));
			pedal_avg = pedal_avg + temp_var_pedal;
		}
		pedal_data_percentage = (pedal_avg/10);
		pedal_data_percentage = pedal_data_percentage;
		//pedal_data_percentage = pedal_data * 20;
		counter_send_return=counter_send_return + 1;
		//if((counter_send_return>SET_SPEED_TIME)||(set_speed_rx_flg==9))//(SET_SPEED_TIME=desired time in ms / 20
		//if((counter_send_return>SET_SPEED_TIME))//(SET_SPEED_TIME=desired time in ms / 20

//		if(rx_uart3_flag==1)
//		{
//			rx_uart3_flag=0;
//			sprintf(send_speed_data,"%u,",can_speedx);
//			HAL_UART_Transmit(&huart1,(uint8_t*)send_speed_data,(uint16_t)(strlen((char*)send_speed_data)), 1000);
//			vTaskDelay(2/portTICK_PERIOD_MS);
//		}

		if((counter_send_return>SET_SPEED_TIME))//if((counter_send_return > PRINT_DEBUG_TIME))
		{
			set_speed_rx_flg=0;
			counter_send_return=0;
			sprintf(send_speed_data,"%03X\n",set_speed);
			//HAL_UART_Transmit(&huart3,(uint8_t*)send_speed_data,(uint16_t)(strlen((char*)send_speed_data)), 1000);
			vTaskDelay(pdMS_TO_TICKS(1));

			//sprintf((char*)send_speed_data,"D=%3.2f--%3.2f--%3.3f--%3.2f--%u\n",frequency,speed_rng,ppr_value,output_pedal_value,set_speed);
			if(inside_pid_loop_flag==9)
			{
				//inside_pid_loop_flag=0;
				if(set_speed>=speed_rng)
				{
					speed_diff=set_speed-speed_rng;
				}
				else
				{
					speed_diff=speed_rng - set_speed;
				}
			//	sprintf((char*)send_speed_data,"C=%3.2f--%3.2f--%3.2f--%3.2f--%3.4f--%3.4f--%3.2f--%3.2f--%d\n",
				//		set_speed,speed_rng,error_value,program_setting_data[pgmr_pointer][3],program_setting_data[pgmr_pointer][4],
					//	program_setting_data[pgmr_pointer][5],pid_steady_val,output_pedal_value,pgmr_pointer);
				sprintf((char*)send_speed_data,"PID=%u--%3.2f--%3.2f--%3.2f--%3.2f--%3.2f--%lu--%d\n",
										set_speed,speed_rng,frequency,output_pedal_value,error_value,error_percentage,Timer_count,pgmr_pointer);
				log_message(send_speed_data);
				//HAL_UART_Transmit(&debug_UART,(uint8_t*)send_speed_data,(uint16_t)(strlen((char*)send_speed_data)), 1000);
			}
			else
			{
				//sprintf((char*)send_speed_data,"D=%3.2f--%3.2f--%3.2f--%f--%u--%d\n",
					//	program_setting_data[0][8],program_setting_data[1][8],program_setting_data[0][10],program_setting_data[1][10],set_speed,pgmr_pointer);
				sprintf((char*)send_speed_data,"NOR=%u-%3.2f-%3.2f-%3.2f-%3.2f-%d-%3.2f    ",
														set_speed,speed_rng,frequency,pedal_data_percentage,pedal_disconnection,pgmr_pointer,pid_result);
				//HAL_UART_Transmit(&huart2,(uint8_t*)send_speed_data,(uint16_t)(strlen((char*)send_speed_data)), 1000);
				log_message(send_speed_data);
				sprintf((char*)send_speed_data,"PID Ctrl Data=%3.3f-%3.3f-%3.3f-%3.3f-%3.3f\n",
										program_setting_data[pgmr_pointer][0],program_setting_data[pgmr_pointer][1],
														program_setting_data[pgmr_pointer][5],program_setting_data[pgmr_pointer][6],
														program_setting_data[pgmr_pointer][7]);
				log_message(send_speed_data);
				//HAL_UART_Transmit(&huart2,(uint8_t*)send_speed_data,(uint16_t)(strlen((char*)send_speed_data)), 1000);
			}
			vTaskDelay(pdMS_TO_TICKS(3));
			/*
			UBaseType_t watermark = uxTaskGetStackHighWaterMark(NULL);
			sprintf(send_speed_data,"IHAND: %lu\n", (unsigned long)watermark);
			HAL_UART_Transmit(&huart1,(uint8_t*)send_speed_data,(uint16_t)(strlen((char*)send_speed_data)), 1000);
			vTaskDelay(2/portTICK_PERIOD_MS);
			*/
		}
	}
}

uint8_t high_speed_notification=0;

void SYHandTask( void *pvParameters )
{
	int i=0;
	int value_enable_flag=0;
	int pid_loop_entry=0;

	int programer_entering_flag=9;
	int buzz_off_flag=0;
	int pedal_exit_flag=0;

	char test_arr[25];
	int ret_flag;
	int flag_condition=1;
	uint16_t fz_flag_counter=0;
	int fz_flag_limit=0;
	ret_flag=0;
	uint32_t serial_array_size=0;
	int return_function=0;
	serial_array_size=500;//sizeof(serial_data_uart2)/sizeof(serial_data_uart2[0]);

	store_config_flag=0;
	pid_loop_condition=3;
	pid_loop_entry=1;
	PRINT_DEBUG_TIME = 100;
	//HAL_GPIO_WritePin(VALVE_OUT_GPIO_Port, VALVE_OUT_Pin,GPIO_PIN_SET);
	pedal_disconnection=0;
	value_enable_flag=0;
	ivms_uart3_flag=0;

	current_COUNT=DMA_BUFFER_SIZE;
	previous_COUNT = current_COUNT;



	 vTaskDelay(pdMS_TO_TICKS(1000));

	 store_config_flag=1;

	pgmr_pointer=0;
	data_conversion(pgmr_pointer);
	//show_data();

	pgmr_pointer=1;
	data_conversion(pgmr_pointer);
	//show_data();
	//show_data();
	 pgmr_pointer=1;

	for(;;)
	{
		programmer_detection = HAL_GPIO_ReadPin(PGMR_CONE_GPIO_Port, PGMR_CONE_Pin);
		if(programmer_detection==0)
		{
			programer_entering_flag=0;
			HAL_NVIC_DisableIRQ(EXTI2_IRQn);
			//interuppt_disable_flag=9;
			//sprintf((char*)test_arr,"TEST LOOP\n");
			//HAL_UART_Transmit(&huart1,(uint8_t*)test_arr,(uint16_t)(strlen((char*)test_arr)), 1000);
			vTaskDelay(pdMS_TO_TICKS(50));
			__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
			//HAL_NVIC_EnableIRQ(USART2_IRQn);
			//HAL_NVIC_EnableIRQ(GPDMA1_Channel5_IRQn);
			vTaskDelay(pdMS_TO_TICKS(100));
			return_function = hand_held_programmer();
			//HAL_GPIO_WritePin(CTRL_INDICATION_GPIO_Port, CTRL_INDICATION_Pin, GPIO_PIN_RESET);
			if(return_function==9)
			{

				data_conversion(store_config_flag);
				//Flash_Write_Read_Test(store_config_flag,dummy_data_l);
			 	//HAL_Delay(150);
			 	//data_conversion(store_config_flag);
			}
			HAL_NVIC_EnableIRQ(EXTI2_IRQn);
			vTaskDelay(10/portTICK_PERIOD_MS);
			//set_speed = program_setting_data[1][8];
			//min_high_ref = generatePedal2(program_setting_data[pgmr_pointer][0],pedal_type);
			//max_high_ref = generatePedal2(program_setting_data[pgmr_pointer][1],pedal_type);
			//pgmr_pointer=1;
			pgmr_pointer=1;
		}
		else
		{
			if(programer_entering_flag==0)
			{
				__HAL_UART_DISABLE_IT(&huart2, UART_IT_IDLE);
				//HAL_NVIC_DisableIRQ(USART2_IRQn);
				//HAL_NVIC_DisableIRQ(GPDMA1_Channel5_IRQn);

				if(store_config_flag == 0)
				{
					store_config_flag=1;
				}
				else if(store_config_flag == 1)
				{
					store_config_flag=0;
				}
			}
			programer_entering_flag=9;

		}

		///////////////////////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////////////
		if(pedal_disconnection<=(program_setting_data[pgmr_pointer][0]+5))
		{
			PRINT_DEBUG_TIME=300;
		}
		else
		{
			PRINT_DEBUG_TIME=100;
		}
		/*
		if(pedal_disconnection>=(program_setting_data[pgmr_pointer][0]+5))
		{
			if(frequency==0)
			{
				signal_disconnection.st_CFlg=1;
				if(signal_disconnection.st_SFlg==1)
				{
					HAL_GPIO_WritePin(OUTPUT_BUZZER_GPIO_Port, OUTPUT_BUZZER_Pin,GPIO_PIN_SET);
				}
			}
			else if(signal_disconnection.st_CFlg==1)
			{
				signal_disconnection.st_CFlg=0;
				signal_disconnection.st_SFlg=0;
				HAL_GPIO_WritePin(OUTPUT_BUZZER_GPIO_Port, OUTPUT_BUZZER_Pin,GPIO_PIN_RESET);
			}
		}
		else if(signal_disconnection.st_CFlg==1)
		{
			signal_disconnection.st_CFlg=0;
			signal_disconnection.st_SFlg=0;
			HAL_GPIO_WritePin(OUTPUT_BUZZER_GPIO_Port, OUTPUT_BUZZER_Pin,GPIO_PIN_RESET);
		}
		*/


		/*
		 * frequency_flag will be 9 when there is not fz detected on the pin
		 * when the fz flag is 1 we made a limit to 15 means LED blinks every 300ms
		 * 300 = 15 * 20 ms - 15 counter limit and 20 task loop delay
		 * checking if greater than 60000 just for safer side that the VAR not to cross
		 * uint16 limit during any condition
		 */
		if(frequency_flag==9)
		{
			fz_flag_limit = 0;
		}
		else if(frequency_flag==1)
		{
			fz_flag_limit = 5;
		}

		fz_flag_counter=fz_flag_counter+1;
		if(fz_flag_counter>60000)
		{
			fz_flag_counter=0;
		}

		if(fz_flag_limit==0)
		{
			HAL_GPIO_WritePin(SGL_INDICATION_GPIO_Port, SGL_INDICATION_Pin, GPIO_PIN_RESET);
		}
		else if(fz_flag_counter>=fz_flag_limit)
		{
			if(flag_condition==1)
			{
				flag_condition=0;
				HAL_GPIO_WritePin(SGL_INDICATION_GPIO_Port, SGL_INDICATION_Pin, GPIO_PIN_SET);
				fz_flag_counter=0;
			}
			else if(flag_condition==0)
			{
				flag_condition=1;
				HAL_GPIO_WritePin(SGL_INDICATION_GPIO_Port, SGL_INDICATION_Pin, GPIO_PIN_RESET);
				fz_flag_counter=0;
			}
		}

		if(program_setting_data[0][10]==1)
		{
			if(set_speed>program_setting_data[0][8])
			{
				pgmr_pointer=1;
			}
			else
			{
				pgmr_pointer=0;
			}
		}
		else
		{
			pgmr_pointer=1;
		}

		/*
		 * uart2_read will read a single byte data from the device for set speed
		 * uart1_data[0] will have the set speed single data
		 */
//		ret_flag = uart2_read(uart1_data,10);
//		//if(ret_flag==1)
//		if((ret_flag==1)&&(uart1_data[0]>1))
//		{
//			//sprintf((char*)test_arr,"U1_data=%d\n",uart1_data[0]);
//			//HAL_UART_Transmit(&huart1,(uint8_t*)test_arr,(uint16_t)(strlen((char*)test_arr)), 1000);
//			if(uart1_data[0]<=200)
//			{
//				ser_speed = uart1_data[0];
//				set_speed_flag=1;
//				serial_speed_receive.st_CFlg=0;
//				serial_data_true=1;
//				serial_speed_receive.st_counter=0;
//				//set_speed_rx_flg=9;
//			}
//		}
		//ivms_uart3_flag=0;
		if(ivms_uart3_flag==1)
		{
			ivms_uart3_flag=0;
			//sprintf(send_speed_data,"%x",rx_byte_interupt);
			//HAL_UART_Transmit(&huart1,(uint8_t*)send_speed_data,(uint16_t)(strlen((char*)send_speed_data)), 1000);
			//vTaskDelay(2/portTICK_PERIOD_MS);
			//ret_flag = uart2_read(uart1_data,10);
			//if(ret_flag==1)
			//if((ret_flag==1)&&(uart1_data[0]>1))
			//{
			//sprintf((char*)test_arr,"U1_data=%d\n",uart1_data[0]);
			//HAL_UART_Transmit(&huart1,(uint8_t*)test_arr,(uint16_t)(strlen((char*)test_arr)), 1000);
			//if(uart1_data[0]<=200)
			//{
			if(can_speedx>5)
			{
				ser_speed = can_speedx;
				set_speed_flag=1;
				serial_speed_receive.st_CFlg=0;
				serial_data_true=1;
				serial_speed_receive.st_counter=0;
			}
				//set_speed_rx_flg=9;
			//}
		}
		else
		{
			serial_speed_receive.st_CFlg=1;
		}
		if(serial_speed_receive.st_SFlg==1)
		{
			set_speed_flag=0;
			serial_speed_receive.st_SFlg=0;
			//sprintf((char*)test_arr,"L0\n");
			//HAL_UART_Transmit(&huart1,(uint8_t*)test_arr,(uint16_t)(strlen((char*)test_arr)), 1000);
		}

		if(((set_speed_flag==0) && (cf==0) && (set_speed>=set_speed_eep)) ||((set_speed_flag==0) && (cf==0) && (set_speed<set_speed_eep )))
		{
			//current set speed >= default speed
			//current set speed < default speed
			set_speed=set_speed_eep;
			//sprintf((char*)test_arr,"L1=%u\n",set_speed_flag);
			//HAL_UART_Transmit(&huart1,(uint8_t*)test_arr,(uint16_t)(strlen((char*)test_arr)), 1000);

		}

		else if((set_speed_flag==0) && (cf==1) && (set_speed>=set_speed_eep))
		{

			//current set speed >= default speed
			//no problem- no delay required

			set_speed=set_speed_eep;
			//sprintf((char*)test_arr,"L2=%u\n",set_speed_flag);
			//HAL_UART_Transmit(&huart1,(uint8_t*)test_arr,(uint16_t)(strlen((char*)test_arr)), 1000);

		}
		else if(set_speed_flag==0 && cf==1 && set_speed<set_speed_eep)
		{
			//current set speed < default speed
			//here we should provde delay and beep
			spd_change=1;
			// cf=0;
			//set_speed=set_speed_eep;
		}
		else if(set_speed_flag==1 && cf==0 && set_speed>=ser_speed)
		{
			//current set speed >= default speed
			set_speed=ser_speed;
			high_speed_notification=0;
		}
		else if(set_speed_flag==1 && cf==0 && set_speed<ser_speed)
		{
			//current set speed < default speed
			set_speed=ser_speed;
			high_speed_notification=0;
		}
		else if(set_speed_flag==1 && cf==1&& set_speed>=ser_speed)
		{
			//current set speed >= default speed
			set_speed=ser_speed;
			high_speed_notification=0;
		}
		else if(set_speed_flag==1 && cf==1&& set_speed<ser_speed)
		{
			//current set speed < default speed
			//here we should provde delay and beep
			spd_change=1;
			high_speed_notification=9;
			//cf=0;
			//set_speed=ser_speed;
		}

		if(spd_change==1)
		{
			buzz=1;
		}
		//This is to control buzzer for indication
		//if(buzz)
		//if(buzz && set_speed_flag==0)
		if((buzz && set_speed_flag==0) || spd_change)
		{
			//HAL_GPIO_WritePin(OUTPUT_BUZZER_GPIO_Port, OUTPUT_BUZZER_Pin, GPIO_PIN_SET);
			rx_high_set_speed_flag=1;
		}
		else
		{
			//HAL_GPIO_WritePin(OUTPUT_BUZZER_GPIO_Port, OUTPUT_BUZZER_Pin, GPIO_PIN_RESET);
			rx_high_set_speed_flag=0;
		}

		//if((set_speed==10) && (if_spd_in==1))
		//if((set_speed==10) && (if_spd_in==1) && (if_spd_in_10==0))
		/*
		if((if_spd_in==1) && (if_spd_in_10==0))
		{
			eeprom_start_loc=EEPROM_SEC_START_LOC;//80;
			eeprom_end_loc=EEPROM_SEC_END_LOC;
			pgmr_pointer=1;
		//data_conversion();
		}
		else
		{
			eeprom_start_loc=_EEPROM_START_LOC;
			eeprom_end_loc=_EEPROM_END_LOC;
			pgmr_pointer=0;
		}
		*/

		for(int qui=0;qui<4;qui++)
		{
		speed_rng=getSpeed();


		if(speed_rng>=set_speed)
		{
			HAL_GPIO_WritePin(VALVE_OUT_GPIO_Port, VALVE_OUT_Pin,GPIO_PIN_RESET);
			if(pedal_disconnection<(program_setting_data[pgmr_pointer][0]+2))
			{
				HAL_GPIO_WritePin(CTRL_INDICATION_GPIO_Port, CTRL_INDICATION_Pin, GPIO_PIN_SET);
				control_buzz_onoff.st_CFlg=1;
				value_enable_flag=9;
			}
			else
			{
				value_enable_flag=0;
			}
		}
		else if(speed_rng<=(set_speed-0.5))
		{
			HAL_GPIO_WritePin(VALVE_OUT_GPIO_Port, VALVE_OUT_Pin,GPIO_PIN_SET);
			if(pedal_disconnection<(program_setting_data[pgmr_pointer][0]+2))
			{
				HAL_GPIO_WritePin(CTRL_INDICATION_GPIO_Port, CTRL_INDICATION_Pin, GPIO_PIN_RESET);
				control_buzz_onoff.st_CFlg=0;
				value_enable_flag=0;
			}
			else
			{
				value_enable_flag=0;
			}
		}

		if(set_speed<=50)
		{
			pid_loop_condition=1;
		}
		else
		{
			pid_loop_condition=1;
		}


		if(cf==0)
		{

			//pgmr_pointer=1;
			//clear integral term
			I_=0;
			//clear clamping flag
			I_Clamping=0;
			if(value_enable_flag!=9)
			{
				HAL_GPIO_WritePin(CTRL_INDICATION_GPIO_Port, CTRL_INDICATION_Pin, GPIO_PIN_RESET);
			}
			if(speed_rng>=((float)(set_speed-pid_loop_condition)))
			{
				if((speed_rng>=((float)(set_speed-pid_loop_condition)))|| (speed_exit_flag==1))
				{
					i++;
					//feedback_pedal(program_setting_data[pgmr_pointer][7]);//cruise mode  need to change to steady val
					//feedback_pedal(pedal_data_percentage);
					feedback_pedal(program_setting_data[pgmr_pointer][0]);
					output_pedal_value = program_setting_data[pgmr_pointer][0];

				}
			}
			else
			{
				buzz=0;
				//HAL_GPIO_WritePin(OUTPUT_BUZZER_GPIO_Port, OUTPUT_BUZZER_Pin, GPIO_PIN_RESET);
				speed_exit_flag=0;
			}
			if(i>=6  && spd_change!=1)
			{
				buzz=!buzz;
				i=0;
			}

			float ped=0;
			ped=pedal_data_percentage;//get_pedal1(hadc1);
			//pedal_co=ped;
			//if(speed_rng<(set_speed-3))
			if(speed_rng<((float)(set_speed-pid_loop_condition)) && speed_exit_flag==0)
			{
				feedback_pedal(ped);//normal mode
				output_pedal_value = ped;
				//HAL_GPIO_WritePin(CTRL_INDICATION_GPIO_Port, CTRL_INDICATION_Pin, GPIO_PIN_SET);

			}
			//compare current speed and set speed.
			if((((speed_rng>=(set_speed-pid_loop_condition)) && (speed_exit_flag==0)) || ((speed_rng<=set_speed+1) && (speed_exit_flag==1))) && (ped>=program_setting_data[pgmr_pointer][0]))
			{
				//loop_exiting_pedal_pos=set_speed_pedal*0.5;
				//switch to cruise mode
				cf=1;
				//printf("le:%f",loop_exiting_pedal_pos);
				//set_timer0(0);
				//timer0_count=0;
			}
		}
		else if(cf==1)
		{
				inside_pid_loop_flag=9;
				PRINT_DEBUG_TIME = 15;
				//PID controller section
				HAL_GPIO_WritePin(CTRL_INDICATION_GPIO_Port, CTRL_INDICATION_Pin, GPIO_PIN_SET);
				control_buzz_onoff.st_CFlg=1;
				speed_exit_flag=1;
				i++;
				if(i>=3 && spd_change!=1)
				{
					buzz=!buzz;
					i=0;
				}

				float pedd=0;

				pedd=pedal_data_percentage;//get_pedal1(hadc1);
				if(pedd >(program_setting_data[pgmr_pointer][0]+5))
				{
					pid_pedal_exit.st_CFlg=0;
					pedal_exit_flag=1;

				}
				else
				{
					pid_pedal_exit.st_CFlg=1;
				}
				//if(pedd >(program_setting_data[pgmr_pointer][0]+5))//2
				if((pid_pedal_exit.st_SFlg==0)&&(pedal_exit_flag==1))
				{
					//pid_pedal_exit.st_SFlg=0;
					if(ch_ok)
					{
						ch_ok=0;
						/*
						timer0_value=get_timer0();
						step_time=(float)(timer0_count*32640)+(timer0_value * 128);//128=.5*256
						time_dt=(float)(step_time/1000000.0);
						*/

						// pid_controller(float Kp,float Ki,float Kd,float set_speed,float current_speed,float pedal_voltage_at_rated_speed)
						//pid_return=pid_controller(program_setting_data[dp][3],program_setting_data[dp][4],program_setting_data[dp][5],set_speed,speed_rng,program_setting_data[dp][0]);

						//steady_value=(((program_setting_data[dp][7]-program_setting_data[dp][0])/110.0)*set_speed)+;
						//steady_value=(0.03636*set_speed)+15.6363;

						float pp=0;
						float temp=0;
						//steady_value=(((program_setting_data[dp][7]-program_setting_data[0][0])/(max_speed-min_speed))*set_speed)+(program_setting_data[0][0]-(((program_setting_data[0][7]-program_setting_data[0][0])/(max_speed-min_speed))));

						pp=program_setting_data[pgmr_pointer][7]-program_setting_data[pgmr_pointer][0];
						temp=pp/120.0;
						steady_value=temp*set_speed;

						//pp=program_setting_data[dp][7]-temp;
						pp=program_setting_data[pgmr_pointer][0]-temp;

						//steady_value+=pp;
						steady_value=steady_value+pp;
						pid_steady_val=steady_value;

//						pp=program_setting_data[pgmr_pointer][7];
//						temp=pp/120.0;
//						steady_value=temp*set_speed;
//						pid_steady_val=steady_value;

						//steady_value=(((program_setting_data[0][7]-program_setting_data[0][0])/(max_speed-min_speed))*(set_speed+1))+(program_setting_data[0][0]-(((program_setting_data[0][7]-program_setting_data[0][0])/(max_speed-min_speed))));

						//subj_speed
						//steady_value=(((program_setting_data[0][7]-program_setting_data[0][0])/(subj_speed-min_speed))*set_speed)+(program_setting_data[0][0]-(((program_setting_data[0][7]-program_setting_data[0][0])/(subj_speed-min_speed))));
						//float pp=0;
						pp=0;

						//pp=program_setting_data[0][3]*(set_speed/120) ;
						//pp=program_setting_data[0][3]*(((0.45 * set_speed) + (set_speed +45.4546))/100);

						//if(set_speed==10 && program_setting_data[0][10]==1 )
						//if(set_speed==10 && if_spd_in==1)

						//if(if_spd_in==1)
						//if((set_speed==10 || set_speed==20)&& if_spd_in==1)
						//if((set_speed==10 )&& (if_spd_in==1))
	//					if((set_speed==10 )&& (if_spd_in==1))
	//					{
	//						//pp=program_setting_data[dp][3];
	//						//program_setting_data[dp][4]=0;
	//						//program_setting_data[dp][5]=0;
	//						//steady_value=program_setting_data[dp][7];
	//
	//						//pp=program_setting_data[0][4]*1*10.0;//0.1;
	//						//steady_value=program_setting_data[0][5]*100.0 ;
	//						pgmr_pointer=1;
	//					}
	//					else
	//					{
	//						//pp=program_setting_data[dp][3];
	//						pgmr_pointer=0;
	//					}
						/*
						if(((set_speed>program_setting_data[0][8]) && (program_setting_data[0][10]==0 && program_setting_data[1][10]==1)) ||((set_speed>program_setting_data[1][8]) && (program_setting_data[0][10]==1 && program_setting_data[1][10]==0)))
						//if((set_speed>program_setting_data[0][8]) && ((program_setting_data[0][10]==0 && program_setting_data[1][10]==1) || (program_setting_data[0][10]==1 && program_setting_data[1][10]==0)))
						//if(((set_speed>program_setting_data[0][8]) && (program_setting_data[0][10]==0 && program_setting_data[1][10]==1)))
						{
							pgmr_pointer=1;
						}
						else
						{
							//pp=program_setting_data[dp][3];
							pgmr_pointer=0;
						}
						*/
						// pp=program_setting_data[0][3];

						//pp=pp*((0.45 * set_speed) + (set_speed +45.4546));

						//pid_return=pid_controller(pp,set_speed,speed_rng,steady_value);//this is the one

						//pid_return=pid_controller(pp,pp*0.01,pp*0.001,set_speed,speed_rng,steady_value);
						//pid_return=pid_controller(pp,pp*0.1,pp*0.01,set_speed,speed_rng,steady_value);
						//pid_return=pid_controller(pp,pp*0.05,pp*0.005,set_speed,speed_rng,steady_value);//for izusu
						//pid_return=pid_controller(pp,pp*0.05,pp*0.000,set_speed,speed_rng,steady_value);


						//the below line is actual line
						pid_return=pid_controller(program_setting_data[pgmr_pointer][3],program_setting_data[pgmr_pointer][4],program_setting_data[pgmr_pointer][5],set_speed,speed_rng,steady_value);

						//sprintf((char*)test_arr,"Stdy Val-%3.2f\n",steady_value);
						//HAL_UART_Transmit(&huart1,(uint8_t*)test_arr,(uint16_t)(strlen((char*)test_arr)), 1000);






						//pid_return=pid_controller(pp,program_setting_data[dp][4],program_setting_data[dp][5],set_speed,speed_rng,steady_value);
						//pid_return=pid_controller(program_setting_data[0][3],program_setting_data[0][4],program_setting_data[0][5],set_speed,speed_rng,program_setting_data[0][0]);

						//pid_return=pid_controller(pp,set_speed,speed_rng,program_setting_data[0][7]);
						// pid_return=pid_controller(pp,set_speed,speed_rng,10.26);
						// pid_return=pid_controller(program_setting_data[0][3],set_speed,speed_rng,steady_value);

						//pid_return=pid_controller(program_setting_data[0][3],program_setting_data[0][4],program_setting_data[0][5],set_speed,speed_rng,steady_value,time_dt);
						//pid_return=pid_controller(pp,program_setting_data[0][4],program_setting_data[0][5],set_speed,speed_rng,steady_value,time_dt);
						//pid_return=pid_controller(program_setting_data[0][3],program_setting_data[0][4],program_setting_data[0][5],set_speed,speed_rng,program_setting_data[0][7],time_dt);
						//printf("%2.2f\n",pid_return);
						//printf("s=%2.2f\n",speed_rng);
						//printf("p=%2.2f\n",pid_return);

						feedback_pedal(pid_return);//cruise mode  --  actual line below just for testing
						output_pedal_value = pid_return;
						//feedback_pedal(pedal_data_percentage);

					}
					if(high_speed_notification==9)
					{
						if(pedal_data_percentage<=(program_setting_data[pgmr_pointer][0]+1))
						{
							///////////////////////////////////////////////
							PRINT_DEBUG_TIME = 100;
							pedal_exit_flag=0;
							pid_pedal_exit.st_CFlg=0;
							pid_pedal_exit.st_SFlg=0;
							//printf("exit\r\n");
							///exit cruise mode.
							if(value_enable_flag!=9)
							{
								HAL_GPIO_WritePin(CTRL_INDICATION_GPIO_Port, CTRL_INDICATION_Pin, GPIO_PIN_RESET);
							}
							control_buzz_onoff.st_CFlg=0;
							cf=0;
							//clear integral term
							I_=0;
							//clear clamping flag
							I_Clamping=0;
							spd_change=0;
							pgmr_pointer=1;
							qui=9;
							pid_loop_entry=1;
							///////////////////////////////////////////////

						}
					}
				}
				else
				{
					inside_pid_loop_flag=0;
					PRINT_DEBUG_TIME = 100;
					pedal_exit_flag=0;
					pid_pedal_exit.st_CFlg=0;
					pid_pedal_exit.st_SFlg=0;
					//printf("exit\r\n");
					///exit cruise mode.
					HAL_GPIO_WritePin(CTRL_INDICATION_GPIO_Port, CTRL_INDICATION_Pin, GPIO_PIN_RESET);
					control_buzz_onoff.st_CFlg=0;
					cf=0;
					//clear integral term
					I_=0;
					//clear clamping flag
					I_Clamping=0;
					spd_change=0;
					pgmr_pointer=1;
					pid_loop_entry=1;
				}

			}
		vTaskDelay(pdMS_TO_TICKS(30));
		}//////for loop close bracket}

		if(control_buzz_onoff.st_CFlg==0)
		{
			HAL_GPIO_WritePin(OUTPUT_BUZZER_GPIO_Port, OUTPUT_BUZZER_Pin,GPIO_PIN_RESET);
		}
		else if(control_buzz_onoff.st_SFlg==1)
		{
			//if(set_speed_flag==0)
			if(serial_data_true==0)
			{
				if(rx_high_set_speed_flag==0)
				{
					HAL_GPIO_TogglePin(OUTPUT_BUZZER_GPIO_Port, OUTPUT_BUZZER_Pin);
					//sprintf((char*)test_arr,"Beep=H\n");
					//HAL_UART_Transmit(&huart1,(uint8_t*)test_arr,(uint16_t)(strlen((char*)test_arr)), 1000);
				}
				else if(rx_high_set_speed_flag==1)
				{
					HAL_GPIO_WritePin(OUTPUT_BUZZER_GPIO_Port, OUTPUT_BUZZER_Pin,GPIO_PIN_SET);
					//sprintf((char*)test_arr,"Beep=T\n");
					//HAL_UART_Transmit(&huart1,(uint8_t*)test_arr,(uint16_t)(strlen((char*)test_arr)), 1000);
				}
			}////below else was commented, uncommented due to false buzzer beep, done on 9th for testing
			else
			{
				HAL_GPIO_WritePin(OUTPUT_BUZZER_GPIO_Port, OUTPUT_BUZZER_Pin,GPIO_PIN_RESET);
			}
			control_buzz_onoff.st_SFlg=0;
		}
		if(high_speed_notification==9)
		{
			HAL_GPIO_WritePin(OUTPUT_BUZZER_GPIO_Port, OUTPUT_BUZZER_Pin,GPIO_PIN_SET);
			buzz_off_flag=9;
		}
		else
		{
			if(buzz_off_flag==9)
			{
				buzz_off_flag=0;
				HAL_GPIO_WritePin(OUTPUT_BUZZER_GPIO_Port, OUTPUT_BUZZER_Pin,GPIO_PIN_RESET);
			}

		}

		stack_size_tester.st_CFlg=1;
//		if(stack_size_tester.st_SFlg==1)
//		{
//			stack_size_tester.st_SFlg=0;
//			UBaseType_t watermark = uxTaskGetStackHighWaterMark(NULL);
//			//sprintf(test_arr,"SYS: %lu\n", (unsigned long)watermark);
//			//HAL_UART_Transmit(&huart1,(uint8_t*)test_arr,(uint16_t)(strlen((char*)test_arr)), 1000);
//			vTaskDelay(2/portTICK_PERIOD_MS);
//		}
	}
}

void HH_dma( void *pvParameters )
{
	char test_arr[200];
	char localBuf[200];
	int programmer_detection = 0;
	vTaskDelay(pdMS_TO_TICKS(500));

	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t*)handheld_serial_data, DMA_BUFFER_SIZE);


	for(;;)
	{
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, handheld_serial_data, DMA_BUFFER_SIZE);
//		 if(xQueueReceive(handheld_uart_queue, localBuf, portMAX_DELAY) == pdPASS)
//		{
//			 //dma_hand_held.st_CFlg=0;
//			 //dma_hand_held.st_SFlg=9;
//			 HAL_UART_Transmit(&huart1, (uint8_t*)localBuf, strlen(localBuf), HAL_MAX_DELAY);
//		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t*)handheld_serial_data, DMA_BUFFER_SIZE);
        if(xSemaphoreTake(handheld_data_ready_sem, pdMS_TO_TICKS(500)) == pdTRUE)
        {
        	vTaskDelay(pdMS_TO_TICKS(150));
        	//xQueueSendFromISR(handheld_uart_queue, &handheld_serial_data, &xHigherPriorityTaskWoken);
        	//HAL_UART_Transmit(&huart1, (uint8_t*)handheld_serial_data, strlen(handheld_serial_data), HAL_MAX_DELAY);
//            if(handheld_serial_data[0]=='@')
//            {
//            	handheld_serial_data[0]='&';
//            	handheld_dma_rx=9;
//            	HAL_UART_Transmit(&huart2, dummy_data_l, strlen(dummy_data_l), HAL_MAX_DELAY);
//            }
        	//xQueueSendFromISR(xChannels, &channels_SBUS, &xHigherPriorityTaskWoken);
            dma_hand_held.st_CFlg=0;
            dma_hand_held.st_SFlg=9;
            __HAL_UART_CLEAR_IDLEFLAG(&huart2);
            __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF);
            HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t*)handheld_serial_data, DMA_BUFFER_SIZE);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
	}
}

//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//	if(huart->Instance == USART2)
//	{
//		//HAL_UART_Transmit(&huart1,(uint8_t*)"G\n" , 2, HAL_MAX_DELAY);
//		uart2_rx_size = Size;
//		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//		xSemaphoreGiveFromISR(handheld_data_ready_sem, &xHigherPriorityTaskWoken);
//		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//	}
//}

//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//
//    if (huart->Instance == USART2)
//    {
//        // Send data to queue in chunks or one-shot
//        xQueueSendFromISR(handheld_uart_queue,handheld_serial_data, &xHigherPriorityTaskWoken);
//
//        // Restart DMA if not using pure circular mode
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, handheld_serial_data, DMA_BUFFER_SIZE);
//    }
//
//    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//}


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3)
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
