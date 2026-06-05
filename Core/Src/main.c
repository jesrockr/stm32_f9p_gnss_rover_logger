
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
#include "fatfs.h"
#include "i2c.h"
#include "sdio.h"
#include "ssd1306.h"
#include "usart.h"
#include "sd_diskio.h"
#include "ffconf.h"

#include "stdio.h"
#include "string.h"

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

////////* raw GNSS stream *///////////////

#define UBX_BUF_SIZE 65535
#define NMEA_FORWARD_ENABLED 1
#define BT_TEST_BEACON_ENABLED 0
#define NMEA_MAX_SENTENCE_LEN 128
#define DEFAULT_ROD_HEIGHT_TENTH_MM 20000LL
#define OLED_ROWS 8
#define OLED_CHARS_PER_ROW 21
#define KEY_DEBOUNCE_MS 50
#define KEY_ARM_DELAY_MS 1500
/*
 * Optional external switch for starting a new log file.
 *
 * The onboard FK407M2-ZGT6 KEY button did not show up reliably as a readable
 * GPIO in testing, so this feature is disabled by default. To use it, wire a
 * momentary switch between a confirmed-free GPIO header pin and GND, then:
 *   1. Set BUTTON_ROTATE_LOG_ENABLED to 1.
 *   2. Set BUTTON_ROTATE_LOG_PORT/PIN to that GPIO.
 *   3. Leave the pin configured with GPIO_PULLUP below.
 *
 * Pressing the switch will briefly show NEW LOG and open the next GNSSxxx.UBX.
 */
#define BUTTON_ROTATE_LOG_ENABLED 1
#define BUTTON_ROTATE_LOG_PORT GPIOE
#define BUTTON_ROTATE_LOG_PIN GPIO_PIN_5
/*
 * Portable base safety option.
 *
 * When enabled, STM32 sends UBX-CFG-RST to the F9P at boot to clear retained
 * navigation state and force a fresh GNSS start. This helps prevent a movable
 * base from silently reusing stale survey/base state after being moved.
 *
 * Wiring required:
 *   STM32 PA9  / USART1 TX  --->  F9P UART1 RX
 *   STM32 PA10 / USART1 RX  <---  F9P UART1 TX
 *
 * The command does not erase the F9P flash configuration, but it does clear
 * retained navigation/orbit/time state. Survey-in will take longer after boot.
 */
#define F9P_COLD_START_ON_BOOT 0
uint8_t ubx_buf[UBX_BUF_SIZE];
volatile uint32_t overflow_counter = 0;
volatile uint32_t uart_dma_wrap_count = 0;
uint32_t log_bytes_written = 0;
uint32_t old_pos = 0;
uint32_t log_start_tick = 0;
uint32_t nmea_sentences_forwarded = 0;
uint32_t nmea_forward_errors = 0;
uint8_t gnss_sat_count = 0;
uint8_t gnss_fix_type = 0;
uint8_t gnss_diff_solution = 0;
uint8_t gnss_carrier_solution = 0;
uint8_t gnss_time_valid = 0;
uint8_t gnss_pos_valid = 0;
uint8_t gnss_pos_uses_hp = 0;
uint8_t gnss_hp_seen = 0;
uint16_t gnss_year = 2015;
uint8_t gnss_month = 6;
uint8_t gnss_day = 4;
uint8_t gnss_hour = 0;
uint8_t gnss_min = 0;
uint8_t gnss_sec = 0;
uint32_t gnss_last_packet_tick = 0;
int64_t gnss_lon_ndeg = 0;
int64_t gnss_lat_ndeg = 0;
int64_t gnss_height_tenth_mm = 0;
int64_t gnss_hmsl_tenth_mm = 0;
uint32_t gnss_hacc_mm = 0;
uint32_t gnss_vacc_mm = 0;
uint8_t gnss_svin_seen = 0;
uint8_t gnss_svin_active = 0;
uint8_t gnss_svin_valid = 0;
uint32_t gnss_svin_duration = 0;
uint32_t gnss_svin_mean_acc = 0;
static char oled_boot_lines[OLED_ROWS][OLED_CHARS_PER_ROW + 1];
#if BUTTON_ROTATE_LOG_ENABLED
static GPIO_TypeDef *key_button_port = BUTTON_ROTATE_LOG_PORT;
static uint16_t key_button_pin = BUTTON_ROTATE_LOG_PIN;
static GPIO_PinState key_button_idle = GPIO_PIN_RESET;
#endif
static uint8_t point_active = 0;
static uint16_t point_index = 0;
static char point_name[5] = "";
static uint32_t point_sample_count = 0;
static int64_t point_sum_lon_ndeg = 0;
static int64_t point_sum_lat_ndeg = 0;
static int64_t point_sum_height_tenth_mm = 0;
static int64_t point_sum_hmsl_tenth_mm = 0;
static uint32_t point_sum_hacc_mm = 0;
static uint32_t point_sum_vacc_mm = 0;
static uint8_t point_best_fix_type = 0;
static uint8_t point_best_carrier_solution = 0;
static uint8_t point_max_sat_count = 0;
static uint16_t point_start_year = 2015;
static uint8_t point_start_month = 6;
static uint8_t point_start_day = 4;
static uint8_t point_start_hour = 0;
static uint8_t point_start_min = 0;
static uint8_t point_start_sec = 0;
static int64_t rod_height_tenth_mm = DEFAULT_ROD_HEIGHT_TENTH_MM;
static char rod_height_display[12] = "2M";
static char point_csv_name[13] = "POINTS.CSV";

/////////////*  *////////////////////

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t GNSS_DMA_GetWriteAbs(void)
{
  uint32_t wraps_a;
  uint32_t wraps_b;
  uint32_t pos;

  do
  {
    wraps_a = uart_dma_wrap_count;
    pos = UBX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    wraps_b = uart_dma_wrap_count;
  } while (wraps_a != wraps_b);

  if (pos >= UBX_BUF_SIZE)
  {
    pos = 0;
  }

  return (wraps_a * UBX_BUF_SIZE) + pos;
}

static void NMEA_ForwardByte(uint8_t byte)
{
#if NMEA_FORWARD_ENABLED
  static uint8_t active = 0;
  static uint8_t sentence[NMEA_MAX_SENTENCE_LEN];
  static uint16_t len = 0;

  if (byte == '$')
  {
    active = 1;
    len = 0;
  }

  if (!active)
  {
    return;
  }

  if (len >= sizeof(sentence))
  {
    active = 0;
    len = 0;
    return;
  }

  sentence[len++] = byte;

  if (byte == '\n')
  {
    if (HAL_UART_Transmit(&huart3, sentence, len, 50) == HAL_OK)
    {
      nmea_sentences_forwarded++;
    }
    else
    {
      nmea_forward_errors++;
    }
    active = 0;
    len = 0;
  }
#else
  (void)byte;
#endif
}

static HAL_StatusTypeDef F9P_ColdStart(void)
{
#if F9P_COLD_START_ON_BOOT
  static const uint8_t cfg_rst_cold_start[] =
  {
      0xB5, 0x62,             /* UBX sync */
      0x06, 0x04,             /* CFG-RST */
      0x04, 0x00,             /* payload length */
      0xFF, 0xFF,             /* clear all nav BBR data */
      0x02,                   /* controlled GNSS-only reset */
      0x00,                   /* reserved */
      0x0E, 0x61              /* checksum */
  };

  return HAL_UART_Transmit(&huart1,
                           (uint8_t *)cfg_rst_cold_start,
                           sizeof(cfg_rst_cold_start),
                           100);
#else
  return HAL_OK;
#endif
}

static FRESULT GNSS_OpenNextLogFile(FIL *file, char *name, UINT name_size)
{
  FILINFO info;
  FRESULT res;

  for (uint16_t index = 1; index <= 999; index++)
  {
    snprintf(name, name_size, "GNSS%03u.UBX", index);

    res = f_stat(name, &info);
    if (res == FR_NO_FILE)
    {
      return f_open(file, name, FA_CREATE_NEW | FA_WRITE);
    }

    if (res != FR_OK)
    {
      return res;
    }
  }

  return FR_DENIED;
}

static FRESULT GNSS_OpenNamedLogFile(FIL *file, const char *name)
{
  return f_open(file, name, FA_CREATE_NEW | FA_WRITE);
}

static FRESULT POINT_OpenNextFile(FIL *file, char *name, UINT name_size)
{
  FILINFO info;
  FRESULT res;

  for (uint16_t index = 1; index <= 999; index++)
  {
    snprintf(name, name_size, "P%03u.UBX", index);

    res = f_stat(name, &info);
    if (res == FR_NO_FILE)
    {
      point_index = index;
      snprintf(point_name, sizeof(point_name), "P%03u", index);
      return GNSS_OpenNamedLogFile(file, name);
    }

    if (res != FR_OK)
    {
      return res;
    }
  }

  return FR_DENIED;
}

static FRESULT POINT_CreateSessionCsv(char *name, UINT name_size)
{
  FIL csv;
  FILINFO info;
  FRESULT res;
  UINT bw;
  static const char header[] =
    "point,ubx_file,start_utc,end_utc,samples,lat_deg,lon_deg,antenna_height_m,antenna_hmsl_m,rod_height_m,point_height_m,point_hmsl_m,fix,carrier,sats,hacc_m,vacc_m\r\n";

  for (uint16_t index = 1; index <= 999; index++)
  {
    snprintf(name, name_size, "POINT%03u.CSV", index);

    res = f_stat(name, &info);
    if (res == FR_NO_FILE)
    {
      res = f_open(&csv, name, FA_CREATE_NEW | FA_WRITE);
      if (res != FR_OK)
      {
        return res;
      }

      res = f_write(&csv, header, strlen(header), &bw);
      if ((res != FR_OK) || (bw != strlen(header)))
      {
        f_close(&csv);
        return (res == FR_OK) ? FR_DISK_ERR : res;
      }

      res = f_sync(&csv);
      f_close(&csv);
      return res;
    }

    if (res != FR_OK)
    {
      return res;
    }
  }

  return FR_DENIED;
}

static void POINT_ResetAverage(void)
{
  point_sample_count = 0;
  point_sum_lon_ndeg = 0;
  point_sum_lat_ndeg = 0;
  point_sum_height_tenth_mm = 0;
  point_sum_hmsl_tenth_mm = 0;
  point_sum_hacc_mm = 0;
  point_sum_vacc_mm = 0;
  point_best_fix_type = 0;
  point_best_carrier_solution = 0;
  point_max_sat_count = 0;
  point_start_year = gnss_year;
  point_start_month = gnss_month;
  point_start_day = gnss_day;
  point_start_hour = gnss_hour;
  point_start_min = gnss_min;
  point_start_sec = gnss_sec;
}

static void POINT_AddCurrentSample(void)
{
  if (!point_active || !gnss_pos_valid)
  {
    return;
  }

  point_sum_lon_ndeg += gnss_lon_ndeg;
  point_sum_lat_ndeg += gnss_lat_ndeg;
  point_sum_height_tenth_mm += gnss_height_tenth_mm;
  point_sum_hmsl_tenth_mm += gnss_hmsl_tenth_mm;
  point_sum_hacc_mm += gnss_hacc_mm;
  point_sum_vacc_mm += gnss_vacc_mm;
  point_sample_count++;

  if (gnss_fix_type > point_best_fix_type)
  {
    point_best_fix_type = gnss_fix_type;
  }

  if (gnss_carrier_solution > point_best_carrier_solution)
  {
    point_best_carrier_solution = gnss_carrier_solution;
  }

  if (gnss_sat_count > point_max_sat_count)
  {
    point_max_sat_count = gnss_sat_count;
  }
}

static void FormatSignedScaled(char *out, size_t out_size, int64_t value, uint32_t scale, uint8_t decimals)
{
  char frac[12];
  long whole;
  uint32_t frac_value;
  int width = (decimals > 9U) ? 9 : decimals;

  if (value < 0)
  {
    value = -value;
    whole = (long)(value / scale);
    frac_value = (uint32_t)(value % scale);
    snprintf(frac, sizeof(frac), "%0*lu", width, (unsigned long)frac_value);
    snprintf(out, out_size, "-%ld.%s", whole, frac);
  }
  else
  {
    whole = (long)(value / scale);
    frac_value = (uint32_t)(value % scale);
    snprintf(frac, sizeof(frac), "%0*lu", width, (unsigned long)frac_value);
    snprintf(out, out_size, "%ld.%s", whole, frac);
  }
}

static void FormatRodHeightDisplay(void)
{
  long whole = (long)(rod_height_tenth_mm / 10000LL);
  long frac = (long)((rod_height_tenth_mm % 10000LL) / 100LL);

  if (frac == 0)
  {
    snprintf(rod_height_display, sizeof(rod_height_display), "%ldM", whole);
  }
  else
  {
    snprintf(rod_height_display, sizeof(rod_height_display), "%ld.%02ldM", whole, frac);
  }
}

static uint8_t ParseRodHeightMeters(const char *text, int64_t *out_tenth_mm)
{
  int64_t whole = 0;
  int64_t frac = 0;
  uint8_t frac_digits = 0;
  uint8_t saw_digit = 0;

  while ((*text == ' ') || (*text == '\t') || (*text == '\r') || (*text == '\n'))
  {
    text++;
  }

  while ((*text >= '0') && (*text <= '9'))
  {
    saw_digit = 1;
    whole = (whole * 10) + (*text - '0');
    text++;
  }

  if (*text == '.')
  {
    text++;
    while ((*text >= '0') && (*text <= '9') && (frac_digits < 4))
    {
      frac = (frac * 10) + (*text - '0');
      frac_digits++;
      text++;
    }
  }

  if (!saw_digit)
  {
    return 0;
  }

  while (frac_digits < 4)
  {
    frac *= 10;
    frac_digits++;
  }

  *out_tenth_mm = (whole * 10000LL) + frac;

  return (*out_tenth_mm >= 0) && (*out_tenth_mm <= 100000LL);
}

static uint8_t LoadRodHeightFromSd(void)
{
  FIL rod_file;
  FRESULT res;
  UINT br = 0;
  char text[32];
  int64_t parsed_height = DEFAULT_ROD_HEIGHT_TENTH_MM;

  rod_height_tenth_mm = DEFAULT_ROD_HEIGHT_TENTH_MM;

  res = f_open(&rod_file, "ROD.TXT", FA_READ);
  if (res != FR_OK)
  {
    FormatRodHeightDisplay();
    return 0;
  }

  memset(text, 0, sizeof(text));
  res = f_read(&rod_file, text, sizeof(text) - 1, &br);
  f_close(&rod_file);

  if ((res == FR_OK) && (br > 0) && ParseRodHeightMeters(text, &parsed_height))
  {
    rod_height_tenth_mm = parsed_height;
    FormatRodHeightDisplay();
    return 1;
  }

  FormatRodHeightDisplay();
  return 0;
}

static void OLED_ShowRodHeightReminder(void)
{
  char msg[32];

  SSD1306_Clear();
  SSD1306_SetCursor(0, 0);
  SSD1306_WriteString("CHECK ROD HEIGHT");
  SSD1306_SetCursor(0, 2);
  snprintf(msg, sizeof(msg), "= %s", rod_height_display);
  SSD1306_WriteString(msg);
  SSD1306_UpdateScreen();
}

static FRESULT POINT_AppendCsv(const char *ubx_name)
{
  FIL csv;
  FRESULT res;
  UINT bw;
  char line[256];
  char lat[24];
  char lon[24];
  char antenna_height[20];
  char antenna_hmsl[20];
  char rod_height[20];
  char point_height[20];
  char point_hmsl[20];
  int64_t avg_height_tenth_mm;
  int64_t avg_hmsl_tenth_mm;

  if (point_sample_count == 0)
  {
    return FR_INVALID_OBJECT;
  }

  res = f_open(&csv, point_csv_name, FA_OPEN_ALWAYS | FA_WRITE);
  if (res != FR_OK)
  {
    return res;
  }

  res = f_lseek(&csv, f_size(&csv));
  if (res != FR_OK)
  {
    f_close(&csv);
    return res;
  }

  FormatSignedScaled(lat, sizeof(lat), point_sum_lat_ndeg / (int64_t)point_sample_count, 1000000000UL, 9);
  FormatSignedScaled(lon, sizeof(lon), point_sum_lon_ndeg / (int64_t)point_sample_count, 1000000000UL, 9);
  avg_height_tenth_mm = point_sum_height_tenth_mm / (int64_t)point_sample_count;
  avg_hmsl_tenth_mm = point_sum_hmsl_tenth_mm / (int64_t)point_sample_count;
  FormatSignedScaled(antenna_height, sizeof(antenna_height), avg_height_tenth_mm, 10000UL, 4);
  FormatSignedScaled(antenna_hmsl, sizeof(antenna_hmsl), avg_hmsl_tenth_mm, 10000UL, 4);
  FormatSignedScaled(rod_height, sizeof(rod_height), rod_height_tenth_mm, 10000UL, 4);
  FormatSignedScaled(point_height, sizeof(point_height), avg_height_tenth_mm - rod_height_tenth_mm, 10000UL, 4);
  FormatSignedScaled(point_hmsl, sizeof(point_hmsl), avg_hmsl_tenth_mm - rod_height_tenth_mm, 10000UL, 4);

  snprintf(line, sizeof(line),
           "%s,%s,%04u-%02u-%02u %02u:%02u:%02u,%04u-%02u-%02u %02u:%02u:%02u,%lu,%s,%s,%s,%s,%s,%s,%s,%u,%u,%u,%lu.%03lu,%lu.%03lu\r\n",
           point_name,
           ubx_name,
           point_start_year, point_start_month, point_start_day,
           point_start_hour, point_start_min, point_start_sec,
           gnss_year, gnss_month, gnss_day,
           gnss_hour, gnss_min, gnss_sec,
           point_sample_count,
           lat,
           lon,
           antenna_height,
           antenna_hmsl,
           rod_height,
           point_height,
           point_hmsl,
           point_best_fix_type,
           point_best_carrier_solution,
           point_max_sat_count,
           (unsigned long)((point_sum_hacc_mm / point_sample_count) / 1000UL),
           (unsigned long)((point_sum_hacc_mm / point_sample_count) % 1000UL),
           (unsigned long)((point_sum_vacc_mm / point_sample_count) / 1000UL),
           (unsigned long)((point_sum_vacc_mm / point_sample_count) % 1000UL));

  res = f_write(&csv, line, strlen(line), &bw);
  if ((res != FR_OK) || (bw != strlen(line)))
  {
    f_close(&csv);
    return (res == FR_OK) ? FR_DISK_ERR : res;
  }

  res = f_sync(&csv);
  f_close(&csv);
  return res;
}

#if BUTTON_ROTATE_LOG_ENABLED
static FRESULT POINT_Start(FIL *file, char *log_name, UINT name_size)
{
  FRESULT res;

  res = f_sync(file);
  if (res != FR_OK)
  {
    return res;
  }

  res = f_close(file);
  if (res != FR_OK)
  {
    return res;
  }

  res = POINT_OpenNextFile(file, log_name, name_size);
  if (res != FR_OK)
  {
    return res;
  }

  POINT_ResetAverage();
  point_active = 1;
  log_bytes_written = 0;
  log_start_tick = HAL_GetTick();
  return FR_OK;
}

static FRESULT POINT_StopAndStore(FIL *file, char *log_name, UINT name_size)
{
  FRESULT res;
  char point_ubx[13];

  snprintf(point_ubx, sizeof(point_ubx), "%s", log_name);

  res = f_sync(file);
  if (res != FR_OK)
  {
    return res;
  }

  res = POINT_AppendCsv(point_ubx);
  if (res != FR_OK)
  {
    return res;
  }

  res = f_close(file);
  if (res != FR_OK)
  {
    return res;
  }

  point_active = 0;
  point_name[0] = '\0';

  res = GNSS_OpenNextLogFile(file, log_name, name_size);
  if (res != FR_OK)
  {
    return res;
  }

  log_bytes_written = 0;
  log_start_tick = HAL_GetTick();
  return FR_OK;
}

static uint8_t KEY_NewLogRequested(void)
{
  static GPIO_PinState last_raw = GPIO_PIN_RESET;
  static GPIO_PinState stable = GPIO_PIN_RESET;
  static uint32_t changed_tick = 0;
  static uint32_t init_tick = 0;
  static uint8_t initialized = 0;
  static uint8_t idle_learned = 0;
  static uint8_t armed = 0;
  GPIO_PinState raw = HAL_GPIO_ReadPin(key_button_port, key_button_pin);

  if (!initialized)
  {
    last_raw = raw;
    stable = raw;
    changed_tick = HAL_GetTick();
    init_tick = changed_tick;
    initialized = 1;
    return 0;
  }

  if ((HAL_GetTick() - init_tick) < KEY_ARM_DELAY_MS)
  {
    return 0;
  }

  if (raw != last_raw)
  {
    last_raw = raw;
    changed_tick = HAL_GetTick();
  }

  if ((HAL_GetTick() - changed_tick) >= KEY_DEBOUNCE_MS)
  {
    stable = raw;
  }

  if (!idle_learned)
  {
    key_button_idle = stable;
    idle_learned = 1;
    armed = 1;
    return 0;
  }

  if ((stable != key_button_idle) && armed)
  {
    armed = 0;
    return 1;
  }

  if (stable == key_button_idle)
  {
    armed = 1;
  }

  return 0;
}
#endif

static void GNSS_ParseByte(uint8_t byte)
{
  static uint8_t state = 0;
  static uint8_t msg_class = 0;
  static uint8_t msg_id = 0;
  static uint16_t payload_len = 0;
  static uint16_t payload_index = 0;
  static uint8_t ck_a = 0;
  static uint8_t ck_b = 0;
  static uint8_t nav_pvt_valid = 0;
  static uint16_t nav_pvt_year = 2015;
  static uint8_t nav_pvt_month = 6;
  static uint8_t nav_pvt_day = 4;
  static uint8_t nav_pvt_hour = 0;
  static uint8_t nav_pvt_min = 0;
  static uint8_t nav_pvt_sec = 0;
  static uint8_t nav_pvt_fix_type = 0;
  static uint8_t nav_pvt_diff_solution = 0;
  static uint8_t nav_pvt_carrier_solution = 0;
  static uint8_t nav_pvt_num_sv = 0;
  static uint32_t nav_pvt_lon_raw = 0;
  static uint32_t nav_pvt_lat_raw = 0;
  static uint32_t nav_pvt_height_raw = 0;
  static uint32_t nav_pvt_hmsl_raw = 0;
  static uint32_t nav_pvt_hacc = 0;
  static uint32_t nav_pvt_vacc = 0;
  static uint8_t nav_svin_active = 0;
  static uint8_t nav_svin_valid = 0;
  static uint32_t nav_svin_duration = 0;
  static uint32_t nav_svin_mean_acc = 0;
  static uint32_t nav_hppos_lon_raw = 0;
  static uint32_t nav_hppos_lat_raw = 0;
  static uint32_t nav_hppos_height_raw = 0;
  static uint32_t nav_hppos_hmsl_raw = 0;
  static int8_t nav_hppos_lon_hp = 0;
  static int8_t nav_hppos_lat_hp = 0;
  static int8_t nav_hppos_height_hp = 0;
  static int8_t nav_hppos_hmsl_hp = 0;
  static uint32_t nav_hppos_hacc = 0;
  static uint32_t nav_hppos_vacc = 0;

  switch (state)
  {
    case 0:
      state = (byte == 0xB5) ? 1 : 0;
      break;

    case 1:
      state = (byte == 0x62) ? 2 : 0;
      break;

    case 2:
      msg_class = byte;
      ck_a = byte;
      ck_b = ck_a;
      state = 3;
      break;

    case 3:
      msg_id = byte;
      ck_a = (ck_a + byte) & 0xFF;
      ck_b = (ck_b + ck_a) & 0xFF;
      state = 4;
      break;

    case 4:
      payload_len = byte;
      ck_a = (ck_a + byte) & 0xFF;
      ck_b = (ck_b + ck_a) & 0xFF;
      state = 5;
      break;

    case 5:
      payload_len |= ((uint16_t)byte << 8);
      payload_index = 0;
      nav_pvt_valid = 0;
      nav_pvt_year = 2015;
      nav_pvt_month = 6;
      nav_pvt_day = 4;
      nav_pvt_hour = 0;
      nav_pvt_min = 0;
      nav_pvt_sec = 0;
      nav_pvt_fix_type = 0;
      nav_pvt_diff_solution = 0;
      nav_pvt_carrier_solution = 0;
      nav_pvt_num_sv = 0;
      nav_pvt_lon_raw = 0;
      nav_pvt_lat_raw = 0;
      nav_pvt_height_raw = 0;
      nav_pvt_hmsl_raw = 0;
      nav_pvt_hacc = 0;
      nav_pvt_vacc = 0;
      nav_svin_active = 0;
      nav_svin_valid = 0;
      nav_svin_duration = 0;
      nav_svin_mean_acc = 0;
      nav_hppos_lon_raw = 0;
      nav_hppos_lat_raw = 0;
      nav_hppos_height_raw = 0;
      nav_hppos_hmsl_raw = 0;
      nav_hppos_lon_hp = 0;
      nav_hppos_lat_hp = 0;
      nav_hppos_height_hp = 0;
      nav_hppos_hmsl_hp = 0;
      nav_hppos_hacc = 0;
      nav_hppos_vacc = 0;
      ck_a = (ck_a + byte) & 0xFF;
      ck_b = (ck_b + ck_a) & 0xFF;
      state = (payload_len == 0) ? 7 : 6;
      break;

    case 6:
      if ((msg_class == 0x01) && (msg_id == 0x07) && (payload_index == 4))
      {
        nav_pvt_year = byte;
      }

      if ((msg_class == 0x01) && (msg_id == 0x07) && (payload_index == 5))
      {
        nav_pvt_year |= ((uint16_t)byte << 8);
      }

      if ((msg_class == 0x01) && (msg_id == 0x07) && (payload_index == 6))
      {
        nav_pvt_month = byte;
      }

      if ((msg_class == 0x01) && (msg_id == 0x07) && (payload_index == 7))
      {
        nav_pvt_day = byte;
      }

      if ((msg_class == 0x01) && (msg_id == 0x07) && (payload_index == 8))
      {
        nav_pvt_hour = byte;
      }

      if ((msg_class == 0x01) && (msg_id == 0x07) && (payload_index == 9))
      {
        nav_pvt_min = byte;
      }

      if ((msg_class == 0x01) && (msg_id == 0x07) && (payload_index == 10))
      {
        nav_pvt_sec = byte;
      }

      if ((msg_class == 0x01) && (msg_id == 0x07) && (payload_index == 11))
      {
        nav_pvt_valid = byte;
      }

      if ((msg_class == 0x01) && (msg_id == 0x07) && (payload_index == 20))
      {
        nav_pvt_fix_type = byte;
      }

      if ((msg_class == 0x01) && (msg_id == 0x07) && (payload_index == 21))
      {
        nav_pvt_diff_solution = (byte & 0x02) ? 1 : 0;
        nav_pvt_carrier_solution = (byte >> 6) & 0x03;
      }

      if ((msg_class == 0x01) && (msg_id == 0x07) && (payload_index == 23))
      {
        nav_pvt_num_sv = byte;
      }

      if ((msg_class == 0x01) && (msg_id == 0x07) &&
          (payload_index >= 24) && (payload_index <= 27))
      {
        nav_pvt_lon_raw |= ((uint32_t)byte << (8U * (payload_index - 24)));
      }

      if ((msg_class == 0x01) && (msg_id == 0x07) &&
          (payload_index >= 28) && (payload_index <= 31))
      {
        nav_pvt_lat_raw |= ((uint32_t)byte << (8U * (payload_index - 28)));
      }

      if ((msg_class == 0x01) && (msg_id == 0x07) &&
          (payload_index >= 32) && (payload_index <= 35))
      {
        nav_pvt_height_raw |= ((uint32_t)byte << (8U * (payload_index - 32)));
      }

      if ((msg_class == 0x01) && (msg_id == 0x07) &&
          (payload_index >= 36) && (payload_index <= 39))
      {
        nav_pvt_hmsl_raw |= ((uint32_t)byte << (8U * (payload_index - 36)));
      }

      if ((msg_class == 0x01) && (msg_id == 0x07) &&
          (payload_index >= 40) && (payload_index <= 43))
      {
        nav_pvt_hacc |= ((uint32_t)byte << (8U * (payload_index - 40)));
      }

      if ((msg_class == 0x01) && (msg_id == 0x07) &&
          (payload_index >= 44) && (payload_index <= 47))
      {
        nav_pvt_vacc |= ((uint32_t)byte << (8U * (payload_index - 44)));
      }

      if ((msg_class == 0x01) && (msg_id == 0x3B) &&
          (payload_index >= 8) && (payload_index <= 11))
      {
        nav_svin_duration |= ((uint32_t)byte << (8U * (payload_index - 8)));
      }

      if ((msg_class == 0x01) && (msg_id == 0x3B) &&
          (payload_index >= 28) && (payload_index <= 31))
      {
        nav_svin_mean_acc |= ((uint32_t)byte << (8U * (payload_index - 28)));
      }

      if ((msg_class == 0x01) && (msg_id == 0x3B) && (payload_index == 36))
      {
        nav_svin_valid = byte ? 1 : 0;
      }

      if ((msg_class == 0x01) && (msg_id == 0x3B) && (payload_index == 37))
      {
        nav_svin_active = byte ? 1 : 0;
      }

      if ((msg_class == 0x01) && (msg_id == 0x14) &&
          (payload_index >= 8) && (payload_index <= 11))
      {
        nav_hppos_lon_raw |= ((uint32_t)byte << (8U * (payload_index - 8)));
      }

      if ((msg_class == 0x01) && (msg_id == 0x14) &&
          (payload_index >= 12) && (payload_index <= 15))
      {
        nav_hppos_lat_raw |= ((uint32_t)byte << (8U * (payload_index - 12)));
      }

      if ((msg_class == 0x01) && (msg_id == 0x14) &&
          (payload_index >= 16) && (payload_index <= 19))
      {
        nav_hppos_height_raw |= ((uint32_t)byte << (8U * (payload_index - 16)));
      }

      if ((msg_class == 0x01) && (msg_id == 0x14) &&
          (payload_index >= 20) && (payload_index <= 23))
      {
        nav_hppos_hmsl_raw |= ((uint32_t)byte << (8U * (payload_index - 20)));
      }

      if ((msg_class == 0x01) && (msg_id == 0x14) && (payload_index == 24))
      {
        nav_hppos_lon_hp = (int8_t)byte;
      }

      if ((msg_class == 0x01) && (msg_id == 0x14) && (payload_index == 25))
      {
        nav_hppos_lat_hp = (int8_t)byte;
      }

      if ((msg_class == 0x01) && (msg_id == 0x14) && (payload_index == 26))
      {
        nav_hppos_height_hp = (int8_t)byte;
      }

      if ((msg_class == 0x01) && (msg_id == 0x14) && (payload_index == 27))
      {
        nav_hppos_hmsl_hp = (int8_t)byte;
      }

      if ((msg_class == 0x01) && (msg_id == 0x14) &&
          (payload_index >= 28) && (payload_index <= 31))
      {
        nav_hppos_hacc |= ((uint32_t)byte << (8U * (payload_index - 28)));
      }

      if ((msg_class == 0x01) && (msg_id == 0x14) &&
          (payload_index >= 32) && (payload_index <= 35))
      {
        nav_hppos_vacc |= ((uint32_t)byte << (8U * (payload_index - 32)));
      }

      ck_a = (ck_a + byte) & 0xFF;
      ck_b = (ck_b + ck_a) & 0xFF;
      payload_index++;

      if (payload_index >= payload_len)
      {
        state = 7;
      }
      break;

    case 7:
      state = (byte == ck_a) ? 8 : 0;
      break;

    case 8:
      if (byte == ck_b)
      {
        if ((msg_class == 0x01) && (msg_id == 0x07) && (payload_len >= 48))
        {
          gnss_time_valid = ((nav_pvt_valid & 0x03) == 0x03) ? 1 : 0;
          gnss_year = nav_pvt_year;
          gnss_month = nav_pvt_month;
          gnss_day = nav_pvt_day;
          gnss_hour = nav_pvt_hour;
          gnss_min = nav_pvt_min;
          gnss_sec = nav_pvt_sec;
          gnss_fix_type = nav_pvt_fix_type;
          gnss_diff_solution = nav_pvt_diff_solution;
          gnss_carrier_solution = nav_pvt_carrier_solution;
          gnss_sat_count = nav_pvt_num_sv;
          gnss_lon_ndeg = ((int64_t)(int32_t)nav_pvt_lon_raw) * 100LL;
          gnss_lat_ndeg = ((int64_t)(int32_t)nav_pvt_lat_raw) * 100LL;
          gnss_height_tenth_mm = ((int64_t)(int32_t)nav_pvt_height_raw) * 10LL;
          gnss_hmsl_tenth_mm = ((int64_t)(int32_t)nav_pvt_hmsl_raw) * 10LL;
          gnss_hacc_mm = nav_pvt_hacc;
          gnss_vacc_mm = nav_pvt_vacc;
          gnss_pos_valid = (nav_pvt_fix_type >= 2) ? 1 : 0;
          gnss_pos_uses_hp = 0;
          gnss_last_packet_tick = HAL_GetTick();
          if (!gnss_hp_seen)
          {
            POINT_AddCurrentSample();
          }
        }

        if ((msg_class == 0x01) && (msg_id == 0x3B) && (payload_len >= 38))
        {
          gnss_svin_seen = 1;
          gnss_svin_active = nav_svin_active;
          gnss_svin_valid = nav_svin_valid;
          gnss_svin_duration = nav_svin_duration;
          gnss_svin_mean_acc = nav_svin_mean_acc;
        }

        if ((msg_class == 0x01) && (msg_id == 0x14) && (payload_len >= 36))
        {
          gnss_hp_seen = 1;
          gnss_lon_ndeg = (((int64_t)(int32_t)nav_hppos_lon_raw) * 100LL) + nav_hppos_lon_hp;
          gnss_lat_ndeg = (((int64_t)(int32_t)nav_hppos_lat_raw) * 100LL) + nav_hppos_lat_hp;
          gnss_height_tenth_mm = (((int64_t)(int32_t)nav_hppos_height_raw) * 10LL) + nav_hppos_height_hp;
          gnss_hmsl_tenth_mm = (((int64_t)(int32_t)nav_hppos_hmsl_raw) * 10LL) + nav_hppos_hmsl_hp;
          gnss_hacc_mm = nav_hppos_hacc / 10UL;
          gnss_vacc_mm = nav_hppos_vacc / 10UL;
          gnss_pos_valid = 1;
          gnss_pos_uses_hp = 1;
          POINT_AddCurrentSample();
        }
      }
      state = 0;
      break;

    default:
      state = 0;
      break;
  }
}

static void GNSS_ParseBytes(const uint8_t *data, uint32_t len)
{
  for (uint32_t i = 0; i < len; i++)
  {
    NMEA_ForwardByte(data[i]);
    GNSS_ParseByte(data[i]);
  }
}

static const char *GNSS_FixLabel(void)
{
  if (gnss_diff_solution)
  {
    return "DGPS";
  }

  switch (gnss_fix_type)
  {
    case 2:
      return "2D";
    case 3:
      return "3D";
    case 4:
      return "GNSS+DR";
    case 5:
      return "TIME";
    default:
      return "NOFIX";
  }
}

static uint8_t GNSS_FixIsPpkOk(void)
{
  return (gnss_diff_solution || (gnss_fix_type == 3) || (gnss_fix_type == 5));
}

static const char *GNSS_SvinLabel(void)
{
  if (!gnss_svin_seen)
  {
    if (gnss_fix_type == 5)
    {
      return "SURVEYING IN";
    }

    return "";
  }

  if (gnss_svin_valid)
  {
    return "SURVEY IN OK";
  }

  if (gnss_svin_active)
  {
    return "SURVEYING IN";
  }

  return "SURVEYING IN";
}

DWORD get_fattime(void)
{
  uint16_t year = gnss_year;
  uint8_t month = gnss_month;
  uint8_t day = gnss_day;
  uint8_t hour = gnss_hour;
  uint8_t min = gnss_min;
  uint8_t sec = gnss_sec;

  if (!gnss_time_valid || (year < 1980) || (year > 2107) ||
      (month < 1) || (month > 12) || (day < 1) || (day > 31))
  {
    year = 2015;
    month = 6;
    day = 4;
    hour = 0;
    min = 0;
    sec = 0;
  }

  return ((DWORD)(year - 1980) << 25) |
         ((DWORD)month << 21) |
         ((DWORD)day << 16) |
         ((DWORD)hour << 11) |
         ((DWORD)min << 5) |
         ((DWORD)(sec / 2));
}

static void OLED_BootSetLine(uint8_t row, const char *text)
{
  if (row >= OLED_ROWS)
  {
    return;
  }

  snprintf(oled_boot_lines[row], sizeof(oled_boot_lines[row]), "%s", text);

  SSD1306_Clear();
  for (uint8_t i = 0; i < OLED_ROWS; i++)
  {
    if (oled_boot_lines[i][0] != '\0')
    {
      SSD1306_SetCursor(0, i);
      SSD1306_WriteString(oled_boot_lines[i]);
    }
  }
  SSD1306_UpdateScreen();
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

  MX_GPIO_Init();
  MX_I2C1_Init();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  char msg[32];

  HAL_Delay(100);
  SSD1306_Init();

  OLED_BootSetLine(0, "BOOT");
  HAL_Delay(500);

  OLED_BootSetLine(1, "UART CFG");
  HAL_Delay(500);

  MX_USART3_UART_Init();

  if (!F9P_COLD_START_ON_BOOT)
  {
      OLED_BootSetLine(1, "UARTS OK");
  }
  else if (F9P_ColdStart() == HAL_OK)
  {
      OLED_BootSetLine(1, "UART OK F9P RST");
  }
  else
  {
      OLED_BootSetLine(1, "F9P RST FAIL");
  }
  HAL_Delay(500);

  /* INIT SD + FATFS (DO NOT CALL HAL_SD OR BSP_SD HERE) */
  MX_SDIO_SD_Init();

  OLED_BootSetLine(2, "SDIO CFG");
  HAL_Delay(200);

  MX_FATFS_Init();

  OLED_BootSetLine(3, "FATFS OK");
  HAL_Delay(500);

  /* -------- MOUNT -------- */

  FRESULT res = f_mount(&SDFatFS, SDPath, 1);
  DSTATUS ds = disk_status(0);

  if ((res == FR_OK) && (ds == 0))
  {
      sprintf(msg, "MNT=OK SD=OK");
  }
  else
  {
      sprintf(msg, "MNT=%d DS=%u", res, ds);
  }
  OLED_BootSetLine(4, msg);
  HAL_Delay(1000);

  if (res != FR_OK)
  {
      SSD1306_Clear();
      sprintf(msg, "MFAIL D=%u", ds);
      SSD1306_WriteString(msg);
      SSD1306_UpdateScreen();
      while (1);
  }

  LoadRodHeightFromSd();

  res = POINT_CreateSessionCsv(point_csv_name, sizeof(point_csv_name));
  if (res == FR_OK)
  {
      OLED_BootSetLine(5, point_csv_name);
  }
  else
  {
      sprintf(msg, "CSV=%d", res);
      OLED_BootSetLine(5, msg);
  }
  HAL_Delay(1000);

  if (res != FR_OK)
  {
      SSD1306_Clear();
      sprintf(msg, "CSV FAIL=%d", res);
      SSD1306_WriteString(msg);
      SSD1306_UpdateScreen();
      while (1);
  }

  /* -------- FREE SPACE -------- */

  DWORD free_clusters;
  FATFS *fs;

  res = f_getfree(SDPath, &free_clusters, &fs);

  SSD1306_Clear();
  if (res == FR_OK)
  {
      DWORD free_sectors = free_clusters * fs->csize;

      uint32_t free_mb = free_sectors / 2048UL;

      sprintf(msg, "FREE=%luMB", free_mb);
  }
  else
  {
      sprintf(msg, "FREE FAIL");
  }

  OLED_BootSetLine(5, msg);
  HAL_Delay(1500);

  /* -------- OPEN FILE -------- */

  FIL log_file;
  UINT bw;
  char log_name[13];

  res = GNSS_OpenNextLogFile(&log_file, log_name, sizeof(log_name));

  if (res == FR_OK)
  {
      sprintf(msg, "OPEN=OK");
  }
  else
  {
      sprintf(msg, "OPEN=%d", res);
  }
  OLED_BootSetLine(6, msg);
  HAL_Delay(1000);

  if (res != FR_OK)
  {
      SSD1306_Clear();
      SSD1306_WriteString("OPEN FAIL");
      SSD1306_UpdateScreen();
      while (1);
  }

  OLED_BootSetLine(7, log_name);
  HAL_Delay(1000);

  res = f_sync(&log_file);

  if (res == FR_OK)
  {
      sprintf(msg, "OPEN=OK SYNC=OK");
  }
  else
  {
      sprintf(msg, "OPEN=OK SYNC=%d", res);
  }
  OLED_BootSetLine(6, msg);
  HAL_Delay(1000);

  if (res != FR_OK)
  {
      SSD1306_Clear();
      SSD1306_WriteString("SYNC FAIL");
      SSD1306_UpdateScreen();
      f_close(&log_file);
      while (1);
  }

  uart_dma_wrap_count = 0;
  overflow_counter = 0;
  log_bytes_written = 0;
  old_pos = 0;

  if (HAL_UART_Receive_DMA(&huart1, ubx_buf, UBX_BUF_SIZE) != HAL_OK)
  {
      SSD1306_Clear();
      SSD1306_WriteString("UART DMA FAIL");
      SSD1306_UpdateScreen();
      f_close(&log_file);
      while (1);
  }
  log_start_tick = HAL_GetTick();
  OLED_BootSetLine(1, "UART RX OK");
  HAL_Delay(500);

  /* -------- DONE -------- */

  sprintf(msg, "%s READY", log_name);
  OLED_BootSetLine(7, msg);
  HAL_Delay(1000);

  OLED_ShowRodHeightReminder();
  HAL_Delay(3000);

  /* USER CODE END 2 */


  ///////////////////////* Infinite loop *//////////////////////////

  old_pos = 0;

  while (1)
  {
      uint32_t write_abs = GNSS_DMA_GetWriteAbs();
      uint32_t available = write_abs - old_pos;

      if (available > UBX_BUF_SIZE)
      {
          overflow_counter++;
          old_pos = write_abs;
          SSD1306_Clear();
          sprintf(msg, "OVR=%lu", overflow_counter);
          SSD1306_WriteString(msg);
          SSD1306_UpdateScreen();
          continue;
      }

      while (available > 0)
      {
          uint32_t read_pos = old_pos % UBX_BUF_SIZE;
          uint32_t chunk = UBX_BUF_SIZE - read_pos;

          if (chunk > available)
          {
              chunk = available;
          }

          GNSS_ParseBytes(&ubx_buf[read_pos], chunk);
          res = f_write(&log_file, &ubx_buf[read_pos], chunk, &bw);

          if ((res != FR_OK) || (bw != chunk))
          {
              SSD1306_Clear();
              sprintf(msg, "LOG ERR=%d", res);
              SSD1306_WriteString(msg);
              SSD1306_UpdateScreen();
              f_close(&log_file);
              while (1);
          }

          old_pos += bw;
          log_bytes_written += bw;
          write_abs = GNSS_DMA_GetWriteAbs();
          available = write_abs - old_pos;

          if (available > UBX_BUF_SIZE)
          {
              overflow_counter++;
              old_pos = write_abs;
              break;
          }
      }

#if BUTTON_ROTATE_LOG_ENABLED
      if (KEY_NewLogRequested())
      {
          SSD1306_Clear();
          SSD1306_SetCursor(0, 0);
          SSD1306_WriteString(point_active ? "STORE POINT" : "START POINT");
          SSD1306_UpdateScreen();

          if (point_active)
          {
              res = POINT_StopAndStore(&log_file, log_name, sizeof(log_name));
          }
          else
          {
              res = POINT_Start(&log_file, log_name, sizeof(log_name));
          }

          if (res != FR_OK)
          {
              SSD1306_Clear();
              sprintf(msg, "POINT ERR=%d", res);
              SSD1306_WriteString(msg);
              SSD1306_UpdateScreen();
              while (1);
          }

          HAL_Delay(700);

          SSD1306_Clear();
          SSD1306_SetCursor(0, 0);
          SSD1306_WriteString(log_name);
          SSD1306_UpdateScreen();
          HAL_Delay(500);
      }
#endif

      static uint32_t last_sync = 0;
      if (HAL_GetTick() - last_sync > 5000)
      {
          res = f_sync(&log_file);
          last_sync = HAL_GetTick();

          if (res != FR_OK)
          {
              SSD1306_Clear();
              sprintf(msg, "SYNC ERR=%d", res);
              SSD1306_WriteString(msg);
              SSD1306_UpdateScreen();
              f_close(&log_file);
              while (1);
          }
      }

#if BT_TEST_BEACON_ENABLED
      static uint32_t last_bt_test = 0;
      if (HAL_GetTick() - last_bt_test > 1000)
      {
          static const uint8_t bt_test[] = "STM32 BT TEST\r\n";
          HAL_UART_Transmit(&huart3, (uint8_t *)bt_test, sizeof(bt_test) - 1, 50);
          last_bt_test = HAL_GetTick();
      }
#endif

      static uint32_t last_oled = 0;
      if (HAL_GetTick() - last_oled > 1000)
      {
          static uint8_t spinner_index = 0;
          const char spinner[] = ".-*+";

          SSD1306_Clear();
          SSD1306_SetCursor(0, 0);
          sprintf(msg, "SD WRITE %luKB %s",
                  log_bytes_written / 1024UL,
                  (overflow_counter == 0) ? "OK" : "BAD");
          SSD1306_WriteString(msg);
          if ((overflow_counter > 0) && ((spinner_index & 0x01) == 0))
          {
              SSD1306_SetCursor(0, 1);
              SSD1306_WriteString("WARNING OVERRUN");
          }
          else if (((gnss_last_packet_tick == 0) ||
                    ((HAL_GetTick() - gnss_last_packet_tick) > 3000)) &&
                   ((spinner_index & 0x01) == 0))
          {
              SSD1306_SetCursor(0, 1);
              SSD1306_WriteString("NO GNSS DATA");
          }
          SSD1306_SetCursor(0, 2);
          SSD1306_WriteString(log_name);
          SSD1306_SetCursor(0, 4);
          uint32_t elapsed = (HAL_GetTick() - log_start_tick) / 1000UL;
          if (point_active)
          {
              snprintf(msg, sizeof(msg), "%s %lu %02lu:%02lu",
                       point_name,
                       point_sample_count,
                       elapsed / 60UL,
                       elapsed % 60UL);
          }
          else
          {
              sprintf(msg, "LOGGING %c%c %02lu:%02lu",
                      spinner[spinner_index],
                      spinner[spinner_index],
                      elapsed / 60UL,
                      elapsed % 60UL);
          }
          SSD1306_WriteString(msg);
          SSD1306_SetCursor(0, 6);
          if (GNSS_SvinLabel()[0] != '\0' && gnss_svin_valid)
          {
              snprintf(msg, sizeof(msg), "SAT=%u %s",
                       gnss_sat_count, GNSS_SvinLabel());
              SSD1306_WriteString(msg);
          }
          else if (GNSS_SvinLabel()[0] == '\0')
          {
              snprintf(msg, sizeof(msg), "SAT=%u", gnss_sat_count);
              SSD1306_WriteString(msg);
          }
          else if ((spinner_index & 0x01) == 0)
          {
              snprintf(msg, sizeof(msg), "SAT=%u %s",
                       gnss_sat_count, GNSS_SvinLabel());
              SSD1306_WriteString(msg);
          }
          else
          {
              snprintf(msg, sizeof(msg), "SAT=%u", gnss_sat_count);
              SSD1306_WriteString(msg);
          }
          SSD1306_SetCursor(0, 7);
          if (GNSS_FixIsPpkOk() || ((spinner_index & 0x01) == 0))
          {
              SSD1306_WriteString(GNSS_FixLabel());
          }
          if (gnss_time_valid)
          {
              sprintf(msg, "UTC=%02u:%02u:%02u", gnss_hour, gnss_min, gnss_sec);
          }
          else
          {
              sprintf(msg, "UTC=--:--:--");
          }
          SSD1306_SetCursor(SSD1306_WIDTH - (strlen(msg) * 6), 7);
          SSD1306_WriteString(msg);
          SSD1306_UpdateScreen();
          spinner_index = (spinner_index + 1) % 4;
          last_oled = HAL_GetTick();
      }
  }
}
    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */


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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */


/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */


/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
#if BUTTON_ROTATE_LOG_ENABLED
  GPIO_InitTypeDef GPIO_InitStruct = {0};
#endif

  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */

#if BUTTON_ROTATE_LOG_ENABLED
  GPIO_InitStruct.Pin = BUTTON_ROTATE_LOG_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_ROTATE_LOG_PORT, &GPIO_InitStruct);
#endif
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uart_dma_wrap_count++;
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
