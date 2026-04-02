/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : H743 4-channel TOF USB bridge bring-up firmware
  ******************************************************************************
  */

#include "main.h"
#include "led.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TOF_CHANNEL_COUNT            4U
#define TOF_FRAME_LENGTH             4U
#define TOF_INVALID_DISTANCE_MM      65535U
#define TOF_INTERBYTE_TIMEOUT_MS     10U
#define TOF_FRAME_TIMEOUT_MS         120U
#define CONTROL_INPUT_TIMEOUT_MS     200U
#define CONTROL_LINE_BUFFER_SIZE     128U
#define TOF_USB_REPORT_PERIOD_MS     30U
#define APP_LED_TOGGLE_PERIOD_MS     250U
#define APP_ESTOP_RELEASED           0U
#define APP_ESTOP_ASSERTED           1U
#define APP_CONFIG_SAVE_DELAY_MS     1000U
#define APP_RELEASE_OVERRIDE_DEFAULT_MS 3000U
#define APP_RELEASE_OVERRIDE_MAX_MS  600000U
#define TOF_SAFE_BASELINE_DEFAULT_MM 391U
#define TOF_SAFE_TOLERANCE_DEFAULT_MM 28U
#define TOF_CONFIG_MAX_MM            (TOF_INVALID_DISTANCE_MM - 1U)
#define TOF_MASK_1                   0x01U
#define TOF_MASK_2                   0x02U
#define TOF_MASK_3                   0x04U
#define TOF_MASK_4                   0x08U
#define TOF_MASK_ALL                 (TOF_MASK_1 | TOF_MASK_2 | TOF_MASK_3 | TOF_MASK_4)
#define CONTROL_EPSILON              0.001f

#define ESTOP_OUT_GPIO_Port          GPIOE
#define ESTOP_OUT_Pin                GPIO_PIN_10

#define APP_CONFIG_MAGIC             0x48474643UL
#define APP_CONFIG_VERSION           0x00010000UL
#define APP_CONFIG_STORAGE_BASE      0x081E0000UL
#define APP_CONFIG_STORAGE_SIZE      FLASH_SECTOR_SIZE
#define APP_CONFIG_STORAGE_END       (APP_CONFIG_STORAGE_BASE + APP_CONFIG_STORAGE_SIZE)
#define APP_CONFIG_STORAGE_BANK      FLASH_BANK_2
#define APP_CONFIG_STORAGE_SECTOR    FLASH_SECTOR_7
#define APP_CONFIG_SLOT_WORD_COUNT   8U
#define APP_CONFIG_SLOT_SIZE         (APP_CONFIG_SLOT_WORD_COUNT * sizeof(uint32_t))

typedef struct
{
  UART_HandleTypeDef *uart;
  uint8_t rx_byte;
  uint8_t last_byte;
  uint8_t frame[TOF_FRAME_LENGTH];
  uint8_t frame_index;
  uint16_t distance_mm;
  uint32_t last_byte_tick;
  uint32_t last_frame_tick;
  uint32_t rx_count;
  uint32_t frame_count;
  uint16_t sync_errors;
  uint16_t checksum_errors;
  uint16_t uart_errors;
  uint8_t has_valid_frame;
  uint8_t consecutive_failures;
} TofChannelContext;

typedef enum
{
  APP_MOTION_IDLE = 0,
  APP_MOTION_FORWARD = 1,
  APP_MOTION_REVERSE = 2,
  APP_MOTION_TURN = 3,
  APP_MOTION_STRAFE = 4,
  APP_MOTION_MIXED = 5,
  APP_MOTION_LINK_LOSS = 6
} AppMotionClass;

typedef struct
{
  uint8_t motion_class;
  uint8_t active_mask;
  uint8_t valid_mask;
  uint8_t fault_mask;
  uint8_t trip_mask;
  uint8_t self_estop;
  uint8_t external_estop;
  uint8_t takeover_enabled;
  uint16_t baseline_mm;
  uint16_t tolerance_mm;
  uint16_t threshold_mm;
  uint32_t release_remaining_ms;
} AppSafetyState;

typedef struct
{
  uint8_t has_motion_class;
  AppMotionClass motion_class;
  uint8_t has_takeover_enable;
  uint8_t takeover_enable;
  uint8_t release_request;
  uint8_t has_release_hold_ms;
  uint32_t release_hold_ms;
  uint8_t has_baseline_mm;
  uint16_t baseline_mm;
  uint8_t has_tolerance_mm;
  uint16_t tolerance_mm;
} AppControlUpdate;

typedef struct
{
  uint32_t magic;
  uint32_t version;
  uint32_t sequence;
  uint32_t baseline_mm;
  uint32_t tolerance_mm;
  uint32_t reserved0;
  uint32_t reserved1;
  uint32_t checksum;
} AppPersistRecord;

_Static_assert(sizeof(AppPersistRecord) == APP_CONFIG_SLOT_SIZE, "Persist record must fit exactly one flash word");

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

static TofChannelContext g_tof_channels[TOF_CHANNEL_COUNT] =
{
  { .uart = &huart1, .distance_mm = TOF_INVALID_DISTANCE_MM },
  { .uart = &huart6, .distance_mm = TOF_INVALID_DISTANCE_MM },
  { .uart = &huart3, .distance_mm = TOF_INVALID_DISTANCE_MM },
  { .uart = &huart2, .distance_mm = TOF_INVALID_DISTANCE_MM },
};

static uint32_t g_usb_sequence = 0U;
static uint32_t g_last_report_tick = 0U;
static uint32_t g_last_led_tick = 0U;
static uint8_t g_estop_state = APP_ESTOP_RELEASED;
static uint32_t g_release_override_until_tick = 0U;
static uint32_t g_last_control_tick = 0U;
static uint32_t g_last_config_change_tick = 0U;
static uint8_t g_has_control_input = 0U;
static AppMotionClass g_motion_class = APP_MOTION_LINK_LOSS;
static AppSafetyState g_safety_state = {0};
static uint8_t g_takeover_enabled = 0U;
static uint16_t g_baseline_mm = TOF_SAFE_BASELINE_DEFAULT_MM;
static uint16_t g_tolerance_mm = TOF_SAFE_TOLERANCE_DEFAULT_MM;
static char g_control_line[CONTROL_LINE_BUFFER_SIZE];
static uint32_t g_control_line_length = 0U;
static uint32_t g_persist_sequence = 0U;
static uint8_t g_config_save_pending = 0U;
static uint8_t g_config_loaded_from_flash = 0U;

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void APP_Init(void);
static void APP_Poll(void);
static void APP_ReportFrame(uint32_t now);
static void APP_StartChannelReceive(TofChannelContext *channel);
static void APP_ProcessChannelByte(TofChannelContext *channel, uint8_t byte, uint32_t now);
static uint8_t APP_ChannelValid(const TofChannelContext *channel, uint32_t now);
static uint16_t APP_ChannelDistance(const TofChannelContext *channel, uint32_t now);
static uint8_t APP_ChannelTripExceeded(const TofChannelContext *channel, uint32_t now);
static uint8_t APP_CalcChecksum(const char *payload);
static uint16_t APP_GetTripThresholdMm(void);
static uint16_t APP_NormalizeMmValue(float value);
static uint32_t APP_NormalizeReleaseHoldMs(unsigned long value);
static uint8_t APP_TickDeadlineActive(uint32_t deadline, uint32_t now);
static uint32_t APP_GetReleaseRemainingMs(uint32_t now);
static void APP_SetEstop(uint8_t asserted);
static TofChannelContext *APP_FindChannel(UART_HandleTypeDef *huart);
static void APP_UART_CommonInit(UART_HandleTypeDef *huart, USART_TypeDef *instance);
static void APP_EvaluateSafety(uint32_t now, AppSafetyState *state);
static AppMotionClass APP_GetEffectiveMotionClass(uint32_t now);
static uint8_t APP_GetActiveMask(AppMotionClass motion_class);
static void APP_ParseControlLine(char *line, uint32_t now);
static uint8_t APP_ParseControlH7Ctl(const char *line, AppControlUpdate *update);
static uint8_t APP_ParseControlJson(const char *line, AppControlUpdate *update);
static uint8_t APP_ParseControlText(const char *line, AppControlUpdate *update);
static AppMotionClass APP_DeriveMotionClass(float vx, float vy, float wz);
static uint8_t APP_TryReadJsonString(const char *json, const char *key, char *out, uint32_t out_size);
static uint8_t APP_TryReadJsonFloat(const char *json, const char *key, float *value);
static uint8_t APP_ParseFloatTriplet(const char *line, float *vx, float *vy, float *wz);
static int32_t APP_Stricmp(const char *left, const char *right);
static char *APP_Trim(char *text);
static char *APP_UnframeControlLine(char *line);
static const char *APP_SkipWhitespace(const char *text);
static uint32_t APP_PersistChecksum(const AppPersistRecord *record);
static uint8_t APP_PersistRecordIsBlank(const AppPersistRecord *record);
static uint8_t APP_PersistRecordIsValid(const AppPersistRecord *record);
static void APP_LoadPersistedConfig(void);
static uint8_t APP_SavePersistedConfig(void);
static void APP_RequestConfigSave(uint32_t now);
static void APP_InvalidateDCache(uint32_t address, uint32_t size);

int main(void)
{
  SCB_EnableICache();
  SCB_EnableDCache();

  HAL_Init();
  SystemClock_Config();

  LED_Init();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_USB_DEVICE_Init();
  APP_Init();

  while (1)
  {
    APP_Poll();
    HAL_Delay(1);
  }
}

static void APP_Init(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t index;

  APP_SetEstop(APP_ESTOP_RELEASED);
  LED1_OFF;

  g_last_report_tick = now;
  g_last_led_tick = now;
  g_release_override_until_tick = 0U;
  g_last_control_tick = now;
  g_has_control_input = 0U;
  g_motion_class = APP_MOTION_LINK_LOSS;
  g_takeover_enabled = 0U;
  g_baseline_mm = TOF_SAFE_BASELINE_DEFAULT_MM;
  g_tolerance_mm = TOF_SAFE_TOLERANCE_DEFAULT_MM;
  g_control_line_length = 0U;
  g_last_config_change_tick = now;
  g_persist_sequence = 0U;
  g_config_save_pending = 0U;
  g_config_loaded_from_flash = 0U;

  APP_LoadPersistedConfig();

  for (index = 0U; index < TOF_CHANNEL_COUNT; ++index)
  {
    g_tof_channels[index].frame_index = 0U;
    g_tof_channels[index].distance_mm = TOF_INVALID_DISTANCE_MM;
    g_tof_channels[index].last_byte_tick = now;
    g_tof_channels[index].last_frame_tick = 0U;
    g_tof_channels[index].last_byte = 0U;
    g_tof_channels[index].rx_count = 0U;
    g_tof_channels[index].frame_count = 0U;
    g_tof_channels[index].sync_errors = 0U;
    g_tof_channels[index].checksum_errors = 0U;
    g_tof_channels[index].uart_errors = 0U;
    g_tof_channels[index].has_valid_frame = 0U;
    g_tof_channels[index].consecutive_failures = 0U;
    APP_StartChannelReceive(&g_tof_channels[index]);
  }
}

static void APP_Poll(void)
{
  uint32_t now = HAL_GetTick();

  if ((g_config_save_pending != 0U) && ((now - g_last_config_change_tick) >= APP_CONFIG_SAVE_DELAY_MS))
  {
    if (APP_SavePersistedConfig() != 0U)
    {
      g_config_save_pending = 0U;
    }
  }

  APP_EvaluateSafety(now, &g_safety_state);
  APP_SetEstop(g_safety_state.self_estop);

  if ((now - g_last_report_tick) >= TOF_USB_REPORT_PERIOD_MS)
  {
    g_last_report_tick = now;
    APP_ReportFrame(now);
  }

  if ((now - g_last_led_tick) >= APP_LED_TOGGLE_PERIOD_MS)
  {
    g_last_led_tick = now;
    LED1_Toggle;
  }
}

static void APP_ReportFrame(uint32_t now)
{
  char payload[384];
  char frame_line[512];
  uint16_t distance_mm[TOF_CHANNEL_COUNT];
  uint32_t index;

  APP_EvaluateSafety(now, &g_safety_state);

  for (index = 0U; index < TOF_CHANNEL_COUNT; ++index)
  {
    distance_mm[index] = APP_ChannelDistance(&g_tof_channels[index], now);
  }

  (void)snprintf(
      payload,
      sizeof(payload),
      "H7TOF,%lu,%u,%u,%u,%u,%u,%02X,%02X,R=%lu/%lu/%lu/%lu,F=%lu/%lu/%lu/%lu,S=%u/%u/%u/%u,C=%u/%u/%u/%u,E=%u/%u/%u/%u,L=%02X/%02X/%02X/%02X",
      (unsigned long)g_usb_sequence++,
      (unsigned int)distance_mm[0],
      (unsigned int)distance_mm[1],
      (unsigned int)distance_mm[2],
      (unsigned int)distance_mm[3],
      (unsigned int)g_estop_state,
      (unsigned int)g_safety_state.valid_mask,
      (unsigned int)g_safety_state.fault_mask,
      (unsigned long)g_tof_channels[0].rx_count,
      (unsigned long)g_tof_channels[1].rx_count,
      (unsigned long)g_tof_channels[2].rx_count,
      (unsigned long)g_tof_channels[3].rx_count,
      (unsigned long)g_tof_channels[0].frame_count,
      (unsigned long)g_tof_channels[1].frame_count,
      (unsigned long)g_tof_channels[2].frame_count,
      (unsigned long)g_tof_channels[3].frame_count,
      (unsigned int)g_tof_channels[0].sync_errors,
      (unsigned int)g_tof_channels[1].sync_errors,
      (unsigned int)g_tof_channels[2].sync_errors,
      (unsigned int)g_tof_channels[3].sync_errors,
      (unsigned int)g_tof_channels[0].checksum_errors,
      (unsigned int)g_tof_channels[1].checksum_errors,
      (unsigned int)g_tof_channels[2].checksum_errors,
      (unsigned int)g_tof_channels[3].checksum_errors,
      (unsigned int)g_tof_channels[0].uart_errors,
      (unsigned int)g_tof_channels[1].uart_errors,
      (unsigned int)g_tof_channels[2].uart_errors,
      (unsigned int)g_tof_channels[3].uart_errors,
      (unsigned int)g_tof_channels[0].last_byte,
      (unsigned int)g_tof_channels[1].last_byte,
      (unsigned int)g_tof_channels[2].last_byte,
      (unsigned int)g_tof_channels[3].last_byte);

  (void)snprintf(
      frame_line,
      sizeof(frame_line),
      "%s,TM=%02X,A=%02X,M=%u,SE=%u,EE=%u,TK=%u,RT=%lu,B=%u,T=%u,TH=%u",
      payload,
      (unsigned int)g_safety_state.trip_mask,
      (unsigned int)g_safety_state.active_mask,
      (unsigned int)g_safety_state.motion_class,
      (unsigned int)g_safety_state.self_estop,
      (unsigned int)g_safety_state.external_estop,
      (unsigned int)g_safety_state.takeover_enabled,
      (unsigned long)g_safety_state.release_remaining_ms,
      (unsigned int)g_safety_state.baseline_mm,
      (unsigned int)g_safety_state.tolerance_mm,
      (unsigned int)g_safety_state.threshold_mm);

  USB_printf("$%s*%02X\r\n", frame_line, APP_CalcChecksum(frame_line));
}

static void APP_StartChannelReceive(TofChannelContext *channel)
{
  if ((channel == NULL) || (channel->uart == NULL))
  {
    return;
  }

  (void)HAL_UART_Receive_IT(channel->uart, &channel->rx_byte, 1U);
}

static void APP_ProcessChannelByte(TofChannelContext *channel, uint8_t byte, uint32_t now)
{
  uint16_t parsed_distance_mm;
  uint8_t sum;

  if (channel == NULL)
  {
    return;
  }

  if ((now - channel->last_byte_tick) > TOF_INTERBYTE_TIMEOUT_MS)
  {
    channel->frame_index = 0U;
  }

  channel->last_byte_tick = now;
  channel->last_byte = byte;
  channel->rx_count++;

  if (channel->frame_index == 0U)
  {
    if (byte != 0xFFU)
    {
      if (channel->sync_errors < 0xFFFFU)
      {
        channel->sync_errors++;
      }
      if (channel->consecutive_failures < 0xFFU)
      {
        channel->consecutive_failures++;
      }
      return;
    }
  }

  channel->frame[channel->frame_index++] = byte;

  if (channel->frame_index < TOF_FRAME_LENGTH)
  {
    return;
  }

  channel->frame_index = 0U;
  sum = (uint8_t)(channel->frame[0] + channel->frame[1] + channel->frame[2]);

  if (sum != channel->frame[3])
  {
    if (channel->checksum_errors < 0xFFFFU)
    {
      channel->checksum_errors++;
    }
    if (channel->consecutive_failures < 0xFFU)
    {
      channel->consecutive_failures++;
    }
    return;
  }

  parsed_distance_mm = (uint16_t)(((uint16_t)channel->frame[1] << 8) | channel->frame[2]);
  channel->distance_mm = parsed_distance_mm;
  channel->last_frame_tick = now;
  channel->frame_count++;
  channel->consecutive_failures = 0U;

  /* Some TOF modules report 0 when the target is out of range; treat that as invalid. */
  if ((parsed_distance_mm == 0U) || (parsed_distance_mm == TOF_INVALID_DISTANCE_MM))
  {
    channel->distance_mm = TOF_INVALID_DISTANCE_MM;
    channel->has_valid_frame = 0U;
    return;
  }

  channel->has_valid_frame = 1U;
}

static uint8_t APP_ChannelValid(const TofChannelContext *channel, uint32_t now)
{
  if ((channel == NULL) || (channel->has_valid_frame == 0U))
  {
    return 0U;
  }

  if ((now - channel->last_frame_tick) > TOF_FRAME_TIMEOUT_MS)
  {
    return 0U;
  }

  return 1U;
}

static uint8_t APP_ChannelTripExceeded(const TofChannelContext *channel, uint32_t now)
{
  if (APP_ChannelValid(channel, now) == 0U)
  {
    return 0U;
  }

  return (channel->distance_mm > APP_GetTripThresholdMm()) ? 1U : 0U;
}

static uint16_t APP_ChannelDistance(const TofChannelContext *channel, uint32_t now)
{
  if (APP_ChannelValid(channel, now) == 0U)
  {
    return TOF_INVALID_DISTANCE_MM;
  }

  return channel->distance_mm;
}

static uint8_t APP_CalcChecksum(const char *payload)
{
  uint8_t checksum = 0U;

  while ((payload != NULL) && (*payload != '\0'))
  {
    checksum ^= (uint8_t)(*payload);
    payload++;
  }

  return checksum;
}

static uint16_t APP_GetTripThresholdMm(void)
{
  uint32_t threshold = (uint32_t)g_baseline_mm + (uint32_t)g_tolerance_mm;

  if (threshold > TOF_CONFIG_MAX_MM)
  {
    threshold = TOF_CONFIG_MAX_MM;
  }

  return (uint16_t)threshold;
}

static uint16_t APP_NormalizeMmValue(float value)
{
  uint32_t rounded;

  if (value <= 0.0f)
  {
    return 0U;
  }

  rounded = (uint32_t)(value + 0.5f);
  if (rounded > TOF_CONFIG_MAX_MM)
  {
    rounded = TOF_CONFIG_MAX_MM;
  }

  return (uint16_t)rounded;
}

static uint32_t APP_NormalizeReleaseHoldMs(unsigned long value)
{
  uint32_t normalized = (uint32_t)value;

  if (normalized > APP_RELEASE_OVERRIDE_MAX_MS)
  {
    normalized = APP_RELEASE_OVERRIDE_MAX_MS;
  }

  return normalized;
}

static uint8_t APP_TickDeadlineActive(uint32_t deadline, uint32_t now)
{
  if (deadline == 0U)
  {
    return 0U;
  }

  return (((int32_t)(deadline - now)) > 0) ? 1U : 0U;
}

static uint32_t APP_GetReleaseRemainingMs(uint32_t now)
{
  if (APP_TickDeadlineActive(g_release_override_until_tick, now) == 0U)
  {
    return 0U;
  }

  return (uint32_t)(g_release_override_until_tick - now);
}

static void APP_SetEstop(uint8_t asserted)
{
  g_estop_state = (asserted != 0U) ? APP_ESTOP_ASSERTED : APP_ESTOP_RELEASED;
  HAL_GPIO_WritePin(
      ESTOP_OUT_GPIO_Port,
      ESTOP_OUT_Pin,
      (g_estop_state == APP_ESTOP_ASSERTED) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void APP_ControlInput(const uint8_t *data, uint32_t length)
{
  uint32_t index;

  if (data == NULL)
  {
    return;
  }

  for (index = 0U; index < length; ++index)
  {
    char ch = (char)data[index];

    if (ch == '\r')
    {
      continue;
    }

    if (ch == '\n')
    {
      if (g_control_line_length > 0U)
      {
        g_control_line[g_control_line_length] = '\0';
        APP_ParseControlLine(g_control_line, HAL_GetTick());
        g_control_line_length = 0U;
      }
      continue;
    }

    if ((uint8_t)ch < 0x20U)
    {
      continue;
    }

    if (g_control_line_length < (CONTROL_LINE_BUFFER_SIZE - 1U))
    {
      g_control_line[g_control_line_length++] = ch;
    }
    else
    {
      g_control_line_length = 0U;
    }
  }
}

static TofChannelContext *APP_FindChannel(UART_HandleTypeDef *huart)
{
  uint32_t index;

  for (index = 0U; index < TOF_CHANNEL_COUNT; ++index)
  {
    if (g_tof_channels[index].uart == huart)
    {
      return &g_tof_channels[index];
    }
  }

  return NULL;
}

static void APP_EvaluateSafety(uint32_t now, AppSafetyState *state)
{
  uint32_t index;

  if (state == NULL)
  {
    return;
  }

  state->motion_class = (uint8_t)APP_GetEffectiveMotionClass(now);
  state->active_mask = APP_GetActiveMask((AppMotionClass)state->motion_class);
  state->valid_mask = 0U;
  state->fault_mask = 0U;
  state->trip_mask = 0U;
  state->takeover_enabled = g_takeover_enabled;
  state->baseline_mm = g_baseline_mm;
  state->tolerance_mm = g_tolerance_mm;
  state->threshold_mm = APP_GetTripThresholdMm();
  state->release_remaining_ms = APP_GetReleaseRemainingMs(now);

  for (index = 0U; index < TOF_CHANNEL_COUNT; ++index)
  {
    uint8_t bit = (uint8_t)(1U << index);
    uint8_t channel_valid = APP_ChannelValid(&g_tof_channels[index], now);

    if (channel_valid != 0U)
    {
      state->valid_mask |= bit;
    }

    if ((state->active_mask & bit) != 0U)
    {
      if (channel_valid == 0U)
      {
        state->fault_mask |= bit;
      }
      else if (APP_ChannelTripExceeded(&g_tof_channels[index], now) != 0U)
      {
        state->trip_mask |= bit;
        state->fault_mask |= bit;
      }
    }
  }

  state->self_estop = ((state->fault_mask != 0U) && (state->release_remaining_ms == 0U)) ? 1U : 0U;
  state->external_estop = 0U;
}

static AppMotionClass APP_GetEffectiveMotionClass(uint32_t now)
{
  if ((g_has_control_input == 0U) || ((now - g_last_control_tick) > CONTROL_INPUT_TIMEOUT_MS))
  {
    return APP_MOTION_LINK_LOSS;
  }

  return g_motion_class;
}

static uint8_t APP_GetActiveMask(AppMotionClass motion_class)
{
  (void)motion_class;
  return TOF_MASK_ALL;
}

static void APP_ParseControlLine(char *line, uint32_t now)
{
  AppControlUpdate update = {0};
  char *trimmed;
  char *payload;
  uint16_t previous_baseline_mm;
  uint16_t previous_tolerance_mm;
  uint8_t config_changed = 0U;
  uint32_t release_hold_ms = APP_RELEASE_OVERRIDE_DEFAULT_MS;

  if (line == NULL)
  {
    return;
  }

  trimmed = APP_Trim(line);
  if ((trimmed == NULL) || (*trimmed == '\0'))
  {
    return;
  }

  payload = APP_UnframeControlLine(trimmed);
  if ((payload == NULL) || (*payload == '\0'))
  {
    return;
  }

  if ((APP_ParseControlH7Ctl(payload, &update) == 0U) &&
      (APP_ParseControlJson(payload, &update) == 0U) &&
      (APP_ParseControlText(payload, &update) == 0U))
  {
    return;
  }

  previous_baseline_mm = g_baseline_mm;
  previous_tolerance_mm = g_tolerance_mm;

  if (update.has_motion_class != 0U)
  {
    g_motion_class = update.motion_class;
  }

  if (update.has_takeover_enable != 0U)
  {
    g_takeover_enabled = update.takeover_enable;
  }

  if (update.has_baseline_mm != 0U)
  {
    g_baseline_mm = update.baseline_mm;
  }

  if (update.has_tolerance_mm != 0U)
  {
    g_tolerance_mm = update.tolerance_mm;
  }

  if (update.has_release_hold_ms != 0U)
  {
    release_hold_ms = update.release_hold_ms;
  }

  if ((g_baseline_mm != previous_baseline_mm) || (g_tolerance_mm != previous_tolerance_mm))
  {
    config_changed = 1U;
  }

  g_has_control_input = 1U;
  g_last_control_tick = now;
  if (update.release_request != 0U)
  {
    if (release_hold_ms == 0U)
    {
      release_hold_ms = APP_RELEASE_OVERRIDE_DEFAULT_MS;
    }
    g_release_override_until_tick = now + release_hold_ms;
  }

  if (config_changed != 0U)
  {
    APP_RequestConfigSave(now);
  }
}

static uint8_t APP_ParseControlH7Ctl(const char *line, AppControlUpdate *update)
{
  unsigned long seq_value;
  long vx_mmps;
  long vy_mmps;
  long wz_mradps;
  unsigned long release_req;
  unsigned long takeover_enable;
  unsigned long baseline_mm = 0UL;
  unsigned long tolerance_mm = 0UL;
  unsigned long release_hold_ms = 0UL;
  int matched;

  if ((line == NULL) || (update == NULL))
  {
    return 0U;
  }

  matched = sscanf(
      line,
      "H7CTL,%lu,%ld,%ld,%ld,%lu,%lu,%lu,%lu,%lu",
      &seq_value,
      &vx_mmps,
      &vy_mmps,
      &wz_mradps,
      &release_req,
      &takeover_enable,
      &baseline_mm,
      &tolerance_mm,
      &release_hold_ms);

  if (matched < 6)
  {
    return 0U;
  }

  (void)seq_value;
  update->has_motion_class = 1U;
  update->motion_class = APP_DeriveMotionClass((float)vx_mmps, (float)vy_mmps, (float)wz_mradps);
  update->has_takeover_enable = 1U;
  update->takeover_enable = (takeover_enable != 0UL) ? 1U : 0U;
  update->release_request = (release_req != 0UL) ? 1U : 0U;

  if (matched >= 7)
  {
    if (baseline_mm > (unsigned long)TOF_CONFIG_MAX_MM)
    {
      baseline_mm = (unsigned long)TOF_CONFIG_MAX_MM;
    }
    update->has_baseline_mm = 1U;
    update->baseline_mm = (uint16_t)baseline_mm;
  }

  if (matched >= 8)
  {
    if (tolerance_mm > (unsigned long)TOF_CONFIG_MAX_MM)
    {
      tolerance_mm = (unsigned long)TOF_CONFIG_MAX_MM;
    }
    update->has_tolerance_mm = 1U;
    update->tolerance_mm = (uint16_t)tolerance_mm;
  }

  if (matched >= 9)
  {
    update->has_release_hold_ms = 1U;
    update->release_hold_ms = APP_NormalizeReleaseHoldMs(release_hold_ms);
  }

  return 1U;
}

static uint8_t APP_ParseControlJson(const char *line, AppControlUpdate *update)
{
  char mode[16];
  float vx = 0.0f;
  float vy = 0.0f;
  float wz = 0.0f;
  float number_value;
  uint8_t parsed = 0U;
  uint8_t has_vx;
  uint8_t has_vy;
  uint8_t has_wz;

  if ((line == NULL) || (update == NULL) || (*line != '{'))
  {
    return 0U;
  }

  if ((APP_TryReadJsonString(line, "\"mode\"", mode, sizeof(mode)) != 0U) ||
      (APP_TryReadJsonString(line, "\"motion\"", mode, sizeof(mode)) != 0U))
  {
    parsed = APP_ParseControlText(mode, update);
  }

  has_vx = APP_TryReadJsonFloat(line, "\"vx\"", &vx);
  has_vy = APP_TryReadJsonFloat(line, "\"vy\"", &vy);
  has_wz = APP_TryReadJsonFloat(line, "\"wz\"", &wz);
  if (has_wz == 0U)
  {
    has_wz = APP_TryReadJsonFloat(line, "\"omega\"", &wz);
  }

  if ((has_vx != 0U) || (has_vy != 0U) || (has_wz != 0U))
  {
    update->has_motion_class = 1U;
    update->motion_class = APP_DeriveMotionClass(vx, vy, wz);
    parsed = 1U;
  }

  if ((APP_TryReadJsonFloat(line, "\"baseline_mm\"", &number_value) != 0U) ||
      (APP_TryReadJsonFloat(line, "\"baseline\"", &number_value) != 0U))
  {
    update->has_baseline_mm = 1U;
    update->baseline_mm = APP_NormalizeMmValue(number_value);
    parsed = 1U;
  }

  if ((APP_TryReadJsonFloat(line, "\"tolerance_mm\"", &number_value) != 0U) ||
      (APP_TryReadJsonFloat(line, "\"tolerance\"", &number_value) != 0U) ||
      (APP_TryReadJsonFloat(line, "\"tol\"", &number_value) != 0U))
  {
    update->has_tolerance_mm = 1U;
    update->tolerance_mm = APP_NormalizeMmValue(number_value);
    parsed = 1U;
  }

  if ((APP_TryReadJsonFloat(line, "\"release_req\"", &number_value) != 0U) ||
      (APP_TryReadJsonFloat(line, "\"release\"", &number_value) != 0U))
  {
    if (number_value > 0.5f)
    {
      update->release_request = 1U;
    }
    parsed = 1U;
  }

  if ((APP_TryReadJsonFloat(line, "\"release_hold_ms\"", &number_value) != 0U) ||
      (APP_TryReadJsonFloat(line, "\"release_ms\"", &number_value) != 0U) ||
      (APP_TryReadJsonFloat(line, "\"hold_ms\"", &number_value) != 0U))
  {
    if (number_value >= 0.0f)
    {
      update->has_release_hold_ms = 1U;
      update->release_hold_ms = APP_NormalizeReleaseHoldMs((unsigned long)(number_value + 0.5f));
    }
    parsed = 1U;
  }

  if ((APP_TryReadJsonFloat(line, "\"takeover_enable\"", &number_value) != 0U) ||
      (APP_TryReadJsonFloat(line, "\"takeover\"", &number_value) != 0U))
  {
    update->has_takeover_enable = 1U;
    update->takeover_enable = (number_value > 0.5f) ? 1U : 0U;
    parsed = 1U;
  }

  return parsed;
}

static uint8_t APP_ParseControlText(const char *line, AppControlUpdate *update)
{
  float vx;
  float vy;
  float wz;

  if ((line == NULL) || (update == NULL))
  {
    return 0U;
  }

  if ((APP_Stricmp(line, "FWD") == 0) ||
      (APP_Stricmp(line, "FORWARD") == 0) ||
      (APP_Stricmp(line, "MODE:FWD") == 0) ||
      (APP_Stricmp(line, "MODE:FORWARD") == 0) ||
      (APP_Stricmp(line, "CMD,FWD") == 0) ||
      (APP_Stricmp(line, "CMD,FORWARD") == 0))
  {
    update->has_motion_class = 1U;
    update->motion_class = APP_MOTION_FORWARD;
    return 1U;
  }

  if ((APP_Stricmp(line, "REV") == 0) ||
      (APP_Stricmp(line, "REVERSE") == 0) ||
      (APP_Stricmp(line, "BACKWARD") == 0) ||
      (APP_Stricmp(line, "MODE:REV") == 0) ||
      (APP_Stricmp(line, "MODE:REVERSE") == 0) ||
      (APP_Stricmp(line, "CMD,REV") == 0) ||
      (APP_Stricmp(line, "CMD,REVERSE") == 0))
  {
    update->has_motion_class = 1U;
    update->motion_class = APP_MOTION_REVERSE;
    return 1U;
  }

  if ((APP_Stricmp(line, "TURN") == 0) ||
      (APP_Stricmp(line, "ROTATE") == 0) ||
      (APP_Stricmp(line, "MODE:TURN") == 0) ||
      (APP_Stricmp(line, "CMD,TURN") == 0))
  {
    update->has_motion_class = 1U;
    update->motion_class = APP_MOTION_TURN;
    return 1U;
  }

  if ((APP_Stricmp(line, "STRAFE") == 0) ||
      (APP_Stricmp(line, "SHIFT") == 0) ||
      (APP_Stricmp(line, "MODE:STRAFE") == 0) ||
      (APP_Stricmp(line, "CMD,STRAFE") == 0))
  {
    update->has_motion_class = 1U;
    update->motion_class = APP_MOTION_STRAFE;
    return 1U;
  }

  if ((APP_Stricmp(line, "MIXED") == 0) ||
      (APP_Stricmp(line, "MODE:MIXED") == 0) ||
      (APP_Stricmp(line, "CMD,MIXED") == 0))
  {
    update->has_motion_class = 1U;
    update->motion_class = APP_MOTION_MIXED;
    return 1U;
  }

  if ((APP_Stricmp(line, "IDLE") == 0) ||
      (APP_Stricmp(line, "STOP") == 0) ||
      (APP_Stricmp(line, "STATIC") == 0) ||
      (APP_Stricmp(line, "MODE:IDLE") == 0) ||
      (APP_Stricmp(line, "CMD,IDLE") == 0))
  {
    update->has_motion_class = 1U;
    update->motion_class = APP_MOTION_IDLE;
    return 1U;
  }

  if ((APP_Stricmp(line, "REL") == 0) ||
      (APP_Stricmp(line, "RELEASE") == 0) ||
      (APP_Stricmp(line, "CMD,REL") == 0) ||
      (APP_Stricmp(line, "CMD,RELEASE") == 0))
  {
    update->release_request = 1U;
    return 1U;
  }

  if (APP_ParseFloatTriplet(line, &vx, &vy, &wz) != 0U)
  {
    update->has_motion_class = 1U;
    update->motion_class = APP_DeriveMotionClass(vx, vy, wz);
    return 1U;
  }

  return 0U;
}

static AppMotionClass APP_DeriveMotionClass(float vx, float vy, float wz)
{
  uint8_t has_vx = ((vx > CONTROL_EPSILON) || (vx < -CONTROL_EPSILON)) ? 1U : 0U;
  uint8_t has_vy = ((vy > CONTROL_EPSILON) || (vy < -CONTROL_EPSILON)) ? 1U : 0U;
  uint8_t has_wz = ((wz > CONTROL_EPSILON) || (wz < -CONTROL_EPSILON)) ? 1U : 0U;

  if ((has_vx == 0U) && (has_vy == 0U) && (has_wz == 0U))
  {
    return APP_MOTION_IDLE;
  }

  if ((has_vx != 0U) && (has_vy == 0U) && (has_wz == 0U))
  {
    return (vx > 0.0f) ? APP_MOTION_FORWARD : APP_MOTION_REVERSE;
  }

  if ((has_vx == 0U) && (has_vy != 0U) && (has_wz == 0U))
  {
    return APP_MOTION_STRAFE;
  }

  if ((has_vx == 0U) && (has_vy == 0U) && (has_wz != 0U))
  {
    return APP_MOTION_TURN;
  }

  return APP_MOTION_MIXED;
}

static uint8_t APP_TryReadJsonString(const char *json, const char *key, char *out, uint32_t out_size)
{
  const char *start;
  const char *end;
  uint32_t length;

  if ((json == NULL) || (key == NULL) || (out == NULL) || (out_size < 2U))
  {
    return 0U;
  }

  start = strstr(json, key);
  if (start == NULL)
  {
    return 0U;
  }

  start = strchr(start, ':');
  if (start == NULL)
  {
    return 0U;
  }

  start = APP_SkipWhitespace(start + 1);
  if (*start != '"')
  {
    return 0U;
  }

  start++;
  end = strchr(start, '"');
  if (end == NULL)
  {
    return 0U;
  }

  length = (uint32_t)(end - start);
  if (length >= out_size)
  {
    length = out_size - 1U;
  }

  (void)memcpy(out, start, length);
  out[length] = '\0';
  return 1U;
}

static uint8_t APP_TryReadJsonFloat(const char *json, const char *key, float *value)
{
  const char *start;
  char *end;

  if ((json == NULL) || (key == NULL) || (value == NULL))
  {
    return 0U;
  }

  start = strstr(json, key);
  if (start == NULL)
  {
    return 0U;
  }

  start = strchr(start, ':');
  if (start == NULL)
  {
    return 0U;
  }

  start = APP_SkipWhitespace(start + 1);
  *value = strtof(start, &end);
  return (end != start) ? 1U : 0U;
}

static uint8_t APP_ParseFloatTriplet(const char *line, float *vx, float *vy, float *wz)
{
  if ((line == NULL) || (vx == NULL) || (vy == NULL) || (wz == NULL))
  {
    return 0U;
  }

  if ((sscanf(line, "%f,%f,%f", vx, vy, wz) == 3) ||
      (sscanf(line, "%f %f %f", vx, vy, wz) == 3))
  {
    return 1U;
  }

  return 0U;
}

static int32_t APP_Stricmp(const char *left, const char *right)
{
  char left_char;
  char right_char;

  if ((left == NULL) || (right == NULL))
  {
    return -1;
  }

  while ((*left != '\0') && (*right != '\0'))
  {
    left_char = *left;
    right_char = *right;

    if ((left_char >= 'a') && (left_char <= 'z'))
    {
      left_char = (char)(left_char - ('a' - 'A'));
    }
    if ((right_char >= 'a') && (right_char <= 'z'))
    {
      right_char = (char)(right_char - ('a' - 'A'));
    }

    if (left_char != right_char)
    {
      return (int32_t)((unsigned char)left_char - (unsigned char)right_char);
    }

    left++;
    right++;
  }

  return (int32_t)((unsigned char)(*left) - (unsigned char)(*right));
}

static char *APP_Trim(char *text)
{
  char *end;

  if (text == NULL)
  {
    return NULL;
  }

  while ((*text == ' ') || (*text == '\t'))
  {
    text++;
  }

  end = text + strlen(text);
  while ((end > text) && ((end[-1] == ' ') || (end[-1] == '\t')))
  {
    end--;
  }
  *end = '\0';
  return text;
}

static char *APP_UnframeControlLine(char *line)
{
  char *payload;
  char *checksum_text;
  char *endptr;
  unsigned long expected_checksum;

  if (line == NULL)
  {
    return NULL;
  }

  payload = line;
  if (*payload == '$')
  {
    payload++;
  }

  checksum_text = strchr(payload, '*');
  if (checksum_text != NULL)
  {
    *checksum_text = '\0';
    expected_checksum = strtoul(checksum_text + 1, &endptr, 16);
    if ((endptr == (checksum_text + 1)) || ((uint8_t)expected_checksum != APP_CalcChecksum(payload)))
    {
      return NULL;
    }
  }

  return APP_Trim(payload);
}

static const char *APP_SkipWhitespace(const char *text)
{
  while ((text != NULL) && ((*text == ' ') || (*text == '\t')))
  {
    text++;
  }

  return text;
}

static uint32_t APP_PersistChecksum(const AppPersistRecord *record)
{
  const uint32_t *words;
  uint32_t checksum = 2166136261UL;
  uint32_t word_index;
  uint32_t byte_index;

  if (record == NULL)
  {
    return 0U;
  }

  words = (const uint32_t *)record;
  for (word_index = 0U; word_index < (APP_CONFIG_SLOT_WORD_COUNT - 1U); ++word_index)
  {
    uint32_t word = words[word_index];

    for (byte_index = 0U; byte_index < 4U; ++byte_index)
    {
      checksum ^= (word & 0xFFU);
      checksum *= 16777619UL;
      word >>= 8U;
    }
  }

  return checksum;
}

static uint8_t APP_PersistRecordIsBlank(const AppPersistRecord *record)
{
  const uint32_t *words;
  uint32_t index;

  if (record == NULL)
  {
    return 1U;
  }

  words = (const uint32_t *)record;
  for (index = 0U; index < APP_CONFIG_SLOT_WORD_COUNT; ++index)
  {
    if (words[index] != 0xFFFFFFFFUL)
    {
      return 0U;
    }
  }

  return 1U;
}

static uint8_t APP_PersistRecordIsValid(const AppPersistRecord *record)
{
  if (record == NULL)
  {
    return 0U;
  }

  if ((record->magic != APP_CONFIG_MAGIC) || (record->version != APP_CONFIG_VERSION))
  {
    return 0U;
  }

  if ((record->baseline_mm > TOF_CONFIG_MAX_MM) || (record->tolerance_mm > TOF_CONFIG_MAX_MM))
  {
    return 0U;
  }

  return (record->checksum == APP_PersistChecksum(record)) ? 1U : 0U;
}

static void APP_LoadPersistedConfig(void)
{
  const AppPersistRecord *latest = NULL;
  uint32_t address;

  APP_InvalidateDCache(APP_CONFIG_STORAGE_BASE, APP_CONFIG_STORAGE_SIZE);

  for (address = APP_CONFIG_STORAGE_BASE;
       address < APP_CONFIG_STORAGE_END;
       address += APP_CONFIG_SLOT_SIZE)
  {
    const AppPersistRecord *record = (const AppPersistRecord *)address;

    if (APP_PersistRecordIsBlank(record) != 0U)
    {
      break;
    }

    if (APP_PersistRecordIsValid(record) != 0U)
    {
      if ((latest == NULL) || (record->sequence >= latest->sequence))
      {
        latest = record;
      }
    }
  }

  if (latest != NULL)
  {
    g_baseline_mm = (uint16_t)latest->baseline_mm;
    g_tolerance_mm = (uint16_t)latest->tolerance_mm;
    g_persist_sequence = latest->sequence;
    g_config_loaded_from_flash = 1U;
  }
}

static uint8_t APP_SavePersistedConfig(void)
{
  ALIGN_32BYTES(static AppPersistRecord write_record);
  FLASH_EraseInitTypeDef erase_init = {0};
  uint32_t sector_error = 0xFFFFFFFFUL;
  uint32_t address;
  HAL_StatusTypeDef status;

  write_record.magic = APP_CONFIG_MAGIC;
  write_record.version = APP_CONFIG_VERSION;
  write_record.sequence = g_persist_sequence + 1U;
  write_record.baseline_mm = g_baseline_mm;
  write_record.tolerance_mm = g_tolerance_mm;
  write_record.reserved0 = 0xFFFFFFFFUL;
  write_record.reserved1 = 0xFFFFFFFFUL;
  write_record.checksum = APP_PersistChecksum(&write_record);

  for (address = APP_CONFIG_STORAGE_BASE;
       address < APP_CONFIG_STORAGE_END;
       address += APP_CONFIG_SLOT_SIZE)
  {
    if (APP_PersistRecordIsBlank((const AppPersistRecord *)address) != 0U)
    {
      break;
    }
  }

  status = HAL_FLASH_Unlock();
  if (status != HAL_OK)
  {
    return 0U;
  }

  if (address >= APP_CONFIG_STORAGE_END)
  {
    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.Banks = APP_CONFIG_STORAGE_BANK;
    erase_init.Sector = APP_CONFIG_STORAGE_SECTOR;
    erase_init.NbSectors = 1U;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    status = HAL_FLASHEx_Erase(&erase_init, &sector_error);
    if (status != HAL_OK)
    {
      (void)HAL_FLASH_Lock();
      return 0U;
    }

    APP_InvalidateDCache(APP_CONFIG_STORAGE_BASE, APP_CONFIG_STORAGE_SIZE);
    address = APP_CONFIG_STORAGE_BASE;
  }

  SCB_CleanDCache_by_Addr((uint32_t *)&write_record, (int32_t)sizeof(write_record));
  status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address, (uint32_t)&write_record);
  (void)HAL_FLASH_Lock();
  if (status != HAL_OK)
  {
    return 0U;
  }

  APP_InvalidateDCache(address, APP_CONFIG_SLOT_SIZE);

  if (APP_PersistRecordIsValid((const AppPersistRecord *)address) == 0U)
  {
    return 0U;
  }

  g_persist_sequence = write_record.sequence;
  g_config_loaded_from_flash = 1U;
  return 1U;
}

static void APP_RequestConfigSave(uint32_t now)
{
  g_config_save_pending = 1U;
  g_last_config_change_tick = now;
}

static void APP_InvalidateDCache(uint32_t address, uint32_t size)
{
  uint32_t aligned_address;
  uint32_t aligned_size;

  aligned_address = address & ~31UL;
  aligned_size = ((address + size + 31UL) & ~31UL) - aligned_address;

  SCB_InvalidateDCache_by_Addr((uint32_t *)aligned_address, (int32_t)aligned_size);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  TofChannelContext *channel = APP_FindChannel(huart);

  if (channel != NULL)
  {
    APP_ProcessChannelByte(channel, channel->rx_byte, HAL_GetTick());
    APP_StartChannelReceive(channel);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  TofChannelContext *channel = APP_FindChannel(huart);

  if (channel != NULL)
  {
    channel->frame_index = 0U;
    if (channel->uart_errors < 0xFFFFU)
    {
      channel->uart_errors++;
    }
    if (channel->consecutive_failures < 0xFFU)
    {
      channel->consecutive_failures++;
    }

    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    APP_StartChannelReceive(channel);
  }
}

static void APP_UART_CommonInit(UART_HandleTypeDef *huart, USART_TypeDef *instance)
{
  huart->Instance = instance;
  huart->Init.BaudRate = 115200;
  huart->Init.WordLength = UART_WORDLENGTH_8B;
  huart->Init.StopBits = UART_STOPBITS_1;
  huart->Init.Parity = UART_PARITY_NONE;
  huart->Init.Mode = UART_MODE_TX_RX;
  huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart->Init.OverSampling = UART_OVERSAMPLING_16;
  huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart->Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(huart) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_UARTEx_SetTxFifoThreshold(huart, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_UARTEx_SetRxFifoThreshold(huart, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_UARTEx_DisableFifoMode(huart) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART1_UART_Init(void)
{
  APP_UART_CommonInit(&huart1, USART1);
}

static void MX_USART2_UART_Init(void)
{
  APP_UART_CommonInit(&huart2, USART2);
}

static void MX_USART3_UART_Init(void)
{
  APP_UART_CommonInit(&huart3, USART3);
}

static void MX_USART6_UART_Init(void)
{
  APP_UART_CommonInit(&huart6, USART6);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();

  HAL_GPIO_WritePin(ESTOP_OUT_GPIO_Port, ESTOP_OUT_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = ESTOP_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ESTOP_OUT_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                              | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_PWREx_EnableUSBVoltageDetector();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
