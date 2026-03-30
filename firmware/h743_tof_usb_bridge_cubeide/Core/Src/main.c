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
#define CONTROL_LINE_BUFFER_SIZE     64U
#define TOF_USB_REPORT_PERIOD_MS     30U
#define APP_LED_TOGGLE_PERIOD_MS     250U
#define APP_ESTOP_RELEASED           0U
#define APP_ESTOP_ASSERTED           1U
#define TOF_SAFE_BASELINE_MM         391U
#define TOF_SAFE_TOLERANCE_MM        28U
#define TOF_SAFE_MIN_MM              (TOF_SAFE_BASELINE_MM - TOF_SAFE_TOLERANCE_MM)
#define TOF_SAFE_MAX_MM              (TOF_SAFE_BASELINE_MM + TOF_SAFE_TOLERANCE_MM)
#define TOF_MASK_1                   0x01U
#define TOF_MASK_2                   0x02U
#define TOF_MASK_3                   0x04U
#define TOF_MASK_4                   0x08U
#define TOF_MASK_ALL                 (TOF_MASK_1 | TOF_MASK_2 | TOF_MASK_3 | TOF_MASK_4)
#define TOF_MASK_FORWARD             (TOF_MASK_1 | TOF_MASK_4)
#define TOF_MASK_REVERSE             (TOF_MASK_2 | TOF_MASK_3)
#define CONTROL_EPSILON              0.001f

#define ESTOP_OUT_GPIO_Port          GPIOE
#define ESTOP_OUT_Pin                GPIO_PIN_10
#define ESTOP_IN_GPIO_Port           GPIOE
#define ESTOP_IN_Pin                 GPIO_PIN_11

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
  uint8_t self_estop;
  uint8_t external_estop;
} AppSafetyState;

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
static uint32_t g_last_control_tick = 0U;
static uint8_t g_has_control_input = 0U;
static AppMotionClass g_motion_class = APP_MOTION_LINK_LOSS;
static AppSafetyState g_safety_state = {0};
static char g_control_line[CONTROL_LINE_BUFFER_SIZE];
static uint32_t g_control_line_length = 0U;

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
static uint8_t APP_CalcChecksum(const char *payload);
static void APP_SetEstop(uint8_t asserted);
static uint8_t APP_ExternalEstopRequested(void);
static TofChannelContext *APP_FindChannel(UART_HandleTypeDef *huart);
static void APP_UART_CommonInit(UART_HandleTypeDef *huart, USART_TypeDef *instance);
static void APP_EvaluateSafety(uint32_t now, AppSafetyState *state);
static uint8_t APP_ChannelOutOfWindow(const TofChannelContext *channel, uint32_t now);
static AppMotionClass APP_GetEffectiveMotionClass(uint32_t now);
static uint8_t APP_GetActiveMask(AppMotionClass motion_class);
static void APP_ParseControlLine(char *line, uint32_t now);
static uint8_t APP_ParseControlJson(const char *line, AppMotionClass *motion_class);
static uint8_t APP_ParseControlText(const char *line, AppMotionClass *motion_class);
static AppMotionClass APP_DeriveMotionClass(float vx, float vy, float wz);
static uint8_t APP_TryReadJsonString(const char *json, const char *key, char *out, uint32_t out_size);
static uint8_t APP_TryReadJsonFloat(const char *json, const char *key, float *value);
static uint8_t APP_ParseFloatTriplet(const char *line, float *vx, float *vy, float *wz);
static int32_t APP_Stricmp(const char *left, const char *right);
static char *APP_Trim(char *text);
static const char *APP_SkipWhitespace(const char *text);

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
  g_last_control_tick = now;
  g_has_control_input = 0U;
  g_motion_class = APP_MOTION_LINK_LOSS;
  g_control_line_length = 0U;

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

  APP_EvaluateSafety(now, &g_safety_state);
  APP_SetEstop((g_safety_state.self_estop != 0U) || (g_safety_state.external_estop != 0U));

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
  char payload[256];
  char frame_line[320];
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
      "%s,A=%02X,M=%u,SE=%u,EE=%u",
      payload,
      (unsigned int)g_safety_state.active_mask,
      (unsigned int)g_safety_state.motion_class,
      (unsigned int)g_safety_state.self_estop,
      (unsigned int)g_safety_state.external_estop);

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

  channel->distance_mm = (uint16_t)(((uint16_t)channel->frame[1] << 8) | channel->frame[2]);
  channel->last_frame_tick = now;
  channel->frame_count++;
  channel->has_valid_frame = 1U;
  channel->consecutive_failures = 0U;
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

static uint8_t APP_ChannelOutOfWindow(const TofChannelContext *channel, uint32_t now)
{
  uint16_t distance_mm;

  if (APP_ChannelValid(channel, now) == 0U)
  {
    return 1U;
  }

  distance_mm = channel->distance_mm;
  if ((distance_mm < TOF_SAFE_MIN_MM) || (distance_mm > TOF_SAFE_MAX_MM))
  {
    return 1U;
  }

  return 0U;
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

static void APP_SetEstop(uint8_t asserted)
{
  g_estop_state = (asserted != 0U) ? APP_ESTOP_ASSERTED : APP_ESTOP_RELEASED;
  HAL_GPIO_WritePin(
      ESTOP_OUT_GPIO_Port,
      ESTOP_OUT_Pin,
      (g_estop_state == APP_ESTOP_ASSERTED) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static uint8_t APP_ExternalEstopRequested(void)
{
  return (HAL_GPIO_ReadPin(ESTOP_IN_GPIO_Port, ESTOP_IN_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
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

  for (index = 0U; index < TOF_CHANNEL_COUNT; ++index)
  {
    uint8_t bit = (uint8_t)(1U << index);

    if (APP_ChannelValid(&g_tof_channels[index], now) != 0U)
    {
      state->valid_mask |= bit;
    }

    if ((state->active_mask & bit) != 0U)
    {
      if (APP_ChannelOutOfWindow(&g_tof_channels[index], now) != 0U)
      {
        state->fault_mask |= bit;
      }
    }
  }

  state->self_estop = (state->fault_mask != 0U) ? 1U : 0U;
  state->external_estop = APP_ExternalEstopRequested();
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
  switch (motion_class)
  {
    case APP_MOTION_FORWARD:
      return TOF_MASK_FORWARD;

    case APP_MOTION_REVERSE:
      return TOF_MASK_REVERSE;

    case APP_MOTION_TURN:
    case APP_MOTION_STRAFE:
    case APP_MOTION_MIXED:
    case APP_MOTION_IDLE:
    case APP_MOTION_LINK_LOSS:
    default:
      return TOF_MASK_ALL;
  }
}

static void APP_ParseControlLine(char *line, uint32_t now)
{
  AppMotionClass motion_class;
  char *trimmed;

  if (line == NULL)
  {
    return;
  }

  trimmed = APP_Trim(line);
  if ((trimmed == NULL) || (*trimmed == '\0'))
  {
    return;
  }

  if ((APP_ParseControlJson(trimmed, &motion_class) == 0U) &&
      (APP_ParseControlText(trimmed, &motion_class) == 0U))
  {
    return;
  }

  g_motion_class = motion_class;
  g_has_control_input = 1U;
  g_last_control_tick = now;
}

static uint8_t APP_ParseControlJson(const char *line, AppMotionClass *motion_class)
{
  char mode[16];
  float vx;
  float vy;
  float wz;

  if ((line == NULL) || (motion_class == NULL) || (*line != '{'))
  {
    return 0U;
  }

  if ((APP_TryReadJsonString(line, "\"mode\"", mode, sizeof(mode)) != 0U) ||
      (APP_TryReadJsonString(line, "\"motion\"", mode, sizeof(mode)) != 0U))
  {
    return APP_ParseControlText(mode, motion_class);
  }

  if ((APP_TryReadJsonFloat(line, "\"vx\"", &vx) != 0U) ||
      (APP_TryReadJsonFloat(line, "\"vy\"", &vy) != 0U) ||
      (APP_TryReadJsonFloat(line, "\"wz\"", &wz) != 0U) ||
      (APP_TryReadJsonFloat(line, "\"omega\"", &wz) != 0U))
  {
    if (APP_TryReadJsonFloat(line, "\"vx\"", &vx) == 0U)
    {
      vx = 0.0f;
    }
    if (APP_TryReadJsonFloat(line, "\"vy\"", &vy) == 0U)
    {
      vy = 0.0f;
    }
    if ((APP_TryReadJsonFloat(line, "\"wz\"", &wz) == 0U) &&
        (APP_TryReadJsonFloat(line, "\"omega\"", &wz) == 0U))
    {
      wz = 0.0f;
    }

    *motion_class = APP_DeriveMotionClass(vx, vy, wz);
    return 1U;
  }

  return 0U;
}

static uint8_t APP_ParseControlText(const char *line, AppMotionClass *motion_class)
{
  float vx;
  float vy;
  float wz;

  if ((line == NULL) || (motion_class == NULL))
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
    *motion_class = APP_MOTION_FORWARD;
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
    *motion_class = APP_MOTION_REVERSE;
    return 1U;
  }

  if ((APP_Stricmp(line, "TURN") == 0) ||
      (APP_Stricmp(line, "ROTATE") == 0) ||
      (APP_Stricmp(line, "MODE:TURN") == 0) ||
      (APP_Stricmp(line, "CMD,TURN") == 0))
  {
    *motion_class = APP_MOTION_TURN;
    return 1U;
  }

  if ((APP_Stricmp(line, "STRAFE") == 0) ||
      (APP_Stricmp(line, "SHIFT") == 0) ||
      (APP_Stricmp(line, "MODE:STRAFE") == 0) ||
      (APP_Stricmp(line, "CMD,STRAFE") == 0))
  {
    *motion_class = APP_MOTION_STRAFE;
    return 1U;
  }

  if ((APP_Stricmp(line, "MIXED") == 0) ||
      (APP_Stricmp(line, "MODE:MIXED") == 0) ||
      (APP_Stricmp(line, "CMD,MIXED") == 0))
  {
    *motion_class = APP_MOTION_MIXED;
    return 1U;
  }

  if ((APP_Stricmp(line, "IDLE") == 0) ||
      (APP_Stricmp(line, "STOP") == 0) ||
      (APP_Stricmp(line, "STATIC") == 0) ||
      (APP_Stricmp(line, "MODE:IDLE") == 0) ||
      (APP_Stricmp(line, "CMD,IDLE") == 0))
  {
    *motion_class = APP_MOTION_IDLE;
    return 1U;
  }

  if (APP_ParseFloatTriplet(line, &vx, &vy, &wz) != 0U)
  {
    *motion_class = APP_DeriveMotionClass(vx, vy, wz);
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

static const char *APP_SkipWhitespace(const char *text)
{
  while ((text != NULL) && ((*text == ' ') || (*text == '\t')))
  {
    text++;
  }

  return text;
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

  GPIO_InitStruct.Pin = ESTOP_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ESTOP_IN_GPIO_Port, &GPIO_InitStruct);
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
