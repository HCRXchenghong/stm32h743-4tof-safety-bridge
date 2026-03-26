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

#define TOF_CHANNEL_COUNT            4U
#define TOF_FRAME_LENGTH             4U
#define TOF_INVALID_DISTANCE_MM      65535U
#define TOF_INTERBYTE_TIMEOUT_MS     10U
#define TOF_FRAME_TIMEOUT_MS         120U
#define TOF_USB_REPORT_PERIOD_MS     30U
#define APP_LED_TOGGLE_PERIOD_MS     250U
#define APP_ESTOP_RELEASED           0U
#define APP_ESTOP_ASSERTED           1U

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
static uint8_t APP_ChannelFault(const TofChannelContext *channel, uint32_t now);
static uint16_t APP_ChannelDistance(const TofChannelContext *channel, uint32_t now);
static uint8_t APP_CalcChecksum(const char *payload);
static void APP_SetEstop(uint8_t asserted);
static uint8_t APP_ExternalEstopRequested(void);
static TofChannelContext *APP_FindChannel(UART_HandleTypeDef *huart);
static void APP_UART_CommonInit(UART_HandleTypeDef *huart, USART_TypeDef *instance);

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

  APP_SetEstop(APP_ExternalEstopRequested());

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
  uint8_t valid_mask = 0U;
  uint8_t fault_mask = 0U;
  uint16_t distance_mm[TOF_CHANNEL_COUNT];
  uint32_t index;

  for (index = 0U; index < TOF_CHANNEL_COUNT; ++index)
  {
    if (APP_ChannelValid(&g_tof_channels[index], now) != 0U)
    {
      valid_mask |= (uint8_t)(1U << index);
    }

    if (APP_ChannelFault(&g_tof_channels[index], now) != 0U)
    {
      fault_mask |= (uint8_t)(1U << index);
    }

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
      (unsigned int)valid_mask,
      (unsigned int)fault_mask,
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

  USB_printf("$%s*%02X\r\n", payload, APP_CalcChecksum(payload));
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

static uint8_t APP_ChannelFault(const TofChannelContext *channel, uint32_t now)
{
  if (channel == NULL)
  {
    return 1U;
  }

  if (APP_ChannelValid(channel, now) == 0U)
  {
    return 1U;
  }

  return (channel->consecutive_failures > 0U) ? 1U : 0U;
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
