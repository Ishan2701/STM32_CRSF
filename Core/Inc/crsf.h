/**
  ******************************************************************************
  * @file    crsf.h
  * @brief   CRSF (Crossfire / ExpressLRS) receiver driver - HAL, UART + DMA.
  ******************************************************************************
  * Receives CRSF frames from an ELRS receiver (e.g. RadioMaster RP1) on a UART
  * using HAL_UARTEx_ReceiveToIdle_DMA with a circular DMA buffer, and decodes
  * the 16 RC channels and the link statistics.
  *
  * Wiring (NUCLEO-F412ZG, USART2):
  *   RP1 TX  -> PA3 (USART2_RX, CN9 "A0")
  *   RP1 RX  -> PA2 (USART2_TX, ST morpho header)
  *   RP1 5V  -> 5V,  RP1 GND -> GND
  *
  * Radio notes:
  *   - ELRS receivers default to 420000 baud; the receiver's serial baud must
  *     be set to match the UART (115200 here).
  *   - At 115200 baud an RC frame (26 bytes) takes ~2.3 ms on the wire, so keep
  *     the ELRS packet rate at 250 Hz or lower.
  *   - Channel order (CH1..CH4) follows the model's mixer on the transmitter.
  *
  * Usage:
  *   1. CRSF_Init(&huart2) after MX_USART2_UART_Init().
  *   2. Forward HAL_UARTEx_RxEventCallback() to CRSF_RxEventCallback() and
  *      HAL_UART_ErrorCallback() to CRSF_ErrorCallback().
  *   3. Read channels with CRSF_GetChannelUs() / CRSF_GetChannels() and gate
  *      their use on CRSF_IsLinkActive().
  ******************************************************************************
  */

#ifndef __CRSF_H
#define __CRSF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define CRSF_NUM_CHANNELS           16U

/* Raw 11-bit channel range as sent by ELRS / Crossfire */
#define CRSF_CHANNEL_MIN            172U    /*  988 us */
#define CRSF_CHANNEL_MID            992U    /* 1500 us */
#define CRSF_CHANNEL_MAX            1811U   /* 2012 us */

/* Link is reported lost when no RC frame arrived for this long */
#define CRSF_FAILSAFE_TIMEOUT_MS    100U

/* Payload of the LINK_STATISTICS frame (0x14) */
typedef struct
{
  uint8_t uplink_rssi_ant1;       /* RSSI antenna 1, in -dBm */
  uint8_t uplink_rssi_ant2;       /* RSSI antenna 2, in -dBm */
  uint8_t uplink_link_quality;    /* packet success rate, % */
  int8_t  uplink_snr;             /* dB */
  uint8_t active_antenna;         /* 0 or 1 */
  uint8_t rf_mode;                /* packet-rate index */
  uint8_t uplink_tx_power;        /* TX power index */
  uint8_t downlink_rssi;          /* telemetry RSSI, in -dBm */
  uint8_t downlink_link_quality;  /* telemetry LQ, % */
  int8_t  downlink_snr;           /* dB */
} CRSF_LinkStats_t;

/* Driver diagnostics counters */
typedef struct
{
  uint32_t frames_ok;             /* frames with a valid CRC */
  uint32_t crc_errors;            /* frames dropped on bad CRC */
  uint32_t uart_errors;           /* UART errors (reception restarted) */
} CRSF_Stats_t;

HAL_StatusTypeDef CRSF_Init(UART_HandleTypeDef *huart);
void CRSF_RxEventCallback(UART_HandleTypeDef *huart, uint16_t pos);
void CRSF_ErrorCallback(UART_HandleTypeDef *huart);

/* ch is 0-based: 0 = CH1 ... 15 = CH16. Invalid ch returns 0. */
uint16_t CRSF_GetChannelRaw(uint8_t ch);
uint16_t CRSF_GetChannelUs(uint8_t ch);
void CRSF_GetChannels(uint16_t out[CRSF_NUM_CHANNELS]);

bool CRSF_IsLinkActive(void);
void CRSF_GetLinkStats(CRSF_LinkStats_t *out);
void CRSF_GetStats(CRSF_Stats_t *out);

/* Converts a raw CRSF value to microseconds (172 -> 988, 992 -> 1500, 1811 -> 2012) */
static inline uint16_t CRSF_RawToUs(uint16_t raw)
{
  return (uint16_t)((((uint32_t)raw * 5U) + 4U) / 8U + 880U);
}

#ifdef __cplusplus
}
#endif

#endif /* __CRSF_H */
