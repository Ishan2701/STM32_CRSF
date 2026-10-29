/**
  ******************************************************************************
  * @file    crsf.c
  * @brief   CRSF (Crossfire / ExpressLRS) receiver driver - HAL, UART + DMA.
  ******************************************************************************
  * Frame format:  [sync][len][type][payload ...][crc8]
  *   len   = number of bytes after itself (type + payload + crc)
  *   crc8  = DVB-S2 (poly 0xD5) over type + payload
  ******************************************************************************
  */

#include "crsf.h"
#include <string.h>

#define CRSF_RX_BUF_SIZE                    128U  /* circular DMA buffer */
#define CRSF_MAX_FRAME_SIZE                 64U   /* sync + len + up to 62 bytes */
#define CRSF_FRAME_TIMEOUT_MS               5U    /* drop a partial frame after this gap */

#define CRSF_SYNC_BYTE                      0xC8U /* address: flight controller */
#define CRSF_SYNC_BYTE_ALT                  0xEEU /* address: CRSF transmitter */

#define CRSF_FRAMETYPE_LINK_STATISTICS      0x14U
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED   0x16U

#define CRSF_LINK_STATISTICS_PAYLOAD_SIZE   10U
#define CRSF_RC_CHANNELS_PAYLOAD_SIZE       22U   /* 16 channels x 11 bits */

static UART_HandleTypeDef *crsf_huart;

/* DMA ring buffer and read index */
static uint8_t  rx_buf[CRSF_RX_BUF_SIZE];
static uint16_t rx_tail;

/* Frame assembly */
static uint8_t  frame[CRSF_MAX_FRAME_SIZE];
static uint8_t  frame_pos;
static uint32_t frame_tick;

/* Decoded data, written from interrupt context */
static volatile uint16_t channels[CRSF_NUM_CHANNELS];
static volatile uint32_t last_rc_tick;
static volatile bool     have_rc;
static CRSF_LinkStats_t  link_stats;
static volatile bool     have_link_stats;
static CRSF_Stats_t      stats;

static uint8_t CRSF_Crc8(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0U;

  while (len--)
  {
    crc ^= *data++;
    for (uint8_t i = 0U; i < 8U; i++)
    {
      crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0xD5U) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static void CRSF_UnpackChannels(const uint8_t *payload)
{
  /* 11-bit values packed LSB first, little-endian */
  uint32_t acc = 0U;
  uint8_t bits = 0U;
  uint8_t ch = 0U;

  for (uint8_t i = 0U; i < CRSF_RC_CHANNELS_PAYLOAD_SIZE; i++)
  {
    acc |= (uint32_t)payload[i] << bits;
    bits += 8U;
    while (bits >= 11U)
    {
      channels[ch++] = (uint16_t)(acc & 0x7FFU);
      acc >>= 11;
      bits -= 11U;
    }
  }

  last_rc_tick = HAL_GetTick();
  have_rc = true;
}

static void CRSF_ParseLinkStats(const uint8_t *payload)
{
  link_stats.uplink_rssi_ant1      = payload[0];
  link_stats.uplink_rssi_ant2      = payload[1];
  link_stats.uplink_link_quality   = payload[2];
  link_stats.uplink_snr            = (int8_t)payload[3];
  link_stats.active_antenna        = payload[4];
  link_stats.rf_mode               = payload[5];
  link_stats.uplink_tx_power       = payload[6];
  link_stats.downlink_rssi         = payload[7];
  link_stats.downlink_link_quality = payload[8];
  link_stats.downlink_snr          = (int8_t)payload[9];
  have_link_stats = true;
}

static void CRSF_HandleFrame(void)
{
  uint8_t len = frame[1];
  uint8_t type = frame[2];
  uint8_t payload_len = len - 2U;

  if (CRSF_Crc8(&frame[2], len - 1U) != frame[len + 1U])
  {
    stats.crc_errors++;
    return;
  }
  stats.frames_ok++;

  switch (type)
  {
    case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
      if (payload_len == CRSF_RC_CHANNELS_PAYLOAD_SIZE)
      {
        CRSF_UnpackChannels(&frame[3]);
      }
      break;

    case CRSF_FRAMETYPE_LINK_STATISTICS:
      if (payload_len >= CRSF_LINK_STATISTICS_PAYLOAD_SIZE)
      {
        CRSF_ParseLinkStats(&frame[3]);
      }
      break;

    default:
      break;
  }
}

static void CRSF_ParseByte(uint8_t b, uint32_t now)
{
  if ((frame_pos > 0U) && ((now - frame_tick) > CRSF_FRAME_TIMEOUT_MS))
  {
    frame_pos = 0U;
  }
  frame_tick = now;

  if (frame_pos == 0U)
  {
    if ((b == CRSF_SYNC_BYTE) || (b == CRSF_SYNC_BYTE_ALT))
    {
      frame[frame_pos++] = b;
    }
    return;
  }

  if (frame_pos == 1U)
  {
    if ((b < 2U) || (b > (CRSF_MAX_FRAME_SIZE - 2U)))
    {
      /* Invalid length: resync, this byte may itself be a sync byte */
      frame_pos = 0U;
      if ((b == CRSF_SYNC_BYTE) || (b == CRSF_SYNC_BYTE_ALT))
      {
        frame[frame_pos++] = b;
      }
      return;
    }
  }

  frame[frame_pos++] = b;

  if (frame_pos == (uint8_t)(frame[1] + 2U))
  {
    CRSF_HandleFrame();
    frame_pos = 0U;
  }
}

static void CRSF_Feed(const uint8_t *data, uint16_t len, uint32_t now)
{
  while (len--)
  {
    CRSF_ParseByte(*data++, now);
  }
}

static HAL_StatusTypeDef CRSF_StartReception(void)
{
  rx_tail = 0U;
  frame_pos = 0U;

  /* The receiver may already be streaming: clear a pending overrun first */
  __HAL_UART_CLEAR_OREFLAG(crsf_huart);

  return HAL_UARTEx_ReceiveToIdle_DMA(crsf_huart, rx_buf, CRSF_RX_BUF_SIZE);
}

/**
  * @brief  Starts CRSF reception on a UART that has an RX DMA stream linked
  *         in circular mode.
  * @param  huart UART handle connected to the receiver.
  * @retval HAL status
  */
HAL_StatusTypeDef CRSF_Init(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart->hdmarx == NULL))
  {
    return HAL_ERROR;
  }

  crsf_huart = huart;

  for (uint8_t i = 0U; i < CRSF_NUM_CHANNELS; i++)
  {
    channels[i] = CRSF_CHANNEL_MID;
  }
  have_rc = false;
  have_link_stats = false;
  memset(&link_stats, 0, sizeof(link_stats));
  memset(&stats, 0, sizeof(stats));

  return CRSF_StartReception();
}

/**
  * @brief  Consumes new DMA data. Call from HAL_UARTEx_RxEventCallback().
  * @param  huart UART handle passed to the HAL callback.
  * @param  pos   Size argument of the HAL callback (DMA write position).
  */
void CRSF_RxEventCallback(UART_HandleTypeDef *huart, uint16_t pos)
{
  uint32_t now;

  if ((crsf_huart == NULL) || (huart != crsf_huart))
  {
    return;
  }

  /* pos == buffer size means the DMA wrapped back to index 0 */
  if (pos >= CRSF_RX_BUF_SIZE)
  {
    pos = 0U;
  }

  now = HAL_GetTick();

  if (pos > rx_tail)
  {
    CRSF_Feed(&rx_buf[rx_tail], pos - rx_tail, now);
  }
  else if (pos < rx_tail)
  {
    CRSF_Feed(&rx_buf[rx_tail], CRSF_RX_BUF_SIZE - rx_tail, now);
    CRSF_Feed(rx_buf, pos, now);
  }

  rx_tail = pos;
}

/**
  * @brief  Restarts reception after a UART error. Call from
  *         HAL_UART_ErrorCallback(); HAL stops DMA reception on any RX error.
  * @param  huart UART handle passed to the HAL callback.
  */
void CRSF_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((crsf_huart == NULL) || (huart != crsf_huart))
  {
    return;
  }

  stats.uart_errors++;

  if (huart->RxState == HAL_UART_STATE_READY)
  {
    (void)CRSF_StartReception();
  }
}

/**
  * @brief  Returns the raw value (172..1811) of a channel (0 = CH1).
  */
uint16_t CRSF_GetChannelRaw(uint8_t ch)
{
  if (ch >= CRSF_NUM_CHANNELS)
  {
    return 0U;
  }
  return channels[ch];
}

/**
  * @brief  Returns a channel in microseconds (~988..2012) (0 = CH1).
  */
uint16_t CRSF_GetChannelUs(uint8_t ch)
{
  if (ch >= CRSF_NUM_CHANNELS)
  {
    return 0U;
  }
  return CRSF_RawToUs(channels[ch]);
}

/**
  * @brief  Copies all raw channel values from the same frame.
  */
void CRSF_GetChannels(uint16_t out[CRSF_NUM_CHANNELS])
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  for (uint8_t i = 0U; i < CRSF_NUM_CHANNELS; i++)
  {
    out[i] = channels[i];
  }
  __set_PRIMASK(primask);
}

/**
  * @brief  True while RC frames keep arriving and the uplink LQ is non-zero.
  */
bool CRSF_IsLinkActive(void)
{
  if (!have_rc)
  {
    return false;
  }
  if ((HAL_GetTick() - last_rc_tick) >= CRSF_FAILSAFE_TIMEOUT_MS)
  {
    return false;
  }
  if (have_link_stats && (link_stats.uplink_link_quality == 0U))
  {
    return false;
  }
  return true;
}

/**
  * @brief  Copies the latest link statistics.
  */
void CRSF_GetLinkStats(CRSF_LinkStats_t *out)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  *out = link_stats;
  __set_PRIMASK(primask);
}

/**
  * @brief  Copies the driver diagnostics counters.
  */
void CRSF_GetStats(CRSF_Stats_t *out)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  *out = stats;
  __set_PRIMASK(primask);
}
