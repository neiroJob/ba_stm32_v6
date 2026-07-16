#include "dwin.h"
#include <string.h>

/* Если после начала кадра (заголовок уже принят) новых байт нет дольше этого
 * времени — считаем кадр битым/линию оборванной и сбрасываем автомат.
 * Значение с большим запасом относительно скорости 115200 8N1 (кадр из
 * DWIN_MAX_PAYLOAD_LEN=16 байт передаётся физически за доли миллисекунды);
 * 200 мс — это уже явно "тишина", а не пауза между байтами одного кадра. */
#define DWIN_RX_STUCK_TIMEOUT_MS 200

void DWIN_Channel_Init(dwin_channel_t *ch, UART_HandleTypeDef *huart,
                        int32_t addr_offset, const volatile uint8_t *enabled_flag) {
  memset((void *)ch, 0, sizeof(*ch));
  ch->huart = huart;
  ch->addr_offset = addr_offset;
  ch->enabled_flag = enabled_flag;
  ch->rx_state = DWIN_RX_WAIT_HEADER1;
  ch->last_rx_tick = HAL_GetTick();

  /* Запускаем приём первого байта. Дальше цепочка сама себя поддерживает:
   * DWIN_Channel_RxCpltFromISR перевооружает приём в конце каждого вызова,
   * DWIN_Channel_ErrorFromISR — в конце обработки ошибки. */
  HAL_UART_Receive_IT(ch->huart, &ch->rx_byte, 1);
}

uint8_t DWIN_Channel_IsEnabled(const dwin_channel_t *ch) {
  return (ch->enabled_flag == NULL) || (*ch->enabled_flag != 0);
}

void DWIN_Channel_RxCpltFromISR(dwin_channel_t *ch) {
  uint8_t data = ch->rx_byte;
  ch->last_rx_tick = HAL_GetTick();

  switch (ch->rx_state) {
  case DWIN_RX_WAIT_HEADER1:
    if (data == DWIN_FRAME_HEADER1) {
      ch->rx_state = DWIN_RX_WAIT_HEADER2;
    }
    /* иначе — просто продолжаем ждать первый байт заголовка */
    break;

  case DWIN_RX_WAIT_HEADER2:
    if (data == DWIN_FRAME_HEADER2) {
      ch->rx_state = DWIN_RX_WAIT_LEN;
    } else {
      /* не тот байт после 0x5A — заголовок ложный, начинаем поиск заново */
      ch->rx_state = DWIN_RX_WAIT_HEADER1;
    }
    break;

  case DWIN_RX_WAIT_LEN:
    ch->rx_len = data;
    if (ch->rx_len >= 3 && ch->rx_len <= DWIN_MAX_PAYLOAD_LEN) {
      ch->rx_index = 0;
      ch->rx_state = DWIN_RX_RECEIVING;
    } else {
      /* некорректная длина кадра (0/1/2 или больше буфера) — отбрасываем */
      ch->rx_state = DWIN_RX_WAIT_HEADER1;
    }
    break;

  case DWIN_RX_RECEIVING:
    if (ch->rx_index < DWIN_MAX_PAYLOAD_LEN) {
      ch->rx_buffer[ch->rx_index] = data;
      ch->rx_index++;
    }
    if (ch->rx_index >= ch->rx_len) {
      /* Кадр собран целиком — кладём в очередь, если есть свободное место. */
      uint8_t next_head = (uint8_t)((ch->queue_head + 1) % DWIN_QUEUE_SIZE);
      if (next_head != ch->queue_tail) {
        ch->queue[ch->queue_head].length = ch->rx_len;
        memcpy(ch->queue[ch->queue_head].data, ch->rx_buffer, ch->rx_len);
        ch->queue_head = next_head;
      }
      /* Иначе очередь переполнена — кадр молча теряется, как и в исходной
       * реализации (packet_queue в main.c вела себя так же). */
      ch->rx_state = DWIN_RX_WAIT_HEADER1;
    }
    break;
  }

  /* HAL_UART_Receive_IT(...,1) принимает РОВНО один байт и останавливается —
   * обязательно перевооружаем приём следующего байта в конце КАЖДОГО вызова. */
  HAL_UART_Receive_IT(ch->huart, &ch->rx_byte, 1);
}

void DWIN_Channel_ErrorFromISR(dwin_channel_t *ch) {
  ch->error_count++;
  ch->last_error_tick = HAL_GetTick();
  /* Недособранный кадр всё равно уже не восстановить — сбрасываем автомат,
   * чтобы следующий валидный заголовок 5A A5 гарантированно подхватился. */
  ch->rx_state = DWIN_RX_WAIT_HEADER1;
  /* При ошибке (framing/noise/overrun) HAL сам переводит UART в состояние
   * "не принимает" — без повторного HAL_UART_Receive_IT канал молча умрёт
   * навсегда после первой же наведённой помехи на линии. */
  HAL_UART_Receive_IT(ch->huart, &ch->rx_byte, 1);
}

uint8_t DWIN_Channel_PopPacket(dwin_channel_t *ch, dwin_packet_t *out) {
  if (ch->queue_tail == ch->queue_head) {
    return 0; /* очередь пуста */
  }
  *out = ch->queue[ch->queue_tail];
  ch->queue_tail = (uint8_t)((ch->queue_tail + 1) % DWIN_QUEUE_SIZE);
  return 1;
}

uint16_t DWIN_Channel_ToLocalAddr(const dwin_channel_t *ch, uint16_t raw_addr) {
  return (uint16_t)((int32_t)raw_addr - ch->addr_offset);
}

void DWIN_WriteVariable(dwin_channel_t *ch, uint16_t local_addr, uint16_t data) {
  if (!DWIN_Channel_IsEnabled(ch)) {
    return; /* экрана физически нет на этом канале — отправлять некуда */
  }

  uint16_t wire_addr = (uint16_t)((int32_t)local_addr + ch->addr_offset);
  uint8_t tx_buffer[8] = {DWIN_FRAME_HEADER1,
                           DWIN_FRAME_HEADER2,
                           0x05,
                           DWIN_CMD_WRITE_VP,
                           (uint8_t)(wire_addr >> 8),
                           (uint8_t)(wire_addr & 0xFF),
                           (uint8_t)(data >> 8),
                           (uint8_t)(data & 0xFF)};
  HAL_UART_Transmit(ch->huart, tx_buffer, sizeof(tx_buffer), 20);
}

void DWIN_SendRaw(dwin_channel_t *ch, const uint8_t *frame, uint16_t len) {
  if (!DWIN_Channel_IsEnabled(ch)) {
    return; /* экрана физически нет на этом канале — отправлять некуда */
  }
  HAL_UART_Transmit(ch->huart, (uint8_t *)frame, len, 20);
}

void DWIN_Channel_Poll(dwin_channel_t *ch, uint32_t now_tick) {
  if (ch->rx_state != DWIN_RX_WAIT_HEADER1) {
    uint32_t idle_ms = now_tick - ch->last_rx_tick; /* корректно и при переполнении HAL_GetTick() */
    if (idle_ms > DWIN_RX_STUCK_TIMEOUT_MS) {
      /* Кадр начали собирать, но линия "замолчала" на середине — либо помеха,
       * либо обрыв связи. Сбрасываем автомат и на всякий случай ещё раз
       * вооружаем приём (пассивное восстановление, без переинициализации
       * самой периферии USART — см. обсуждение архитектуры канала). */
      ch->rx_state = DWIN_RX_WAIT_HEADER1;
      ch->restart_count++;
      HAL_UART_Receive_IT(ch->huart, &ch->rx_byte, 1);
    }
  }
}
