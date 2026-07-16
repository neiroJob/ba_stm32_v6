#include "dwin.h"
#include <string.h>

/* Если после начала кадра (заголовок уже принят) новых байт нет дольше этого
 * времени — считаем кадр битым/линию оборванной и сбрасываем автомат.
 * Значение с большим запасом относительно скорости 115200 8N1 (кадр из
 * DWIN_MAX_PAYLOAD_LEN=16 байт передаётся физически за доли миллисекунды);
 * 200 мс — это уже явно "тишина", а не пауза между байтами одного кадра.
 * Это ДЕФОЛТ для DWIN_Channel_Init() — подходит для прямого провода к
 * физическому экрану. Каналы через внешний транспорт (см. stuck_timeout_ms
 * в dwin.h) переопределяют его через DWIN_Channel_SetStuckTimeout(). */
#define DWIN_RX_STUCK_TIMEOUT_MS_DEFAULT 200

void DWIN_Channel_Init(dwin_channel_t *ch, UART_HandleTypeDef *huart,
                        int32_t addr_offset, const volatile uint8_t *enabled_flag) {
  memset((void *)ch, 0, sizeof(*ch));
  ch->huart = huart;
  ch->addr_offset = addr_offset;
  ch->enabled_flag = enabled_flag;
  ch->rx_state = DWIN_RX_WAIT_HEADER1;
  ch->last_rx_tick = HAL_GetTick();
  ch->needs_resync = 1; /* свежий канал — вызывающий код должен один раз послать актуальное состояние целиком */
  ch->stuck_timeout_ms = DWIN_RX_STUCK_TIMEOUT_MS_DEFAULT; /* см. DWIN_Channel_SetStuckTimeout() для переопределения */

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
  /* usart_error: канал только что сбоил — после восстановления экран может
   * оказаться с устаревшими данными (пропустил всё, что слалось, пока была
   * ошибка). needs_resync просит вызывающий код досослать состояние заново
   * целиком, а не только то, что реально изменится в будущем. */
  ch->needs_resync = 1;
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

uint8_t DWIN_Channel_ConsumeResyncFlag(dwin_channel_t *ch) {
  /* needs_resync выставляется из ISR (DWIN_Channel_ErrorFromISR) и читается
   * из задачи — короткая критическая секция на случай, если запись из ISR
   * попадёт ровно между чтением и сбросом. Секция длится единицы тактов
   * (чтение байта, сравнение, запись), Poll-ы других каналов и системный
   * тик от этого не пострадают. */
  __disable_irq();
  uint8_t was_set = ch->needs_resync;
  ch->needs_resync = 0;
  __enable_irq();
  return was_set;
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

void DWIN_PatchFrameAddress(uint8_t *payload, uint8_t payload_len, int32_t delta) {
  if (payload_len < 3) {
    return; /* нет байта CMD + 2 байт адреса VP в этой позиции */
  }
  if (payload[0] != DWIN_CMD_WRITE_VP && payload[0] != DWIN_CMD_READ_VP) {
    return; /* не команда с адресом VP (например, служебный кадр) — не трогаем */
  }
  uint16_t addr = (uint16_t)((payload[1] << 8) | payload[2]);
  addr = (uint16_t)((int32_t)addr + delta);
  payload[1] = (uint8_t)(addr >> 8);
  payload[2] = (uint8_t)(addr & 0xFF);
}

void DWIN_SendRawFrame(dwin_channel_t *ch, const uint8_t *payload, uint8_t payload_len) {
  if (!DWIN_Channel_IsEnabled(ch)) {
    return; /* экрана физически нет на этом канале — отправлять некуда */
  }
  if (payload_len > DWIN_MAX_PAYLOAD_LEN) {
    return; /* защита от переполнения — не должно случаться, т.к. payload_len
             * всегда приходит из DWIN_Channel_PopPacket(), который сам
             * ограничен тем же DWIN_MAX_PAYLOAD_LEN */
  }
  uint8_t tx_buffer[2 + 1 + DWIN_MAX_PAYLOAD_LEN];
  tx_buffer[0] = DWIN_FRAME_HEADER1;
  tx_buffer[1] = DWIN_FRAME_HEADER2;
  tx_buffer[2] = payload_len;
  memcpy(&tx_buffer[3], payload, payload_len);
  HAL_UART_Transmit(ch->huart, tx_buffer, (uint16_t)(3 + payload_len), 20);
}

void DWIN_Channel_SetStuckTimeout(dwin_channel_t *ch, uint32_t timeout_ms) {
  ch->stuck_timeout_ms = timeout_ms;
}

void DWIN_Channel_Poll(dwin_channel_t *ch, uint32_t now_tick) {
  if (ch->rx_state != DWIN_RX_WAIT_HEADER1) {
    uint32_t idle_ms = now_tick - ch->last_rx_tick; /* корректно и при переполнении HAL_GetTick() */
    if (idle_ms > ch->stuck_timeout_ms) {
      /* Кадр начали собирать, но линия "замолчала" на середине — либо помеха,
       * либо обрыв связи. Сбрасываем автомат и на всякий случай ещё раз
       * вооружаем приём (пассивное восстановление, без переинициализации
       * самой периферии USART — см. обсуждение архитектуры канала). */
      ch->rx_state = DWIN_RX_WAIT_HEADER1;
      ch->restart_count++;
      ch->needs_resync = 1; /* usart_error: как и после ErrorFromISR — досослать состояние заново */
      HAL_UART_Receive_IT(ch->huart, &ch->rx_byte, 1);
    }
  }
}
