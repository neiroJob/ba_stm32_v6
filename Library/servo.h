#ifndef SERVO_H
#define SERVO_H

#include "main.h"
#include "cmsis_os.h"
#include "pool_types.h"  // < подключаем общие типы

/* === Внешние зависимости из main.c === */
extern UART_HandleTypeDef huart1;
extern osMutexId_t MutexStateHandle;
extern volatile uint8_t isNeedToRefresh;
extern pool_state_t g_pool_state;

/* === Глобальные флаги модуля === */
extern volatile uint8_t servo_sequence_active;
extern volatile uint8_t servo_error_flag;

/* === Публичный API === */
void Servo_Init(void);
void Servo_ProcessRxByte(uint8_t data);
void Servo_RequestBackwash(void);

/* === Внешние функции из main.c === */
extern void write_variable(uint16_t vp_address, uint16_t data);
extern void save_pool_state_to_flash(void);

#endif /* SERVO_H */