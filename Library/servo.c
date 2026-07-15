#include "servo.h"
#include <string.h>
#include <stdio.h>

/* === Конфигурация === */
#define SERVO_RX_BUF_SIZE 32
#define SERVO_TIMEOUT_MS  180000  // 3 минуты
#define PAUSE_RUCKSP_MS   30000
#define PAUSE_KLARSP_MS   50000

/* === Глобальные переменные модуля === */
volatile uint8_t servo_sequence_active = 0;
volatile uint8_t servo_error_flag = 0;

/* === Внутренние переменные === */
static char servo_rx_buf[SERVO_RX_BUF_SIZE];
static volatile uint16_t servo_rx_idx = 0;
static volatile uint8_t servo_line_ready = 0;
static volatile uint8_t backwash_req = 0;
uint8_t uart1_rx_byte;  // Байт для прерывания (глобальный для HAL_UART_RxCpltCallback)

/* === Прототипы === */
static void ServoTask(void *argument);
static void Servo_SetAllEquipmentOff(void);
static void Servo_SetPumpsOnlyOn(void);
static uint8_t Servo_WaitOk(uint32_t timeout_ms);

/* === Инициализация === */
void Servo_Init(void) {
    // Запускаем приём USART1 в прерываниях
    HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
    
    // Создаём задачу
    static const osThreadAttr_t servo_attrs = {
        .name = "ServoTask",
        .stack_size = 512,
        .priority = osPriorityNormal,
    };
    osThreadNew(ServoTask, NULL, &servo_attrs);
}

/* === Обработчик прерывания USART1 === */
void Servo_ProcessRxByte(uint8_t data) {
    // Если строка уже готова, но ещё не прочитана задачей — игнорируем новые данные,
    // но ОБЯЗАТЕЛЬНО перезапускаем приём!
    if (servo_line_ready) {
        HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
        return;
    }

    if (data == '\r' || data == '\n') {
        if (servo_rx_idx > 0) {
            servo_rx_buf[servo_rx_idx] = '\0';  // Null-terminate
            servo_line_ready = 1;  // Сигнал задаче
        }
        servo_rx_idx = 0;
    } else if (servo_rx_idx < SERVO_RX_BUF_SIZE - 1) {
        servo_rx_buf[servo_rx_idx++] = data;
    }
    
    // ПЕРЕЗАПУСК ПРИЁМА — ВСЕГДА, в самом конце
    HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
}

/* === Запрос промывки === */
void Servo_RequestBackwash(void) {
    backwash_req = 1;
}

/* === Управление оборудованием === */
static void Servo_SetAllEquipmentOff(void) {
    HAL_GPIO_WritePin(GPIOA, NASOS1_Pin | ST_DOZ_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, NASOS2_Pin | HEATING_Pin | UFO_Pin | DOLIV_Pin, GPIO_PIN_RESET);
    write_variable(DWIN_ADDR_ICO_FILTERN, 0x0000);
    write_variable(DWIN_ADDR_ICO_HEATING, 0x0000);
}

static void Servo_SetPumpsOnlyOn(void) {
    HAL_GPIO_WritePin(GPIOA, NASOS1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, NASOS2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, ST_DOZ_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, HEATING_Pin | UFO_Pin | DOLIV_Pin, GPIO_PIN_RESET);
    write_variable(DWIN_ADDR_ICO_FILTERN, 0x0011);
    write_variable(DWIN_ADDR_ICO_HEATING, 0x0000);
}

/* === Ожидание ответа servo_ok (встроенный while, без отдельной функции) === */
#define WAIT_SERVO_OK(cmd_str, cmd_len) do { \
    servo_line_ready = 0; \
    servo_rx_buf[0] = '\0'; \
    servo_rx_idx = 0; \
    uint32_t wait_start = osKernelGetTickCount(); \
    uint8_t cmd_ok = 0; \
    HAL_UART_Transmit(&huart1, (uint8_t*)(cmd_str), (cmd_len), 50); \
    osDelay(5); /* Дадим прерыванию "догнать" отправку */ \
    while (osKernelGetTickCount() - wait_start < SERVO_TIMEOUT_MS) { \
        if (servo_line_ready) { \
            servo_line_ready = 0; \
            if (strstr(servo_rx_buf, "servo_ok") != NULL) { \
                cmd_ok = 1; \
            } \
            break; /* Выходим из while в любом случае: получили ответ или таймаут */ \
        } \
        osDelay(10); \
    } \
    if (!cmd_ok) { \
        servo_error_flag = 1; \
        sequence_completed = 1; /* Флаг для выхода из последовательности */ \
    } \
} while(0)

/* === Основная задача === */
void ServoTask(void *argument) {
    pool_mode_t saved_mode = POOL_MODE_OFF;
    uint32_t pause_start = 0;

    for (;;) {
        // Ждём запроса
        if (!backwash_req) {
            osDelay(50);
            continue;
        }

        backwash_req = 0;
        servo_error_flag = 0;
        servo_sequence_active = 1;

        // Сохраняем режим и "замораживаем" управление
        if (osMutexAcquire(MutexStateHandle, pdMS_TO_TICKS(100)) == osOK) {
            saved_mode = g_pool_state.mode;
            g_pool_state.mode = POOL_MODE_OFF;
            osMutexRelease(MutexStateHandle);
        }

        Servo_SetAllEquipmentOff();
        osDelay(100);

        uint8_t sequence_completed = 0; /* Флаг завершения/ошибки последовательности */

        /* === ШАГ 1: servo_rucksp === */ 
        if (!sequence_completed) {
            WAIT_SERVO_OK("servo_rucksp\r\n", 14);
            
            if (!sequence_completed) {
                // Пауза 30 сек, насосы ВКЛ
                Servo_SetPumpsOnlyOn();
                pause_start = osKernelGetTickCount();
                while (osKernelGetTickCount() - pause_start < PAUSE_RUCKSP_MS) {
                    osDelay(50);
                }
            }
        }

        /* === ШАГ 2: servo_klarsp === */
        if (!sequence_completed) {
            Servo_SetAllEquipmentOff();
            osDelay(100);
            WAIT_SERVO_OK("servo_klarsp\r\n", 14);
            
            if (!sequence_completed) {
                // Пауза 50 сек, насосы ВКЛ
                Servo_SetPumpsOnlyOn();
                pause_start = osKernelGetTickCount();
                while (osKernelGetTickCount() - pause_start < PAUSE_KLARSP_MS) {
                    osDelay(50);
                }
            }
        }

        /* === ШАГ 3: servo_filtern === */
        if (!sequence_completed) {
            Servo_SetAllEquipmentOff();
            osDelay(100);
            WAIT_SERVO_OK("servo_filtern\r\n", 15);
        }

        /* === Завершение последовательности === */
        Servo_SetAllEquipmentOff();

        // Восстанавливаем режим
        if (osMutexAcquire(MutexStateHandle, pdMS_TO_TICKS(100)) == osOK) {
            g_pool_state.mode = saved_mode;
            save_pool_state_to_flash();
            isNeedToRefresh = 1;
            osMutexRelease(MutexStateHandle);
        }

        servo_sequence_active = 0;
        osDelay(200);
    }
}