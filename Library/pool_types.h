#ifndef POOL_TYPES_H
#define POOL_TYPES_H

#include "main.h"
/* === Режимы работы === */
typedef enum {
    POOL_MODE_OFF = 0,
    POOL_MODE_AUTO = 5,
    POOL_MODE_MANUAL = 6
} pool_mode_t;

/* === Состояние системы === */
typedef struct {
    pool_mode_t mode;
    uint8_t target_temp;
    uint8_t delta_target_temp;
    uint8_t filling_active;
    uint8_t filling_timeout_min;		//Авария если долив больше указанного времени, мин
    uint8_t filling_error;
    uint8_t filling_svet;
    uint8_t filling_heating;
    uint8_t filling_auto_time;
    uint8_t filling_heating_priority;
    uint8_t filling_doliv_time;
		uint16_t count_refresh_eeprom;		//Кол-во записи в EEPROM
    // === dwin2, Шаг 2: чекбоксы "экран подключен" ===
    // Настраиваются галочками на экране (см. DWIN_ADDR_MAIN_DISPLAY_EN/
    // DWIN_ADDR_REMOTE_DISPLAY_EN ниже), сохраняются во flash вместе с
    // остальным состоянием. Управляют DWIN-каналами main.c: dwin_main
    // (huart3) читает main_display_enabled, dwin_remote (huart1) читает
    // remote_display_enabled — см. DWIN_Channel_Init()/enabled_flag в
    // Library/dwin.h. Если экран выключен — канал не шлёт в него
    // (DWIN_WriteVariable) и игнорирует то, что из него принято.
    uint8_t main_display_enabled;   // 1 = основной экран (USART3) физически подключен и используется
    uint8_t remote_display_enabled; // 1 = выносной экран (USART1/плата-посредник) физически подключен и используется
} pool_state_t;
/* === Адреса DWIN (для переносимости) === */
#define DWIN_ADDR_MODE 						0x5000 // Режим работы на главном экране (0=OFF, 1=MANUAL, 2=AUTO)
#define DWIN_ADDR_TARGET_TEMP 		0x5002 // Заданная температура на главном экране
#define DWIN_ADDR_TEMP 						0x5010 // Температура на главном экране
#define DWIN_ADDR_GISTEREZIS 			0x5001 // Гистерезис хранится на странице "Температура подогрева"
#define DWIN_ADDR_FILLING_RESET 	0x5016 // Кнопка "Перезагрузить долив" на главном экране
#define DWIN_ADDR_SVET 						0x5050 // Кнопка "Подсветка бассейна" на главном экране
#define DWIN_ADDR_ICO_ERROR 			0x5015 // Иконка ошибки на главном экране
#define DWIN_ADDR_ICO_FILTERN 		0x5011 // Иконка фильтрации на главном экране
#define DWIN_ADDR_ICO_SETTINGS 		0x5014 // Иконка настройки(шестеренка) на главном экране
#define DWIN_ADDR_ICO_HEATING 		0x5013 // Иконка нагрева на главном экране
#define DWIN_ADDR_ICO_DOLIV 			0x5012 // Иконка долива на главном экране
#define DWIN_ADDR_FLUSHING_ON 		0x5023 // Кнопка "Промывки" на экране режимы работы
#define DWIN_ADDR_TIMEOUT_ERR 		0x5026 // Время наполнения бассейна до аварии, 10  минут
#define DWIN_ADDR_DOLIV_TIME 			0x5025 // Время задержки включения долива
#define DWIN_ADDR_MAIN_DISPLAY_EN 0x5900 // Чекбокс "Основной экран подключен" (main_display_enabled)
#define DWIN_ADDR_REMOTE_DISPLAY_EN 0x5901 // Чекбокс "Выносной экран подключен" (remote_display_enabled)

/* === dwin2: смещение VP-адресов канала выносного экрана (USART1) === */
// Выносной экран физически видит те же VP, что и главный (0x5000...), но на
// проводе между платами адрес идёт со смещением +0x1000 (0x5000 -> 0x6000),
// чтобы отличать "команду от выносного экрана" от "команды от главного" ещё
// до разбора кадра. См. dwin_channel_t.addr_offset в Library/dwin.h и
// DWIN_Channel_Init(&dwin_remote, &huart1, DWIN_REMOTE_ADDR_OFFSET, ...) в main.c.
#define DWIN_REMOTE_ADDR_OFFSET 0x1000

/*----Flash память----*/
// Адрес эмулированной EEPROM (последняя страница flash: 64 КБ STM32F103C8T6 ->
// 0x08000000 + 0x10000 = 0x08010000, но безопаснее использовать 0x0800FC00)
#define POOL_STATE_FLASH_ADDR ((uint32_t *)0x0800F800)
#define TEMP_QUEUE_WAIT_MS 350 	//Время захвата очереди в GlobalTask

#endif /* POOL_TYPES_H */
