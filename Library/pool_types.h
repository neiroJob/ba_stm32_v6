#ifndef POOL_TYPES_H
#define POOL_TYPES_H

#include "main.h"
/* === Режимы работы === */
typedef enum {
    POOL_MODE_OFF = 0,
    POOL_MODE_AUTO = 5,
    POOL_MODE_MANUAL = 6
} pool_mode_t;

/* === dwin2: версия layout'а pool_state_t для безопасной миграции flash ===
 * load_pool_state_from_flash() делает СЫРОЙ memcpy этой структуры из
 * фиксированного адреса flash (POOL_STATE_FLASH_ADDR). При обычной заливке
 * прошивки через Keil (Download, без полного стирания чипа) эта flash-
 * страница НЕ стирается — в ней остаются байты, записанные ЕЩЁ СТАРОЙ
 * версией прошивки. Если структуру расширили (добавили поля в конец, как
 * main_display_enabled/remote_display_enabled в Шаге 2) — эти новые поля
 * при memcpy заполнятся не гарантированным нулём/единицей, а произвольным
 * "мусором" из старой flash-страницы. Ровно это и вызвало баг "оба DWIN-
 * канала внезапно перестали реагировать после апдейта прошивки" (мусор в
 * *_display_enabled случайно оказался равен 0 => DWIN_Channel_IsEnabled()
 * решила, что оба экрана выключены).
 * Решение: version-барьер. struct_version — ПЕРВОЕ поле структуры.
 * load_pool_state_from_flash() после memcpy сверяет его с POOL_STATE_VERSION
 * (константа этой прошивки) — при несовпадении (или после chip erase)
 * структура принудительно переинициализируется чистыми дефолтами
 * (init_pool_state()+save), а не доверяет частично valid/частично мусорным
 * байтам. Правило на будущее: КАЖДЫЙ раз, когда меняется состав/порядок
 * полей pool_state_t, нужно увеличивать POOL_STATE_VERSION на 1. */
#define POOL_STATE_VERSION 2

/* === Состояние системы === */
typedef struct {
    uint16_t struct_version; // см. POOL_STATE_VERSION выше; ДОЛЖНО быть первым полем
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

/* === usart_error: служебная псевдо-команда "пришли состояние заново" ===
 * НЕ соответствует никакому реальному виджету ни на одном DWIN-экране —
 * используется только между прошивками. Плата-посредник шлёт её на этот
 * адрес (со сдвигом +0x1000, как и всё остальное на канале dwin_remote)
 * после своего старта/переподключения — см. DwinRemoteBridge_Process() в
 * ba_stm32_dwin_eth. Специально выбран далеко от занятого диапазона
 * 0x5000-0x5901, чтобы не столкнуться с новыми реальными VP в будущем.
 * ВАЖНО: значение должно совпадать с DWIN_RESYNC_REQUEST_ADDR в
 * Library/dwin_remote_bridge.h платы-посредника! */
#define DWIN_ADDR_RESYNC_REQUEST 0x5FFE

/* === dwin2: смещение VP-адресов канала выносного экрана (USART1) === */
// Выносной экран физически видит те же VP, что и главный (0x5000...), но на
// проводе между платами адрес идёт со смещением +0x1000 (0x5000 -> 0x6000),
// чтобы отличать "команду от выносного экрана" от "команды от главного" ещё
// до разбора кадра. См. dwin_channel_t.addr_offset в Library/dwin.h и
// DWIN_Channel_Init(&dwin_remote, &huart1, DWIN_REMOTE_ADDR_OFFSET, ...) в main.c.
#define DWIN_REMOTE_ADDR_OFFSET 0x1000

/* === usart_error: период фолбэк-ресинка экранов === */
// Помимо мгновенного ресинка по факту обнаруженного сбоя канала
// (dwin_channel_t.needs_resync, см. dwin.h) — раз в этот период ВСЕ
// "настроечные" поля принудительно рассылаются заново на оба канала,
// даже если основной блок не видит явной ошибки на СВОЁМ канале. Это
// подстраховка от сбоев, которые происходят не на связи "основной блок -
// плата-посредник", а глубже — например, между самой платой-посредником и
// её локальным дисплеем (такое основной блок обнаружить не может, у него
// свой USART1 при этом выглядит полностью исправным).
#define DWIN_FULL_RESYNC_PERIOD_MS 60000

/* === dwin2: тайм-аут ожидания ACK от сервера (isNeedToRefresh) ===
 * Баг из практики: пользователь меняет tempSet на СЕРВЕРЕ, устройство очень
 * долго (пока не перезагрузят/пока само не "рассосётся") не применяет
 * изменение — при этом mode/light применяются нормально в других случаях.
 * Причина: isNeedToRefresh (наш, публикуемый) — это флаг "жду подтверждения
 * своего локального изменения с экрана" (см. handle_dwin_command() и
 * parse_and_apply_json_command() в main.c). Пока он равен 1, ЛЮБЫЕ
 * "тихие" изменения от сервера (включая совсем не связанный с исходным
 * локальным изменением tempSet) игнорируются — так задумано, чтобы не
 * откатить свежее нажатие на экране устаревшим снимком с сервера. Но если
 * ACK от сервера на исходное локальное изменение потерян (обрыв связи,
 * реконнект NT1-M со сбросом буфера, падение сервера) — флаг остаётся 1
 * НАВСЕГДА, и устройство перестаёт принимать вообще любые независимые
 * изменения с сервера до перезагрузки.
 * Решение: если ждём ACK дольше этого тайм-аута — считаем его потерянным,
 * сбрасываем isNeedToRefresh сами и возвращаемся к обычной работе. Это не
 * теряет наше локальное изменение (StartMqttWrite и так публикует текущее
 * состояние КАЖДЫЙ цикл независимо от флага) — просто перестаём его ждать. */
#define ISNEEDTOREFRESH_ACK_TIMEOUT_MS 15000

/*----Flash память----*/
// Адрес эмулированной EEPROM (последняя страница flash: 64 КБ STM32F103C8T6 ->
// 0x08000000 + 0x10000 = 0x08010000, но безопаснее использовать 0x0800FC00)
#define POOL_STATE_FLASH_ADDR ((uint32_t *)0x0800F800)
#define TEMP_QUEUE_WAIT_MS 350 	//Время захвата очереди в GlobalTask

#endif /* POOL_TYPES_H */
