/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "queue.h"
#include "stdio.h"
#include <string.h>
#include <stdlib.h>
#include "pool_types.h"
#include "dwin.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* Definitions for ReadNTC */
osThreadId_t ReadNTCHandle;
const osThreadAttr_t ReadNTC_attributes = {
  .name = "ReadNTC",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GlobalTask */
osThreadId_t GlobalTaskHandle;
const osThreadAttr_t GlobalTask_attributes = {
  .name = "GlobalTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for FloatTask */
osThreadId_t FloatTaskHandle;
const osThreadAttr_t FloatTask_attributes = {
  .name = "FloatTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MqttWrite */
osThreadId_t MqttWriteHandle;
const osThreadAttr_t MqttWrite_attributes = {
  .name = "MqttWrite",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TemperatureQueue */
osMessageQueueId_t TemperatureQueueHandle;
const osMessageQueueAttr_t TemperatureQueue_attributes = {
  .name = "TemperatureQueue"
};
/* Definitions for MutexState */
osMutexId_t MutexStateHandle;
const osMutexAttr_t MutexState_attributes = {
  .name = "MutexState"
};
/* Definitions for MutexFlash */
osMutexId_t MutexFlashHandle;
const osMutexAttr_t MutexFlash_attributes = {
  .name = "MutexFlash"
};
/* USER CODE BEGIN PV */
// === Для обработки поплавка ===
volatile uint8_t float_interrupt_flag = 0;
volatile uint8_t float_new_state = 0;
//Переменные для MQTT
volatile uint8_t isNeedToRefresh = 0;
// Тик HAL_GetTick(), когда isNeedToRefresh был выставлен в 1 — см.
// ISNEEDTOREFRESH_ACK_TIMEOUT_MS в Library/pool_types.h и его использование
// в parse_and_apply_json_command(): без этой метки время неизвестно, сколько
// уже ждём ACK от сервера, и тайм-аут самовосстановления не сработать.
static uint32_t isNeedToRefresh_set_tick = 0;
// === Текущее время от дисплея ===
volatile uint8_t current_hour = 0;        // Текущий час (0-23)
volatile uint8_t current_day_of_week = 1; // День недели (1=Пн, 7=Вс)
// === DWIN: канал главного экрана (USART3, смещение VP-адресов = 0) ===
// Раньше здесь была россыпь глобальных переменных под ручную стейт-машину
// разбора кадра (uart_rx_state, packet_buffer, packet_index, packet_queue...).
// Теперь весь этот стейт инкапсулирован в dwin_channel_t (Library/dwin.c/.h),
// а сам канал инициализируется в StartGlobalTask() через DWIN_Channel_Init().
// Диспетчеризация приёма — в HAL_UART_RxCpltCallback() ниже.
dwin_channel_t dwin_main;
// === DWIN: канал выносного экрана / платы-посредника (USART1, смещение
// VP-адресов = +0x1000, см. DWIN_REMOTE_ADDR_OFFSET в pool_types.h) ===
dwin_channel_t dwin_remote;
// === Для приёма команд по USART2 ===
#define USART2_RX_BUFFER_SIZE 128
uint8_t usart2_rx_byte;
char usart2_rx_buffer[USART2_RX_BUFFER_SIZE];
char usart2_cmd_copy[USART2_RX_BUFFER_SIZE];
volatile uint16_t usart2_rx_index = 0;
volatile uint8_t usart2_command_ready = 0;
pool_state_t g_pool_state;
pool_schedule_t g_schedule; // Расписание автоматического режима (см. Library/pool_types.h)
// Какой день недели сейчас редактируется на экране "Время фильтрации"
// (0=Пн ... 6=Вс, см. индексацию g_schedule.days[]). Чекбоксы часов
// (DWIN_ADDR_SCHEDULE_HOURS_LOW/HIGH) на экране ОБЩИЕ для всех дней — сам
// экран не знает, к какому дню относится присланное значение, поэтому это
// отслеживает контроллер: вкладка дня переключает эту переменную и просит
// контроллер прислать обратно на экран сохранённое состояние ИМЕННО этого
// дня (см. schedule_push_day_to_screen() ниже). По умолчанию Понедельник —
// экран открывается на нём же.
static uint8_t schedule_edit_day = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);
void StartReadNTC(void *argument);
void StartGlobalTask(void *argument);
void StartTaskFloat(void *argument);
void StartMqttWrite(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of MutexState */
  MutexStateHandle = osMutexNew(&MutexState_attributes);

  /* creation of MutexFlash */
  MutexFlashHandle = osMutexNew(&MutexFlash_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of TemperatureQueue */
  TemperatureQueueHandle = osMessageQueueNew (1, sizeof(uint8_t), &TemperatureQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ReadNTC */
  ReadNTCHandle = osThreadNew(StartReadNTC, NULL, &ReadNTC_attributes);

  /* creation of GlobalTask */
  GlobalTaskHandle = osThreadNew(StartGlobalTask, NULL, &GlobalTask_attributes);

  /* creation of FloatTask */
  FloatTaskHandle = osThreadNew(StartTaskFloat, NULL, &FloatTask_attributes);

  /* creation of MqttWrite */
  MqttWriteHandle = osThreadNew(StartMqttWrite, NULL, &MqttWrite_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ST_DOZ_Pin|NASOS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DOLIV_Pin|HEATING_Pin|LED_Pin|UFO_Pin
                          |NASOS2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ST_DOZ_Pin NASOS1_Pin */
  GPIO_InitStruct.Pin = ST_DOZ_Pin|NASOS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : POPLAVOK_Pin */
  GPIO_InitStruct.Pin = POPLAVOK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(POPLAVOK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DOLIV_Pin HEATING_Pin LED_Pin UFO_Pin
                           NASOS2_Pin */
  GPIO_InitStruct.Pin = DOLIV_Pin|HEATING_Pin|LED_Pin|UFO_Pin
                          |NASOS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief  Записываем данные на Dwin.
 * @param  argument: Адрес ячейки VP и данные
 * @retval None
 */
void write_variable(uint16_t vp_address, uint16_t data) {
  // Тонкая обёртка над универсальной DWIN-библиотекой ради обратной
  // совместимости имени/сигнатуры со всем остальным кодом main.c
  // (вызовов write_variable() по проекту много, менять их все ни к чему).
  // Сама отправка (сборка кадра 5A A5 05 82 VPH VPL DH DL и HAL_UART_Transmit)
  // теперь в DWIN_WriteVariable() — см. Library/dwin.c.
  //
  // dwin2, Шаг 2: экраны зеркалируют друг друга, поэтому write_variable()
  // теперь шлёт в ОБА канала — DWIN_WriteVariable() сама прибавит нужное
  // смещение адреса (0 для главного экрана, +0x1000 для выносного) и сама
  // же тихо пропустит отправку в канал, чей экран сейчас выключен в
  // настройках (main_display_enabled/remote_display_enabled) — см.
  // DWIN_Channel_IsEnabled() в Library/dwin.c.
  DWIN_WriteVariable(&dwin_main, vp_address, data);
  DWIN_WriteVariable(&dwin_remote, vp_address, data);
}
/**
 * @brief  Отправляет на экран сохранённое состояние чекбоксов часов ОДНОГО
 *         дня расписания (2 слова: DWIN_ADDR_SCHEDULE_HOURS_LOW/HIGH).
 *         Нужна каждый раз, когда меняется, какой день сейчас показан на
 *         экране "Время фильтрации" — сам экран не хранит семь наборов
 *         чекбоксов, у него один общий грид, который надо перерисовать под
 *         выбранный день (см. schedule_edit_day выше).
 * @param  day: индекс дня недели (0=Пн ... 6=Вс)
 * @retval None
 */
void schedule_push_day_to_screen(uint8_t day) {
  if (day >= 7) {
    return; // защита от мусорного индекса — такого дня не существует
  }
  write_variable(DWIN_ADDR_SCHEDULE_HOURS_LOW, g_schedule.days[day].hours_0_15);
  write_variable(DWIN_ADDR_SCHEDULE_HOURS_HIGH, g_schedule.days[day].hours_16_23);
}
/**
 * @brief  Проверяет, разрешена ли фильтрация в указанный день недели/час
 *         по сохранённому расписанию (см. Этап 6, применение в авто-режиме).
 * @param  day_of_week_1_7: день недели в соглашении RTC-ответа DWIN (1=Пн ... 7=Вс)
 * @param  hour: час (0-23)
 * @retval 1, если час разрешён (чекбокс отмечен); 0, если не разрешён ИЛИ
 *         входные данные некорректны (безопасное поведение по умолчанию —
 *         считать час запрещённым, а не разрешённым).
 */
uint8_t schedule_hour_allowed(uint8_t day_of_week_1_7, uint8_t hour) {
  if (day_of_week_1_7 < 1 || day_of_week_1_7 > 7 || hour > 23) {
    return 0;
  }
  const day_schedule_t *day = &g_schedule.days[day_of_week_1_7 - 1]; // 1..7 -> 0..6
  if (hour < 16) {
    return (uint8_t)((day->hours_0_15 >> hour) & 0x1);
  }
  return (uint8_t)((day->hours_16_23 >> (hour - 16)) & 0x1);
}
/**
 * @brief  Функция инициализации начальными значениями структуры.
 * @param  argument: Данные
 * @retval None
 */
void init_pool_state(void) {
  g_pool_state.mode = POOL_MODE_AUTO; 				// Автоматический режим
  g_pool_state.target_temp = 27;      				// Нагревать до 30°C
  g_pool_state.delta_target_temp = 2; 				// Гистерезис 2°C → включать при ≤28°C
  g_pool_state.filling_active = 0; 						// Долив выключен
  g_pool_state.filling_timeout_min = 10; 			// Авария, если долив 120 > 2 минут
  g_pool_state.filling_error = 0; 						// Ошибка долива, флаг
  g_pool_state.filling_svet = 0; 							// Свет в бассейне (0- выкл, 1 - вкл)
  g_pool_state.filling_heating = 0; 					// Флаг нагрев бассейне (0 - выкл, 1 - вкл)
  g_pool_state.filling_auto_time = 0; 				// Флаг времени промывки в режиме авто
  g_pool_state.filling_heating_priority = 1;	// Флаг приоритета нагрева
  g_pool_state.filling_doliv_time = 5; 				// Время задержки включения долива(сек)
	g_pool_state.count_refresh_eeprom = 0;			//Количество записи в Flash
	// dwin2: по умолчанию (первая прошивка/первый запуск) активен только
	// основной экран — выносной пользователь включает сам галочкой на
	// экране (взаимоисключающее переключение настроено в самом DGUS-проекте,
	// Synchrodata Return между VP 0x5900/0x5901 — см. обсуждение фичи).
	g_pool_state.main_display_enabled = 1;
	g_pool_state.remote_display_enabled = 0;
	// Пишем версию структуры ПОСЛЕДНЕЙ строкой — если позже кто-то добавит ещё
	// присваивания после этого места по невнимательности, version всё равно
	// останется корректно выставленной. См. POOL_STATE_VERSION в pool_types.h.
	g_pool_state.struct_version = POOL_STATE_VERSION;
}
/**
 * @brief  Функция записи данных в память.
 * @param  argument: Данные
 * @retval None
 */
void save_pool_state_to_flash(void) {
	// 1. Захватываем мьютекс. Если другая задача уже пишет, мы ждем её завершения.
  if (osMutexAcquire(MutexFlashHandle, pdMS_TO_TICKS(150)) != osOK) {
      return; 
  }
  HAL_FLASH_Unlock();
	g_pool_state.count_refresh_eeprom += 1;
	// 3. Очищаем флаги ошибок Flash перед началом операций
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

  // Стирание страницы (1 КБ)
  FLASH_EraseInitTypeDef EraseInitStruct = {0};
  uint32_t PageError = 0;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  // dwin2: 0xFFFFFC00 — выравнивание на границу СТРАНИЦЫ flash (1 КБ у
  // STM32F103C8, FLASH_PAGE_SIZE=0x400 в stm32f1xx_hal_flash_ex.h). Раньше
  // тут стояла 0xFFFFF800 (выравнивание на 2 КБ) — для ЭТОГО конкретного
  // адреса (0x0800F800, он и так кратен 2 КБ) результат совпадал, поэтому
  // баг был незаметен. Но та же маска, скопированная в
  // save_pool_schedule_to_flash() для адреса 0x0800FC00 (кратен 1 КБ, но НЕ
  // 2 КБ), схлопывала вычисленный адрес страницы обратно на 0x0800F800 —
  // то есть каждое сохранение расписания стирало страницу pool_state_t
  // вместо своей собственной. См. разбор инцидента в истории коммитов dwin2.
  EraseInitStruct.PageAddress = (uint32_t)POOL_STATE_FLASH_ADDR & 0xFFFFFC00;
  EraseInitStruct.NbPages = 1;

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
    HAL_FLASH_Lock();
		// === ОСВОБОЖДЕНИЕ МЬЮТЕКСА ===
		osMutexRelease(MutexFlashHandle);
    return; //Ошибка стирания
  }

  // Запись по словам (4 байта за раз)
  uint32_t *src = (uint32_t *)&g_pool_state;
  uint32_t *dst = POOL_STATE_FLASH_ADDR;
  uint32_t words = (sizeof(pool_state_t) + 3) / 4; // округление вверх

  for (uint32_t i = 0; i < words; i++) {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)&dst[i], src[i]);
  }

  HAL_FLASH_Lock();
	// === ОСВОБОЖДЕНИЕ МЬЮТЕКСА ===
  osMutexRelease(MutexFlashHandle);
}
/**
 * @brief  Функция загрузки данных из памяти.
 * @param  argument: Данные
 * @retval None
 */
void load_pool_state_from_flash(void) {
  // Проверяем, не стёрта ли память (все 0xFF)
  if (*(uint32_t *)POOL_STATE_FLASH_ADDR == 0xFFFFFFFF) {
    // Первая загрузка — инициализируем значениями по умолчанию
    init_pool_state();
    save_pool_state_to_flash(); // сохраняем первый раз
  } else {
    // Копируем из flash в RAM
    memcpy(&g_pool_state, (void *)POOL_STATE_FLASH_ADDR, sizeof(pool_state_t));
    // dwin2: защита от "мусора" в структуре после обновления прошивки БЕЗ
    // полного стирания чипа. Обычная заливка (Keil Download) не стирает эту
    // flash-страницу — в ней остаются байты от СТАРОЙ версии прошивки. Если
    // с тех пор поменялся состав/порядок полей pool_state_t (например,
    // Шаг 2 добавил main_display_enabled/remote_display_enabled в конец
    // структуры) — свежедобавленные поля заполнятся произвольным мусором,
    // а не гарантированным нулём/дефолтом (см. struct_version в
    // pool_types.h — там разобран конкретный инцидент). Поэтому при
    // несовпадении версии НЕ доверяем содержимому вообще и переинициализируем
    // структуру целиком чистыми дефолтами — лучше один раз сбросить
    // настройки пользователя, чем работать с непредсказуемым состоянием.
    if (g_pool_state.struct_version != POOL_STATE_VERSION) {
      init_pool_state();
      save_pool_state_to_flash();
    }
  }
}
/**
 * @brief  Функция инициализации расписания автоматического режима значениями
 *         по умолчанию (ни один час ни в один день изначально не разрешён —
 *         пользователь настраивает расписание сам на экране "Время фильтрации").
 * @retval None
 */
void init_pool_schedule(void) {
  memset(&g_schedule, 0, sizeof(g_schedule));
  g_schedule.struct_version = POOL_SCHEDULE_VERSION;
}
/**
 * @brief  Функция записи расписания автоматического режима в память.
 *         Пишет в ОТДЕЛЬНУЮ flash-страницу (POOL_SCHEDULE_FLASH_ADDR, см.
 *         Library/pool_types.h) — та же логика стирания+записи по словам,
 *         что и save_pool_state_to_flash(), но не трогает страницу pool_state_t.
 * @retval None
 */
void save_pool_schedule_to_flash(void) {
	// Захватываем тот же мьютекс, что и save_pool_state_to_flash() — операции
	// с flash в принципе нельзя выполнять параллельно с любой стороны, даже
	// если страницы разные (HAL_FLASH_Unlock/Lock общие на всю периферию).
  if (osMutexAcquire(MutexFlashHandle, pdMS_TO_TICKS(150)) != osOK) {
      return;
  }
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

  // Стирание страницы (1 КБ)
  FLASH_EraseInitTypeDef EraseInitStruct = {0};
  uint32_t PageError = 0;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = (uint32_t)POOL_SCHEDULE_FLASH_ADDR & 0xFFFFFC00; // выравнивание на 1 КБ, см. пояснение у save_pool_state_to_flash()
  EraseInitStruct.NbPages = 1;

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
    HAL_FLASH_Lock();
		osMutexRelease(MutexFlashHandle);
    return; //Ошибка стирания
  }

  // Запись по словам (4 байта за раз)
  uint32_t *src = (uint32_t *)&g_schedule;
  uint32_t *dst = POOL_SCHEDULE_FLASH_ADDR;
  uint32_t words = (sizeof(pool_schedule_t) + 3) / 4; // округление вверх

  for (uint32_t i = 0; i < words; i++) {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)&dst[i], src[i]);
  }

  HAL_FLASH_Lock();
	osMutexRelease(MutexFlashHandle);
}
/**
 * @brief  Функция загрузки расписания автоматического режима из памяти.
 *         Version-барьер — та же защита от "мусора" после неполной
 *         перезаливки прошивки, что и у load_pool_state_from_flash().
 * @retval None
 */
void load_pool_schedule_from_flash(void) {
  if (*(uint32_t *)POOL_SCHEDULE_FLASH_ADDR == 0xFFFFFFFF) {
    // Первая загрузка (страница ещё ни разу не записывалась) — дефолты
    init_pool_schedule();
    save_pool_schedule_to_flash();
  } else {
    memcpy(&g_schedule, (void *)POOL_SCHEDULE_FLASH_ADDR, sizeof(pool_schedule_t));
    if (g_schedule.struct_version != POOL_SCHEDULE_VERSION) {
      init_pool_schedule();
      save_pool_schedule_to_flash();
    }
  }
}
/**
 * @brief  Обработчик команд от дисплея DWIN
 * @param  addr: адрес переменной (например, 0x5000)
 * @param  value: значение (16 бит)
 * @retval None
 */
void handle_dwin_command(uint16_t addr, uint16_t value) {
  uint8_t changed = 0;
	// === ЗАХВАТ МЬЮТЕКСА ДО ЛЮБОГО ОБРАЩЕНИЯ К g_pool_state ===
  if (osMutexAcquire(MutexStateHandle, pdMS_TO_TICKS(50)) != osOK) {
      return; // Не удалось захватить — пропускаем команду
  }

  switch (addr) {
  case DWIN_ADDR_MODE: // 0x5000 - Режим работы
    if (value <= POOL_MODE_MANUAL && g_pool_state.mode != (pool_mode_t)value) {
      g_pool_state.mode = (pool_mode_t)value;
      changed = 1;
    }
    break;

  case DWIN_ADDR_TARGET_TEMP: // 0x5002 - Заданная температура
    if (value <= 50 && g_pool_state.target_temp != (uint8_t)value) {
      g_pool_state.target_temp = (uint8_t)value;
      changed = 1;
    }
    break;

  case DWIN_ADDR_TIMEOUT_ERR: // 0x5026 - Время наполнения бассейна, поплавок
    if (value <= 60 && g_pool_state.filling_timeout_min != (uint8_t)value) {
      g_pool_state.filling_timeout_min = (uint8_t)value;
      changed = 1;
    }
    break;
		
  case DWIN_ADDR_DOLIV_TIME: // 0x5025 - Время задержки включения долива, поплавок
    if (value <= 9 && g_pool_state.filling_doliv_time != (uint8_t)value) {
      g_pool_state.filling_doliv_time = (uint8_t)value;
      changed = 1;
    }
    break;

  case DWIN_ADDR_SVET: // 0x5050 - Нажали вкл\выкл свет в бассейне
    if (g_pool_state.filling_svet != (uint8_t)value) {
      g_pool_state.filling_svet = (uint8_t)value;
      changed = 1;
    }
    break;

  case DWIN_ADDR_HEATING_PRIORITY: // 0x5064 - Чекбокс "Приоритет нагрева"
    // true (1): в авто-режиме температура может форсировать насосы+нагрев
    // даже вне расписания (защита воды от переохлаждения важнее графика).
    // false (0): в авто-режиме нагрев/насосы работают строго по расписанию.
    // См. применение в блоке выбора режима работы (POOL_MODE_AUTO) ниже.
    if (g_pool_state.filling_heating_priority != (value != 0)) {
      g_pool_state.filling_heating_priority = (value != 0) ? 1 : 0;
      changed = 1;
    }
    break;

  case DWIN_ADDR_GISTEREZIS: // 0x5001 - Гистерезис
    if (value <= 10 && g_pool_state.delta_target_temp != (uint8_t)value) {
      g_pool_state.delta_target_temp = (uint8_t)value;
      changed = 1;
    }

    break;

  case DWIN_ADDR_FILLING_RESET: // 0x5016 - Управление доливом
    if (value != 0 && g_pool_state.filling_error) {
      g_pool_state.filling_error = 0; // Сброс флага аварии
      changed = 1;
    }
    break;
		
  case DWIN_ADDR_FLUSHING_ON: // 0x5019 - Кнопка промывки
    // Модуль сервопривода обратной промывки временно удалён (см. dwin2).
    // Вернётся позже как отдельная подсистема, управляемая VP-адресами 0x7000-0x73E7,
    // по той же общей DWIN-архитектуре (см. Library/dwin.h) — тогда сюда добавится
    // вызов нового обработчика вместо старого Servo_RequestBackwash().
    break;

  case DWIN_ADDR_MAIN_DISPLAY_EN: // 0x5900 - Чекбокс "Основной экран подключен"
    if (g_pool_state.main_display_enabled != (value != 0)) {
      g_pool_state.main_display_enabled = (value != 0) ? 1 : 0;
      changed = 1;
      // Оба экрана одновременно работать не должны (взаимоисключение уже
      // настроено на самом экране через Synchrodata Return, но дублируем
      // здесь на всякий случай — например, если DGUS-конфиг когда-нибудь
      // поменяют/собьют, или команда придёт из другого источника). Если
      // основной только что включился, а выносной всё ещё оставался
      // включён — гасим его. Само изменение попадёт на экран автоматически
      // через живое зеркалирование ниже в цикле (сравнение с last_mirrored_*).
      if (g_pool_state.main_display_enabled && g_pool_state.remote_display_enabled) {
        g_pool_state.remote_display_enabled = 0;
      }
    }
    break;

  case DWIN_ADDR_REMOTE_DISPLAY_EN: // 0x5901 - Чекбокс "Выносной экран подключен"
    if (g_pool_state.remote_display_enabled != (value != 0)) {
      g_pool_state.remote_display_enabled = (value != 0) ? 1 : 0;
      changed = 1;
      // См. пояснение в DWIN_ADDR_MAIN_DISPLAY_EN выше — то же самое, но
      // в обратную сторону.
      if (g_pool_state.remote_display_enabled && g_pool_state.main_display_enabled) {
        g_pool_state.main_display_enabled = 0;
      }
    }
    break;

  case DWIN_ADDR_RESYNC_REQUEST: // служебная команда от платы-посредника после её старта/переподключения
    // Не пользовательская настройка — не трогаем g_pool_state и НЕ ставим
    // changed (иначе бы зря дёрнули save_pool_state_to_flash()/isNeedToRefresh
    // ниже). Просто просим канал выносного экрана переслать всё состояние
    // заново на следующем проходе цикла — тем же механизмом, что уже
    // используется после сбоя UART (см. DWIN_Channel_RequestResync()).
    DWIN_Channel_RequestResync(&dwin_remote);
    break;

  // === Расписание автоматического режима (экран "Время фильтрации") ===
  // Все case ниже НЕ ставят changed — у расписания своя, отдельная от
  // pool_state_t логика сохранения (пишется в СВОЮ flash-страницу и только
  // по явному нажатию "Сохранить", см. DWIN_ADDR_SCHEDULE_SAVE ниже, Этап 5).

  case DWIN_ADDR_SCHEDULE_HOURS_LOW: // 0x5100 - часы 00-15 (битовая маска) редактируемого дня
    g_schedule.days[schedule_edit_day].hours_0_15 = value;
    break;

  case DWIN_ADDR_SCHEDULE_HOURS_HIGH: // 0x5101 - часы 16-23 (битовая маска) редактируемого дня
    g_schedule.days[schedule_edit_day].hours_16_23 = value;
    break;

  case DWIN_ADDR_SCHEDULE_DAY_MON: // 0x5102 - вкладка "Понедельник"
  case DWIN_ADDR_SCHEDULE_DAY_TUE: // 0x5103 - вкладка "Вторник"
  case DWIN_ADDR_SCHEDULE_DAY_WED: // 0x5104 - вкладка "Среда"
  case DWIN_ADDR_SCHEDULE_DAY_THU: // 0x5105 - вкладка "Четверг"
  case DWIN_ADDR_SCHEDULE_DAY_FRI: // 0x5106 - вкладка "Пятница"
  case DWIN_ADDR_SCHEDULE_DAY_SAT: // 0x5107 - вкладка "Суббота"
  case DWIN_ADDR_SCHEDULE_DAY_SUN: // 0x5108 - вкладка "Воскресенье"
    // value тут — служебная константа экрана (на практике 0x0011), не
    // полезные данные, см. комментарий у DWIN_ADDR_SCHEDULE_* в
    // Library/pool_types.h. Индекс дня — по смещению адреса от MON.
    schedule_edit_day = (uint8_t)(addr - DWIN_ADDR_SCHEDULE_DAY_MON);
    schedule_push_day_to_screen(schedule_edit_day); // показать сохранённое состояние выбранного дня
    break;

  case DWIN_ADDR_SCHEDULE_CLEAR_ALL: // 0x5060 - "Очистить всё" (текущий редактируемый день)
    g_schedule.days[schedule_edit_day].hours_0_15 = 0;
    g_schedule.days[schedule_edit_day].hours_16_23 = 0;
    schedule_push_day_to_screen(schedule_edit_day);
    break;

  case DWIN_ADDR_SCHEDULE_FILL_ALL: // 0x5061 - "Заполнить всё" (текущий редактируемый день)
    g_schedule.days[schedule_edit_day].hours_0_15 = 0xFFFF;
    g_schedule.days[schedule_edit_day].hours_16_23 = 0x00FF; // значимы только младшие 8 бит (часы 16-23)
    schedule_push_day_to_screen(schedule_edit_day);
    break;

  case DWIN_ADDR_SCHEDULE_COPY_ALL: // 0x5063 - "Копировать на каждый день"
    for (uint8_t d = 0; d < 7; d++) {
      g_schedule.days[d] = g_schedule.days[schedule_edit_day];
    }
    // Push на экран не нужен: источник копирования — сам текущий показанный
    // день, визуально ничего не меняется. Остальные 6 дней корректно
    // покажутся при следующем переключении вкладки — они уже в g_schedule.
    break;

  case DWIN_ADDR_SCHEDULE_SAVE: // 0x5062 - "Сохранить" (фиксирует ВСЕ 7 дней разом во flash)
    // К этому моменту g_schedule уже содержит актуальные данные (чекбоксы
    // часов шлют 0x5100/0x5101 на контроллер сразу при изменении, см. case
    // выше) — здесь просто переносим текущее ОЗУ-состояние в постоянную
    // память. save_pool_schedule_to_flash() сама берёт MutexFlashHandle —
    // вложенный захват (снаружи уже держим MutexStateHandle) безопасен,
    // это тот же порядок захвата мьютексов, что уже используется ниже для
    // save_pool_state_to_flash().
    save_pool_schedule_to_flash();
    break;

  default:
    // Неизвестная команда — игнорируем
    break;
  }

  // Сохраняем только при реальном изменении
  if (changed) {
    save_pool_state_to_flash();
		isNeedToRefresh = 1;
		isNeedToRefresh_set_tick = HAL_GetTick(); // см. ISNEEDTOREFRESH_ACK_TIMEOUT_MS
  }
	// === ОСВОБОЖДЕНИЕ МЬЮТЕКСА ===
  osMutexRelease(MutexStateHandle);
}
/**
  * @brief  Парсинг и применение JSON-команды из внешнего источника
  * @param  json_str: строка в формате {"mode":"manual","tempSet":38,"light":"off"}
  * @retval None
  */
void parse_and_apply_json_command(char* json_str)
{
	uint8_t Refrash = 0; //Переменная перезаписи Eeprom
    // Защищаем доступ к структуре состояния
    //if (osMutexAcquire(MutexStateHandle, pdMS_TO_TICKS(50)) != osOK) {
        //return; // Не удалось захватить мьютекс — пропускаем команду
    //}

    // === 0. Флаг "у сервера есть свежие данные" (isNeedToRefresh) ===
    // Протокол двусторонний, isNeedToRefresh в слушаемом топике имеет РОВНО
    // ОДИН смысл — это ПОДТВЕРЖДЕНИЕ (ACK) сервером НАШЕГО же изменения:
    //   1. Пользователь меняет mode/light/tempSet(/toppingUpTime/
    //      toppingUpFilling) на экране -> handle_dwin_command выставляет
    //      НАШ публикуемый isNeedToRefresh=1 (см. save_pool_state_to_flash()
    //      там же — сейчас это делается широко, при любом сохранении в
    //      flash, без сужения списка полей, чтобы новые поля в будущем
    //      автоматически подхватились).
    //   2. StartMqttWrite() отправляет это на сервер.
    //   3. Сервер применяет у себя и ставит true В СВОЁМ топике, который мы
    //      слушаем здесь — ИСКЛЮЧИТЕЛЬНО как эхо/квитанцию на п.2, никогда
    //      проактивно. Значит mode/light/tempSet в ЭТОМ сообщении заведомо
    //      совпадают с тем, что мы только что сами отправили — разбирать их
    //      незачем, а вот сбросить свой isNeedToRefresh в 0 нужно.
    //   4. Сервер у себя тоже сбрасывает isNeedToRefresh в false — цикл
    //      завершён, оба isNeedToRefresh снова false = "тихо", можно
    //      доверять содержимому топика как источнику НЕЗАВИСИМЫХ изменений
    //      (сделанных в приложении на сервере, а не с экрана).
    // Отсюда правило: mode/light/tempSet из топика применяем ТОЛЬКО когда
    // ОБА isNeedToRefresh (наш и серверный) равны false одновременно — это
    // и есть признак "сейчас ничего не в процессе синхронизации, топик
    // отражает намеренное изменение с сервера". Если наш isNeedToRefresh
    // ещё 1 (ждём ACK) — только что произошедшее локальное изменение с
    // экрана НЕЛЬЗЯ перезаписать устаревшим снимком топика, который ещё не
    // догнал это изменение (тот самый баг "перещёлкивания" при медленном
    // интернете, который раньше чинился более грубым фильтром).
    uint8_t server_has_fresh_data = 0;
    char* refresh = strstr(json_str, "\"isNeedToRefresh\":");
    if (refresh) {
        refresh += 18; // Пропускаем "\"isNeedToRefresh\":"
        while (*refresh == ' ' || *refresh == '\t' || *refresh == '\n' || *refresh == '\r') {
            refresh++;
        }
        if (strncmp(refresh, "true", 4) == 0) {
            server_has_fresh_data = 1;
        }
    }
    if (server_has_fresh_data) {
        // Механизм 1: это ACK нашего изменения — mode/light/tempSet в этом
        // сообщении заведомо равны тому, что мы сами только что отправили,
        // парсить и применять их не нужно (и опасно — см. поле "тихо
        // обновить" ниже, тут наоборот явно требуется просигналить, что
        // цикл синхронизации завершён).
        isNeedToRefresh = 0;
        return;
    }
    if (isNeedToRefresh != 0) {
        // У нас самих есть ещё не подтверждённое сервером локальное
        // изменение (ждём ACK, см. п.3 выше) — топик сейчас нельзя считать
        // авторитетным снимком, он может быть устаревшим. Ничего не трогаем,
        // дождёмся ACK на следующих сообщениях.
        //
        // dwin2: НО если ждём уже дольше ISNEEDTOREFRESH_ACK_TIMEOUT_MS —
        // ACK, скорее всего, потерян навсегда (обрыв связи, реконнект NT1-M
        // со сбросом буфера, падение сервера) — см. подробное обоснование у
        // константы в Library/pool_types.h. Не ждём вечно: сдаёмся, сбрасываем
        // свой флаг и НЕ выходим — проваливаемся ниже в Механизм 2, чтобы
        // применить актуальные данные уже из этого сообщения, а не только
        // начиная со следующего.
        if ((uint32_t)(HAL_GetTick() - isNeedToRefresh_set_tick) < ISNEEDTOREFRESH_ACK_TIMEOUT_MS) {
            return;
        }
        isNeedToRefresh = 0;
    }

    // === Механизм 2: оба isNeedToRefresh false -> "тихо" =================
    // Это независимое изменение, сделанное на сервере/в приложении (не с
    // экрана) — сравниваем с текущим состоянием и применяем разницу.
    // "Тихо обновить" означает не трогать isNeedToRefresh в этой ветке (он
    // и так уже 0 с обеих сторон) — экранам всё равно эхо шлём как обычно
    // через write_variable(), чтобы они сразу показали новое значение.

    // === 1. Парсинг поля "mode" ===
    char* mode_start = strstr(json_str, "\"mode\":\"");
    if (mode_start) {
        mode_start += 8; // Пропускаем "\"mode\":\""
        
        if (strncmp(mode_start, "off", 3) == 0) {
					if (g_pool_state.mode != POOL_MODE_OFF) {
						g_pool_state.mode = POOL_MODE_OFF;
            write_variable(DWIN_ADDR_MODE, g_pool_state.mode);
						Refrash = 1;
					}
        } 
        else if (strncmp(mode_start, "manual", 6) == 0) {
					if (g_pool_state.mode != POOL_MODE_MANUAL) {
						g_pool_state.mode = POOL_MODE_MANUAL;
            write_variable(DWIN_ADDR_MODE, g_pool_state.mode);
						Refrash = 1;
					}
        } 
        else if (strncmp(mode_start, "auto", 4) == 0) {
					if (g_pool_state.mode != POOL_MODE_AUTO) {
						g_pool_state.mode = POOL_MODE_AUTO;
            write_variable(DWIN_ADDR_MODE, g_pool_state.mode);
						Refrash = 1;
					}
        }
    }

    // === 2. Парсинг поля "light" ===
    char* light_start = strstr(json_str, "\"light\":\"");
    if (light_start) {
        light_start += 9; // Пропускаем "\"light\":\""
        
        if (strncmp(light_start, "on", 2) == 0) {
					if (g_pool_state.filling_svet != 1) {
						g_pool_state.filling_svet = 1;
            write_variable(DWIN_ADDR_SVET, g_pool_state.filling_svet);
            HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_SET); // Включаем свет
						Refrash = 1;
					}
        } 
        else if (strncmp(light_start, "off", 3) == 0) {
					if (g_pool_state.filling_svet != 0) {
						g_pool_state.filling_svet = 0;
            write_variable(DWIN_ADDR_SVET, g_pool_state.filling_svet);
            HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_RESET); // Выключаем свет
						Refrash = 1;
					}
        }
    }

    // === 3. Парсинг поля "tempSet" ===
    char* temp_start = strstr(json_str, "\"tempSet\":");
    if (temp_start) {
        temp_start += 10; // Пропускаем "\"tempSet\":"
        
        // Извлекаем число до запятой или закрывающей скобки
        uint8_t temp_value = (uint8_t)atoi(temp_start);
        if (temp_value <= 50) { // Ограничение по спецификации датчика
					if (g_pool_state.target_temp != temp_value) {
						g_pool_state.target_temp = temp_value;
            write_variable(DWIN_ADDR_TARGET_TEMP, g_pool_state.target_temp);
						Refrash = 1;
					}
        }
    }
    // === 4. Сохраняем изменения во флеш НЕМЕДЛЕННО ===
		//Сохранять нужно только при условии если параметры топиков отличаются т.е. когда топики не равны
		if (Refrash == 1) {
			save_pool_state_to_flash();
			Refrash = 0;
		}
    // Освобождаем мьютекс
    //osMutexRelease(MutexStateHandle);
}
/**
 * @brief  Обработчик прерывания приёма UART (вызывается автоматически)
 * @param  huart: указатель на структуру UART
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART3) {
    // Главный экран DWIN. Разбор кадра (стейт-машина + очередь) теперь общий
    // код в Library/dwin.c — здесь только диспетчеризация в нужный канал.
    // Сама функция перевооружает приём следующего байта в конце.
    DWIN_Channel_RxCpltFromISR(&dwin_main);
  }
	if (huart->Instance == USART2)
    {
        uint8_t data = usart2_rx_byte;
        /* Защита от переполнения */
        if (usart2_rx_index >= USART2_RX_BUFFER_SIZE - 1)
        {
            usart2_rx_index = 0;
        }
        usart2_rx_buffer[usart2_rx_index++] = data;
        /* Завершение по } */
        if (data == '}')
        {
            usart2_rx_buffer[usart2_rx_index] = '\0';
            memcpy(usart2_cmd_copy, usart2_rx_buffer, usart2_rx_index + 1);
            usart2_command_ready = 1;
            usart2_rx_index = 0;
        }
        HAL_UART_Receive_IT(&huart2, &usart2_rx_byte, 1);
    }
	if (huart->Instance == USART1) {
		// Выносной экран / плата-посредник. Тот же общий код разбора кадра,
		// что и для главного экрана — просто другой канал (dwin_remote,
		// смещение адреса +0x1000).
		DWIN_Channel_RxCpltFromISR(&dwin_remote);
	}

}
/**
 * @brief  Обработчик ошибок UART (вызывается автоматически из HAL).
 *         Framing/noise/overrun error — HAL сам останавливает приём при
 *         таких ошибках, поэтому диспетчер здесь обязателен: без него канал,
 *         на линии которого случилась ХОТЬ ОДНА наведённая помеха, замолчит
 *         навсегда (переставший обновляться dwin_main.rx_state).
 * @param  huart: указатель на структуру UART
 * @retval None
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART3) {
    DWIN_Channel_ErrorFromISR(&dwin_main);
  }
  if (huart->Instance == USART1) {
    DWIN_Channel_ErrorFromISR(&dwin_remote);
  }
  if (huart->Instance == USART2) {
    // dwin2: тот же класс сбоя, что и у DWIN-каналов выше (framing/noise/
    // overrun) — HAL сам останавливает приём при таких ошибках, а без
    // ре-армирования здесь линия JSON/MQTT-моста молча умирала бы навсегда
    // после первой же помехи (ровно тот баг, который раньше уже нашли и
    // починили для USART1/USART3 — до этой правки для USART2 обработчика
    // не было вовсе). Недособранный кадр всё равно не восстановить —
    // сбрасываем индекс приёма, чтобы следующий байт начинал новое
    // сообщение с чистого буфера, а не дописывался к обрывку старого.
    usart2_rx_index = 0;
    HAL_UART_Receive_IT(&huart2, &usart2_rx_byte, 1);
  }
}
/**
 * @brief  Запрос текущего часа у дисплея
 * @retval None
 */
void request_rtc_time(void) {
  // Команда чтения системного регистра 0x0010 (текущее время RTC экрана) —
  // это НЕ пользовательский VP, поэтому шлём как "сырой" кадр без трансляции
  // адреса (DWIN_WriteVariable тут не подходит, см. DWIN_SendRaw).
  //
  // dwin2: запрашиваем ОБА канала (главный экран и выносной через плату-
  // посредник), а не только главный — иначе при схеме "только выносной
  // экран подключен, главного нет" время вообще не обновлялось бы (тем более
  // критично, когда одновременно нет и связи с сервером через интернет, и
  // расписанию совсем не от чего брать текущий час). DWIN_SendRaw сама
  // тихо пропускает отправку в выключенный канал (main_display_enabled/
  // remote_display_enabled), так что при обычной схеме "оба экрана есть"
  // лишний трафик безвреден — ответят оба, оба раза обновим одно и то же.
  uint8_t tx_buffer[7] = {0x5A, 0xA5, 0x04, 0x83, 0x00, 0x10, 0x04};
  DWIN_SendRaw(&dwin_main, tx_buffer, sizeof(tx_buffer));
  DWIN_SendRaw(&dwin_remote, tx_buffer, sizeof(tx_buffer));
}
/**
 * @brief  Считывает и возвращает температуру в целых градусах Цельсия (0..60)
 * @retval uint8_t: температура в °C, или 255 при ошибке
 */
uint8_t read_temperature_celsius_rounded(void) {
  // 1. Запуск АЦП
  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
    return 255; // ошибка (255 = недопустимое значение для 0..60)

  uint32_t adc_val = HAL_ADC_GetValue(&hadc1); // 0..4095

  // 2. Напряжение на входе АЦП
  float voltage = (float)adc_val * 3.3f / 4095.0f;

  // 3. Проверка корректности напряжения
  if (voltage <= 0.0f || voltage >= 3.3f)
    return 255;

  // 4. Расчёт сопротивления NTC (ваша схема: R_ref сверху, NTC к GND)
  float R_ntc = 10000.0f * voltage / (3.3f - voltage);

  // 5. β-модель
  const float R0 = 10000.0f;        // 10 кОм при 25°C
  const float T0 = 25.0f + 273.15f; // 298.15 K
  const float BETA = 3950.0f;

  float ln_r = logf(R_ntc / R0);
  float T_inv = (1.0f / T0) + (ln_r / BETA);
  float T_kelvin = 1.0f / T_inv;
  float T_celsius = T_kelvin - 273.15f;

  // 6. Проверка диапазона
  if (T_celsius < 0.0f || T_celsius > 60.0f)
    return 255;

  // 7. Округление до ближайшего целого
  // 7.1 Поправочный коэффициент
  float Delta = -2.0f;
	//
	float corrected = T_celsius + 0.5f + Delta; // 17.6 → 18-2=16, 17.4 → 17-2=15 (с учётом коррекции)
  if (corrected < 0.0f) {
		corrected = 0.0f; 	// не даём уйти в отрицательные значения перед приведением к uint8_t
	}
	
		//
  //uint8_t temp_rounded = (uint8_t)(T_celsius + 0.5f) + Delta; // 17.6 → 18, 17.4 → 17	// Вводим поправочный коэффициент
	uint8_t temp_rounded = (uint8_t)corrected;
  return temp_rounded;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == POPLAVOK_Pin) {
    // Вариант Б: 1 = низкий уровень (нужен долив), 0 = норма
    uint8_t new_state = HAL_GPIO_ReadPin(POPLAVOK_GPIO_Port, POPLAVOK_Pin);
    float_new_state = new_state;
    float_interrupt_flag = 1; // Флаг для задачи обработки
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartReadNTC */
/**
 * @brief  Измеряем температуру
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartReadNTC */
void StartReadNTC(void *argument)
{
  /* USER CODE BEGIN 5 */
  uint8_t temp = 255;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for (;;) {
    temp = read_temperature_celsius_rounded();
    if (temp != 255 && (temp >= 4 )) // если нет ошибки
    {
      xQueueOverwrite(TemperatureQueueHandle, &temp);
    } else {
			temp = 255;
			xQueueOverwrite(TemperatureQueueHandle, &temp);
		}
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(300));
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartGlobalTask */
/**
 * @brief Основной цикл.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartGlobalTask */
void StartGlobalTask(void *argument)
{
  /* USER CODE BEGIN StartGlobalTask */
  // Переменные для управлениия логикой работы
  uint8_t pumps_running = 0; // Флаг: работают ли насосы (+ УФ + дозация)
  uint8_t heating_allowed = 0; // Флаг: разрешён ли нагрев

  // Переменные для работы с датчиком температуры
  uint8_t temp = 255; // 255 = "ещё нет валидных данных с NTC" (безопасное значение по умолчанию)
  uint8_t last_temp = 250; // флаг "ещё не выводили" температуру

  // Флаги состояний кнопок и иконок на дисплее
  uint8_t active_button_reset = 0; // Кнопка "Перезагрузить долив"(0 - не ктивна, 1 - активна)

  // === Локальный "снимок" разделяемого состояния g_pool_state ===
  // Заполняется одним захватом MutexStateHandle за проход цикла (см. ниже) и
  // используется всеми последующими операциями (иконки DWIN, GPIO, выбор режима),
  // чтобы они видели ОДНО согласованное состояние, а не рассинхронизированные
  // между несколькими отдельными критическими секциями значения.
  // Объявлены вне for(;;), поэтому если однажды не получится взять мьютекс за
  // отведённый таймаут — используются значения с предыдущего (успешного) прохода,
  // а не мусор/нули.
	uint8_t last_ico_doliv = 0xFF;
  uint8_t last_ico_filtern = 0xFF;
  uint8_t last_ico_heating = 0xFF;

  // dwin2, живое зеркалирование настроек между экранами: последнее
  // значение каждого поля, которое реально ушло в write_variable(). Пока
  // на снимке (s_*) то же самое значение — ничего не шлём; как только
  // где-то (с любого из двух экранов ИЛИ из приложения) поле поменяли —
  // на следующем проходе цикла (до ~300 мс) значение уйдёт на ОБА канала
  // (write_variable сама решит, в какой канал реально слать — см.
  // DWIN_Channel_IsEnabled: неактивный/невыключенный канал пропускается
  // мгновенно, без блокировки на HAL_UART_Transmit, так что при схемах
  // "только один экран" или "экрана вообще нет" это зеркалирование не
  // создаёт никакой задержки в цикле).
  // 0xFFFF — сентинел ("ещё ни разу не отправляли"), гарантирует, что на
  // самом первом проходе цикла будет как минимум одна рассылка каждого поля.
  uint16_t last_mirrored_mode = 0xFFFF;
  uint16_t last_mirrored_target_temp = 0xFFFF;
  uint16_t last_mirrored_gisterezis = 0xFFFF;
  uint16_t last_mirrored_svet = 0xFFFF;
  uint16_t last_mirrored_timeout_err = 0xFFFF;
  uint16_t last_mirrored_doliv_time = 0xFFFF;
  uint16_t last_mirrored_main_display_en = 0xFFFF;
  uint16_t last_mirrored_remote_display_en = 0xFFFF;
  uint16_t last_mirrored_heating_priority = 0xFFFF;

	uint8_t s_filling_active = 0;
  uint8_t s_filling_error = 0;
  uint8_t s_filling_svet = 0;

  uint8_t s_filling_heating_priority = 0;
  uint8_t s_filling_heating = 0;
	pool_mode_t s_mode = POOL_MODE_OFF;
  // dwin2: остальные поля-"настройки", которые тоже нужно живьём зеркалировать
  // на оба экрана (см. last_mirrored_* выше) — раньше они читались/писались
  // только один раз при старте задачи.
  uint8_t s_target_temp = 0;
  uint8_t s_delta_target_temp = 0;
  uint8_t s_filling_timeout_min = 0;
  uint8_t s_filling_doliv_time = 0;
  uint8_t s_main_display_enabled = 0;
  uint8_t s_remote_display_enabled = 0;

  // Тут нужно загрузить состояние системы из EEPROM
  load_pool_state_from_flash(); // ← загружаем состояние при старте
  load_pool_schedule_from_flash(); // ← загружаем расписание автоматического режима
  // Ждем пока заставка на дисплее прогрузится
  osDelay(2000);
  // Инициализация DWIN-каналов. ВАЖНО: делать это нужно ДО первого
  // write_variable() ниже — он теперь шлёт в ОБА канала сразу (см. dwin2,
  // Шаг 2), и если хотя бы один из них не инициализирован,
  // DWIN_WriteVariable() обратится к нулевому huart (dwin_remote.huart==NULL
  // до первого DWIN_Channel_Init) — крах. Сама инициализация уже запускает
  // приём по прерыванию на соответствующем UART, поэтому отдельных
  // HAL_UART_Receive_IT(...) для USART3/USART1 в коде больше нет.
  //
  // Главный экран (USART3, смещение адресов VP = 0). enabled_flag указывает
  // на pool_state — экран может быть физически не подключен (см. схему
  // инсталляции "только выносной экран"), тогда main_display_enabled=0 и
  // DWIN_WriteVariable в этот канал ничего слать не будет.
  DWIN_Channel_Init(&dwin_main, &huart3, 0, &g_pool_state.main_display_enabled);
  // Выносной экран / плата-посредник (USART1, смещение адресов VP = +0x1000).
  DWIN_Channel_Init(&dwin_remote, &huart1, DWIN_REMOTE_ADDR_OFFSET,
                     &g_pool_state.remote_display_enabled);
  // Тут проверяем нужно выключать кнопку "Перезагрузить" долив или нет
  if (g_pool_state.filling_error) {
    write_variable(DWIN_ADDR_FILLING_RESET, 0x0000); // Если есть ошибка отображаем кнопку на главном экране
    active_button_reset = 1;
  } else {
    write_variable(DWIN_ADDR_FILLING_RESET, 0x0011); // Если нет ошибки выключаем кнопку на главном экране
    active_button_reset = 0;
  }
  // Выводим все необходимые параметры при старте дисплея
  osDelay(50);
  write_variable(DWIN_ADDR_TARGET_TEMP, g_pool_state.target_temp); // Выводим заданную температуру на
                                            // главный экран delta_target_temp
	//Выводим данные на страничку настройки попловка
	osDelay(50);
  write_variable(DWIN_ADDR_TIMEOUT_ERR, g_pool_state.filling_timeout_min); // Выводим заданную температуру на
	osDelay(50);
  write_variable(DWIN_ADDR_DOLIV_TIME, g_pool_state.filling_doliv_time); // Выводим заданную температуру на
	
  osDelay(50);
  write_variable(DWIN_ADDR_MODE, g_pool_state.mode); // Выводим режим работы, меняем состояние
                                     // кнопок (Авто\Ручной\Выкл)
  osDelay(50);
  write_variable(DWIN_ADDR_GISTEREZIS, g_pool_state.delta_target_temp); // Выводим гистерезис
  osDelay(50);
  write_variable(DWIN_ADDR_SVET, g_pool_state.filling_svet); // Выводим состояние кнопки освещения бассейна
  osDelay(50);
  write_variable(DWIN_ADDR_MAIN_DISPLAY_EN, g_pool_state.main_display_enabled); // Чекбокс "Основной экран подключен"
  osDelay(50);
  write_variable(DWIN_ADDR_REMOTE_DISPLAY_EN, g_pool_state.remote_display_enabled); // Чекбокс "Выносной экран подключен"
  osDelay(50);
  write_variable(DWIN_ADDR_HEATING_PRIORITY, g_pool_state.filling_heating_priority); // Чекбокс "Приоритет нагрева"
  osDelay(50);
  schedule_push_day_to_screen(schedule_edit_day); // Расписание: чекбоксы часов дня по умолчанию (Понедельник)
  osDelay(50);
  osDelay(100);
  request_rtc_time(); // Запрос данных с rtc
  osDelay(200);
  uint32_t last_rtc_request = osKernelGetTickCount();
  // usart_error: точка отсчёта периодического фолбэк-ресинка экранов (см. ниже в цикле)
  uint32_t last_full_resync_tick = osKernelGetTickCount();

	HAL_UART_Receive_IT(&huart2, &usart2_rx_buffer[0], 1); // Запускаем приём по USART2 (внешнее управление)
  // Выводим данные долива
  // Выводим данные настройки датчика NTC
  /* Infinite loop */
  for (;;) {
    HAL_IWDG_Refresh(&hiwdg);
    //  Периодический запрос времени — см. RTC_REQUEST_PERIOD_MS в pool_types.h
    if (osKernelGetTickCount() - last_rtc_request > RTC_REQUEST_PERIOD_MS) {
      request_rtc_time();
      last_rtc_request = osKernelGetTickCount();
    }
    // === ПАССИВНЫЙ КОНТРОЛЬ СВЯЗИ DWIN-КАНАЛОВ ===
    // Без активных ping-пакетов: если приёмный автомат канала "завис" посреди
    // кадра дольше таймаута (см. DWIN_RX_STUCK_TIMEOUT_MS в dwin.c) — канал
    // мягко перезапускается сам. Здесь только регулярный вызов "тика".
    DWIN_Channel_Poll(&dwin_main, HAL_GetTick());
    DWIN_Channel_Poll(&dwin_remote, HAL_GetTick());

    // === usart_error: РЕСИНК ЭКРАНОВ ПОСЛЕ ВОССТАНОВЛЕНИЯ СВЯЗИ ===
    // Обычная diff-логика зеркалирования ниже (last_mirrored_*) шлёт на экран
    // только то, что РЕАЛЬНО изменилось с прошлого раза. Если канал сбоил
    // (обрыв/помеха на линии, или неисправность где-то внутри платы-
    // посредника) и восстановился, а пока он был недоступен, сервер или
    // другой экран поменяли значения — эта diff-логика молчит, потому что с
    // ЕЁ точки зрения ничего не изменилось (просто некому было это увидеть).
    // Экран в этом случае так и останется со старыми данными до следующего
    // ПОДЛИННОГО изменения. Чтобы так не происходило:
    //  - как только dwin_main или dwin_remote сообщают о восстановлении
    //    после сбоя (DWIN_Channel_ConsumeResyncFlag — см. dwin.c:
    //    выставляется в DWIN_Channel_ErrorFromISR и в авто-рестарте внутри
    //    DWIN_Channel_Poll), сбрасываем last_mirrored_*/last_ico_doliv/
    //    last_temp в сентинелы — на следующем проходе блок зеркалирования
    //    ниже отправит вообще ВСЁ состояние заново на оба канала;
    //  - вдобавок раз в DWIN_FULL_RESYNC_PERIOD_MS (см. pool_types.h) это же
    //    происходит безусловно — подстраховка от сбоя, который случился не
    //    на "нашем" канале USART1, а глубже (например, между платой-
    //    посредником и её локальным дисплеем), и который основной блок со
    //    своей стороны обнаружить не может в принципе.
    {
      uint8_t force_full_resync = 0;
      if (DWIN_Channel_ConsumeResyncFlag(&dwin_main)) {
        force_full_resync = 1;
      }
      if (DWIN_Channel_ConsumeResyncFlag(&dwin_remote)) {
        force_full_resync = 1;
      }
      if (osKernelGetTickCount() - last_full_resync_tick > DWIN_FULL_RESYNC_PERIOD_MS) {
        force_full_resync = 1;
        last_full_resync_tick = osKernelGetTickCount();
      }
      if (force_full_resync) {
        last_mirrored_mode = 0xFFFF;
        last_mirrored_target_temp = 0xFFFF;
        last_mirrored_gisterezis = 0xFFFF;
        last_mirrored_svet = 0xFFFF;
        last_mirrored_timeout_err = 0xFFFF;
        last_mirrored_doliv_time = 0xFFFF;
        last_mirrored_main_display_en = 0xFFFF;
        last_mirrored_remote_display_en = 0xFFFF;
        last_mirrored_heating_priority = 0xFFFF;
        last_ico_doliv = 0xFF;
        last_temp = 250; // тот же "ещё не выводили" сентинел, что и при объявлении last_temp выше
      }
    }

    // === ОБРАБОТКА ВСЕХ НАКОПИВШИХСЯ ПАКЕТОВ ГЛАВНОГО ЭКРАНА (USART3) ===
    {
      dwin_packet_t pkt;
      while (DWIN_Channel_PopPacket(&dwin_main, &pkt)) {
        // Экран выключен в настройках ("физически его нет") — просто
        // выбрасываем всё, что накопилось в очереди, ничего не применяя.
        if (!DWIN_Channel_IsEnabled(&dwin_main)) {
          continue;
        }
        // --- Ответ на запрос текущего времени (системный регистр 0x0010) ---
        // Это НЕ пользовательский VP, поэтому смещение канала (addr_offset)
        // тут не применяется — разбирается как отдельный частный случай ДО
        // общего разбора VP-команд ниже.
        // Структура ответа: 83 00 10 04 [8 байт данных]; байт 7 = день недели
        // (1-7), байт 8 = час (0-23). Пример: 5A A5 0C 83 00 10 04 1A 02 07 06 0F 2E 01 00
        if (pkt.length >= 12 && pkt.data[0] == 0x83 && pkt.data[1] == 0x00 &&
            pkt.data[2] == 0x10) {
          uint8_t day_of_week = pkt.data[7];
          uint8_t hour = pkt.data[8];
          if (day_of_week >= 1 && day_of_week <= 7) {
            current_day_of_week = day_of_week;
          }
          if (hour <= 23) {
            current_hour = hour;
          }
          continue; // служебный ответ, а не команда — под общий разбор не подпадает
        }

        // --- Обычная команда записи/чтения VP (0x82/0x83) ---
        // Адрес переводится из "проводного" домена канала в родной 0x5000-домен
        // через DWIN_Channel_ToLocalAddr(). Для USART3 (dwin_main) смещение
        // равно 0, поэтому число не меняется.
        if (pkt.length >= 5 && (pkt.data[0] == 0x82 || pkt.data[0] == 0x83)) {
          uint16_t raw_addr = (uint16_t)((pkt.data[1] << 8) | pkt.data[2]);
          uint16_t value = (uint16_t)((pkt.data[4] << 8) | pkt.data[5]);
          uint16_t addr = DWIN_Channel_ToLocalAddr(&dwin_main, raw_addr);
          handle_dwin_command(addr, value);
        }
      }
    }
    // === ОБРАБОТКА ВСЕХ НАКОПИВШИХСЯ ПАКЕТОВ ВЫНОСНОГО ЭКРАНА (USART1) ===
    // Код 1:1 повторяет структуру блока главного экрана выше — это и есть
    // переиспользование универсального разбора DWIN, о котором просили в
    // Шаге 1: единственная разница между каналами — addr_offset внутри
    // dwin_remote/dwin_main, весь остальной код идентичен. dwin2: раньше тут
    // не было разбора RTC-ответа ("время запрашивается только у главного
    // экрана") — это было ограничением, из-за которого схема "только
    // выносной экран подключен, главного нет" вообще не получала время.
    // Теперь request_rtc_time() спрашивает оба канала (см. её же), и ответ
    // от выносного экрана (через плату-посредник) тоже разбирается здесь.
    {
      dwin_packet_t pkt;
      while (DWIN_Channel_PopPacket(&dwin_remote, &pkt)) {
        // Выносной экран выключен в настройках ("физически его нет") —
        // накопленные пакеты выбрасываем, не применяя (см. main_display_enabled
        // выше — тот же принцип, отдельный флаг remote_display_enabled).
        if (!DWIN_Channel_IsEnabled(&dwin_remote)) {
          continue;
        }
        // --- Ответ на запрос текущего времени (системный регистр 0x0010) ---
        // Как и у главного экрана: это НЕ пользовательский VP, поэтому
        // addr_offset тут не применяется — тот же формат ответа. Адрес
        // 0x0010 приходит НЕсдвинутым, потому что плата-посредник теперь не
        // патчит системные регистры при трансляции (см. DWIN_PatchFrameAddress
        // в Library/dwin.c — правка на обеих платах, main и посредник).
        if (pkt.length >= 12 && pkt.data[0] == 0x83 && pkt.data[1] == 0x00 &&
            pkt.data[2] == 0x10) {
          uint8_t day_of_week = pkt.data[7];
          uint8_t hour = pkt.data[8];
          if (day_of_week >= 1 && day_of_week <= 7) {
            current_day_of_week = day_of_week;
          }
          if (hour <= 23) {
            current_hour = hour;
          }
          continue; // служебный ответ, а не команда — под общий разбор не подпадает
        }
        if (pkt.length >= 5 && (pkt.data[0] == 0x82 || pkt.data[0] == 0x83)) {
          uint16_t raw_addr = (uint16_t)((pkt.data[1] << 8) | pkt.data[2]);
          uint16_t value = (uint16_t)((pkt.data[4] << 8) | pkt.data[5]);
          uint16_t addr = DWIN_Channel_ToLocalAddr(&dwin_remote, raw_addr);
          handle_dwin_command(addr, value);
        }
      }
    }

    // === Единый снимок разделяемого состояния (ОДИН захват мьютекса вместо четырёх) ===
    // Раньше здесь было 4 отдельных osMutexAcquire/Release подряд для логически связанных
    // операций (иконка долива, иконка ошибки, свет, пересчёт нагрева), а блок выбора режима
    // работы ниже вообще читал g_pool_state.mode/filling_auto_time/filling_heating_priority
    // без какой-либо защиты (см. старый комментарий "Тут нужно сделать мьютекс!!!").
    // Проблемы старого подхода:
    //  - 4 лишних lock/unlock за один проход цикла (накладные расходы FreeRTOS);
    //  - между отдельными критическими секциями другая задача (обработчик команд DWIN/JSON)
    //    могла изменить g_pool_state, и в рамках ОДНОГО прохода цикла иконка долива,
    //    иконка ошибки и расчёт нагрева могли оказаться рассчитаны по не согласованному
    //    между собой состоянию (данные "поехали" в середине прохода);
    //  - блок выбора режима читал состояние вообще без мьютекса.
    // Теперь: один короткий захват, пересчитываем filling_heating и копируем всё нужное
    // в локальные переменные s_*, отпускаем мьютекс — а все "медленные" операции
    // (write_variable по UART, HAL_GPIO_WritePin) выполняются уже вне мьютекса,
    // на согласованном локальном снимке.
    if (osMutexAcquire(MutexStateHandle, pdMS_TO_TICKS(20)) == osOK) {
      // Пересчитываем нагрев по температуре здесь же, чтобы s_filling_heating ниже
      // был согласован с остальными полями снимка в рамках этого же прохода.
      if (temp != 255) {
        if (temp <= (g_pool_state.target_temp - g_pool_state.delta_target_temp)) {
          g_pool_state.filling_heating = 1;
        }
        if (temp >= g_pool_state.target_temp) {
          g_pool_state.filling_heating = 0;
        }
      } else {
        // Нет валидной температуры — нагрев принудительно выключен (безопасное поведение)
        g_pool_state.filling_heating = 0;
      }

      s_filling_active           = g_pool_state.filling_active;
      s_filling_error            = g_pool_state.filling_error;
      s_filling_svet             = g_pool_state.filling_svet;
      s_mode                     = g_pool_state.mode;
      s_filling_heating_priority = g_pool_state.filling_heating_priority;
      s_filling_heating          = g_pool_state.filling_heating;
      // dwin2: снимок полей-"настроек" для живого зеркалирования на оба экрана
      s_target_temp              = g_pool_state.target_temp;
      s_delta_target_temp        = g_pool_state.delta_target_temp;
      s_filling_timeout_min      = g_pool_state.filling_timeout_min;
      s_filling_doliv_time       = g_pool_state.filling_doliv_time;
      s_main_display_enabled     = g_pool_state.main_display_enabled;
      s_remote_display_enabled   = g_pool_state.remote_display_enabled;

      osMutexRelease(MutexStateHandle);
    }
    // Если мьютекс не удалось взять за 20 мс — s_* остаются от предыдущего (успешного)
    // прохода цикла (300 мс назад), это безопаснее, чем работать с нулями/мусором.

    // Иконка долива (на согласованном снимке, без мьютекса)
    if (s_filling_active) {
      write_variable(DWIN_ADDR_ICO_DOLIV, 0x0011); // Долив ВКЛ
    } else {
      write_variable(DWIN_ADDR_ICO_DOLIV, 0x0000); // Долив ВЫКЛ
    }

    // Иконка долива (на согласованном снимке, без мьютекса) — шлём по UART только при изменении
    if (s_filling_active != last_ico_doliv) {
      write_variable(DWIN_ADDR_ICO_DOLIV, s_filling_active ? 0x0011 : 0x0000); // Долив ВКЛ/ВЫКЛ
      last_ico_doliv = s_filling_active;
    }

    // Иконка ошибки и кнопка сброса
    if (s_filling_error && !active_button_reset) {
      write_variable(DWIN_ADDR_ICO_ERROR, 0x0011); // Иконка ошибки ВКЛ
      osDelay(pdMS_TO_TICKS(20));
      write_variable(DWIN_ADDR_FILLING_RESET, 0x0000); // Кнопка сброса ВИДИМА
      active_button_reset = 1;
    }
    if (!s_filling_error && active_button_reset) {
      write_variable(DWIN_ADDR_ICO_ERROR, 0x0000); // Иконка ошибки ВЫКЛ
      osDelay(pdMS_TO_TICKS(20));
      write_variable(DWIN_ADDR_FILLING_RESET, 0x0011); // Кнопка сброса СКРЫТА
      active_button_reset = 0;
    }

    // === Живое зеркалирование "настроечных" полей на оба экрана ===
    // Раньше mode/target_temp/гистерезис/свет/timeout/doliv_time/чекбоксы
    // писались на экран только один раз при старте задачи — если поменять их
    // с ОДНОГО экрана, второй (или тот же самый после ручного изменения на
    // сервере) не узнавал об этом, пока устройство не перезагружали. Теперь,
    // как и иконки выше, они сверяются с последним отправленным значением
    // каждый проход цикла (~300 мс) и досылаются при реальном изменении —
    // независимо от того, КТО их поменял: любой из двух экранов или JSON
    // с сервера (parse_and_apply_json_command трогать не стали — см. dwin2:
    // её собственный write_variable для mode/light/tempSet просто станет
    // тут избыточным на один лишний проход, это не ошибка).
    // write_variable() сама решает, в какой из двух каналов реально писать
    // (DWIN_Channel_IsEnabled) — если экрана на канале нет физически,
    // соответствующая отправка пропускается мгновенно, без задержек.
    if ((uint16_t)s_mode != last_mirrored_mode) {
      write_variable(DWIN_ADDR_MODE, s_mode);
      last_mirrored_mode = (uint16_t)s_mode;
    }
    if ((uint16_t)s_target_temp != last_mirrored_target_temp) {
      write_variable(DWIN_ADDR_TARGET_TEMP, s_target_temp);
      last_mirrored_target_temp = (uint16_t)s_target_temp;
    }
    if ((uint16_t)s_delta_target_temp != last_mirrored_gisterezis) {
      write_variable(DWIN_ADDR_GISTEREZIS, s_delta_target_temp);
      last_mirrored_gisterezis = (uint16_t)s_delta_target_temp;
    }
    if ((uint16_t)s_filling_svet != last_mirrored_svet) {
      write_variable(DWIN_ADDR_SVET, s_filling_svet);
      last_mirrored_svet = (uint16_t)s_filling_svet;
    }
    if ((uint16_t)s_filling_timeout_min != last_mirrored_timeout_err) {
      write_variable(DWIN_ADDR_TIMEOUT_ERR, s_filling_timeout_min);
      last_mirrored_timeout_err = (uint16_t)s_filling_timeout_min;
    }
    if ((uint16_t)s_filling_doliv_time != last_mirrored_doliv_time) {
      write_variable(DWIN_ADDR_DOLIV_TIME, s_filling_doliv_time);
      last_mirrored_doliv_time = (uint16_t)s_filling_doliv_time;
    }
    if ((uint16_t)s_main_display_enabled != last_mirrored_main_display_en) {
      write_variable(DWIN_ADDR_MAIN_DISPLAY_EN, s_main_display_enabled);
      last_mirrored_main_display_en = (uint16_t)s_main_display_enabled;
    }
    if ((uint16_t)s_remote_display_enabled != last_mirrored_remote_display_en) {
      write_variable(DWIN_ADDR_REMOTE_DISPLAY_EN, s_remote_display_enabled);
      last_mirrored_remote_display_en = (uint16_t)s_remote_display_enabled;
    }
    if ((uint16_t)s_filling_heating_priority != last_mirrored_heating_priority) {
      write_variable(DWIN_ADDR_HEATING_PRIORITY, s_filling_heating_priority);
      last_mirrored_heating_priority = (uint16_t)s_filling_heating_priority;
    }

    // Свет в бассейне вкл\выкл
    if (s_filling_svet) {
      HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_SET); // Включаем свет в бассейне
    } else {
      HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_RESET); // Выключаем свет в бассейне
    }

    // Выводим значения температуры с NTC на DWIN
    // ВАЖНО: НЕ используем portMAX_DELAY — очередь обновляется задачей ReadNTC раз в 300 мс,
    // и если датчик перестанет давать валидные значения (read_temperature_celsius_rounded()
    // постоянно возвращает 255 и xQueueOverwrite не вызывается), portMAX_DELAY заблокирует
    // ВЕСЬ цикл GlobalTask навсегда: перестанут обрабатываться команды DWIN, не будут
    // обновляться иконки/насосы/нагрев, и главное — перестанет вызываться HAL_IWDG_Refresh(),
    // что приведёт к неконтролируемой перезагрузке по сторожевому таймеру.
    // Таймаут TEMP_QUEUE_WAIT_MS чуть больше периода ReadNTC (300 мс), чтобы в норме
    // почти всегда получать свежее значение, но не виснуть, если данных нет.
    if (xQueueReceive(TemperatureQueueHandle, &temp, pdMS_TO_TICKS(TEMP_QUEUE_WAIT_MS)) == pdTRUE) {
      if (temp != last_temp) {
        write_variable(DWIN_ADDR_TEMP, temp);
        last_temp = temp;
      }
    }
    // Если новых данных не было (таймаут) — temp сохраняет предыдущее известное значение
    // (переменная объявлена вне цикла), поэтому логика нагрева в СЛЕДУЮЩЕМ проходе (см. выше,
    // блок под мьютексом) безопасно продолжит работать на последнем известном значении.

    // Тут основной цикл выбора режима работы — теперь на согласованном снимке s_*,
    // а не на "живом" g_pool_state без защиты, как было раньше.
    // Раньше здесь была проверка `if (!servo_sequence_active)`, которая приостанавливала
    // насосы/нагрев на время цикла обратной промывки сервопривода. Модуль сервопривода
    // временно удалён (см. dwin2) — условие снято, блок выполняется безусловно.
    {
      // --- Режим OFF ---
      if (s_mode == POOL_MODE_OFF) {
        pumps_running = 0;
        heating_allowed = 0;
      }
      // --- Ручной режим ---
      else if (s_mode == POOL_MODE_MANUAL) {
        pumps_running = 1; // Насосы/УФ/дозация ВСЕГДА включены
        heating_allowed = s_filling_heating; // Нагрев по температуре
      }
      // --- Автоматический режим ---
      // dwin2, Этап 6: расписание (g_schedule) заменило старую эвристику
      // filling_auto_time (то поле никогда фактически не выставлялось в 1
      // нигде в коде — было заготовкой именно под эту задачу).
      else if (s_mode == POOL_MODE_AUTO) {
        uint8_t schedule_allows = schedule_hour_allowed(current_day_of_week, current_hour);
        if (s_filling_heating_priority) {
          // Приоритет нагрева: температура может форсировать насосы даже
          // вне расписания — защита воды от переохлаждения важнее графика.
          pumps_running = schedule_allows || s_filling_heating;
        } else {
          // Без приоритета — строго по расписанию, без исключений по температуре.
          pumps_running = schedule_allows;
        }

        // Нагрев всегда разрешён только по температуре — а физически
        // включится только если ещё и pumps_running=1 (см. heating_running
        // ниже), поэтому и без приоритета нагрев де-факто ограничен
        // расписанием (насосов нет вне расписания -> нагреву не на чём работать).
        heating_allowed = s_filling_heating;
      }

      // === 4. Безопасное включение оборудования ===
      // НАГРЕВ МОЖЕТ РАБОТАТЬ ТОЛЬКО С НАСОСАМИ! (защита от перегрева)
      uint8_t heating_running = heating_allowed && pumps_running;

      // --- Включение насосов, УФ, станции дозации ---
      if (pumps_running) {
        HAL_GPIO_WritePin(GPIOB, NASOS2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, NASOS1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, UFO_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, ST_DOZ_Pin, GPIO_PIN_SET);
        write_variable(DWIN_ADDR_ICO_FILTERN, 0x0011); // Иконка фильтрации ВКЛ
      } else {
        HAL_GPIO_WritePin(GPIOB, NASOS2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, NASOS1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, UFO_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, ST_DOZ_Pin, GPIO_PIN_RESET);
        write_variable(DWIN_ADDR_ICO_FILTERN, 0x0000); // Иконка фильтрации ВЫКЛ
      }

      // --- Включение нагрева (ТОЛЬКО если работают насосы!) ---
      if (heating_running) {
        HAL_GPIO_WritePin(GPIOB, HEATING_Pin, GPIO_PIN_SET);
        write_variable(DWIN_ADDR_ICO_HEATING, 0x0011); // Иконка нагрева ВКЛ
      } else {
        HAL_GPIO_WritePin(GPIOB, HEATING_Pin, GPIO_PIN_RESET);
        write_variable(DWIN_ADDR_ICO_HEATING, 0x0000); // Иконка нагрева ВЫКЛ
      }
    }
		//Если пришли данные по MQTT, то парсим их
		if (usart2_command_ready)
		{
			if (osMutexAcquire(MutexStateHandle, pdMS_TO_TICKS(50)) == osOK) {
				usart2_command_ready = 0;
				parse_and_apply_json_command(usart2_cmd_copy);
				// Освобождаем мьютекс
				osMutexRelease(MutexStateHandle);
			}
		}
    osDelay(pdMS_TO_TICKS(300));
  }
  /* USER CODE END StartGlobalTask */
}

/* USER CODE BEGIN Header_StartTaskFloat */
/**
 * @brief Задача обработки поплавка и управления доливом
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskFloat */
void StartTaskFloat(void *argument)
{
  /* USER CODE BEGIN StartTaskFloat */
  uint8_t local_float_state = 0; 		// Текущее состояние поплавка: 0 = норма, 1 = низкий уровень
  uint32_t float_confirm_start = 0; // Время начала подтверждения состояния
  uint32_t filling_doliv_start = 0; // Время начала долива (тики)
	osDelay(2000);
	// Инициализация состояния поплавка ПОД МЬЮТЕКСОМ
  if (osMutexAcquire(MutexStateHandle, pdMS_TO_TICKS(50)) == osOK) {
  // === ИНИЦИАЛИЗАЦИЯ СОСТОЯНИЯ ПОПЛАВКА ПРИ СТАРТЕ ===
  local_float_state = HAL_GPIO_ReadPin(POPLAVOK_GPIO_Port, POPLAVOK_Pin);

  // Если поплавок опущен (низкий уровень) — начинаем отсчёт подтверждения
  if (local_float_state == 0 && !g_pool_state.filling_error) {
    float_confirm_start = osKernelGetTickCount();
  }
	osMutexRelease(MutexStateHandle);
  }
  /* Infinite loop */
  for (;;) {
    // === ОБРАБОТКА ПРЕРЫВАНИЯ ПОПЛАВКА ===
    if (float_interrupt_flag) {
      float_interrupt_flag = 0;
      uint8_t new_state = float_new_state; // 0 = низкий уровень, 1 = норма

      // Состояние изменилось?
      if (new_state != local_float_state) {
        local_float_state = new_state;
        float_confirm_start = osKernelGetTickCount();
      }
    }
		//Захватываем мьютекс и работаем с структурой
		if (osMutexAcquire(MutexStateHandle, pdMS_TO_TICKS(20)) == osOK) {
    // === АНТИДРЕБЕЗГ: ПОДТВЕРЖДЕНИЕ СОСТОЯНИЯ ===
			if (float_confirm_start != 0 && !g_pool_state.filling_error) {
				uint32_t elapsed = osKernelGetTickCount() - float_confirm_start;

				// Подтверждён НИЗКИЙ уровень → ВКЛЮЧАЕМ долив
				if (local_float_state == 0 &&
						elapsed >= (g_pool_state.filling_doliv_time * 1000) &&
						!g_pool_state.filling_active) {
					HAL_GPIO_WritePin(GPIOB, DOLIV_Pin, GPIO_PIN_SET);
					g_pool_state.filling_active = 1;
					filling_doliv_start = osKernelGetTickCount();
				}
				// Подтверждён НОРМАЛЬНЫЙ уровень → ВЫКЛЮЧАЕМ долив
				else if (local_float_state == 1 &&
								elapsed >= (g_pool_state.filling_doliv_time * 1000) &&
								g_pool_state.filling_active) {
								HAL_GPIO_WritePin(GPIOB, DOLIV_Pin, GPIO_PIN_RESET);
								g_pool_state.filling_active = 0;
				}

				// Сброс таймера подтверждения после обработки
				if ((local_float_state == 0 && g_pool_state.filling_active) ||
						(local_float_state == 1 && !g_pool_state.filling_active)) {
					float_confirm_start = 0;
				}
			
			}
			// === КОНТРОЛЬ ТАЙМАУТА ДОЛИВА ===
			if (g_pool_state.filling_active && !g_pool_state.filling_error) {
				uint32_t elapsed = osKernelGetTickCount() - filling_doliv_start;
				if (elapsed >= (g_pool_state.filling_timeout_min * 1000 * 60)) {
					// Авария по таймауту
					HAL_GPIO_WritePin(GPIOB, DOLIV_Pin, GPIO_PIN_RESET);
					g_pool_state.filling_active = 0;
					g_pool_state.filling_error = 1; // Устанавливаем флаг аварии
					save_pool_state_to_flash(); //Сохраняем данные о аварии 
				}
			}
		}
		osMutexRelease(MutexStateHandle);
    osDelay(100);
  }
  /* USER CODE END StartTaskFloat */
}

/* USER CODE BEGIN Header_StartMqttWrite */
/**
* @brief Function implementing the MqttWrite thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMqttWrite */
void StartMqttWrite(void *argument)
{
  /* USER CODE BEGIN StartMqttWrite */
	uint8_t temp = 0;  // Текущая температура
  char json_buffer[256];  // Буфер для формирования JSON
  int json_len;
	uint8_t local_isNeedToRefresh;  // Локальная копия флага
  /* Infinite loop */
  for(;;)
  {
		// === 1. Получаем текущую температуру из очереди (неблокирующий вызов) ===
        // Используем копию последней известной температуры, если очередь пуста
        if (osMessageQueueGet(TemperatureQueueHandle, &temp, NULL, 0) != osOK) {
            // Если очередь пуста — оставляем предыдущее значение температуры
            // (или можно установить 255 как "ошибка датчика")
        }
        
        // === 2. Защищённое чтение структуры состояния ===
        if (osMutexAcquire(MutexStateHandle, pdMS_TO_TICKS(20)) == osOK) {
					
            
          // Определяем строковое представление режима
					const char* mode_str = "off";
					switch (g_pool_state.mode) {
						case POOL_MODE_OFF:   
							mode_str = "off";
						break;
						case POOL_MODE_MANUAL:
							mode_str = "manual";
						break;
						case POOL_MODE_AUTO:  
							mode_str = "auto";
						break;
					 default:              
						mode_str = "off";
					 break;
				}
				// Сохраняем локальную копию флага для отправки
				local_isNeedToRefresh = isNeedToRefresh;
            
            // === 3. Формируем JSON-строку ===
            json_len = snprintf(json_buffer, sizeof(json_buffer),
                "{\"temp\":%d,"
                "\"tempSet\":%u,"
                "\"mode\":\"%s\","
                "\"isHeating\":%s,"
                "\"isToppingUp\":%s,"
                "\"light\":\"%s\","
                "\"isToppingUpError\":%s,"
                "\"toppingUpTime\":%u,"
                "\"toppingUpFilling\":%u,"
								"\"countRefrashEeprom\":%u,"
								"\"dayOfWeek\":%u,"
								"\"hour\":%u,"
								"\"isNeedToRefresh\":%s"
                "}\r\n",
                temp,
                g_pool_state.target_temp,
                mode_str,
                g_pool_state.filling_heating ? "true" : "false",      // Булево: true/false
								g_pool_state.filling_active ? "true" : "false",        // Булево: true/false
								g_pool_state.filling_svet ? "on" : "off",              // Строка: on/off
								g_pool_state.filling_error ? "true" : "false",         // Булево: true/false
								g_pool_state.filling_doliv_time,
								g_pool_state.filling_timeout_min,
								g_pool_state.count_refresh_eeprom,
								current_day_of_week, // 1=Пн ... 7=Вс (см. current_day_of_week в USER CODE BEGIN PV)
								current_hour,        // 0-23
								local_isNeedToRefresh ? "true" : "false"
            );
            
            // === 4. Отправляем по USART2 (неблокирующая передача) ===
            if (json_len > 0 && json_len < sizeof(json_buffer)) {
                HAL_UART_Transmit(&huart2, (uint8_t*)json_buffer, (uint16_t)json_len, 20);
            }
            // Сбрасываем флаг после формирования пакета
						//if (local_isNeedToRefresh) {
							//isNeedToRefresh = 0;
						//}
            // Освобождаем мьютекс
            osMutexRelease(MutexStateHandle);
        }
        
        // === 5. Ждём 500 мс до следующей отправки ===
        osDelay(pdMS_TO_TICKS(800));
    
  }
  /* USER CODE END StartMqttWrite */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
