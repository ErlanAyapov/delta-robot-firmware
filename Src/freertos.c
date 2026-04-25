/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "motor.h"
#include "rs485.h"
#include "queue.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LIMIT_SWITCH_BEEP_MS   250U
#define LIMIT_SWITCH_BEEP_DEBOUNCE_MS  80U
#define BUZZER_QUEUE_LEN       8U
#define BUZZER_ON_STATE        GPIO_PIN_SET
#define BUZZER_OFF_STATE       GPIO_PIN_RESET
#define BUZZER_HALF_PERIOD_MS  1U
#define BUZZER_DC_BURST_MS     100U
#define KEY_DEBOUNCE_MS        25U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static QueueHandle_t buzzerQueue = NULL;
static uint8_t prevLimitSwitches[4] = {1, 1, 1, 1};
static uint8_t limitSwitchStateReady = 0;
static TickType_t lastLimitSwitchBeepTick = 0;
static uint8_t keyStablePressed[5] = {0};
static uint8_t keyRawPressed[5] = {0};
static TickType_t keyLastChangeTick[5] = {0};

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId LimitSwitchHandle;
osThreadId ListenCommandHandle;
osThreadId BuzzerHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void Buzzer_RequestMs(uint16_t duration_ms);
static uint8_t Key_ReadRawPressed(uint8_t keyIndex);
static void Keys_Init(void);
static uint8_t Key_GetPressEvent(uint8_t keyIndex, TickType_t now);
static uint8_t AxisMovingTowardLimit(uint8_t axisIndex);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartLimitSwitch(void const * argument);
void StartListenCommand(void const * argument);
void StartBuzzer(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  buzzerQueue = xQueueCreate(BUZZER_QUEUE_LEN, sizeof(uint16_t));
  if (buzzerQueue == NULL) {
    AddLog(0x4111);
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityHigh, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of LimitSwitch */
  osThreadDef(LimitSwitch, StartLimitSwitch, osPriorityRealtime, 0, 512);
  LimitSwitchHandle = osThreadCreate(osThread(LimitSwitch), NULL);

  /* definition and creation of ListenCommand */
  osThreadDef(ListenCommand, StartListenCommand, osPriorityRealtime, 0, 512);
  ListenCommandHandle = osThreadCreate(osThread(ListenCommand), NULL);

  /* definition and creation of Buzzer */
  osThreadDef(Buzzer, StartBuzzer, osPriorityRealtime, 0, 128);
  BuzzerHandle = osThreadCreate(osThread(Buzzer), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

static uint8_t Key_ReadRawPressed(uint8_t keyIndex)
{
  switch (keyIndex)
  {
    case 0: return (KEY1_pressed() == 0) ? 1U : 0U;
    case 1: return (KEY2_pressed() == 0) ? 1U : 0U;
    case 2: return (KEY3_pressed() == 0) ? 1U : 0U;
    case 3: return (KEY4_pressed() == 0) ? 1U : 0U;
    case 4: return (KEY5_pressed() == 0) ? 1U : 0U;
    default: return 0U;
  }
}

static void Keys_Init(void)
{
  TickType_t now = xTaskGetTickCount();

  for (uint8_t i = 0; i < 5U; ++i)
  {
    keyRawPressed[i] = 0U;
    keyStablePressed[i] = 0U;
    keyLastChangeTick[i] = now;
  }
}

static uint8_t Key_GetPressEvent(uint8_t keyIndex, TickType_t now)
{
  uint8_t raw = Key_ReadRawPressed(keyIndex);

  if (raw != keyRawPressed[keyIndex])
  {
    keyRawPressed[keyIndex] = raw;
    keyLastChangeTick[keyIndex] = now;
  }

  if ((now - keyLastChangeTick[keyIndex]) >= pdMS_TO_TICKS(KEY_DEBOUNCE_MS))
  {
    if (keyStablePressed[keyIndex] != keyRawPressed[keyIndex])
    {
      keyStablePressed[keyIndex] = keyRawPressed[keyIndex];
      if (keyStablePressed[keyIndex] != 0U)
      {
        return 1U;
      }
    }
  }

  return 0U;
}

static uint8_t AxisMovingTowardLimit(uint8_t axisIndex)
{
  if (axisIndex >= 3U)
  {
    return 0U;
  }

  if ((run[axisIndex].pf == NULL) || (run[axisIndex].left == 0U))
  {
    return 0U;
  }

  /* For this machine, dir=0 is the direction that pushes into limit switches. */
  return (run[axisIndex].pf[run[axisIndex].seg].dir == 0) ? 1U : 0U;
}
/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;
	AddLog(0x1001);
  AxisCounters_Init();
  Keys_Init();
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = pdMS_TO_TICKS(1);
  /* Infinite loop */
  for(;;)
  {
    TickType_t now = xTaskGetTickCount();

    if (Key_GetPressEvent(0, now) != 0U)
    {
      is_on = 1;
      alarm = 0;
      ready = 0;
      rs_mode = 1;
      MotorEnable();

      if (CalibrationRequest())
      {
        UART_SendString("KEY1 pressed > calibration start\r\n");
        AddLog(0x1012);
      }
      else
      {
        UART_SendString("KEY1 pressed > calibration busy\r\n");
        AddLog(0x2012);
      }
    }

    if ((Key_GetPressEvent(1, now) != 0U) && is_on && !alarm && ready)
    {
      rs_mode = 1;
      time_captured = 0;
      real_time_us = 0;
      BuildProfileFromAngleSegments(angle_segments, 50, 5000);
      UART_SendString("KEY2 pressed > Counterclockwise\r\n");
      AddLog(0x1005);
    }

    if ((Key_GetPressEvent(2, now) != 0U) && is_on && !alarm && ready)
    {
      rs_mode = 1;
      MotorStart();
      rotateRevers();
      StartTrajectory();
      UART_SendString("KEY3 pressed > Clockwise\r\n");
      AddLog(0x1006);
    }

    if (Key_GetPressEvent(3, now) != 0U)
    {
      MotorDisable();
      UART_SendString("Timer 8 stopped\r\n");
      HAL_TIM_OC_Stop_IT(&htim8, TIM_CHANNEL_1);
      HAL_TIM_OC_Stop_IT(&htim8, TIM_CHANNEL_2);
      HAL_TIM_OC_Stop_IT(&htim8, TIM_CHANNEL_3);
      HAL_TIM_OC_Stop_IT(&htim8, TIM_CHANNEL_4);

      for (int i = 0; i < 4; i++)
      {
        reversingInProgress[i] = 0;
      }

      is_on = 0;
      ready = 0;
      UART_SendString("KEY4 pressed > driver deactivated\r\n");
      UART_SendString("All motors disabled\r\n");
      AddLog(0x1007);
    }

    if (Key_GetPressEvent(4, now) != 0U)
    {
      PrintLastErrors();
      UART_SendString("KEY5 pressed > display logs\r\n");
      AddLog(0x1008);
    }

    RefreshReadyFlag();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartLimitSwitch */
/**
* @brief Function implementing the LimitSwitch thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLimitSwitch */
void StartLimitSwitch(void const * argument)
{
  /* USER CODE BEGIN StartLimitSwitch */
	AddLog(0x1002);
	const TickType_t delay = pdMS_TO_TICKS(2);

  for(;;)
  { 
    CheckInsideLimitSwitches();  

    if (!limitSwitchStateReady)
    {
      for (uint8_t i = 0; i < 4; ++i)
      {
        prevLimitSwitches[i] = limitSwitches[i];
      }
      lastLimitSwitchBeepTick = xTaskGetTickCount();
      limitSwitchStateReady = 1;
    }
    else
    {
      for (uint8_t i = 0; i < 4; ++i)
      {
        if (prevLimitSwitches[i] == 1 && limitSwitches[i] == 0)
        {
          TickType_t now = xTaskGetTickCount();
          if ((now - lastLimitSwitchBeepTick) >= pdMS_TO_TICKS(LIMIT_SWITCH_BEEP_DEBOUNCE_MS))
          {
            AddLog((uint16_t)(0x1100U + i));
            Buzzer_RequestMs(LIMIT_SWITCH_BEEP_MS);
            lastLimitSwitchBeepTick = now;
          }
        }
        prevLimitSwitches[i] = limitSwitches[i];
      }
    }
     
		if (calibState == CAL_IDLE)
    {
		  for (uint8_t i = 0; i < 3; ++i)   /* motors 1-3 (index 0-2) */
      {
          /* Generic safety stop is disabled during calibration,
             otherwise release trajectory (back-off) is blocked. */
          if (limitSwitches[i] == 0 &&
              g_motorRunning[i] &&
              !reversingInProgress[i] &&
              (AxisMovingTowardLimit(i) != 0U))
          {
              MotorStopSingle(i);
              vTaskDelay(pdMS_TO_TICKS(10));
          }
      }
    }
		CalibrationTick();
    RefreshReadyFlag();
		
		vTaskDelay(delay);
  }
  /* USER CODE END StartLimitSwitch */
}

/* USER CODE BEGIN Header_StartListenCommand */
/**
* @brief Function implementing the ListenCommand thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartListenCommand */
void StartListenCommand(void const * argument)
{
  /* USER CODE BEGIN StartListenCommand */
	AddLog(0x1003);
	TickType_t xLastWakeTime = xTaskGetTickCount();
	const TickType_t xFrequency = pdMS_TO_TICKS(1); 
  /* Infinite loop */
	for(;;)
		{ 
			RefreshReadyFlag();
			RS485_USARTx_ProcessModbus();
			if (is_on && ready) {
				MotorEnable(); 
			//	display_system_info();
			}
			vTaskDelayUntil(&xLastWakeTime, xFrequency); 
		}
  /* USER CODE END StartListenCommand */
}

void StartBuzzer(void const * argument)
{
  (void)argument;
  uint16_t duration_ms = 0;
  HAL_GPIO_WritePin(Beep_GPIO_Port, Beep_Pin, BUZZER_OFF_STATE);
  AddLog(0x1111);

  for (;;)
  {
    if ((buzzerQueue != NULL) &&
        (xQueueReceive(buzzerQueue, &duration_ms, portMAX_DELAY) == pdPASS))
    {
      uint16_t elapsed_ms = 0;
      uint16_t dc_burst_ms = duration_ms;
      if (dc_burst_ms > BUZZER_DC_BURST_MS)
      {
        dc_burst_ms = BUZZER_DC_BURST_MS;
      }

      AddLog(0x1110);

      HAL_GPIO_WritePin(Beep_GPIO_Port, Beep_Pin, BUZZER_ON_STATE);
      vTaskDelay(pdMS_TO_TICKS(dc_burst_ms));
      elapsed_ms = dc_burst_ms;

      while (elapsed_ms < duration_ms)
      {
        HAL_GPIO_WritePin(Beep_GPIO_Port, Beep_Pin, BUZZER_ON_STATE);
        vTaskDelay(pdMS_TO_TICKS(BUZZER_HALF_PERIOD_MS));
        elapsed_ms = (uint16_t)(elapsed_ms + BUZZER_HALF_PERIOD_MS);

        if (elapsed_ms >= duration_ms)
        {
          break;
        }

        HAL_GPIO_WritePin(Beep_GPIO_Port, Beep_Pin, BUZZER_OFF_STATE);
        vTaskDelay(pdMS_TO_TICKS(BUZZER_HALF_PERIOD_MS));
        elapsed_ms = (uint16_t)(elapsed_ms + BUZZER_HALF_PERIOD_MS);
      }

      HAL_GPIO_WritePin(Beep_GPIO_Port, Beep_Pin, BUZZER_OFF_STATE);
    }
  }
}

static void Buzzer_RequestMs(uint16_t duration_ms)
{
  if (duration_ms == 0U)
  {
    return;
  }

  if (buzzerQueue == NULL)
  {
    AddLog(0x4111);
    return;
  }

  if (xQueueSend(buzzerQueue, &duration_ms, 0) != pdPASS)
  {
    AddLog(0x4110);
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    Motor_UpdateCompare(htim);
}
/* USER CODE END Application */
