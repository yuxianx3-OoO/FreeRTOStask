/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "tim.h"
#include "adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    uint16_t adc_val;
    float voltage;
    uint32_t timestamp;
} ADC_Data_t;
volatile uint32_t g_sentCount = 0;
volatile uint32_t g_processedCount = 0;
volatile uint32_t g_queueFullCount = 0;
volatile uint32_t g_adcErrorCount = 0;
volatile uint32_t g_lastVoltageMv = 0;
volatile uint32_t g_uartTxErrorCount = 0;
#define EVT_DATA_READY  (1U << 0)
#define EVT_ALARM       (1U << 1)
#define EVT_SYSTEM_READY (1U << 2)
// 初始阈值 1650mV，后续按传感器实测值调整
#define ALARM_THRESHOLD_MV 1650U
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for SampleTask */
osThreadId_t SampleTaskHandle;
const osThreadAttr_t SampleTask_attributes = {
  .name = "SampleTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ProcessTask */
osThreadId_t ProcessTaskHandle;
const osThreadAttr_t ProcessTask_attributes = {
  .name = "ProcessTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for LedTask */
osThreadId_t LedTaskHandle;
const osThreadAttr_t LedTask_attributes = {
  .name = "LedTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for KeyTask */
osThreadId_t KeyTaskHandle;
const osThreadAttr_t KeyTask_attributes = {
  .name = "KeyTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for adcDataQueue */
osMessageQueueId_t adcDataQueueHandle;
const osMessageQueueAttr_t adcDataQueue_attributes = {
  .name = "adcDataQueue"
};
/* Definitions for ledCmdQueue */
osMessageQueueId_t ledCmdQueueHandle;
const osMessageQueueAttr_t ledCmdQueue_attributes = {
  .name = "ledCmdQueue"
};
/* Definitions for uartMutex */
osMutexId_t uartMutexHandle;
const osMutexAttr_t uartMutex_attributes = {
  .name = "uartMutex"
};
/* Definitions for keySem */
osSemaphoreId_t keySemHandle;
const osSemaphoreAttr_t keySem_attributes = {
  .name = "keySem"
};
/* Definitions for timSem */
osSemaphoreId_t timSemHandle;
const osSemaphoreAttr_t timSem_attributes = {
  .name = "timSem"
};
/* Definitions for sensorEvent */
osEventFlagsId_t sensorEventHandle;
const osEventFlagsAttr_t sensorEvent_attributes = {
  .name = "sensorEvent"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void App_Log(const char *tag, uint32_t value);

/* USER CODE END FunctionPrototypes */

void StartSampleTask(void *argument);
void StartProcessTask(void *argument);
void StartLedTask(void *argument);
void StartKeyTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of uartMutex */
  uartMutexHandle = osMutexNew(&uartMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of keySem */
  keySemHandle = osSemaphoreNew(1, 0, &keySem_attributes);

  /* creation of timSem */
  timSemHandle = osSemaphoreNew(5, 0, &timSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
 
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of adcDataQueue */
  adcDataQueueHandle = osMessageQueueNew (8, 12, &adcDataQueue_attributes);

  /* creation of ledCmdQueue */
  ledCmdQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &ledCmdQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of SampleTask */
  SampleTaskHandle = osThreadNew(StartSampleTask, NULL, &SampleTask_attributes);

  /* creation of ProcessTask */
  ProcessTaskHandle = osThreadNew(StartProcessTask, NULL, &ProcessTask_attributes);

  /* creation of LedTask */
  LedTaskHandle = osThreadNew(StartLedTask, NULL, &LedTask_attributes);

  /* creation of KeyTask */
  KeyTaskHandle = osThreadNew(StartKeyTask, NULL, &KeyTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of sensorEvent */
  sensorEventHandle = osEventFlagsNew(&sensorEvent_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartSampleTask */
/**
  * @brief  Function implementing the SampleTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSampleTask */
void StartSampleTask(void *argument)
{
  /* USER CODE BEGIN StartSampleTask */
  
  ADC_Data_t data;
  // STM32F1 ADC 启动前进行一次校准
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
      Error_Handler();
  }

  // TIM2 只启动一次
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
      Error_Handler();
  }
  /* Infinite loop */
  for(;;)
  {
    if (osSemaphoreAcquire(
                timSemHandle,
                osWaitForever) == osOK)
    {
        if (HAL_ADC_Start(&hadc1) == HAL_OK)
        {
            if (HAL_ADC_PollForConversion(
                    &hadc1, 20) == HAL_OK)//（轮询）函数，专门用来等待 ADC 硬件把模拟信号转换成数字信号
            {
                data.adc_val =
                    (uint16_t)HAL_ADC_GetValue(&hadc1);

                data.voltage =
                    data.adc_val * 3.3f / 4095.0f;

                data.timestamp =
                    osKernelGetTickCount();

                if (osMessageQueuePut(
                        adcDataQueueHandle,
                        &data,
                        0,
                        0) == osOK)
                {
                    g_sentCount++;
                    osEventFlagsSet(sensorEventHandle, EVT_DATA_READY);
                }
                else
                {
                    g_queueFullCount++;
                }
            }
            else
            {
                g_adcErrorCount++;
            }

            HAL_ADC_Stop(&hadc1);
        }
        else
        {
            g_adcErrorCount++;
        }
    }

  }
  /* USER CODE END StartSampleTask */
}

/* USER CODE BEGIN Header_StartProcessTask */
/**
* @brief Function implementing the ProcessTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartProcessTask */
void StartProcessTask(void *argument)
{
  /* USER CODE BEGIN StartProcessTask */
  uint32_t flags;

  ADC_Data_t rxData;
  
  /* Infinite loop */
  for(;;)
  {
    flags = osEventFlagsWait(
            sensorEventHandle,
            EVT_DATA_READY,
            osFlagsWaitAny,
            osWaitForever);

        if ((flags & osFlagsError) != 0U)
        {
            continue;
        }

        // 2. 一次性处理队列中现有的所有数据
        while (osMessageQueueGet(
                   adcDataQueueHandle,
                   &rxData,
                   NULL,
                   0) == osOK)
        {
            g_processedCount++;

            g_lastVoltageMv =
                (uint32_t)rxData.adc_val
                * 3300U / 4095U;
            App_Log("ADC_mV", g_lastVoltageMv);   
            // 3. 判断是否超过报警阈值
            if (g_lastVoltageMv > ALARM_THRESHOLD_MV)
            {
                osEventFlagsSet(
                    sensorEventHandle,
                    EVT_ALARM);
            }
            else
            {
                osEventFlagsClear(
                    sensorEventHandle,
                    EVT_ALARM);
            }
        }
  }
  /* USER CODE END StartProcessTask */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
* @brief Function implementing the LedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedTask */
void StartLedTask(void *argument)
{
  /* USER CODE BEGIN StartLedTask */
  uint32_t flags;

  uint8_t ledState;
  
  uint32_t lastBlink = osKernelGetTickCount();

  /* Infinite loop */
  for(;;)
  {
    if (osMessageQueueGet(
            ledCmdQueueHandle,
            &ledState,
            NULL,
            50) == osOK)
    {
        // PC13：低电平点亮
        HAL_GPIO_WritePin(
            LED_GPIO_Port,
            LED_Pin,
            ledState ?
                GPIO_PIN_RESET : GPIO_PIN_SET);
    }

    // 2. 查询报警状态
    flags = osEventFlagsGet(sensorEventHandle);

    if ((flags & osFlagsError) == 0U &&
        (flags & EVT_ALARM) != 0U)
    {
        uint32_t now = osKernelGetTickCount();

        // 每 100 tick 翻转一次
        if ((uint32_t)(now - lastBlink) >= 100U)
        {
            lastBlink = now;

            HAL_GPIO_TogglePin(
                GPIOB,
                GPIO_PIN_12);
        }
    }
    else
    {
        // 无报警时 PB12 熄灭
        HAL_GPIO_WritePin(
            GPIOB,
            GPIO_PIN_12,
            GPIO_PIN_RESET);

        lastBlink = osKernelGetTickCount();
    }
  }
  /* USER CODE END StartLedTask */
}

/* USER CODE BEGIN Header_StartKeyTask */
/**
* @brief Function implementing the KeyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKeyTask */
void StartKeyTask(void *argument)
{
  /* USER CODE BEGIN StartKeyTask */

  uint8_t keyState=0;
  
  /* Infinite loop */
  for(;;)
  {

    if (osSemaphoreAcquire(
            keySemHandle,
            osWaitForever) == osOK)
    {
      osDelay(20);//消抖延时
      if(HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)==GPIO_PIN_RESET)
      {
        keyState = !keyState; // 按下
        
        if(osMessageQueuePut(
            ledCmdQueueHandle,
            &keyState,
            0,
            0) == osOK)
        {
            App_Log("KEY_STATE", keyState);
        }
      }

    }  
  }
  /* USER CODE END StartKeyTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void App_Log(const char *tag, uint32_t value)
{
    char buf[96];

    int len = snprintf(
        buf,
        sizeof(buf),
        "[%s] value=%lu\r\n",
        tag,
        (unsigned long)value
    );

    if (len <= 0)
        return;

    if (len >= (int)sizeof(buf))
        len = (int)sizeof(buf) - 1;

    if (osMutexAcquire(uartMutexHandle,
                       osWaitForever) == osOK)
    {
        if (HAL_UART_Transmit(
                &huart1,
                (uint8_t *)buf,
                (uint16_t)len,
                100) != HAL_OK)
        {
            g_uartTxErrorCount++;
        }

        osMutexRelease(uartMutexHandle);
    }
}



/* USER CODE END Application */

