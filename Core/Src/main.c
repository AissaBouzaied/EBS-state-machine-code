/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "state_machine_common.h"
#include "callback.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticTimer_t osStaticTimerDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
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
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

/* Definitions for ebs_sm_ */
osThreadId_t ebs_sm_Handle;
uint32_t ebs_sm_Buffer[ 512 ];
osStaticThreadDef_t ebs_sm_ControlBlock;
const osThreadAttr_t ebs_sm__attributes = {
  .name = "ebs_sm_",
  .cb_mem = &ebs_sm_ControlBlock,
  .cb_size = sizeof(ebs_sm_ControlBlock),
  .stack_mem = &ebs_sm_Buffer[0],
  .stack_size = sizeof(ebs_sm_Buffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for can_rx_ */
osThreadId_t can_rx_Handle;
uint32_t can_rx_Buffer[ 256 ];
osStaticThreadDef_t can_rx_ControlBlock;
const osThreadAttr_t can_rx__attributes = {
  .name = "can_rx_",
  .cb_mem = &can_rx_ControlBlock,
  .cb_size = sizeof(can_rx_ControlBlock),
  .stack_mem = &can_rx_Buffer[0],
  .stack_size = sizeof(can_rx_Buffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for can_tx_ */
osThreadId_t can_tx_Handle;
uint32_t can_tx_Buffer[ 256 ];
osStaticThreadDef_t can_tx_ControlBlock;
const osThreadAttr_t can_tx__attributes = {
  .name = "can_tx_",
  .cb_mem = &can_tx_ControlBlock,
  .cb_size = sizeof(can_tx_ControlBlock),
  .stack_mem = &can_tx_Buffer[0],
  .stack_size = sizeof(can_tx_Buffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for angle_sensors_ */
osThreadId_t angle_sensors_Handle;
uint32_t angle_sensors_Buffer[ 512 ];
osStaticThreadDef_t angle_sensors_ControlBlock;
const osThreadAttr_t angle_sensors__attributes = {
  .name = "angle_sensors_",
  .cb_mem = &angle_sensors_ControlBlock,
  .cb_size = sizeof(angle_sensors_ControlBlock),
  .stack_mem = &angle_sensors_Buffer[0],
  .stack_size = sizeof(angle_sensors_Buffer),
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for pressure_sensor */
osThreadId_t pressure_sensorHandle;
uint32_t pressure_sensorBuffer[ 256 ];
osStaticThreadDef_t pressure_sensorControlBlock;
const osThreadAttr_t pressure_sensor_attributes = {
  .name = "pressure_sensor",
  .cb_mem = &pressure_sensorControlBlock,
  .cb_size = sizeof(pressure_sensorControlBlock),
  .stack_mem = &pressure_sensorBuffer[0],
  .stack_size = sizeof(pressure_sensorBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for digital_inputs_ */
osThreadId_t digital_inputs_Handle;
uint32_t digital_inputs_Buffer[ 256 ];
osStaticThreadDef_t digital_inputs_ControlBlock;
const osThreadAttr_t digital_inputs__attributes = {
  .name = "digital_inputs_",
  .cb_mem = &digital_inputs_ControlBlock,
  .cb_size = sizeof(digital_inputs_ControlBlock),
  .stack_mem = &digital_inputs_Buffer[0],
  .stack_size = sizeof(digital_inputs_Buffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for heartbeat_ */
osThreadId_t heartbeat_Handle;
uint32_t heartbeatBuffer[ 256 ];
osStaticThreadDef_t heartbeatControlBlock;
const osThreadAttr_t heartbeat__attributes = {
  .name = "heartbeat_",
  .cb_mem = &heartbeatControlBlock,
  .cb_size = sizeof(heartbeatControlBlock),
  .stack_mem = &heartbeatBuffer[0],
  .stack_size = sizeof(heartbeatBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CANNodeWatchDog */
osThreadId_t CANNodeWatchDogHandle;
uint32_t CANNodeWatchDogBuffer[ 256 ];
osStaticThreadDef_t CANNodeWatchDogControlBlock;
const osThreadAttr_t CANNodeWatchDog_attributes = {
  .name = "CANNodeWatchDog",
  .cb_mem = &CANNodeWatchDogControlBlock,
  .cb_size = sizeof(CANNodeWatchDogControlBlock),
  .stack_mem = &CANNodeWatchDogBuffer[0],
  .stack_size = sizeof(CANNodeWatchDogBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ebs_sm_event_queue_ */
osMessageQueueId_t ebs_sm_event_queue_Handle;
uint8_t ebs_sm_event_queueBuffer[ 16 * sizeof( Event_t ) ];
osStaticMessageQDef_t ebs_sm_event_queueControlBlock;
const osMessageQueueAttr_t ebs_sm_event_queue__attributes = {
  .name = "ebs_sm_event_queue_",
  .cb_mem = &ebs_sm_event_queueControlBlock,
  .cb_size = sizeof(ebs_sm_event_queueControlBlock),
  .mq_mem = &ebs_sm_event_queueBuffer,
  .mq_size = sizeof(ebs_sm_event_queueBuffer)
};
/* Definitions for timeoutTimer */
osTimerId_t timeoutTimerHandle;
osStaticTimerDef_t timeoutTimerControlBlock;
const osTimerAttr_t timeoutTimer_attributes = {
  .name = "timeoutTimer",
  .cb_mem = &timeoutTimerControlBlock,
  .cb_size = sizeof(timeoutTimerControlBlock),
};
/* Definitions for ebs_status_can_message_mutex_ */
osMutexId_t ebs_status_can_message_mutex_Handle;
const osMutexAttr_t ebs_status_can_message_mutex__attributes = {
  .name = "ebs_status_can_message_mutex_"
};
/* Definitions for can_message_mutex_ */
osMutexId_t can_message_mutex_Handle;
const osMutexAttr_t can_message_mutex__attributes = {
  .name = "can_message_mutex_"
};
/* Definitions for angleBufferAvailableSemaphore */
osSemaphoreId_t angleBufferAvailableSemaphoreHandle;
const osSemaphoreAttr_t angleBufferAvailableSemaphore_attributes = {
  .name = "angleBufferAvailableSemaphore"
};
/* Definitions for pressureDmaCpltSemaphore */
osSemaphoreId_t pressureDmaCpltSemaphoreHandle;
const osSemaphoreAttr_t pressureDmaCpltSemaphore_attributes = {
  .name = "pressureDmaCpltSemaphore"
};
/* Definitions for angleDataAvailableSemaphore */
osSemaphoreId_t angleDataAvailableSemaphoreHandle;
const osSemaphoreAttr_t angleDataAvailableSemaphore_attributes = {
  .name = "angleDataAvailableSemaphore"
};
/* Definitions for BrakeNotActiveSemaphore */
osSemaphoreId_t BrakeNotActiveSemaphoreHandle;
osStaticSemaphoreDef_t BrakeNotActiveSemaphoreControlBlock;
const osSemaphoreAttr_t BrakeNotActiveSemaphore_attributes = {
  .name = "BrakeNotActiveSemaphore",
  .cb_mem = &BrakeNotActiveSemaphoreControlBlock,
  .cb_size = sizeof(BrakeNotActiveSemaphoreControlBlock),
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM13_Init(void);
void taskEbsStateMachine(void *argument);
void taskCanRx(void *argument);
void taskCanTx(void *argument);
void taskAngleSensor(void *argument);
void taskPressureSensor(void *argument);
void taskDigitalInputs(void *argument);
void taskHeartbeat(void *argument);
void taskCANNodeWatchDog(void *argument);
void timeoutCallback(void *argument);

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CAN1_Init();
  MX_TIM7_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_TIM1_Init();
  MX_TIM14_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of ebs_status_can_message_mutex_ */
  ebs_status_can_message_mutex_Handle = osMutexNew(&ebs_status_can_message_mutex__attributes);

  /* creation of can_message_mutex_ */
  can_message_mutex_Handle = osMutexNew(&can_message_mutex__attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of angleBufferAvailableSemaphore */
  angleBufferAvailableSemaphoreHandle = osSemaphoreNew(1, 0, &angleBufferAvailableSemaphore_attributes);

  /* creation of pressureDmaCpltSemaphore */
  pressureDmaCpltSemaphoreHandle = osSemaphoreNew(1, 0, &pressureDmaCpltSemaphore_attributes);

  /* creation of angleDataAvailableSemaphore */
  angleDataAvailableSemaphoreHandle = osSemaphoreNew(1, 0, &angleDataAvailableSemaphore_attributes);

  /* creation of BrakeNotActiveSemaphore */
  BrakeNotActiveSemaphoreHandle = osSemaphoreNew(1, 1, &BrakeNotActiveSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  osSemaphoreRelease(angleDataAvailableSemaphoreHandle);
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of timeoutTimer */
  timeoutTimerHandle = osTimerNew(timeoutCallback, osTimerOnce, NULL, &timeoutTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of ebs_sm_event_queue_ */
  ebs_sm_event_queue_Handle = osMessageQueueNew (16, sizeof(Event_t), &ebs_sm_event_queue__attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ebs_sm_ */
  ebs_sm_Handle = osThreadNew(taskEbsStateMachine, NULL, &ebs_sm__attributes);

  /* creation of can_rx_ */
  can_rx_Handle = osThreadNew(taskCanRx, NULL, &can_rx__attributes);

  /* creation of can_tx_ */
  can_tx_Handle = osThreadNew(taskCanTx, NULL, &can_tx__attributes);

  /* creation of angle_sensors_ */
  angle_sensors_Handle = osThreadNew(taskAngleSensor, NULL, &angle_sensors__attributes);

  /* creation of pressure_sensor */
  pressure_sensorHandle = osThreadNew(taskPressureSensor, NULL, &pressure_sensor_attributes);

  /* creation of digital_inputs_ */
  digital_inputs_Handle = osThreadNew(taskDigitalInputs, NULL, &digital_inputs__attributes);

  /* creation of heartbeat_ */
  heartbeat_Handle = osThreadNew(taskHeartbeat, NULL, &heartbeat__attributes);

  /* creation of CANNodeWatchDog */
  CANNodeWatchDogHandle = osThreadNew(taskCANNodeWatchDog, NULL, &CANNodeWatchDog_attributes);

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
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 4;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1343;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 62499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 31250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */




  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 2;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 55999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 167;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 49999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 335;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 50000-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 42000-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 200 - 1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 42000-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 200-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 42000-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 200-1;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 42000-1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 200-1;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 42000-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, RELEASE_BRAKE_1_Pin|RELEASE_BRAKE_2_Pin|SYSTEM_OK_LED_Pin|SYSTEM_FAULT_LED_Pin
                          |GENERAL_LED_Pin|CLOSE_SDC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(WATCHDOG_GPIO_Port, WATCHDOG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SDC_TO_REAR_Pin SDC_FROM_REAR_Pin WATCHDOG_READY_Pin */
  GPIO_InitStruct.Pin = SDC_TO_REAR_Pin|SDC_FROM_REAR_Pin|WATCHDOG_READY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : RELEASE_BRAKE_1_Pin RELEASE_BRAKE_2_Pin SYSTEM_OK_LED_Pin SYSTEM_FAULT_LED_Pin
                           GENERAL_LED_Pin CLOSE_SDC_Pin */
  GPIO_InitStruct.Pin = RELEASE_BRAKE_1_Pin|RELEASE_BRAKE_2_Pin|SYSTEM_OK_LED_Pin|SYSTEM_FAULT_LED_Pin
                          |GENERAL_LED_Pin|CLOSE_SDC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PS1_Pin PS2_Pin */
  GPIO_InitStruct.Pin = PS1_Pin|PS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : WATCHDOG_Pin */
  GPIO_InitStruct.Pin = WATCHDOG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WATCHDOG_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_taskEbsStateMachine */
/**
  * @brief  Function implementing the ebs_sm_ thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_taskEbsStateMachine */
__weak void taskEbsStateMachine(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_taskCanRx */
/**
* @brief Function implementing the can_rx_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskCanRx */
__weak void taskCanRx(void *argument)
{
  /* USER CODE BEGIN taskCanRx */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END taskCanRx */
}

/* USER CODE BEGIN Header_taskCanTx */
/**
* @brief Function implementing the can_tx_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskCanTx */
__weak void taskCanTx(void *argument)
{
  /* USER CODE BEGIN taskCanTx */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END taskCanTx */
}

/* USER CODE BEGIN Header_taskAngleSensor */
/**
* @brief Function implementing the angle_sensors_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskAngleSensor */
__weak void taskAngleSensor(void *argument)
{
  /* USER CODE BEGIN taskAngleSensor */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END taskAngleSensor */
}

/* USER CODE BEGIN Header_taskPressureSensor */
/**
* @brief Function implementing the pressure_sensor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskPressureSensor */
__weak void taskPressureSensor(void *argument)
{
  /* USER CODE BEGIN taskPressureSensor */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END taskPressureSensor */
}

/* USER CODE BEGIN Header_taskDigitalInputs */
/**
* @brief Function implementing the digital_inputs_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskDigitalInputs */
__weak void taskDigitalInputs(void *argument)
{
  /* USER CODE BEGIN taskDigitalInputs */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END taskDigitalInputs */
}

/* USER CODE BEGIN Header_taskHeartbeat */
/**
* @brief Function implementing the heartbeat_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskHeartbeat */
__weak void taskHeartbeat(void *argument)
{
  /* USER CODE BEGIN taskHeartbeat */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END taskHeartbeat */
}

/* USER CODE BEGIN Header_taskCANNodeWatchDog */
/**
* @brief Function implementing the CANNodeWatchDog thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_taskCANNodeWatchDog */
__weak void taskCANNodeWatchDog(void *argument)
{
  /* USER CODE BEGIN taskCANNodeWatchDog */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END taskCANNodeWatchDog */
}

/* timeoutCallback function */
__weak void timeoutCallback(void *argument)
{
  /* USER CODE BEGIN timeoutCallback */

  /* USER CODE END timeoutCallback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	Event_t event = { 0 };

	if (htim->Instance == TIM7) {
		event.eEventType = eWatchdogEvent;
		osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone, 0);
	}

	if (htim->Instance == TIM10) {
		if(TIM10->CNT >0){
			event.eEventType = eASSystemNotOkEvent;
			osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone, 0);
			}
		}

	if (htim->Instance == TIM11) {
			event.eEventType = eRESNotOkEvent;
			osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone, 0);
		}

	if (htim->Instance == TIM12) {
			event.eEventType = eVESCNotOkEvent;
			osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone, 0);
		}

	if (htim->Instance == TIM13) {
			event.eEventType = eDSPACENotOkEvent;
			//osMessageQueuePut(ebs_sm_event_queue_Handle, &event, osPriorityNone, 0);
		}

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
  while (1)
  {
	  HAL_GPIO_WritePin(SYSTEM_FAULT_LED_GPIO_Port, SYSTEM_FAULT_LED_Pin, GPIO_PIN_SET);

  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
