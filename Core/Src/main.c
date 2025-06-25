/* USER CODE BEGIN Header */
  /**
    ******************************************************************************
    * @file           : main.c
    * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
  #include "fonts.h"
  #include "ILI9341_GFX.h"
  #include "ILI9341_STM32_Driver.h"

  #include "stdio.h"
  #include "string.h"
  #include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
  typedef struct{
  	char rainplus[15];
  	char waterplus[15];
  	char distplus[15];;
  	char rssiplus[25];

  	int rssi;
  }lcd;

  typedef struct{
  	float wateraver;
  	float rainaver;
  }sensor;

  typedef struct timemeet
  {
  	int tensec;
  	int tenmin;
  	int tenhour;
  	int tendate;
  	int tenmonth;
  	int realyear;
  }timemeet;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* Definitions for collect */
osThreadId_t collectHandle;
const osThreadAttr_t collect_attributes = {
  .name = "collect",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal5,
};
/* Definitions for tx */
osThreadId_t txHandle;
const osThreadAttr_t tx_attributes = {
  .name = "tx",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for LCDtask */
osThreadId_t LCDtaskHandle;
const osThreadAttr_t LCDtask_attributes = {
  .name = "LCDtask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for time */
osSemaphoreId_t timeHandle;
const osSemaphoreAttr_t time_attributes = {
  .name = "time"
};
/* Definitions for sensor */
osSemaphoreId_t sensorHandle;
const osSemaphoreAttr_t sensor_attributes = {
  .name = "sensor"
};
/* USER CODE BEGIN PV */
  uint16_t waterpp = 0;
  uint8_t timecheck = 0;
  uint16_t branch = 0;
  uint8_t sec = 0;
  uint8_t min = 0;
  uint8_t hour = 0;
  uint8_t date = 0;
  uint8_t month = 0;
  uint8_t year = 0;
  uint8_t tenyear = 0;

  uint8_t stop = 1;
  uint16_t adc_buffer[50];
  uint32_t start = 0;

  float rain = 0;
  float water = 0;
  float watervalue = 0;

  char modem[80] = { 0 };
  char timebuff[40] = { 0 };

  timemeet timedata;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
void collection(void *argument);
void trans(void *argument);
void LCD(void *argument);
void DS1302_WriteByte(uint8_t,uint8_t);

/* USER CODE BEGIN PFP */
  osMessageQueueId_t lcdQueue;
  osMessageQueueId_t sensorvalueQueue;
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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

    ILI9341_Init();

    ILI9341_FillScreen(WHITE);

     HAL_Delay(10);
    ILI9341_FillScreen(WHITE);
    ILI9341_DrawText("Start",FONT4,95,170,BLACK,WHITE);

    HAL_Delay(5000);

    ILI9341_FillScreen(WHITE);

    while(1)
     {
   	  ILI9341_FillScreen(BLACK);

   	  HAL_Delay(1000);

   	  ILI9341_FillScreen(WHITE);

   	  HAL_Delay(1000);

     ILI9341_DrawText("AT",FONT4,100,160,BLACK, WHITE);


     memset(modem,0,sizeof(modem));
     HAL_UART_Transmit(&huart3,(uint8_t*)"AT\r\n",strlen("AT\r\n"),1000);
     HAL_UART_Receive(&huart3,(uint8_t*)modem,sizeof(modem),1000);

     if(strstr((const char*)modem,"OK") == NULL)
     {
   	  ILI9341_DrawText("AT Fail",FONT4,100,160,BLACK,WHITE);
   	  HAL_Delay(1000);

   	  ILI9341_FillScreen(WHITE);

   	  continue;
     }
     memset(modem,0,sizeof(modem));

     ILI9341_DrawText("AT OK",FONT4,100,160,BLACK, WHITE);

     HAL_Delay(1000);

     ILI9341_FillScreen(WHITE);

     ILI9341_DrawText("AT CFUN",FONT4,80,160,BLACK,WHITE);

     HAL_Delay(1000);

     memset(modem,0,sizeof(modem));
     HAL_UART_Transmit(&huart3,(uint8_t*)"AT+CFUN=1,1\r\n",strlen("AT+CFUN=1,1\r\n"), 1000);

     HAL_Delay(1000);

     ILI9341_FillScreen(WHITE);

     ILI9341_DrawText("Modem reseting..",FONT4,60,160,BLACK,WHITE);

     HAL_Delay(30000);

     ILI9341_FillScreen(WHITE);

     ILI9341_DrawText("AT CREG",FONT4,90,160,BLACK,WHITE);

     HAL_Delay(1000);

     memset(modem,0,sizeof(modem));
     HAL_UART_Transmit(&huart3,(uint8_t*)"AT+CREG?\r\n",strlen("AT+CREG?\r\n"),1000);
     HAL_UART_Receive(&huart3,(uint8_t*)modem,sizeof(modem),1000);

     if (strstr((const char*)modem, "0,1") == NULL && strstr((const char*)modem, "0,5") == NULL)
     {
   	  ILI9341_DrawText("AT CREG Fail",FONT4,70,160,BLACK,WHITE);
   	  HAL_Delay(1000);

   	  continue;
     }

     ILI9341_DrawText("AT CREG OK",FONT4,70,160,BLACK,WHITE);

     HAL_Delay(1000);

     ILI9341_FillScreen(WHITE);

     ILI9341_DrawText("AT+CGDCONT",FONT4,60,160,BLACK,WHITE);

     HAL_Delay(1000);

     memset(modem,0,sizeof(modem));
     HAL_UART_Transmit(&huart3,(uint8_t*)"AT+CGDCONT=1,\"IP\",\"3gnet\"\r\n",strlen("AT+CGDCONT=1,\"IP\",\"3gnet\"\r\n"),1000);
     HAL_UART_Receive(&huart3,(uint8_t*)modem,sizeof(modem),1000);

     if(strstr((const char*)modem,"OK") == NULL)
     {
   	  ILI9341_DrawText("AT CGDCONT Fail",FONT4,50,160,BLACK,WHITE);
   	  HAL_Delay(1000);

   	  continue;
     }

     ILI9341_DrawText("AT CGDCONT OK",FONT4,50,160,BLACK,WHITE);

     HAL_Delay(1000);

     ILI9341_FillScreen(WHITE);

     ILI9341_DrawText("AT SICS TYPE",FONT4,50,160,BLACK,WHITE);

     HAL_Delay(1000);

     memset(modem,0,sizeof(modem));

     HAL_UART_Transmit(&huart3,(uint8_t*)"AT^SICS=0,\"conType\",\"GPRS0\"\r\n",strlen("AT^SICS=0,\"conType\",\"GPRS0\"\r\n"),1000);
     HAL_UART_Receive(&huart3,(uint8_t*)modem,sizeof(modem),1000);

     if(strstr((const char*)modem,"OK") == NULL)
     {
   	  ILI9341_DrawText("AT SICS TYPE Fail",FONT4,50,160,BLACK,WHITE);
   	  HAL_Delay(1000);

   	  continue;
     }

     ILI9341_DrawText("AT SICS TYPE OK",FONT4,50,160,BLACK,WHITE);

     HAL_Delay(1000);

     ILI9341_FillScreen(WHITE);

     ILI9341_DrawText("AT SICS APN",FONT4,60,160,BLACK,WHITE);

     HAL_Delay(1000);

     memset(modem,0,sizeof(modem));

     HAL_UART_Transmit(&huart3,(uint8_t*)"AT^SICS=0,\"apn\",\"3gnet\"\r\n",strlen("AT^SICS=0,\"apn\",\"3gnet\"\r\n"),1000);
     HAL_UART_Receive(&huart3,(uint8_t*)modem,sizeof(modem),1000);

     if(strstr((const char*)modem, "OK") == NULL)
     {
   	  ILI9341_DrawText("AT SICS APN Fail",FONT4,50,160,BLACK,WHITE);
   	  HAL_Delay(1000);

   	  continue;
     }

     ILI9341_DrawText("AT SICS APN OK",FONT4,50,160,BLACK,WHITE);

     HAL_Delay(1000);

     ILI9341_FillScreen(WHITE);

     ILI9341_DrawText("AT SISS TYPE",FONT4,60,160,BLACK,WHITE);

     HAL_Delay(1000);
     memset(modem,0,sizeof(modem));
     HAL_UART_Transmit(&huart3,(uint8_t*)"AT^SISS=0,\"srvType\",\"Socket\"\r\n",strlen("AT^SISS=0,\"srvType\",\"Socket\"\r\n"),1000);
     HAL_UART_Receive(&huart3,(uint8_t*)modem,sizeof(modem),1000);

     if(strstr((const char*) modem, "OK") == NULL)
     {
   	  ILI9341_DrawText("AT SISS TYPE Fail",FONT4,50,160,BLACK,WHITE);
   	  HAL_Delay(1000);

   	  continue;
     }

     ILI9341_DrawText("AT SISS TYPE OK",FONT4,60,160,BLACK,WHITE);

     HAL_Delay(1000);

     ILI9341_FillScreen(WHITE);

     ILI9341_DrawText("AT SISS ID",FONT4,70,160,BLACK,WHITE);

     HAL_Delay(1000);
     memset(modem,0,sizeof(modem));
     HAL_UART_Transmit(&huart3,(uint8_t*)"AT^SISS=0,\"conId\",0\r\n",strlen("AT^SISS=0,\"conId\",0\r\n"),1000);
     HAL_UART_Receive(&huart3,(uint8_t*)modem,sizeof(modem),1000);

     if(strstr((const char*)modem,"OK") == NULL)
     {
   	  ILI9341_DrawText("AT SISS ID Fail",FONT4,60,160,BLACK,WHITE);
   	  HAL_Delay(1000);

   	  continue;
     }

     ILI9341_DrawText("AT SISS ID OK",FONT4,70,160,BLACK,WHITE);

     HAL_Delay(1000);

     ILI9341_FillScreen(WHITE);

     ILI9341_DrawText("AT SISS address",FONT4,60,160,BLACK,WHITE);

     HAL_Delay(1000);

     memset(modem,0,sizeof(modem));
     HAL_UART_Transmit(&huart3,(uint8_t*)"AT^SISS=0,\"address\",\"socktcp://49.254.169.71:5001\"\r\n",strlen("AT^SISS=0,\"address\",\"socktcp://49.254.169.71:5001\"\r\n"), 1000);
     HAL_UART_Receive(&huart3,(uint8_t*)modem,sizeof(modem),1000);

     if(strstr((const char*)modem, "OK") == NULL)
     {
   	  ILI9341_DrawText("AT SISS address Fail",FONT4,40,160,BLACK,WHITE);
   	  HAL_Delay(1000);

   	  continue;
     }

     ILI9341_DrawText("AT SISS address OK",FONT4,50,160,BLACK,WHITE);

     HAL_Delay(1000);

     ILI9341_FillScreen(WHITE);

     ILI9341_DrawText("AT+CGATT",FONT4,80,160,BLACK,WHITE);

     HAL_Delay(1000);
     memset(modem,0,sizeof(modem));
     HAL_UART_Transmit(&huart3,(uint8_t*)"AT+CGATT?\r\n",strlen("AT+CGATT?\r\n"), 1000);
     HAL_UART_Receive(&huart3,(uint8_t*)modem,sizeof(modem),1000);

     if(strstr((const char*)modem,"1") == NULL)
     {
   	 ILI9341_DrawText("AT+CGATT Fail",FONT4,70,160,BLACK,WHITE);
   	 HAL_Delay(1000);

   	 continue;
     }

     ILI9341_DrawText("AT CGATT OK",FONT4,70,160,BLACK,WHITE);

     HAL_Delay(1000);

     ILI9341_FillScreen(WHITE);

     ILI9341_DrawText("AT+CGACT",FONT4,70,160,BLACK,WHITE);

     HAL_Delay(1000);
     memset(modem,0,sizeof(modem));
     HAL_UART_Transmit(&huart3,(uint8_t*)"AT+CGACT=1,1\r\n",strlen("AT+CGACT=1,1\r\n"), 1000);
     HAL_UART_Receive(&huart3,(uint8_t*)modem,sizeof(modem),1000);

     HAL_Delay(5000);

     if(strstr((const char*)modem,"1,1") == NULL)
      {
    	  ILI9341_DrawText("AT+CGACT Fail",FONT4,80,160,BLACK,WHITE);
    	  HAL_Delay(1000);

    	  continue;
      }

      ILI9341_DrawText("AT+CGACT OK",FONT4,70,160,BLACK,WHITE);

        HAL_Delay(1000);

        HAL_UART_Transmit(&huart3,(uint8_t*)"AT^SISO=0\r\n",strlen("AT^SISO=0\r\n"),1000);
        HAL_UART_Receive(&huart3,(uint8_t*)modem,sizeof(modem),1000);

        HAL_Delay(3000);

        ILI9341_DrawText("Communication OK?",FONT4,60,160,BLACK,WHITE);

        HAL_Delay(1000);

        HAL_UART_Transmit(&huart3,(uint8_t*)"AT^SISI?\r\n",strlen("AT^SISI?\r\n"),1000);
        HAL_UART_Receive(&huart3,(uint8_t*)modem,sizeof(modem),1000);

        if(strstr((const char*)modem, "0,4") == NULL)
        {
       	 ILI9341_DrawText("Communication Fail",FONT4,40,160,BLACK,WHITE);
       	 HAL_Delay(1000);

       	 continue;
        }
        ILI9341_FillScreen(WHITE);

        ILI9341_DrawText("Communication OK",FONT4,50,160,BLACK,WHITE);

        HAL_Delay(1000);

        ILI9341_FillScreen(WHITE);

        break;
     }

/*  DS1302_WriteByte(0x80,0x30);
    DS1302_WriteByte(0x82,0x21);
    DS1302_WriteByte(0x84,0x11);
    DS1302_WriteByte(0x86,0x23);
    DS1302_WriteByte(0x88,0x06);
    DS1302_WriteByte(0x8C,0x25);
*/
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3,(uint8_t*)modem,sizeof(modem));

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of time */
  timeHandle = osSemaphoreNew(1, 0, &time_attributes);

  /* creation of sensor */
  sensorHandle = osSemaphoreNew(1, 0, &sensor_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    lcdQueue = osMessageQueueNew(4, sizeof(lcd), NULL);
    sensorvalueQueue = osMessageQueueNew(4, sizeof(sensor), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of collect */
  collectHandle = osThreadNew(collection, NULL, &collect_attributes);

  /* creation of tx */
  txHandle = osThreadNew(trans, NULL, &tx_attributes);

  /* creation of LCDtask */
  LCDtaskHandle = osThreadNew(LCD, NULL, &LCDtask_attributes);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_1;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Pin|CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DAT_Pin|CLK_Pin|DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin RESET_Pin CS_Pin */
  GPIO_InitStruct.Pin = LED_Pin|RESET_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RST_Pin */
  GPIO_InitStruct.Pin = RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DAT_Pin CLK_Pin DC_Pin */
  GPIO_InitStruct.Pin = DAT_Pin|CLK_Pin|DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
  void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
  {
    /* Prevent unused argument(s) compilation warning */
    if(HAL_GetTick() - start >= 50)
    {

    if(GPIO_Pin == GPIO_PIN_10)
    {
  	 rain += 0.5;

  	 start = HAL_GetTick();
    }

    }
    else
    {
  	  	return;
    }
    /* NOTE: This function Should not be modified, when the callback is needed,
             the HAL_GPIO_EXTI_Callback could be implemented in the user file
     */
  }

  void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
  {
  	if(hadc->Instance == ADC1)
  	{
  		osSemaphoreRelease(timeHandle);
  	}
  }

  void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
  {
  	 stop = 0;

      HAL_UARTEx_ReceiveToIdle_DMA(&huart3,(uint8_t*)modem,sizeof(modem));
  }

  void ProtectRESET()
  {
  	uint8_t protect = 0x8E;

  	HAL_GPIO_WritePin(RST_GPIO_Port,RST_Pin,GPIO_PIN_SET);

  	for(int i = 0; i < 8; i++)
  	{
  	   HAL_GPIO_WritePin(DAT_GPIO_Port,DAT_Pin,(protect >> i) & 0x01);

  	   HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_SET);
  	   HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_RESET);
  	}

  	for(int i = 0; i < 8; i++)
  	{
  		HAL_GPIO_WritePin(DAT_GPIO_Port,DAT_Pin,0x00);

  		HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_SET);
  	    HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_RESET);
  	}

  	HAL_GPIO_WritePin(RST_GPIO_Port,RST_Pin,GPIO_PIN_RESET);
  }

  void DS1302_RESET()
  {
  	HAL_GPIO_WritePin(RST_GPIO_Port,RST_Pin,GPIO_PIN_RESET);
  	HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_RESET);
  }

  void DS1302_WriteByte(uint8_t ch,uint8_t data)
  {

  	HAL_GPIO_WritePin(RST_GPIO_Port,RST_Pin,GPIO_PIN_SET);
  	for(int i = 0; i < 8; i++)
  	{
  		HAL_GPIO_WritePin(DAT_GPIO_Port,DAT_Pin,(ch >> i) & 0x01);

  		HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_SET);
  		HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_RESET);
  	}

  	HAL_Delay(10);

  	for(int i = 0; i < 8; i++)
  	{
  		HAL_GPIO_WritePin(DAT_GPIO_Port,DAT_Pin,(data >> i) & 0x01);

  		HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_SET);
  		HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_RESET);
  	}

  	HAL_GPIO_WritePin(RST_GPIO_Port,RST_Pin,GPIO_PIN_RESET);
  }

  uint16_t DS1302_ReadByte(uint16_t ch)
  {
  	uint16_t time = 0;

  	HAL_GPIO_WritePin(RST_GPIO_Port,RST_Pin,GPIO_PIN_SET);

  	for(int i = 0; i < 8; i++)
  	{
  		HAL_GPIO_WritePin(DAT_GPIO_Port,DAT_Pin,(ch >> i) & 0x01);
  		HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_SET);
  		HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_RESET);
  	}

  	GPIO_InitTypeDef GPIO_InitStruct = {0};

  	GPIO_InitStruct.Pin = DAT_Pin;
  	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  	GPIO_InitStruct.Pull = GPIO_NOPULL;
  	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  	for(int i = 0; i < 8; i++)
  	{
  	 if(HAL_GPIO_ReadPin(DAT_GPIO_Port,DAT_Pin) == GPIO_PIN_SET)
  	 {
  		time |= 1 << i;
  	 }

  	 HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_SET);
  	 HAL_GPIO_WritePin(CLK_GPIO_Port,CLK_Pin,GPIO_PIN_RESET);
  	}

  	HAL_GPIO_WritePin(RST_GPIO_Port,RST_Pin,GPIO_PIN_RESET);

  	 GPIO_InitStruct.Pin = DAT_Pin;
  	 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  	 GPIO_InitStruct.Pull = GPIO_NOPULL;
  	 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  	 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  	return time;
  }

  uint8_t DS1302_tenone(uint8_t num)
  {
  	uint8_t ten = (num >> 4) * 10;
  	uint8_t one = num & 0x0f;
  	uint8_t sum = ten+one;

  	return sum;
  }

  void Clk()
  {
  	  sec = DS1302_ReadByte(0x81);
  	  min = DS1302_ReadByte(0x83);
  	  hour = DS1302_ReadByte(0x85);
  	  date = DS1302_ReadByte(0x87);
  	  month = DS1302_ReadByte(0x89);
  	  year = DS1302_ReadByte(0x8D);

  	  timedata.tensec = DS1302_tenone(sec);
  	  timedata.tenmin = DS1302_tenone(min);
  	  timedata.tenhour = DS1302_tenone(hour);
  	  timedata.tendate = DS1302_tenone(date);
  	  timedata.tenmonth = DS1302_tenone(month);
  	  tenyear = DS1302_tenone(year);
  	  timedata.realyear = tenyear + 2000;
  }

/* USER CODE END 4 */

/* USER CODE BEGIN Header_collection */
  /**
    * @brief  Function implementing the collect thread.
    * @param  argument: Not used
    * @retval None
    */
/* USER CODE END Header_collection */
void collection(void *argument)
{
  /* USER CODE BEGIN 5 */
    sensor senaver;
    lcd lcdvalue;

    /* Infinite loop */
    for(;;)
    {
  	  watervalue = 0;

  	  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_buffer,50);

  	  osSemaphoreAcquire(timeHandle,osWaitForever);

  	  for(int i = 0; i < 50; i++)
  	  {
  		  watervalue += adc_buffer[i];
  	  }

  	  water = ((3980.0 - (watervalue / 50))  / (3980.0 - 788.0)) * 15.0;

  	  HAL_UART_Transmit_DMA(&huart3, (uint8_t*)"AT+CSQ\r\n",sizeof(modem));

  	  while(1)
  	  {
  	  if(stop == 0)
  	  {
  	  char *csqptr = strstr(modem,":");
  	  sscanf(csqptr,": %d,",&lcdvalue.rssi);

  	  sprintf(lcdvalue.waterplus,"ADC : %.0f",watervalue / 50);
  	  sprintf(lcdvalue.rainplus,"Rain : %.1fmm",rain);
  	  sprintf(lcdvalue.distplus,"Water : %.2fm",water);
  	  sprintf(lcdvalue.rssiplus,"RSSI : %d",lcdvalue.rssi);

  	  break;
  	  }
  	  else
  	  {
  		  HAL_Delay(10);
  	  }
  	  }

  	  senaver.wateraver += water;
  	  senaver.rainaver = rain;
  	  waterpp++;

  	  osMessageQueuePut(lcdQueue,&lcdvalue,0,osWaitForever);

  	  if(timedata.tenmin % 1 == 0)
  	  {
  	 	    	branch++;
  	  }
  	  if(branch == 1)
  	  {
  		  ILI9341_FillScreen(WHITE);

  		  osMessageQueuePut(sensorvalueQueue,&senaver,0,osWaitForever);
  		  osSemaphoreRelease(sensorHandle);

  		  senaver.wateraver = 0;
  		  senaver.rainaver = 0;

  		  if(timedata.tenmin % 1 != 0)
  		  {
  			  branch = 0;
  		  }
  	   }

  	  osDelay(1000);
    }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_trans */
  /**
  * @brief Function implementing the tx thread.
  * @param argument: Not used
  * @retval None
  */
/* USER CODE END Header_trans */
void trans(void *argument)
{
  /* USER CODE BEGIN trans */
    sensor sensorrecv;
    lcd lcdcheck;
    uint8_t datalen;
    uint8_t checksum;
    uint8_t checkstack;
    /* Infinite loop */
    for(;;)
    {
  	  	osSemaphoreAcquire(sensorHandle, osWaitForever);

  	  	osMessageQueueGet(sensorvalueQueue,&sensorrecv,NULL,osWaitForever);
  	  	osMessageQueueGet(lcdQueue,&lcdcheck,NULL,osWaitForever);

  	    char datafield[30] = { 0 };
  	    char send[40] = { 0 };
  	    char transfinal[30] = { 0 };

  	    sensorrecv.wateraver /= (float)waterpp;

  	    memset(modem,0,sizeof(modem));

  		while(stop != 0)
  		{
  		if(stop == 0)
  		{
  		sprintf(datafield,"%.1f %.1f %d",sensorrecv.wateraver,sensorrecv.rainaver,lcdcheck.rssi);

  		datalen = strlen(datafield);
  		checksum = 0;
        checkstack = 0;

  		while(datafield[checkstack] != '\0')
  		{
  			checksum ^= datafield[checkstack];

  			checkstack++;
  		}

  		stop = 1;
  		}
  		else
  		{
  		osDelay(1);
  		}
  		}

  		HAL_UART_Transmit_DMA(&huart3,(uint8_t*)"AT^SISI?\r\n",strlen("AT^SISI?\r\n"));

  		while(stop != 0)
  		{
  		if(stop == 0)
  		{
  		if((strstr((const char*)modem,"0,4") == NULL))
  		{
  			ILI9341_FillScreen(WHITE);
  		    ILI9341_DrawText("Re Connecting..\r\n",FONT4,50,160,BLACK, WHITE);

  			HAL_UART_Transmit_DMA(&huart3,(uint8_t*)"AT^SISC=0\r\n",strlen("AT^SISC=0\r\n"));

  			osDelay(3000);

  			HAL_UART_Transmit_DMA(&huart3,(uint8_t*)"AT^SISO=0\r\n",strlen("AT^SISO=0\r\n"));

  			osDelay(5000);

  			ILI9341_FillScreen(WHITE);

  		}
  		stop = 1;
  		}
  		else
  		{
  			osDelay(1);
  		}
  		}
  		sprintf(transfinal,"#%d%.1f %.1f %d$%d@",datalen,sensorrecv.wateraver,sensorrecv.rainaver,lcdcheck.rssi,checksum);

  		sprintf(send,"AT^SISW=0,%d\r\n",strlen(transfinal));

  		HAL_UART_Transmit_DMA(&huart3,(uint8_t*)send,strlen(send));

  	    osDelay(1000);

  	    HAL_UART_Transmit_DMA(&huart3,(uint8_t*)transfinal,strlen(transfinal));

  		osDelay(1000);
  	}

  /* USER CODE END trans */
}

/* USER CODE BEGIN Header_LCD */
  /**
  * @brief Function implementing the LCDtask thread.
  * @param argument: Not used
  * @retval None
  */
/* USER CODE END Header_LCD */
void LCD(void *argument)
{
  /* USER CODE BEGIN LCD */
    lcd lcdrecv;
    /* Infinite loop */
    for(;;)
    {
  	Clk();

  	sprintf(timebuff,"%d-%02d-%02d %02d:%02d:%02d",timedata.realyear,timedata.tenmonth,timedata.tendate,timedata.tenhour,timedata.tenmin,timedata.tensec);

  	osMessageQueueGet(lcdQueue,&lcdrecv,NULL,osWaitForever);

  	ILI9341_DrawText(lcdrecv.waterplus,FONT4,70,40,BLACK,WHITE);
  	ILI9341_DrawText(lcdrecv.rainplus,FONT4,70,80,BLACK,WHITE);
  	ILI9341_DrawText(lcdrecv.distplus,FONT4,70,120,BLACK,WHITE);
  	ILI9341_DrawText(lcdrecv.rssiplus,FONT4,70,200,BLACK,WHITE);
  	ILI9341_DrawText(timebuff,FONT4,20,280,BLACK,WHITE);

    osDelayUntil(1000);
    }
  /* USER CODE END LCD */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
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
    while (1)
    {
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
