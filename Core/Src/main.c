/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "ws2812_led.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define FRAMEBUFFER_SIZE        24 //24led * 3 colors
#define FRAMEBUFFER2_SIZE        19 //24led * 3 colors
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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim2_ch2_ch4;
DMA_HandleTypeDef hdma_tim2_ch1;
DMA_HandleTypeDef hdma_tim2_up;

UART_HandleTypeDef huart1;

osThreadId adcreaderHandle;
osThreadId serialreaderHandle;
osTimerId pidTimerHandle;
osTimerId statusUpdateHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
void adc_reader_task(void const * argument);
void serial_reader_task(void const * argument);
void pid_timer(void const * argument);
void status_update_timer(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t RX1_Char = 0x00;
uint32_t adc_value[7];
int forcethres=20;
struct pixel {
    uint8_t g;
    uint8_t r;
    uint8_t b;
};
struct pixel channel_framebuffers[WS2812_NUM_CHANNELS][FRAMEBUFFER_SIZE];
//serial write buffer
char buffer[30];
float MSG[150];

// IR proximity sensors
  int num_irsensors = 10;
  int irdata_fl[10];
  int irdata_fr[10];
  int data_fl_real, data_fr_real, data_fl_noise,data_fr_noise,data_fl,data_fr;

void make_pretty_colors(struct pixel *framebuffer, int channel, int state)
{
    int red_offset, green_offset, blue_offset, i;

    red_offset = 0;
    green_offset = (FRAMEBUFFER_SIZE / 4) * ((channel) & 0x03);
    blue_offset = (FRAMEBUFFER_SIZE / 4) * ((channel >> 2) & 0x03);

    /* Just generate a different-looking psychedelic-looking pattern for each channel, to
     * test/prove that each channel is receiving a unique pattern
     */
    framebuffer[0].r=255;
    framebuffer[0].g=0;
    framebuffer[0].b=0;

    		  framebuffer[1].r=0;
    		    framebuffer[1].g=255;
    		    framebuffer[1].b=0;

    		    		  framebuffer[2].r=0;
    		    		    framebuffer[2].g=0;
    		    		    framebuffer[2].b=255;



   /* for (i = 0; i < FRAMEBUFFER_SIZE ; i++) {
        framebuffer[(i + red_offset + state) % FRAMEBUFFER_SIZE].r = i;
        framebuffer[(i + green_offset + state) % FRAMEBUFFER_SIZE].g = i;
        framebuffer[(i + blue_offset + state) % FRAMEBUFFER_SIZE].b = i;
    }*/

}

void lightupLED(struct pixel *framebuffer)
{
	for(int i=0;i<5;i++)
	{
		framebuffer[i].r=0;
		framebuffer[i].g=0;
		framebuffer[i].b=255;
	}
	for(int i=5;i<12;i++)
	{
		framebuffer[i].r=0;
		framebuffer[i].g=255;
		framebuffer[i].b=0;
	}
	for(int i=12;i<17;i++)
	{
		framebuffer[i].r=50;
		framebuffer[i].g=50;
		framebuffer[i].b=50;
	}
	for(int i=17;i<24;i++)
	{
		framebuffer[i].r=255;
		framebuffer[i].g=0;
		framebuffer[i].b=0;
	}
}
void lightupLED2(struct pixel *framebuffer)
{
	for(int i=0;i<5;i++)
	{
		framebuffer[i].r=0;
		framebuffer[i].g=0;
		framebuffer[i].b=255;
	}
	for(int i=5;i<12;i++)
	{
		framebuffer[i].r=0;
		framebuffer[i].g=255;
		framebuffer[i].b=0;
	}
	for(int i=12;i<19;i++)
	{
		framebuffer[i].r=255;
		framebuffer[i].g=0;
		framebuffer[i].b=0;
	}
}
//---------[ UART Data Reception Completion CallBackFunc. ]---------
void HAL_USART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(&huart1, &RX1_Char, 1);
}

void open()
{
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		    		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
		    		 HAL_Delay(100);
		    		 while(adc_value[2]<forcethres)
		    		 {
	   		  		  }
	         	  	HAL_Delay(30);
		    	  	while(adc_value[2]<forcethres)
		      		  {
	 		  		  }
		    	  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		    	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
}

void close()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
		    		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		    		  HAL_Delay(100);
		    		  while(adc_value[2]<forcethres)
		    		  {
		    		  }
		    		  HAL_Delay(30);
		    		  while(adc_value[2]<forcethres)
		    		  {
		    		  }
		    		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		    		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

}

void ir_led_on()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
}

void ir_led_off()
{
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

}

void set_mux_fl(value)
{


HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, value & 0b0001);
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, value & 0b0010);
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, value & 0b0100);
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, value & 0b1000);
}
void set_mux_fr(value)
{
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, value & 0b0001);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, value & 0b0010);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, value & 0b0100);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, value & 0b1000);
}
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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  //start ADC conversations
  HAL_ADC_Start_DMA(&hadc1, adc_value, 7);

  //START PWM TIMERS
  /* PWM frequency is set  =Fclk/((ARR+1)*(PSC+1))
   *	Fclk =  clock frequency
   *	ARR = Counter Period (AutoReload Register - 16 bits value, set from Timer-> Parameter Settings
   *	PSC  =  Prescaler value, set from Timer-> Parameter Settings
   *
   *	so for 20Khz,
   *		20000 = 56000000/x
   *		x = 2800, so ARR  =  2799 and PSC=0
   *		now, 2800 will correspond to 100% pwm and can be set either by
   *		TIM4->CCR1 = pwm_value;
   *		or
   *		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, pwm_value);
   *
   */
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_Base_Start(&htim1);

  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);


  	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 10);
  	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 560);
  	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 1080);
  	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 2000);
  	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1560);
  // = {'\0'};
  long X = 0;




  int temp;

  struct led_channel_info led_channels[WS2812_NUM_CHANNELS];

      int ch, animation_state = 0;

      __enable_irq();
      HAL_Delay(200);

      //INITIALIZE NEOPIXELS
      //channel 0
      led_channels[0].framebuffer = channel_framebuffers[0];
      led_channels[0].length = FRAMEBUFFER_SIZE * sizeof(struct pixel);

      //channel1
      led_channels[1].framebuffer = channel_framebuffers[1];
      led_channels[1].length = FRAMEBUFFER2_SIZE * sizeof(struct pixel);

      HAL_Delay(200);
      ws2812_init();
      HAL_Delay(200);
      // SETUP LED COLORS
      lightupLED(channel_framebuffers[0]);
      lightupLED2(channel_framebuffers[1]);

   	  __disable_irq();
   	  ws2812_refresh(led_channels, GPIOB);
   	  __enable_irq();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of pidTimer */
  osTimerDef(pidTimer, pid_timer);
  pidTimerHandle = osTimerCreate(osTimer(pidTimer), osTimerPeriodic, NULL);

  /* definition and creation of statusUpdate */
  osTimerDef(statusUpdate, status_update_timer);
  statusUpdateHandle = osTimerCreate(osTimer(statusUpdate), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  // PID timer runs at 100hz
  osTimerStart(pidTimerHandle, 10);
  //status update timer runs at 100 hz
  osTimerStart(statusUpdateHandle, 10);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of adcreader */
  osThreadDef(adcreader, adc_reader_task, osPriorityNormal, 0, 128);
  adcreaderHandle = osThreadCreate(osThread(adcreader), NULL);

  /* definition and creation of serialreader */
  osThreadDef(serialreader, serial_reader_task, osPriorityIdle, 0, 128);
  serialreaderHandle = osThreadCreate(osThread(serialreader), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*

	  // set ir led on
	 // ir_led_on();
	  // set the selection ports


	  HAL_Delay(1);
	 //sprintf(MSG, "Data = %d \t %d  \t %d %d  \t%d  \t%d \t%d \t \r\n ",adc_value[0],adc_value[1], adc_value[2], adc_value[3], adc_value[4], adc_value[5], adc_value[6]);
	  //sprintf(MSG, "Data = %d \t %d  \t %d \t %d  \t%d  \t%d \t%d \t \r\n ",irdata_fl[0],irdata_fl[1], irdata_fl[2], irdata_fl[3], irdata_fl[4], irdata_fl[5], irdata_fl[6]);
	  sprintf(MSG, "Data = %d \t %d  \t %d \t %d  \t%d  \t%d \t%d \t \r\n ",irdata_fr[0],irdata_fr[1], irdata_fr[2], irdata_fr[3], irdata_fr[4], irdata_fr[5], irdata_fr[6]);
	  //sprintf(MSG, "Hello Dudes! COUNT = %d \r\n ",X);
	 HAL_UART_Transmit(&huart1, MSG, strlen(MSG), 600);
	  //HAL_UART_Transmit(&huart1, (uint8_t*)buffer, sprintf(buffer, "Val is : %d \r\n", X), 10);


	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	    if(RX1_Char == 'c') //close
	    {
	    		close();

	    		  HAL_UART_Receive_IT(&huart1, &RX1_Char, 1);
	    		  RX1_Char = 0x00;
	    }
	    else if(RX1_Char == 'o') //open
	    {
	    		open();

	    	    HAL_UART_Receive_IT(&huart1, &RX1_Char, 1);
	    	    RX1_Char = 0x00;
	    }
	    else if(RX1_Char == 'b')  //brake
	    	    {
	    	    	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	    	    		  	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	    	    		  	  HAL_Delay(100);

	    	    		  	HAL_UART_Receive_IT(&huart1, &RX1_Char, 1);
	    	    		  	RX1_Char = 0x00;

	    	    }
	    else
	    {	//clear buffer by reading out
	    	HAL_UART_Receive_IT(&huart1, &RX1_Char, 1);
	    		    	    		  	RX1_Char = 0x00;
	    }
	    */

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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL7;
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
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 7;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2799;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2799;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11
                           PB12 PB13 PB14 PB15
                           PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_adc_reader_task */
/**
  * @brief  Function implementing the adcreader thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_adc_reader_task */
void adc_reader_task(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {


	  for(int i=0;i<num_irsensors;i++)
	  	  {
	  		  // set IR off
	  		  ir_led_off();

	  		  // select mux channel
	  		  set_mux_fl(i);
	  		  set_mux_fr(i);

	  		  //small delay
	  		  HAL_Delay(1);

	  		  // get initial readings
	  		  data_fl_noise = adc_value[0];
	  		  data_fr_noise = adc_value[1];

	  		  // set IR on
	  		  ir_led_on();
	  		  //small delay
	  		  HAL_Delay(1);

	  		  // get second readings
	  		  data_fl = adc_value[0];
	  		  data_fr = adc_value[1];

	  		  //calculate the real value and set it in ir_data array
	  		  data_fl_real = -1*(data_fl - data_fl_noise);
	  		  data_fr_real = -1*(data_fr - data_fr_noise);

	  		  irdata_fl[i] = data_fl_real;
	  		  irdata_fr[i] = data_fr_real;

	  	  }

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_serial_reader_task */
/**
* @brief Function implementing the serialreader thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_serial_reader_task */
void serial_reader_task(void const * argument)
{
  /* USER CODE BEGIN serial_reader_task */
  /* Infinite loop */
  for(;;)
  {
	//HAL_UART_Transmit(&huart1, (uint8_t*)buffer, sprintf(buffer, "Serial READ \n", 1), 10);
    osDelay(1);
  }
  /* USER CODE END serial_reader_task */
}

/* pid_timer function */
void pid_timer(void const * argument)
{
  /* USER CODE BEGIN pid_timer */
	//HAL_UART_Transmit(&huart1, (uint8_t*)buffer, sprintf(buffer, "TIMER \n", 1), 10);
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
	//HAL_GPIO_WritePin(GPIOB, , value & 0b0001);

  /* USER CODE END pid_timer */
}

/* status_update_timer function */
void status_update_timer(void const * argument)
{
  /* USER CODE BEGIN status_update_timer */

	/* Serial Data Format
	 *  M1-2_current, M1Pos, M2Pos, M3Pos, M4Pos, irsens_left[10], irsens_right[10]
	 *
	 */
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
	//sprintf(MSG, "Data = %d \t %d  \t %d \t %d  \t%d  \t%d \t%d \t \r\n ",
	//		irdata_fr[0],irdata_fr[1], irdata_fr[2], irdata_fr[3], irdata_fr[4], irdata_fr[5], irdata_fr[6]);

	sprintf(MSG, "%d \t%d \t%d \t%d \t%d \t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t \r\n",
			adc_value[2], adc_value[3], adc_value[4], adc_value[5], adc_value[6],
				irdata_fr[0],irdata_fr[1], irdata_fr[2], irdata_fr[3], irdata_fr[4], irdata_fr[5], irdata_fr[6],irdata_fr[7],irdata_fr[8],irdata_fr[9],
				irdata_fl[0],irdata_fl[1], irdata_fl[2], irdata_fl[3], irdata_fl[4], irdata_fl[5], irdata_fl[6],irdata_fl[7],irdata_fl[8],irdata_fl[9]);
		  //sprintf(MSG, "Hello Dudes! COUNT = %d \r\n ",X);
		 HAL_UART_Transmit(&huart1, MSG, strlen(MSG), 600);
  /* USER CODE END status_update_timer */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
