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
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// framebuffer for the two sets of RGB LEDs
#define FRAMEBUFFER_SIZE        24 //24led * 3 colors
#define FRAMEBUFFER2_SIZE        19 //24led * 3 colors

#define UART_TX_BUFFER_SIZE	 256

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

osThreadId adcreaderHandle;
osThreadId serialreaderHandle;
osMessageQId myQueue01Handle;
osTimerId pidTimerHandle;
osTimerId statusUpdateHandle;
osSemaphoreId BinSemHandle;
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

// array to hold values of ADC input ports
uint32_t adc_value[7];

// RGB LED structs
struct pixel {
    uint8_t g;
    uint8_t r;
    uint8_t b;
};
struct pixel channel_framebuffers[WS2812_NUM_CHANNELS][FRAMEBUFFER_SIZE];


// UART TX buffers
uint16_t uartTXBufferIndex = 0;
uint16_t uartTXBufferLength = 0;
uint8_t uartTXBuffer[UART_TX_BUFFER_SIZE];

//flags that are set to indicate whether motors  are
// moving and their direction.
int lrw = 0; // LEFT REVERSE
int lfw = 0; // LEFT FORWARD
int rfw = 0; // RIGHT FORWARD
int rrw = 0; // RIGHT REVERSE
int mgo = 0; // GRIPPER OPENING
int mgc = 0; // GRIPPER CLOSING

//flags to enable/disable PID loops
int lPid = 0;
int rPid = 0;
int gPid = 0;

// for LF & RF Max when extended ; Min when retracted
int LFMaxPos = 3900;
int LFMinPos = 200;
int RFMaxPos = 3900;
int RFMinPos = 200;
// for M1 & M2 Min when extended ; Max when retracted
// fully open at minpos and closed at maxpos
// when closed, value depends on finger geometry. Example values
// when closed is 3299, 3699
/* -----||----- 3299, 3699(gap 0 )
 * ----|  | ---3000,3200 (gap = (3299-3000) + (3699-3200))
 *--|         |-- at full open, theoretical max gap = ((3850-430)+(3850-380)) = 6890
 * so we will scale 0 to 6890 into 0 to 999
 *
 *
 */
int M1MinPos = 430;
int M1MaxPos = 3850;
int M2MinPos = 380;
int M2MaxPos = 3850;
//M1 and M2 positions for zero gap
int M1z = 3850;
int M2z = 3850;
int gripperGapcmd = 0;
int gripperGapDelta = 2;
int GapMax = 6890;
//PID params for gripper gap control
double g_error_prev=0;
double g_error_integral = 0;
double g_Kp = 1.8;
double g_Kd = 0.8;
double g_Ki = 0.0001;

//gripper gap value limits
int gmin = 3500; // -value when closed - depends on finger geometry
int gmax=6500; // value when fully open

// Desired positions & currents
int lPosDesired =0;
int rPosDesired =0;
int gCurDesired = 0;

// Delta position & currents allowed
int lPosDelta = 10;
int rPosDelta = 10;
int gCurDelta = 10;

// Motor control PID loop time period in milliseconds
// with t = 5, PID runs at 200 Hz
uint8_t pid_time_period = 5;

// L & R Finger PID parameters
double l_error_prev=0;
double l_error_integral = 0;
double l_Kp = 1.0;
double l_Kd = 0.8;
double l_Ki = 0.00001;
double r_error_prev=0;
double r_error_integral = 0;
double r_Kp = 1.0;
double r_Kd = 0.8;
double r_Ki = 0.00001;

// gripper motor params
int gForceThres = 0;
int gTimeOut = 10000; // 10 second timeout for gripper to reach threshold current
int startTick = 0;

// IR proximity sensors and the buffers to hold data
int num_irsensors = 10;
int irdata_fl[10];
int irdata_fr[10];
int data_fl_real, data_fr_real, data_fl_noise,data_fr_noise,data_fl,data_fr;


// set the RGB LEDs on Finger 1
void lightupLED1(struct pixel *framebuffer)
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
		framebuffer[i].r=0;
		framebuffer[i].g=0;
		framebuffer[i].b=0;
	}
	for(int i=17;i<24;i++)
	{
		framebuffer[i].r=255;
		framebuffer[i].g=0;
		framebuffer[i].b=0;
	}
}
// set the RGB LEDs on Finger 2
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

void open_gripper(int pwmval)
{

	if((adc_value[3]>M1MinPos)||(adc_value[4]>M2MinPos))
	{
		osSemaphoreWait(BinSemHandle, osWaitForever);
		mgo=1;
		pwmval = scale_val(pwmval,0,100, 0, 2800);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwmval);

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
		osSemaphoreRelease(BinSemHandle);
	}
}

void close_gripper(int pwmval)
{

	if((adc_value[3]<M1MaxPos)||(adc_value[4]<M2MaxPos))
	{
		osSemaphoreWait(BinSemHandle, osWaitForever);
		mgc=1;
		pwmval = scale_val(pwmval,0,100, 0, 2800);
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwmval);

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		osSemaphoreRelease(BinSemHandle);

	}

}
void brake_lf()
{
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 2800);
  	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 2800);
}
void brake_rf()
{
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 2800);
  	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 2800);
}
void stop_lf()
{
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);
  	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 0);
}

void stop_rf()
{
  	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
  	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
}
void brake_gripper()
{
	//reset the GPIO for open-close motors
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2800);
}
void stop_gripper()
{
	//reset the GPIO for open-close motors
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
}
void brake_all()
{/*Stops all motors -  shorts motor terminals -  cannot move by hand*/

	//set the GPIO for open-close motors
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

	//set all PWMs to 2800 - pin high - brake
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 2800);
  	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 2800);
  	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 2800);
  	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 2800);
  	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2800);

}
void stop_all()
{/*Stops all motors -  disconnects motor terminals -  can move by hand*/

	//reset the GPIO for open-close motors
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

	//set all PWMs to 2800 - pin high - brake
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);
  	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 0);
  	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
  	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
  	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);

}

void move_lf(int pwmval)
{
	//	Check whether we are at the end positions
		//LF pos given by adc_value[6]
		// min value is around 100, max value 4000, so we set limits as 120 and 3900
		//scale the value from 0 to 100 => 0 to 2800
	if(adc_value[6]<LFMaxPos)
	{
		//HAL_UART_Transmit(&huart1, (uint8_t*)buffer, sprintf(buffer, "S1 %d  %d \n", adc_value[6],isMoveF_LF ), 100);
		osSemaphoreWait(BinSemHandle, osWaitForever);
		// set the moving flag
		lfw= 1;

		pwmval = scale_val(pwmval,0,100, 0, 2800);
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, pwmval);
		osSemaphoreRelease(BinSemHandle);
		//HAL_UART_Transmit(&huart1, (uint8_t*)buffer, sprintf(buffer, "S2 %d  %d \n", adc_value[6],isMoveF_LF ), 100);
	}

}

void move_lb(int pwmval)
{
	if(adc_value[6]>LFMinPos)
	{
		osSemaphoreWait(BinSemHandle, osWaitForever);
		//isMoveB_LF = 1;
		lrw=1;
		//scale the value from 0 to 100 => 0 to 2800
		pwmval = scale_val(pwmval,0,100, 0, 2800);
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, pwmval);
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 0);
		osSemaphoreRelease(BinSemHandle);

	}

}

void move_rf(int pwmval)
{
	//	Check whether we are at the end positions
		//LF pos given by adc_value[6]
		// min value is around 100, max value 4000, so we set limits as 120 and 3900
		//scale the value from 0 to 100 => 0 to 2800
	if(adc_value[5]<RFMaxPos)
	{
		//HAL_UART_Transmit(&huart1, (uint8_t*)buffer, sprintf(buffer, "S1 %d  %d \n", adc_value[6],isMoveF_LF ), 100);
		osSemaphoreWait(BinSemHandle, osWaitForever);
		// set the moving flag
		rfw= 1;

		pwmval = scale_val(pwmval,0,100, 0, 2800);
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, pwmval);
		osSemaphoreRelease(BinSemHandle);
		//HAL_UART_Transmit(&huart1, (uint8_t*)buffer, sprintf(buffer, "S2 %d  %d \n", adc_value[6],isMoveF_LF ), 100);
	}

}

void move_rb(int pwmval)
{
	if(adc_value[5]>RFMinPos)
	{
		osSemaphoreWait(BinSemHandle, osWaitForever);
		//isMoveB_LF = 1;
		rrw=1;
		//scale the value from 0 to 100 => 0 to 2800
		pwmval = scale_val(pwmval,0,100, 0, 2800);
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwmval);
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
		osSemaphoreRelease(BinSemHandle);

	}

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

int scale_val(int inval, int inmin, int inmax, int outmin, int outmax)
{
	if (inval>=inmax)
	{
		inval = inmax;
	}
	else if(inval<=inmin)
	{
		inval =inmin;
	}

	double slope = 1.0 * (outmax - outmin) / (inmax - inmin);
	return outmin + slope * (inval - inmin);
}

int clamp_val(int inval, int minval, int maxval)
{
	if(inval<minval)
	{
		return minval;
	}
	else if(inval>maxval)
	{
		return maxval;
	}
	else
		return inval;
}

// send data to uart
uint8_t sendData(char* str)
{
	// check whether the buffer is empty after previous transmission
	if(uartTXBufferLength==0)
	{
		uartTXBufferLength = strlen(str);
		uartTXBufferIndex = 0;
		for(int i = 0;i<uartTXBufferLength;i++)
		{
			uartTXBuffer[i]=*str;
			++str;
		}
		//Transmit the first byte, and increment the index
		LL_USART_TransmitData8(USART1, uartTXBuffer[uartTXBufferIndex++]);
		// Enable Interrupt and let it handle the rest
		LL_USART_EnableIT_TXE(USART1);
	}
	else
	{
		// Buffer full, so return error
		return 1;
	}
	return 0 ;
}

/*The following function handles the UART ISR
Open stm32f1xx_it.c and add the following line inside "void USART1_IRQHandler(void)"
 	 LL_USART1_IRQHandler();
*/
void LL_USART1_IRQHandler()
{	// RX Interrupt
	if(LL_USART_IsActiveFlag_RXNE(USART1) == 0x01)
		{
			int data = LL_USART_ReceiveData8(USART1);
			osMessagePut(myQueue01Handle, data, osWaitForever);
		}
	// TX Interrupt
	if(LL_USART_IsEnabledIT_TXE(USART1) == 0x01 && LL_USART_IsActiveFlag_TXE(USART1) == 0x01)
		{
		//check whether we have transmitted all data in the TX  buffer
		if(uartTXBufferIndex>=uartTXBufferLength)
			{
			//if so, reset the buffer length and index
			uartTXBufferLength = 0;
			uartTXBufferIndex = 0;
			//Disable TX done interrupt
			LL_USART_DisableIT_TXE(USART1);
			}
		else
			{
			//Transmit another byte
			LL_USART_TransmitData8(USART1, uartTXBuffer[uartTXBufferIndex++]);
			}
		}
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

  // Start UART interrupts
  /* When UART gets 5 bytes, it calls the function
   * HAL_UART_RxCpltCallback(
   *
   */
  //if(uart_cmd_full=1)
  //{
  //HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 5);
  //}
  //else
  //{
	 // HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 5);
  //}
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


  	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);
  	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 0);
  	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
  	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
  	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);


  struct led_channel_info led_channels[WS2812_NUM_CHANNELS];

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
      lightupLED1(channel_framebuffers[0]);
      lightupLED2(channel_framebuffers[1]);
      // we have to disable interrupts while refreshing LEDs
   	  __disable_irq();
   	  ws2812_refresh(led_channels, GPIOB);
   	  __enable_irq();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of BinSem */
  osSemaphoreDef(BinSem);
  BinSemHandle = osSemaphoreCreate(osSemaphore(BinSem), 1);

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
  // PID timer runs at 200hz
  osTimerStart(pidTimerHandle, pid_time_period);
  //status update timer runs at 100 hz
  osTimerStart(statusUpdateHandle, 10);
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of myQueue01 */
  osMessageQDef(myQueue01, 128, uint16_t);
  myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of adcreader */
  //ADC Reader task is disabled in noir
 // osThreadDef(adcreader, adc_reader_task, osPriorityNormal, 0, 128);
 // adcreaderHandle = osThreadCreate(osThread(adcreader), NULL);

  /* definition and creation of serialreader */
  osThreadDef(serialreader, serial_reader_task, osPriorityHigh, 0, 128);
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

	  //sendData("hello world");
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

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */
  // enable the RX and TX interrupts
  LL_USART_EnableIT_RXNE(USART1);
  LL_USART_EnableIT_TXE(USART1);

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
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
	  		  osDelay(1);

	  		  // get initial readings
	  		  data_fl_noise = adc_value[0];
	  		  data_fr_noise = adc_value[1];

	  		  // set IR on
	  		  ir_led_on();
	  		  //small delay
	  		  osDelay(1);

	  		  // get second readings
	  		  data_fl = adc_value[0];
	  		  data_fr = adc_value[1];

	  		  //calculate the real value and set it in ir_data array
	  		  data_fl_real = -1*(data_fl - data_fl_noise);
	  		  data_fr_real = -1*(data_fr - data_fr_noise);

	  		  //fill the IR data buffers
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
	osEvent messageHandler;

		int numchr = 7;
		char data[numchr];
		int cmd_val=0;

  /* Infinite loop */
  for(;;)
  {

	  /*		Valid serial commands
	   * 	cop000e - open finger
	   * 	cosXXXe -  open finger with XXX speed
	   * 	ccpXXXe - close finger with position hold, XXX current threshold
	   * 	cccXXXe - close finger with current hold
	   * 	ccsXXXe - close finger with XXX speed
	   *    clpXXXe - left finger position roll
	   *    clfXXXe - move left finger forward with XXX speed
	   *    clrXXXe - move left finger reverse with XXX speed
	   *    crpXXXe - right finger position roll
	   *    crfXXXe - move right finger forward with XXX speed
	   *    crrXXXe - move right finger reverse with XXX speed
	   *    csl000e - stop left finger
	   *    csr000e - stop right finger
	   *    csg000e - stop gripper
	   *	csXXXXe - stop all
	   *	cbl000e - brake left finger
	   *    cbr000e - brake right finger
	   *    cbg000e - brake gripper
	   *	cbXXXXe - brake all
	   *	cgz000e - set zero for gripper gap
	   *	cggXXXe - set gripper gap to XXX ( min 0, Max 999)
	   *		  - Reset serial command queue
	   *
	   */

	  messageHandler = osMessageGet(myQueue01Handle, osWaitForever);

	  for(int i =0;i<(numchr-1);i++)
	  	  {
	  		  data[i]=data[i+1];
	  	  }
	  data[numchr-1] =messageHandler.value.p;
		  //check for start and end characters
		  if(data[0]=='c' && data[6]=='e')
		  {
			  // got command
			  // extract the numerical value
			  char val_ar[4]= {data[3], data[4], data[5], NULL};
			  cmd_val = atoi(val_ar);

			  switch(data[1])
			  {
			  case 'c': // CLOSE GRIPPER
			  	  	  {   //close in current control mode
			  	  		  if(data[2]=='c')
			  	  		  {
			  	  			gPid = 1;
			  	  		  }
			  	  		//close in position hold mode
			  	  		  else if(data[2]=='p')
			  	  		  {
			  	  			gForceThres = cmd_val;
			  	  			gPid = 2;
			  	  			close_gripper(100);
			  	  			startTick = HAL_GetTick();
			  	  		  }
			  	  		//close in speed control mode
			  	  		  else if(data[2]=='s')
			  	  		  {
			  	  			gPid = 3;
			  	  			close_gripper(cmd_val);
			  	  			startTick = HAL_GetTick();
			  	  		  }
			  	  	  }break;
			  case 's': // STOP MOTION
				  	  	// motors inputs disconnected, can move by hand
			  		  {
			  	  		  if(data[2]=='g')
			  	  		  {
			  	  			 stop_gripper(); // stop all motors
			  	  		  }
			  	  		  else if(data[2]=='l')
			  	  		  {
			  	  			stop_lf(); // stop left finger
			  	  		  }
			  	  		  else if(data[2]=='r')
			  	  		  {
			  	  			stop_rf(); // stop right finger
			  	  		  }


			  		 }break;
			  case 'b': // BRAKE MOTORS
				  	  	// motor inputs are shorted. cannot move by hand
			  		  {
			  	  		  if(data[2]=='g')
			  	  		  {
			  	  			brake_gripper(); // brake all motors
			  	  		  }
			  	  		  else if(data[2]=='l')
			  	  		  {
			  	  			  brake_lf(); // brake left finger
			  	  		  }
			  	  		  else if(data[2]=='r')
			  	  		  {
			  	  			  brake_rf(); // brake right finger
			  	  		  }
			  		 }break;
			  case 'o': // OPEN THE GRIPPPER
			  		  {
			  	  		  if(data[2]=='p')
			  	  		  {
			  	  			open_gripper(100); // open fully
			  	  			sendData("open");
			  	  		  }
			  	  		  else if(data[2]=='s')
			  	  		  {
			  	  			open_gripper(cmd_val); // open with speed control
			  	  		  }
			  		 }break;
			  case 'r': // UP DOWN CONTROL OF RIGHT FINGER
			  		  {
			  	  		  if(data[2]=='p')
			  	  		  {
			  	  			//right finger position control
			  	  			rPid= 1;
			  	  			rPosDesired =  scale_val(cmd_val,0,200,RFMinPos,RFMaxPos);
			  	  		  }

			  	  		  else if(data[2]=='f')
			  	  		  {
			  	  			  // move right finger at forward velocity
			  	  			move_rf(cmd_val);

			  	  		  }

			  	  		  else if(data[2]=='r')
			  	  		  {
			  	  			  // move right finger at reverse velocity
			  	  			move_rb(cmd_val);
			  	  			rPid=0;

			  	  		  }

			  		 }break;
			  case 'l': // UP DOWN CONTROL OF LEFT FINGER
			  		  {
			  	  		  if(data[2]=='p')
			  	  		  {
			  	  			  //left finger position control
			  	  			lPid = 1;
			  	  			lPosDesired = scale_val(cmd_val,0,200,LFMinPos,LFMaxPos);
			  	  		  }

			  	  		  else if(data[2]=='f')
			  	  		  {
			  	  			  // move left finger at forward velocity
			  	  			move_lf(cmd_val);
			  	  		  }

			  	  		  else if(data[2]=='r')
			  	  		  {
			  	  			  // move left finger at reverse velocity
			  	  			move_lb(cmd_val);
			  	  			lPid=0;
			  	  		  }

			  		 }break;
			  case 'g': // Gripper gap control
			  			  	  	  {
			  			  	  		  //M1z = 234;
			  			  	  		  if(data[2]=='z')
			  			  	  		  {
			  			  	  			  //get the current gap
			  			  	  		int gripper_gap = (M1MaxPos-adc_value[2])+(M2MaxPos-adc_value[3])-(M1MinPos+M2MinPos);
			  			  	  		gmin = gripper_gap-55;
			  			  	  			  //M1z=0;
			  			  	  			  //M2z =0;
			  			  	  			  // calibrate gripper gaps
			  			  	  			  //for(int j=0;j<100;j++){
			  			  	  				  //get the current gripper positions and average them
			  			  	  				//  M1z = M1z+adc_value[2];
			  			  	  				 // M2z = M2z+adc_value[3];

			  			  	  			  //}
			  			  	  			 // M1z = M1z/100.0;
			  			  	  			 // M2z = M2z/100.0;

			  			  	  		  }
			  			  	  		  else if(data[2]=='g')
			  			  	  		  {
			  			  	  			//int gmin = (M1MinPos+M2MinPos); // TODO - can we estimate this from zero position calibration
			  			  	  			//int gmax = (M1MaxPos-M1MinPos)+(M2MaxPos-M2MinPos)-(M1MinPos+M2MinPos);
			  			  	  			  gripperGapcmd = scale_val(cmd_val,0,999,gmin,gmax);;
					  			  	  			  gPid = 4;

			  			  	  		  }

			  			  	  	  }break;

			  }
		  }

	    osDelay(1);
    //HAL_UART_Transmit(&huart1, UART1_rxBuffer, 5, 100);

  }
  /* USER CODE END serial_reader_task */
}

/* pid_timer function */
void pid_timer(void const * argument)
{
	// ENSURE THAT THE MOTORS DOES NOT GO BEYOND MINIMUM/MAX POSITIONS
	//( if exceeded MAXPos, dont allow forward motion,
	//	but only allow reverse motion, similar with MINPos, for all motors)
	//
	// lfw is set only when left finger is moving forward
	// so check whether L finger goes beyond and stop the motion only
	// when L finger is moving forward (and then set lfw=0).
	// if we dont use flags like lfw, lrw, rfw, rrw, mgo, mgc, it will get
	// stuck at a position and cant move

  if((adc_value[6]>LFMaxPos) && (lfw==1))
	{
		osSemaphoreWait(BinSemHandle, osWaitForever);

		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 0);
		lfw=0;
		osSemaphoreRelease(BinSemHandle);
	}
	else if((adc_value[6]<LFMinPos) && (lrw==1))
	{
		osSemaphoreWait(BinSemHandle, osWaitForever);

		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 0);
		lrw=0;
		osSemaphoreRelease(BinSemHandle);
	}

	if((adc_value[5]>RFMaxPos) && (rfw==1))
		{
			osSemaphoreWait(BinSemHandle, osWaitForever);

			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
			rfw=0;
			osSemaphoreRelease(BinSemHandle);
		}
		else if((adc_value[5]<RFMinPos) && (rrw==1))
		{
			osSemaphoreWait(BinSemHandle, osWaitForever);

			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
			rrw=0;
			osSemaphoreRelease(BinSemHandle);
		}

	if((adc_value[3]<M1MinPos) && (mgo==1) &&(adc_value[4]<M2MinPos))
		{
			osSemaphoreWait(BinSemHandle, osWaitForever);

			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
			mgo=0;
			osSemaphoreRelease(BinSemHandle);
		}
		else if((adc_value[3]>M1MaxPos) && (mgc==1) &&(adc_value[4]>M2MaxPos))
		{
			osSemaphoreWait(BinSemHandle, osWaitForever);

			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
			mgc=0;
			osSemaphoreRelease(BinSemHandle);
		}

	//PID position control for LFinger
	if(lPid==1)
	{
		// get the commanded position
		// get current position
		// calculate error
		int error = lPosDesired - adc_value[6];
		l_error_integral = l_error_integral + error;
		int l_error_derivative = error  - l_error_prev;
		// calculate control value
		int l_ctrl   = (l_Kp * error) + ((l_Kd/pid_time_period)* l_error_derivative) + (l_Ki*l_error_integral*pid_time_period);
		// ensure control value is within limits
		l_ctrl = clamp_val(abs(l_ctrl), 20, 80);// constrain to max 80 % PWM since its a 6V motor at 12V
		//HAL_UART_Transmit(&huart1, (uint8_t*)buffer, sprintf(buffer, "PID L %d \t %d \t %d \t %d\n", lPosDesired, adc_value[6],error, l_ctrl), 100);
		// move motors
		if(error>lPosDelta)
		{
			//forward
			move_lf(l_ctrl);
			//HAL_UART_Transmit(&huart1, (uint8_t*)buffer, sprintf(buffer, "MOVE FWD %d\n", 1), 100);
		}
		else if(error<(-1*lPosDelta))
		{
			move_lb(l_ctrl);
			//HAL_UART_Transmit(&huart1, (uint8_t*)buffer, sprintf(buffer, "MOVE BACK %d\n", 1), 100);
		}
		else
		{
			brake_lf();
			l_error_integral = l_error_prev;
			lPid=0;
		//	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, sprintf(buffer, "REACHED L %d \t %d \t %d\n", lPosDesired, l_ctrl, error), 100);

		}
	}
	//PID position control for RFinger
	if(rPid==1)
	{
		// get the commanded position
		// get current position
		// calculate error
		int error = rPosDesired - adc_value[5];
		r_error_integral = r_error_integral + error;
		int r_error_derivative = error  - r_error_prev;
		// calculate control value
		int r_ctrl   = (r_Kp * error) + ((r_Kd/pid_time_period)* r_error_derivative) + (r_Ki*r_error_integral*pid_time_period);
		// ensure control value is within limits
		r_ctrl = clamp_val(abs(r_ctrl), 20, 80);// constrain to max 80 % PWM since its a 6V motor at 12V
		//HAL_UART_Transmit(&huart1, (uint8_t*)buffer, sprintf(buffer, "PID L %d \t %d \t %d \t %d\n", lPosDesired, adc_value[6],error, l_ctrl), 100);
		// move motors
		if(error>rPosDelta)
		{
			//forward
			move_rf(r_ctrl);
			//HAL_UART_Transmit(&huart1, (uint8_t*)buffer, sprintf(buffer, "MOVE FWD %d\n", 1), 100);
		}
		else if(error<(-1*rPosDelta))
		{
			move_rb(r_ctrl);
			//HAL_UART_Transmit(&huart1, (uint8_t*)buffer, sprintf(buffer, "MOVE BACK %d\n", 1), 100);
		}
		else
		{
			brake_rf();
			r_error_integral = r_error_prev;
			rPid=0;
		//	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, sprintf(buffer, "REACHED L %d \t %d \t %d\n", lPosDesired, l_ctrl, error), 100);

		}
	}

	// control of gripper motors
	if(gPid==2)
	{  // for mode in which gripper close, check for current and hold position if current> threshold
		// check for current
		if(adc_value[2]>gForceThres)
		{	// verify again after a short delay
			osDelay(2);
			if(adc_value[2]>gForceThres)
			{
				// brake
				brake_gripper();
				gPid = 0;
			}

		}
		// check for timeout
		if( abs(HAL_GetTick()-startTick)>gTimeOut)
		{
			// brake
			brake_gripper();
			gPid = 0;
		}
	}
	 if(gPid==3)
	{ // for velocity move mode, no current checking --<<< WARNING
		// check for timeout
				if( abs(HAL_GetTick()-startTick)>gTimeOut)
				{
					// brake
					brake_gripper();
					gPid = 0;
				}
	}
	 if(gPid==4)
	 	 {
		 int gripper_gap = (M1MaxPos-adc_value[2])+(M2MaxPos-adc_value[3])-(M1MinPos+M2MinPos);
	 		 int gap_error = gripperGapcmd - gripper_gap;
	 		// TODO  - implement PID
	 		 g_error_integral = g_error_integral + gap_error;
	 		 		int g_error_derivative = gap_error  - g_error_prev;
	 		 		// calculate control value
	 		 		int g_ctrl   = (g_Kp * gap_error) + ((g_Kd/pid_time_period)* g_error_derivative) + (g_Ki*g_error_integral*pid_time_period);

	 		 		g_ctrl = clamp_val(abs(g_ctrl), 75, 100);
	 		 		if(gap_error> gripperGapDelta)
	 		 		{
	 		 			open_gripper(g_ctrl);
	 		 		}
	 		 		else if(gap_error<(-1*gripperGapDelta))
	 		 		{
	 		 			close_gripper(g_ctrl);
	 		 		}
	 		 		else
	 		 		{
	 		 			brake_gripper();
	 		 			gPid = 0;
	 		 		}
	 	 }
  /* USER CODE END pid_timer */
}

/* status_update_timer function */
void status_update_timer(void const * argument)
{
  /* USER CODE BEGIN status_update_timer */

	/* Serial Data Format
	 *  M1-2_current, M1Pos, M2Pos, RFPos, LFPos, irsens_left[10], irsens_right[10]
	 *
	 */
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);

	int gripper_gap = (M1MaxPos-adc_value[2])+(M2MaxPos-adc_value[3])-(M1MinPos+M2MinPos);
	// 3600 when closed
	//6456 when fully open
	//int gmin = (M1MinPos+M2MinPos); // TODO - can we estimate this from zero position calibration
	//int gmax = (M1MaxPos-M1MinPos)+(M2MaxPos-M2MinPos)-(M1MinPos+M2MinPos);
gripper_gap =scale_val(gripper_gap, gmin, gmax, 0, 999);
	char MSG[180];
/*
		sprintf(MSG, "s\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\te\n",
				adc_value[2], adc_value[3], adc_value[4], scale_val(adc_value[5],RFMinPos,RFMaxPos,0,200), scale_val(adc_value[6],LFMinPos,LFMaxPos,0,200),
					irdata_fr[0],irdata_fr[1], irdata_fr[2], irdata_fr[3], irdata_fr[4], irdata_fr[5], irdata_fr[6],irdata_fr[7],irdata_fr[8],irdata_fr[9],
					irdata_fl[0],irdata_fl[1], irdata_fl[2], irdata_fl[3], irdata_fl[4], irdata_fl[5], irdata_fl[6],irdata_fl[7],irdata_fl[8],irdata_fl[9], gripper_gap);

*/

	sprintf(MSG, "s\t%d\t%d\t%d\t%d\t%d\t%d\te\n",
					adc_value[2], adc_value[3], adc_value[4],
					scale_val(adc_value[5],RFMinPos,RFMaxPos,0,200), scale_val(adc_value[6],LFMinPos,LFMaxPos,0,200),
					gripper_gap);


	sendData(MSG);
	//HAL_UART_Transmit_IT(&huart1, MSG, strlen(MSG));


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
