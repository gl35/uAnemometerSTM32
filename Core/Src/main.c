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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdbool.h>
#include "math.h"
#include "i2c-lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define nShutdownDampHigh HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)
#define nShutdownDampLow HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA_HandleTypeDef hdma_tim1_ch2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void blinkLED ();
char *scanInp(void);
void startSpeaker(bool start);
float *calTime(void);
float *delta_T_alg (float pulseW);
float *getTemp(void);
void startSineW(bool start);
void lcd_disp(void);

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define __USE_C99_MATH

//data buffer for rising and falling DMA
#define numval 1000
#define TIMCLOCK 170000000
#define PSCALAR 16
#define MIN(a,b) ((a) < (b) ? (a) : (b))

int riseCaptured = 0;
int fallCaptured = 0;
uint32_t riseData[numval];
uint32_t fallData[numval];
int isMeasured = 0;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//parameter for pulse width cal
volatile uint16_t tim1, tim2;
uint8_t IsFirstCaptured = 0;
float timeFactor = 5.882;//100;  //100ns per tick when fclk = 10Mhz// 5.882ns/tick if 170Mhz clock
volatile uint16_t *deltaT = 0;
float pulseW = 0;

float dist = 0.2; // dist = 0.2m
float freq = 25000;  //25khz
float cal_val = 0;
float v_sound = 343; // speed of sound  is 343m/s, will need to incorporate temperature here,  c = 331+0.61T,
float lambda = 0;
float temp = 0;
float pathDiff = 0;  //k = 0.7288 , d/lambda,  if d=0.2m, k=22.75,  *made pathDiff to 1 eliminated for testing, k=0.881 by experiment
float windspeed = 0;


//code for getchar and putchar properly display
/*volatile uint32_t gGetCharCount = 0;
int __io_getchar(void)
{
	++gGetCharCount;
	return 42;
}*/

/*void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6); //pin 6 are for rising edges
}*/

void blinkLED ()
{
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	HAL_Delay(500);
	return;
}

// using putchar and getchar
char *scanInp(void)
{
	char *ch_buf = malloc (sizeof (char) * 1);
	scanf("%1s", ch_buf);
	//printf("you have entered %s\n\r", ch_buf);   //no longer need to print what have entered
	return ch_buf;
}

//function to start and stop the speaker
/*void startSpeaker(bool start)
{
	if (!start){
		  //start pwm for 25khz - freq = (fmas/psc)/period
		  //HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);  //start TIM4 pwm ch1 - macro expan 0x00000000U
		  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		  nShutdownDampLow;
	}
	else {
		nShutdownDampHigh;
		//HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	}
	  return;
}*/

//put char prototype for printf function
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);  //use dma to speed it up
//	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&ch, 1);
  return ch;
}

/*prototype for scanf function*************************************************/
#ifdef __GNUC__
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif

GETCHAR_PROTOTYPE
{
  uint8_t ch = 0;
  /* Clear the Overrun flag just before receiving the first character */
  __HAL_UART_CLEAR_OREFLAG(&huart2);
  /* Wait for reception of a character on the USART RX line and echo this
   * character on console */
  HAL_UART_Receive(&huart2, (uint8_t *)&ch, 1, 100);
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 100);
//  HAL_UART_Receive_DMA(&huart2, &ch, 1);
//  HAL_UART_Transmit_DMA(&huart2, &ch, 1);

  return ch;
}


/************************************************************************************/

//calculate delta t function
/*float *calTime(void)
{
	// would probably need to do the void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) with proper rising and falling edges of the pin input
	volatile uint32_t tim1, tim2;
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1)
	{
		tim1 = TIM1->CNT;
	}
	else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0)
	{
		tim2 = TIM1->CNT;
	}
	if (tim2 > tim1)
	{
		*deltaT = tim2 - tim1;
	}
	else if (tim1 > tim2)
	{
		*deltaT = (0xffff - tim1) + tim2;
	}
	printf("tim1 = %ld\r\n", tim1);
	printf("tim2 = %ld\r\n", tim2);
	return deltaT;
}*/

// a callback function ** not sure where the pin capture is used for this callback  -- this needs another approach
//this uses change 1 to measure the rising and falling edges without DMA
/*void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		if (IsFirstCaptured == 0)
		{
			tim1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			IsFirstCaptured = 1;
			//__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else
		{
			tim2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			//__HAL_TIM_SET_COUNTER(htim, 0);
			if (tim2 > tim1)
			{
				deltaT = tim2 - tim1;
			}
			else if (tim1 > tim2)
			{
				deltaT = (0xffff - tim1) + tim2;
			}

			pulseW = *deltaT * timeFactor/1000000000;
			windspeed = 0.01/(pulseW*pathDiff);  //d/(t*path diff)
			IsFirstCaptured = 0;
			__HAL_TIM_SET_COUNTER(htim, 0);
			//__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			//__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		}
	}
}
*/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	//if the interrupt is triggered by 1st Channel
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		riseCaptured = 1;
	}
	//if the interrupt is triggered by 2nd Channel
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		fallCaptured = 1;
	}

	if ((riseCaptured) && (fallCaptured))
	{
		// calculate the reference clock
		//float refClock = TIMCLOCK/(PSCALAR+1);
		int indxr = 0;
		int indxf = 0;
		int countr = 0;
		int countrf = 0;

		float riseavg = 0;
		float rfavg = 0;
		/* In case of high Frequencies, the DMA sometimes captures 0's in the beginning.
		 * increment the index until some useful data shows up
		*/
		while (riseData[indxr] == 0) indxr++;

		/* Again at very high frequencies, sometimes the values don't change
		* So we will wait for the update among the values
		 */
		while ( (MIN( (riseData[indxr+1]-riseData[indxr]), (riseData[indxr+2]-riseData[indxr+1]) ) ) == 0) indxr++;
		/* riseavg is the difference in the 2 consecutive rise Time */

		/* Assign a start value to riseavg */
		riseavg += MIN((riseData[indxr+1]-riseData[indxr]), (riseData[indxr+2]-riseData[indxr+1]));
		indxr++;
		countr++;
		/* start adding the values to the riseavg */
		while (indxr < (numval))
		{
			riseavg += MIN((riseData[indxr+1]-riseData[indxr]), riseavg/countr);
			countr++;
			indxr++;
		}
		/* Find the average riseavg, the average time between 2 RISE */
		riseavg = riseavg/countr;
		indxr = 0;
		/* The calculation for the Falling pulse on second channel */
		/* If the fall time is lower than rise time,
		 * Then there must be some error and we will increment
		 * both, until the error is gone
		*/
		if (fallData[indxf] < riseData[indxr])
		{
			indxf+=2;
			indxr+=2;
			while (fallData[indxf] < riseData[indxr]) indxf++;
		}

		else if (fallData[indxf] > riseData[indxr])
		{
			indxf+=2;
			indxr+=2;
			while (fallData[indxf] > riseData[indxr+1]) indxr++;
		}

		/* The method used for the calculation below is as follows:
		* If Fall time < Rise Time, increment Fall counter
		* If Fall time - Rise Time is in between 0 and (difference between 2 Rise times), then its a success
		 * If fall time > Rise time, but is also > (difference between 2 Rise times), then increment Rise Counter
		 */
		while ((indxf < (numval)) && (indxr < (numval)))
		{
			/* If the Fall time is lower than rise time, increment the fall indx */
			while ((int16_t)(fallData[indxf]-riseData[indxr]) < 0)
			{
				indxf++;
			}
			/* If the Difference in fall time and rise time is >0 and less than rise average,
			 * Then we will register it as a success and increment the countrf (the number of successes)
			 */
			if (((int16_t)(fallData[indxf]-riseData[indxr]) >= 0) && (((int16_t)(fallData[indxf]-riseData[indxr]) <= riseavg)))
			{
				rfavg += MIN((fallData[indxf]-riseData[indxr]), (fallData[indxf+1]-riseData[indxr+1]));
				indxf++;
				indxr++;
				countrf++;
			}
			else
			{
				indxr++;
			}
		}
		/* Calculate the Average time between 2 Rise */
		rfavg = rfavg/countrf;
		v_sound = 331+0.61*temp;
		//pulseW = *deltaT * timeFactor/1000000000;
		pulseW = (rfavg)*timeFactor/1000000000;  //converting ns to s

//		deltaT_real = *delta_T_alg(pulseW);
		//windspeed = dist/((pulseW)*cali_val);  //d/(t*path diff)
//		windspeed = dist/(deltaT_real*cal_val) - v_sound; //might need to make this an array and take the average*/
		windspeed = dist/(pulseW*cal_val) - v_sound; //might need to make this an array and take the average
		riseCaptured = 0;
		fallCaptured = 0;
		isMeasured = 1;
		}
	}

float T = 0;
float pluseW_pre =0;
float deltaT_pre = 0;
float deltaT_prim = 0;

float *delta_T_alg (float deltaT)
{
	T = 1/freq;
	if (deltaT <= deltaT_pre/1000000000 + T)
	{
		deltaT_prim = deltaT;
	}
	else {
		deltaT_prim = deltaT - T;
	}
	deltaT_pre = deltaT_prim;
	return &deltaT_pre;
}

// isr callback function -- uncomment if using interrupt mode or dma mode.  Right now we are using polling mode.
/*void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//DMA mode
	//adc_val = buffer;  //store the value in adc_val

	//interrupt mode
	adc_val = HAL_ADC_GetValue(&hadc1);  //read adc value
	 if the continuos conversion is disabled, we need to start the ADC again here
	HAL_ADC_Start_IT(&hadc1);
	return;
}*/

//get adc value for temperature

float adc_val;
//uint32_t buffer;
//float adc_val_f;
//uint16_t adc_val;

float *getTemp(void)
{
	//HAL_ADC_Start_IT(&hadc1); //start the adc in interrupt mode
	//HAL_ADC_Start_DMA(&hadc1, &buffer, 1);  //start in DMA mode

	//using polling method***********************************

	HAL_ADC_Start(&hadc1); // start the adc
	HAL_ADC_PollForConversion(&hadc1, 100); // poll for conversion
	adc_val = HAL_ADC_GetValue(&hadc1); // get the adc value
	temp = (adc_val/4095) *(125-40);

	HAL_ADC_Stop(&hadc1); // stop adc
	/**********************************************************************/

	//usig DMA method
	//HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	//HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_val, 1);
	//adc_val_f = adc_val;
//	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_val, 1);
//	temp = (adc_val/4095) *(125-40);

	//printf("adc value = %f\r\n", temp);
	//HAL_Delay (100); // wait for 500ms
//temp = (adc_val/4095) *(125-40);
	return &temp;
}
/*void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  adc_val = HAL_ADC_GetValue(&hadc1);
//	adc_val = buffer;
	//HAL_ADC_Stop_DMA(&hadc1);

  return;
  If continuousconversion mode is DISABLED uncomment below
  //HAL_ADC_Start_IT (&hadc1);
}*/

//dac sine**************************************************/
uint32_t sine_val[100];
#define PI 3.1415926

float dac_val = 1.2;
uint32_t var;

void startSineW(bool start)
{
	if (!start)
	{
		//HAL_TIM_Base_Stop(&htim2);
		HAL_DAC_Stop_DMA(&hdac1, DAC1_CHANNEL_1);
	}
	else {
		 //HAL_TIM_Base_Start(&htim2);
		 HAL_DAC_Start_DMA(&hdac1, DAC1_CHANNEL_1, sine_val, 100, DAC_ALIGN_12B_R);
		 for (int i=0; i<100; i++)
		 {
			 sine_val[i] = ((sin(i*2*PI/100) + 1)*(4096/2));
		 }
	}
}
/******************************************************************/
// lcd display*****************************************************/

void lcd_disp(void)
{
	char * fltChar = malloc (sizeof (char) * 7);
	//lcd_send_cmd (0x80);
	//char fltChar [7];
	sprintf(fltChar, "%.4f", windspeed);
	lcd_put_cur(0,0);
	lcd_send_string("Windspeed=");
	//lcd_send_data((windspeed/10) +48);
	lcd_send_string(fltChar);
	lcd_send_string(" ");
	lcd_put_cur(1,0);
	lcd_send_string("m/s");
	HAL_Delay(500);

	return;
}
/********************************************************************/

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
 enum State {IDLE = 0, START = 1, CALTIME = 2, STOP = 3}; //define the number of states
 char State = IDLE;
 char inp;

 setvbuf(stdin, NULL, _IONBF, 0);

lambda = v_sound/freq;  //wavelength
pathDiff = (dist/lambda)-0.5;  //destructive interference, L/lambda-0.5=delta_L

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
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_1, riseData, numval);
  HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_2, fallData, numval);

  //say something
  //uart_buf_len = sprintf(uart_buf, "Timer test\r\n");
  //HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

  //start pwm timer
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);  //start TIM4 pwm ch1 - macro expan 0x00000000U

  lcd_init(); 														//initialize the lcd
  //start timer 2 for the sinewave
  HAL_TIM_Base_Start(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  blinkLED();  //call the blinkLED function
	  getTemp();
	  lcd_disp();

	  if (isMeasured)
	  {
		  TIM1->CNT = 0;
		  HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_1, riseData, numval);
		  HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_2, fallData, numval);
		  isMeasured = 0;
	  }
	  //scan for user inpt for the state machine
	 inp = *scanInp();
	  //wait again so we don't flood the serial terminal
	  //HAL_Delay(100);

	  if (inp == 'i')
	  {
		  State = IDLE;
	  }
	  else if (inp == 's')
	  {
		  State = START;
	  }
	  else if (inp == 't')
	  {
		  State = STOP;
	  } else {};

	  //State Machine starts here
	  switch (State)
	  {
	  case IDLE:
		 // uart_buf_len = sprintf(uart_buf, "In IDLE state\r\n");
		  printf("In IDLE State\r\n");
		  //HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
		  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); /// disable nShutdown pin for digital amp
		  //startSpeaker(0); //turn off speaker
		  startSineW(0); //using sinewave instead
		  HAL_Delay(1000);

		  break;

	  case START:
		  //nShutdownDamp = 1; // start digital amplifier
		  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		  printf("In START State\r\n"); // print status in terminal
		  //startSpeaker(1);
		  startSineW(1); //using sinewave instead
		  //uart_buf_len = sprintf(uart_buf, "In Start State\r\n");
		  //HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
		  /*the following is use to do pulse output and measure the very beginning****
		  //HAL_Delay(100);  //wait 20ms
		  //startSpeaker(0);
		   */
		  cal_val = dist/(pulseW*v_sound);  //get the calibration value from the drift and temperature
		  State = CALTIME;
		  break;

	  case CALTIME:
		  //deltaT = *calTime();
		  //printf("Delay is = %f\r\n", deltaT);
		  //HAL_TIM_IC_CaptureCallback(&htim1);
		  //printf("time1 val = %d \r\n", tim1);
		  //printf("time2 val = %d \r\n", tim2);
		  //printf("delay is %ld\r\n", deltaT);
		  //printf("pulseW is %.8f\r\n", pulseW);
		  //printf("windspeed is %f\r\n", windspeed);
		  //HAL_Delay(1000);  //wait 100ms
		  //State = START;
		  break;

	  case STOP:
		  //nShutdownDamp = 0; //stop digital amplifier
		  //startSpeaker(0);
		  startSineW(0); //using sinewave instead
		  printf("In STOP State\r\n"); // print status in terminal
		  HAL_Delay(100);  //wait 100ms
		  break;

	  default:
		  /*uart_buf_len = sprintf(uart_buf, "In Default State\r\n");
  		  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);*/
		  printf("In DEFAULT State\r\n"); // print status in terminal
		  HAL_Delay(100);  //wait 100ms
		  break;

	  }

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x40B285C2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
 // timer for PLL pulse width input capture. If we use prescaler 0, fclk = 170Mhz, then 5.882ns/tick
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
	//this is used to generate a sinewave, f_sys_clk/(prescalar * 100)/4 = 25khz
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 17-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  //6800-1  , this gives 25khz, f_pwm = fclk/(psc*(arr-1))
  //arr/2-1 = 6800/2-1
  //timer 4 is unused right now since we are using the sinewave from timer 2

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 6799;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 3399;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_ENABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// called when first half of buffer is filled

/* USER CODE END 4 */

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
