/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#define __FPU_PRESENT 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "arm_math.h"
#include "st7735.h"

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
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint32_t ADCbuf[BUFSIZE];
float32_t ADCdata[BUFSIZE];
float32_t FFTdata[BUFSIZE];
float32_t FFTdatamag[BUFSIZE/2];

float32_t MaxFreq = 0;
float32_t MaxVal = 0;
uint32_t MaxIndex = 0;
float32_t AvgMaxFreq = 0;
float32_t Speed = 0;
float32_t AvgSpeedBuf[20] = {0};
float32_t AvgSpeed = 0;

arm_rfft_fast_instance_f32 rfft;
arm_status status;

volatile uint8_t button_pressed = 0;
volatile uint8_t button_released = 0;
uint8_t n = 0;

/* Initial settings */
uint8_t menu = OFF;
uint8_t cutoff = OFF;
uint16_t threshold = 5000;
uint8_t average = OFF;
uint8_t yMax = 100;

enum item{item0, item1, item2, item3, item4, item5, item6, item7, item8};
enum item menu_item = item1;
uint8_t no_menu_items = 7;

enum modes{speed, fft, dual};
enum modes mode = speed;

plot pl = {25, 20, 125, 100, 100};	//One plot
plot pl1 = {25, 20, 125, 50, 50};	//Two plots, upper
plot pl2 = {25, 74, 125, 50, 50};	//Two plots, lower

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void drawfft_plot(plot p, float32_t arr[BUFSIZE]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len){
	  HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	  return len;
}

void init_plot(plot p){
	//Frame
	//Upper Horizontal line
	uint16_t framecolor = ST7735_GREEN;
	ST7735_DrawLine(p.posx, p.posy, p.posx+p.width, p.posy, framecolor);
	ST7735_DrawLine(p.posx, p.posy-1, p.posx+p.width, p.posy-1, framecolor);
	//Lower Horizontal line
	ST7735_DrawLine(p.posx, p.posy+p.height, p.posx+p.width, p.posy+p.height, framecolor);
	ST7735_DrawLine(p.posx, p.posy+p.height+1, p.posx+p.width, p.posy+p.height+1, framecolor);
	//Left Vertical line
	ST7735_DrawLine(p.posx, p.posy, p.posx, p.posy+p.height, framecolor);
	ST7735_DrawLine(p.posx-1, p.posy, p.posx-1, p.posy+p.height, framecolor);
	//Right Vertical line
	ST7735_DrawLine(p.posx+p.width, p.posy, p.posx+p.width, p.posy+p.height, framecolor);
	ST7735_DrawLine(p.posx+p.width+1, p.posy, p.posx+p.width+1, p.posy+p.height, framecolor);

	//Y-axis values
	char maxValStr[3];
	itoa(p.maxVal, maxValStr, 10);
	char midValStr[3];
	itoa(p.maxVal/2, midValStr, 10);
	char minValStr[2] = "0";
	ST7735_WriteString((p.posx-7*strlen(maxValStr)-2), p.posy, maxValStr, Font_7x10, ST7735_WHITE, ST7735_BLACK);
	ST7735_WriteString((p.posx-7*strlen(midValStr)-2), p.posy-3+(p.height/2), midValStr, Font_7x10, ST7735_WHITE, ST7735_BLACK);
	ST7735_WriteString((p.posx-7-2), p.posy+p.height-7, minValStr, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	//White background
	ST7735_FillRectangle(p.posx+1, p.posy+1, p.width-1, p.height-1, ST7735_WHITE);

	//Grid lines
	for(uint8_t i=p.posx+1; i<p.posx+p.width; i++){
		for(uint8_t j=p.posy+1; j<p.posy+p.height; j++){
			//Horizontal grid-lines
			if (j == p.posy+(p.height/4)){
				if (i%3 == 0){
					ST7735_DrawPixel(i, j, ST7735_BLACK);
				}else{
					ST7735_DrawPixel(i, j, ST7735_WHITE);
				}
			}

			if (j == p.posy+2*(p.height/4)){
				if (i%3 == 0){
					ST7735_DrawPixel(i, j, ST7735_BLACK);
				}else{
					ST7735_DrawPixel(i, j, ST7735_WHITE);
				}
			}

			if (j == p.posy+3*(p.height/4)){
				if (i%3 == 0){
					ST7735_DrawPixel(i, j, ST7735_BLACK);
				}else{
					ST7735_DrawPixel(i, j, ST7735_WHITE);
				}
			}

			//Vertical grid-lines
			if (i == 1+p.posx+(p.width/4)){
				if (j%3 == 0){
					ST7735_DrawPixel(i, j, ST7735_BLACK);
				}else{
					ST7735_DrawPixel(i, j, ST7735_WHITE);
				}
			}
			if (i == 1+p.posx+2*(p.width/4)){
				if (j%3 == 0){
					ST7735_DrawPixel(i, j, ST7735_BLACK);
				}else{
					ST7735_DrawPixel(i, j, ST7735_WHITE);
				}
			}
			if (i == 1+p.posx+3*(p.width/4)){
				if (j%3 == 0){
					ST7735_DrawPixel(i, j, ST7735_BLACK);
				}else{
					ST7735_DrawPixel(i, j, ST7735_WHITE);
				}
			}
		}
	}
}

void draw_plot(plot p, uint8_t val){

	if (n == 0){
		n = p.posx+1;
	}else if (n > p.posx+p.width-1){
		n = p.posx+1;
		init_plot(p);
	}

	int div = p.maxVal*10/p.height;

	int datapoint = val*10/div;
	datapoint = p.posy+p.height-datapoint;

	//Boundaries
	if (datapoint < p.posy+1){
		datapoint = p.posy+1;
	} else if (datapoint > p.posy+p.height-1){
		datapoint = p.posy+p.height-1;
	}

	//Don't draw pixels beyound boundaries
	if (datapoint > p.posy+1){
		ST7735_DrawPixel(n, datapoint-1, ST7735_BLUE);
	}
	if (datapoint < p.posy+p.height-1){
		ST7735_DrawPixel(n, datapoint+1, ST7735_BLUE);
	}
	ST7735_DrawPixel(n+1, datapoint, ST7735_BLUE);
	ST7735_DrawPixel(n-1, datapoint, ST7735_BLUE);
	ST7735_DrawPixel(n, datapoint, ST7735_BLUE);
}

void drawfft_plot(plot p, float32_t arr[]){

	int div = p.maxVal*10/p.height;
	int datapoint = 0;

	for (int i = 1; i<p.width; i++){
		datapoint = arr[i]/100/div;
		datapoint = p.posy+p.height-datapoint;

		//Boundaries
		if (datapoint < p.posy+1){
			datapoint = p.posy+1;
		} else if (datapoint > p.posy+p.height-1){
			datapoint = p.posy+p.height-1;
		}

		ST7735_DrawLine(i+p.posx, p.posy+p.height-1, i+p.posx, datapoint, ST7735_BLUE);
	}

	datapoint = p.posy+p.height-(MaxVal/100/div);
	if (datapoint < p.posy+1){
		datapoint = p.posy+1;
	} else if (datapoint > p.posy+p.height-1){
		datapoint = p.posy+p.height-1;
	}
	ST7735_DrawLine(MaxIndex+p.posx, p.posy+p.height-1, MaxIndex+p.posx, datapoint, ST7735_RED);
}

void draw_menu(void){
	char TFT_line1[22] = {0};
	char TFT_line2[22] = {0};
	char TFT_line3[22] = {0};
	char TFT_line4[22] = {0};
	char TFT_line5[22] = {0};
	char TFT_line6[22] = {0};
	char TFT_line7[22] = {0};
	char whitespace[22] = "                     ";

	/* Menu Title */
	strcpy(TFT_line1, "Settings");
	ST7735_WriteString((ST7735_WIDTH/2)-(strlen(TFT_line1)/2*7), 0, TFT_line1, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	/* Menu item 1 */
	strcpy(TFT_line2, "Mode: ");
	if (mode == speed)
		strcat(TFT_line2, "Speed");
	if (mode == fft)
		strcat(TFT_line2, "FFT");
	if (mode == dual)
		strcat(TFT_line2, "Dual");
	strncat(TFT_line2, whitespace, 21-strlen(TFT_line2));
	if (menu_item != item1)
		ST7735_WriteString(0, 20, TFT_line2, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	/* Menu item 2 */
	strcpy(TFT_line3, "Max plot val: ");
	char yMaxStr[4];
	itoa((int)yMax, yMaxStr, 10);
	strcat(TFT_line3, yMaxStr);

	strncat(TFT_line3, whitespace, 21-strlen(TFT_line3));
	if (menu_item != item2)
		ST7735_WriteString(0, 30, TFT_line3, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	/* Menu item 3 */
	strcpy(TFT_line4, "Cutoff freq: ");

	char cutoffStr[5];
	itoa((int)cutoff*FFTRESOLUTION, cutoffStr, 10);
	strcat(TFT_line4, cutoffStr);
	strcat(TFT_line4, " Hz");

	strncat(TFT_line4, whitespace, 21-strlen(TFT_line4));
	if (menu_item != item3)
		ST7735_WriteString(0, 40, TFT_line4, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	/* Menu item 4 */
	strcpy(TFT_line5, "Threshold: ");

	char thresholdStr[5];
	itoa((int)threshold, thresholdStr, 10);
	strcat(TFT_line5, thresholdStr);

	strncat(TFT_line5, whitespace, 21-strlen(TFT_line5));
	if (menu_item != item4)
		ST7735_WriteString(0, 50, TFT_line5, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	/* Menu item 5 */
	strcpy(TFT_line6, "Averaging: ");

	char averageStr[5];
	itoa((int)average, averageStr, 10);
	strcat(TFT_line6, averageStr);

	strncat(TFT_line6, whitespace, 21-strlen(TFT_line6));
	if (menu_item != item5)
		ST7735_WriteString(0, 60, TFT_line6, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	/* Menu item 6 */
	strcpy(TFT_line7, "Save & Exit          ");
	if (menu_item != item6)
		ST7735_WriteString(0, 70, TFT_line7, Font_7x10, ST7735_WHITE, ST7735_BLACK);

	/* Highlighted item */
	switch (menu_item){
		case item1:
			ST7735_WriteString(0, 20, TFT_line2, Font_7x10, ST7735_BLACK, ST7735_WHITE);
			break;
		case item2:
			ST7735_WriteString(0, 30, TFT_line3, Font_7x10, ST7735_BLACK, ST7735_WHITE);
			break;
		case item3:
			ST7735_WriteString(0, 40, TFT_line4, Font_7x10, ST7735_BLACK, ST7735_WHITE);
			break;
		case item4:
			ST7735_WriteString(0, 50, TFT_line5, Font_7x10, ST7735_BLACK, ST7735_WHITE);
			break;
		case item5:
			ST7735_WriteString(0, 60, TFT_line6, Font_7x10, ST7735_BLACK, ST7735_WHITE);
			break;
		case item6:
			ST7735_WriteString(0, 70, TFT_line7, Font_7x10, ST7735_BLACK, ST7735_WHITE);
			break;
		case item7:
			break;
		case item8:
			break;
		default:
			break;
	}

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){

	/* Copy ADC data */
	for (int i = 0; i<BUFSIZE; i++){
		ADCdata[i] = ADCbuf[i];
	}

	/* FFT */
	if ((status = arm_rfft_fast_init_f32(&rfft, BUFSIZE)) == ARM_MATH_SUCCESS){
		arm_rfft_fast_f32(&rfft, ADCdata, FFTdata, 0);
	}

	/* Remove DC component */
	FFTdata[0] = 0;

	/* Get magnitude */
	arm_cmplx_mag_f32(FFTdata, FFTdatamag, BUFSIZE);

	/* Remove frequencies if cutoff enabled */
	if (cutoff > 0){
		for (int i = 0; i<=cutoff; i++)
		FFTdatamag[i] = 0;
	}

	/* Get strongest frequency */
    arm_max_f32(FFTdatamag, BUFSIZE/2, &MaxVal, &MaxIndex);
    MaxFreq = MaxIndex*FFTRESOLUTION;

    /* Calculate speed */
    if (MaxVal > threshold){
    	Speed = MaxFreq/44;
    }else{
    	Speed = 0;
    }

    if (average > 0){
    	static int k;
    	AvgSpeedBuf[k++] = Speed;

    	if (k == average-1){
    		k = 0;

    		for (int i = 0; i<average; i++){
    			AvgSpeed += AvgSpeedBuf[i];
    		}
			AvgSpeed /= average;
    	}
    }


	if (button_pressed == 1){
	  if (mode == speed){
		  init_plot(pl);
	  }else if(mode == fft){
		  init_plot(pl);
	  }else if (mode == dual){
		  init_plot(pl1);
		  init_plot(pl2);
	  }
	  button_pressed = 0;
	}

    if (menu == OFF){
    	if (mode == speed){
    			  if (!HAL_GPIO_ReadPin(PORT_TRIGGERBUTTON, PIN_TRIGGERBUTTON)){
    				  char TFT_line1[22] = {0};
    				  if (average == OFF){
    					  itoa((int)Speed, TFT_line1, 10);
    				  	  strcat(TFT_line1, " km/h   ");
    				  	  ST7735_WriteString((ST7735_WIDTH/2)-(3*11), 0, TFT_line1, Font_11x18, ST7735_WHITE, ST7735_BLACK);
    				  	  draw_plot(pl, (uint8_t)Speed);
    				  }else{
    					  itoa((int)AvgSpeed, TFT_line1, 10);
    				  	  strcat(TFT_line1, " km/h   ");
    				  	  ST7735_WriteString((ST7735_WIDTH/2)-(3*11), 0, TFT_line1, Font_11x18, ST7735_WHITE, ST7735_BLACK);
    				  	  draw_plot(pl, (uint8_t)AvgSpeed);
    				  }
    		  	  	  n++;
    		  	  }
    		  }else if (mode == fft){
    			  if (!HAL_GPIO_ReadPin(PORT_TRIGGERBUTTON, PIN_TRIGGERBUTTON)){
    				  init_plot(pl);

    				  char TFT_line1[22] = {0};
    			  	  itoa((int)MaxFreq, TFT_line1, 10);
    			  	  strcat(TFT_line1, " Hz   ");

    				  char MaxValStr[6];
    				  itoa((int)MaxVal, MaxValStr, 10);
    			  	  strcat(TFT_line1, MaxValStr);
    			  	  strncat(TFT_line1, "           ", 16-strlen(TFT_line1));

    			  	  ST7735_WriteString(6*7, 5, TFT_line1, Font_7x10, ST7735_WHITE, ST7735_BLACK);

    			  	  drawfft_plot(pl, FFTdatamag);
    			  }
    		  }else if (mode == dual){
    			  if (!HAL_GPIO_ReadPin(PORT_TRIGGERBUTTON, PIN_TRIGGERBUTTON)){
    				  char TFT_line1[22] = {0};
    				  itoa((int)Speed, TFT_line1, 10);
    				  strcat(TFT_line1, " km/h   ");

    				  char MaxValStr[6];
    				  itoa((int)MaxVal, MaxValStr, 10);
    				  strcat(TFT_line1, MaxValStr);
    				  strncat(TFT_line1, "           ", 16-strlen(TFT_line1));

    				  ST7735_WriteString(5*7, 5, TFT_line1, Font_7x10, ST7735_WHITE, ST7735_BLACK);
    				  draw_plot(pl1, (uint8_t)Speed);
    				  n++;

    				  init_plot(pl2);
    				  drawfft_plot(pl2, FFTdatamag);
    			  }
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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  /* Init TFT */
  ST7735_Init();
  ST7735_FillScreen(ST7735_BLACK);

  /* Start in speed mode */
  mode = speed;

  /* Draw a blank plot */
  init_plot(pl);

  /* Enable TFT-LED */
  HAL_GPIO_WritePin(PORT_TFTLED, PIN_TFTLED, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (button_released == 1){
		  n = 0;
		  button_released = 0;
	  }

	  /* Settings menu */
	  if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)){
		  	  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

		  	  menu = ON;
			  menu_item = item1;
			  ST7735_FillScreen(ST7735_BLACK);
			  draw_menu();
			  HAL_Delay(200);

			  while(menu){
				  if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)){
					  menu_item++;
					  if (menu_item >= no_menu_items){
						  menu_item = item1;
					  }
					  draw_menu();
					  HAL_Delay(120);
				  }

				  if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)){
					  switch (menu_item){
					  case item1:
						  mode++;
						  if (mode > dual){
							  mode = speed;
						  }
						  draw_menu();
						  break;
					  case item2:
						  yMax = yMax +20;
						  if (yMax > 200){
							  yMax = 20;
						  }
						  draw_menu();
						  break;
					  case item3:
						  cutoff+=2;
						  if (cutoff > 30){
							  cutoff = 0;
						  }
						  draw_menu();
						  break;
					  case item4:
						  threshold = threshold+500;
						  if (threshold > 5000){
							  threshold = 0;
						  }
						  draw_menu();
						  break;
					  case item5:
						  average = average + 2;
						  if (average > 10){
							  average = 0;
						  }
						  draw_menu();
						  break;
					  case item6:
						  menu = OFF;
						  //update y-max in plots
						  pl.maxVal = yMax;
						  pl1.maxVal = yMax;
						  pl2.maxVal = yMax;

						  //Redraw plot when exiting menu
						  ST7735_FillScreen(ST7735_BLACK);
						  switch(mode){
						  	  case speed:
						  		  init_plot(pl);
						  		  break;
						  	  case fft:
						  		  init_plot(pl);
						  		  break;
						  	  case dual:
						  		  init_plot(pl1);
						  		  init_plot(pl2);
						  		  break;
						  }
						  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
						  break;
					  default:
						  break;
					  }
					  HAL_Delay(120);
				  }
			  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim2.Init.Prescaler = 8;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 443;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  huart1.Init.BaudRate = 256000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TFT_CS_Pin|TFT_RST_Pin|TFT_A0_Pin|TFT_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 BTN_MENU_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|BTN_MENU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_TRIGGER_Pin */
  GPIO_InitStruct.Pin = BTN_TRIGGER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_TRIGGER_GPIO_Port, &GPIO_InitStruct);
  //HAL_GPIO_WritePin(GPIOB, BTN_TRIGGER_Pin, BTN_TRIGGER_GPIO_Port);

  /*Configure GPIO pins : TFT_CS_Pin TFT_RST_Pin TFT_A0_Pin TFT_LED_Pin */
  GPIO_InitStruct.Pin = TFT_CS_Pin|TFT_RST_Pin|TFT_A0_Pin|TFT_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
