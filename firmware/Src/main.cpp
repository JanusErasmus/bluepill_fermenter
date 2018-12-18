/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include <time.h>

#include "Utils/cli.h"
#include "Utils/terminal_serial.h"
#include "stm32f1xx_hal.h"

#include "Utils/terminal.h"
#include "Utils/utils.h"
#include "usb_device.h"

#include "interface_nrf24.h"
#include "stm32_tm1637.h"
#include "fermenter.h"

uint8_t netAddress[] = {0x03, 0x44, 0x55};
#define payload_length 16

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;
SPI_HandleTypeDef hspi1;
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
extern "C" {
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
}

/* Private function prototypes -----------------------------------------------*/
typedef struct {
	uint32_t timestamp;		//4
	uint8_t inputs;			//1
	uint8_t outputs;		//1
	uint16_t voltages[4];	//8
	uint16_t temperature;	//2
}__attribute__((packed, aligned(4))) nodeData_s;

void sampleAnalog(double &temperature, double &voltage0, double &voltage1, double &voltage2)
{
	uint32_t adc0 = 0;
	uint32_t adc1 = 0;
	uint32_t adc2 = 0;
	uint32_t adc3 = 0;
	uint32_t adc4 = 0;

	HAL_ADCEx_Calibration_Start(&hadc1);

	for (int k = 0; k < 16; ++k)
	{
		HAL_ADC_Start(&hadc1);
		if(HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
		{
			adc0 += HAL_ADC_GetValue(&hadc1);
			//printf("ADC0: %d\n", adc0);
		}


		HAL_ADC_Start(&hadc1);
		if(HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
		{
			adc1 += HAL_ADC_GetValue(&hadc1);
			//printf("ADC1: %d\n", adc1);
		}

		HAL_ADC_Start(&hadc1);
		if(HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
		{
			adc2 += HAL_ADC_GetValue(&hadc1);
			//printf("ADC0: %d\n", adc2);
		}

		HAL_ADC_Start(&hadc1);
		if(HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
		{
			adc3 += HAL_ADC_GetValue(&hadc1);
			//printf("ADC1: %d\n", adc3);
		}

		HAL_ADC_Start(&hadc1);
		if(HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
		{
			adc4 += HAL_ADC_GetValue(&hadc1);
			//printf("ADC2: %d\n", adc4);
		}

		HAL_Delay(100);
	}

	adc0 >>= 4;
	adc1 >>= 4;
	adc2 >>= 4;
	adc3 >>= 4;
	adc4 >>= 4;

	//this amount of steps measure 1.2V
	double step = 1.2 / adc0;
	//printf("vref %d - ADC Step %d\n", (int)adc0, (int)step);

	//printf(" .	%d\n", (int)adc1);
	double voltage = ((double)adc1 * step);
	//printf(" *	%0.3f\n", voltage);
	voltage = 1.43 - voltage;
	//printf(" -	%0.3f\n", voltage);
	voltage /= 0.0043;
	//printf(" /	%0.3f\n", voltage);
	temperature = (25.0 + voltage) - 11;

	//measure raw voltage
	voltage0 = (((double)adc2 * step) + 0.01);
	voltage1 = (((double)adc3 * step) + 0.01);
	voltage2 = (((double)adc4 * step) + 0.01);

	HAL_ADC_Stop(&hadc1);
}

uint32_t lastSampleTime = 0;
double lastCPU, lastTemp0, lastTemp1, lastOnboard;

void sampleTemperatures(double &cpu, double &onboard, double &temp0, double &temp1)
{
	double v0, v1, v2;
	//only sample every 5 seconds
	if(lastSampleTime)
	{
		if(HAL_GetTick() > (lastSampleTime + 5000))
		{
			sampleAnalog(cpu, v0, v1, v2);
		}
		else
		{
			cpu = lastCPU;
			onboard = lastOnboard;
			temp0 = lastTemp0;
			temp1 = lastTemp1;
			return;
		}
	}
	else
	{
		sampleAnalog(cpu, v0, v1, v2);
	}

	printf("Voltages:\n");
	printf(" 0: %0.3f\n", v0);
	printf(" 1: %0.3f\n", v1);
	printf(" 2: %0.3f\n", v2);

	temp0 = (((v0 * 2) - 2.73) * 100.0);
	temp1 = (((v1 * 2) - 2.73) * 100.0);
	onboard = ((v2 * 2) - 2.73) * 100.0;

	lastSampleTime = HAL_GetTick();
	lastCPU = cpu;
	lastOnboard = onboard;
	lastTemp0 = temp0;
	lastTemp1 = temp1;
}

void sampleFermenter(double &temp)
{
	double cpu, onboard, temp0;
	sampleTemperatures(cpu, onboard, temp0, temp);
}

//uint32_t lastReport = 0;
Fermenter *_fermenter = 0;

void report(uint8_t *address)
{
//	//Report 60 seconds apart
//	if(lastReport)
//	{
//		if((lastReport + 60000) > HAL_GetTick())
//			return;
//	}
//	lastReport = HAL_GetTick();

	double cpu, onboard, temp0, temp1;
	sampleTemperatures(cpu, onboard, temp0, temp1);
	nodeData_s pay;
	memset(&pay, 0, 16);
	pay.timestamp = HAL_GetTick();
	pay.temperature = cpu * 1000;

	pay.voltages[0] = onboard * 1000;
	pay.voltages[1] = temp0 * 1000;
	pay.voltages[2] = temp1 * 1000;
	pay.voltages[3] = 30000;


	if(HAL_GPIO_ReadPin(COOLER_GPIO_Port, COOLER_Pin))
	{
		pay.outputs |= 1;
		if(_fermenter)
			pay.voltages[3] = _fermenter->get() * 1000;
	}

	int result = -3;
	int retries = 3;
	do
	{
		result = InterfaceNRF24::get()->transmit(address, (uint8_t*)&pay, 16);
		if(result < 0)
		{
			printf("  retry tx...\n");
			HAL_Delay(1000);
		}
	}while((result < 0) && (retries--));

	printf("TX result %d\n", result);
}

void reportNow()
{
	report(netAddress);
}

void coolerControl(bool state)
{
	printf("Mains cooler: %s\n", state?"ON":"OFF");
	if(state)
		HAL_GPIO_WritePin(COOLER_GPIO_Port, COOLER_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(COOLER_GPIO_Port, COOLER_Pin, GPIO_PIN_RESET);

	reportNow();
}

void heaterControl(bool state)
{
	printf("Mains heater: %s\n", state?"ON":"OFF");
	if(state)
		HAL_GPIO_WritePin(HEATER_GPIO_Port, HEATER_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(HEATER_GPIO_Port, HEATER_Pin, GPIO_PIN_RESET);

	reportNow();
}

bool isDay()
{
	RTC_TimeTypeDef sTime;
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

	printf("H: %d\n", sTime.Hours);

	if((sTime.Hours > 17) || (5 > sTime.Hours))
		return false;

	return true;
}

bool NRFreceivedCB(int pipe, uint8_t *data, int len)
{
	printf("RCV PIPE# %d\n", (int)pipe);
	printf(" PAYLOAD: %d\n", len);
	diag_dump_buf(data, len);

	nodeData_s down;
	memcpy(&down, data, len);
	int hour = (down.timestamp >> 8) & 0xFF;
	int min = (down.timestamp) & 0xFF;
	printf("Set Time %d:%d\n", hour, min);

	RTC_TimeTypeDef sTime;
	sTime.Hours = hour;
	sTime.Minutes = min;
	sTime.Seconds = 0;
	HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

	//Broadcast pipe
	if(pipe == 1)
	{
		report(netAddress);
	}

	return false;
}

int main(void)
{
  /* MCU Configuration----------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  HAL_Delay(1000);

  MX_USB_DEVICE_Init();
  sTerminalInterface_t usb = {
		  MX_USB_DEVICE_ready,
		  MX_USB_DEVICE_transmit
    };

  terminal_serial_Init();
  sTerminalInterface_t serial = {
		  terminal_serial_ready,
		  terminal_serial_transmit
  };

  sTerminalInterface_t *interfaces[] = {
		  &serial,
		  &usb,
		  0
  };

  terminal_init((sTerminalInterface_t **)&interfaces);

  MX_SPI1_Init();
  MX_ADC1_Init();
  tm1637Init();
  tm1637DisplayDecimal(8888, 0);

//  //configure node address up to PIPE5
//  int nodeAddr = 0;
//  if(HAL_GPIO_ReadPin(NRF_ADDR0_GPIO_Port, NRF_ADDR0_Pin) == GPIO_PIN_RESET)
//	  {
//	  printf("ADDR0: L\n");
//	  nodeAddr |= 0x01;
//	  }
//
//  if(HAL_GPIO_ReadPin(NRF_ADDR1_GPIO_Port, NRF_ADDR1_Pin) == GPIO_PIN_RESET)
//	  {
//	  printf("ADDR1: L\n");
//	  nodeAddr |= 0x02;
//	  }

  InterfaceNRF24::init(&hspi1, netAddress, 3);
  InterfaceNRF24::get()->setRXcb(NRFreceivedCB);

  Fermenter fermenter(sampleFermenter,
  		  coolerControl,
  		  heaterControl);

  _fermenter = &fermenter;

  printf("Bluepills @ %dHz\n", (int)HAL_RCC_GetSysClockFreq());
  printf(" - APB2 %dHz\n", (int)HAL_RCC_GetPCLK1Freq());
  printf(" - APB1 %dHz\n", (int)HAL_RCC_GetPCLK2Freq());
  MX_RTC_Init();



  int sendTemp = 0;
  double prevTemp = 0;
  bool flag = false;
  sampleFermenter(prevTemp);
  tm1637DisplayDecimal(prevTemp * 100, 1);
  prevTemp = 0;

  /* Infinite loop */
  while (1)
  {
	  if(flag)
	  {
		  flag = false;
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	  }
	  else
	  {

		  flag = true;
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	  }

	  terminal_run();
	  InterfaceNRF24::get()->run();
	  _fermenter->run();

	  HAL_Delay(100);

      //send temperature samples every minute;
      if(sendTemp++ > 600)
      {
    	  sendTemp = 0;
    	  double cpu, temp0, temp1, onboard;
    	  sampleTemperatures(cpu, onboard, temp0, temp1);
    	  //only report if temperature changed by a degree
    	  if(((prevTemp - 0.5) > temp1) || (temp1 > (prevTemp + 0.5)))
    	  {
    		  tm1637DisplayDecimal(temp1 * 100, 1);
    		  prevTemp = temp1;
    		  reportNow();
    	  }
      }
  }
}


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* RTC init function */
static void MX_RTC_Init(void)
{
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  HAL_RTCEx_DeactivateTamper(&hrtc, RTC_TAMPER_1);

    /**Initialize RTC and set the Time and Date 
    */
  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) == 0x32F2)
  {
	  printf(GREEN("RTC: "));

	  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	  printf("RTC date");
	  printf(" - %04d-%02d-%02d ", 2000 + sDate.Year, sDate.Month, sDate.Date);
	  printf("%02d:%02d:%02d\n", sTime.Hours, sTime.Minutes, sTime.Seconds);
  }
  else
  {
	  printf(RED("RTC: Not set\n"));
	  sTime.Hours = 0;
	  sTime.Minutes = 0;
	  sTime.Seconds = 0;

	  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	  {
		  _Error_Handler(__FILE__, __LINE__);
	  }

	  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
	  sDate.Year = 0;
	  sDate.Month = RTC_MONTH_JANUARY;
	  sDate.Date = 0;

	  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
	  {
		  _Error_Handler(__FILE__, __LINE__);
	  }

	  HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR1,0x32F2);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	//USB disable
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : SPI1_CS_Pin */
	GPIO_InitStruct.Pin = SPI1_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : NRF_CE_Pin */
	GPIO_InitStruct.Pin = NRF_CE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(NRF_CE_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : NRF_IRQ_Pin */
	GPIO_InitStruct.Pin = NRF_IRQ_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(NRF_IRQ_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : COOLER_Pin */
	GPIO_InitStruct.Pin = COOLER_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(COOLER_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : HEATER_Pin */
	GPIO_InitStruct.Pin = HEATER_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(HEATER_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : ADC12_IN0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : ADC12_IN1 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : ADC12_IN2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{
  hadc1.Instance = ADC1;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE	;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  ADC_ChannelConfTypeDef sConfig;
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }

  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }

  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	  _Error_Handler(__FILE__, __LINE__);
  }
  //printf("%08X\n", ADC1->SQR1);
}

const char *getDayName(int week_day)
{
	switch(week_day)
	{
	case RTC_WEEKDAY_MONDAY:
		return "Monday";
	case RTC_WEEKDAY_TUESDAY:
		return "Tuesday";
	case RTC_WEEKDAY_WEDNESDAY:
		return "Wednesday";
	case RTC_WEEKDAY_THURSDAY:
		return "Thursday";
	case RTC_WEEKDAY_FRIDAY:
		return "Friday";
	case RTC_WEEKDAY_SATURDAY:
		return "Saturday";
	case RTC_WEEKDAY_SUNDAY:
		return "Sunday";
	}

	return 0;
}

#ifdef __cplusplus
 extern "C" {
#endif

void nrf(uint8_t argc, char **argv)
{
	if(InterfaceNRF24::get())
	{
		uint8_t address[5];
		memcpy(address, netAddress, 5);

		if(argc > 1)
		{
			address[0] = strtoul(argv[1], 0, 16);
		}

		report(address);

	}
}

void adc(uint8_t argc, char **argv)
{
//	int temperature, voltage0, voltage1;
//	sampleAnalog(temperature, voltage0, voltage1);
//
//
//	printf("temp: %d\n", temperature);
//	printf("voltage0: %d\n", voltage0);
//	printf("voltage1: %d\n", voltage1);

	double cpu, temp0, temp1, onboard;
	sampleTemperatures(cpu, onboard, temp0, temp1);
	printf("Temperatures:\n");
	printf(" cpu  : %0.3f\n", cpu);
	//printf("onbrd: %0.3f\n", onboard);
	printf(" temp0: %0.3f\n", temp0);
	printf(" temp1: %0.3f\n", temp1);
	printf("      :         %0.3f\n", temp0 - temp1);
}


void rtc_debug(uint8_t argc, char **argv)
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

	if(argc > 5)
	{
		printf("Setting date %d\n", atoi(argv[5]));

		sDate.WeekDay = RTC_WEEKDAY_MONDAY;
		sDate.Year = atoi(argv[1]) - 2000;
		sDate.Month = atoi(argv[2]);
		sDate.Date = atoi(argv[3]);
		sTime.Hours = atoi(argv[4]);
		sTime.Minutes = atoi(argv[5]);
		sTime.Seconds = 0;

		HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	}


	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	printf("RTC date: %s %d\n", getDayName(sDate.WeekDay), (int)HAL_RTC_SecondsSinceEpoch(sDate, sTime));
	printf(" - %04d-%02d-%02d ", 2000 +sDate.Year, sDate.Month, sDate.Date);
	printf("%02d:%02d:%02d\n", sTime.Hours, sTime.Minutes, sTime.Seconds);
}

void setFerment(uint8_t argc, char **argv)
{
	if(argc > 1)
	{
		int setPoint = atoi(argv[1]);
		printf("Setting temperature %d\n",setPoint);

		_fermenter->set(setPoint);
	}
	else
	{
		printf("Fermenter set: %d\n", _fermenter->get());
	}
}

#ifdef __cplusplus
 }
#endif


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif


/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

