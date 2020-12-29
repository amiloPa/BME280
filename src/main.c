/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f10x.h"
#include "UART/UART.h"
#include "I2C/I2C.h"
#include "BME280/BME280.h"
#include "COMMON/common_var.h"
#include "SPI/SPI.h"


ErrorStatus HSEStartUpStatus;
#define F_PCLK2  72000000
#define MEASURE_PERIOD 1000 // measure period in ms



void RCC_Conf(void);
void GPIO_Conf(void);
void NVIC_Conf(void);
void SysTick_Conf(void);

uint32_t allow_for_measure = 0;
volatile uint8_t flag = 1;
uint32_t start_measure = 0;
uint16_t result_time = 0;
char measure_time[15];

int main(void)
{

	uint8_t result_BME_conf;
	uint8_t result;

	RCC_Conf();
	SysTick_Conf();
	GPIO_Conf();
	UART_Conf(UART_BAUD);
	NVIC_Conf();

#if BME280_I2C
	I2C_Conf(400);
#endif

#if BME280_SPI
	SPI_Conf();
#endif

	do
	{
		result_BME_conf = BME280_Conf(&conf_BME280, &bme);
	}
	while(result_BME_conf == 3);

	while(1)
	{
		if(flag)
		{
			flag = 0;

			if(result_BME_conf)
			{
				uart_puts("Sensor configuration error:");
				if(bme.err_conf == calib_reg)  uart_puts(" calibration coefficients includes zero value,");
				if(bme.err_conf == config_reg) uart_puts(" configuration registers error,");
				if(bme.err_conf == both) uart_puts(" calibration coefficients includes zero value and configuration registers error,");
				uart_puts("\n\r");
			}
			else
			{
				start_measure = source_time;
				result = BME280_ReadTPH(&bme);

				switch(result)
				{
				case 1:
					uart_puts("Sensor error. Initialization phase didn't go properly.");
					uart_puts("\n\r");
					flag = 0;
					break;

				case 2:
					break;

				case 3:
					switch(bme.err_boundaries_T)
					{
					case T_lower_limit:
						uart_puts(" Measured raw value of temperature is lower than minimum value (0x00000),");
						break;
					case T_over_limit:
						uart_puts(" Measured raw value of temperature is over than maximum value (0x800000),");
						break;
					}

					switch(bme.err_boundaries_T)
					{
					case P_lower_limit:
						uart_puts(" Measured raw value of pressure is lower than minimum value (0x00000),");
						break;
					case P_over_limit:
						uart_puts(" Measured raw value of pressure is over than maximum value (0x800000),");
						break;
					}
					break;
				case 4:
					uart_puts(" Try to divide by 0 (measuring is intermittent.)");
					break;

				default:
					result_time = source_time - start_measure;

					uart_puts(bme.temp2str);
					uart_puts("C");
					uart_puts("  ");
					uart_puts(bme.pressure2str);
					uart_puts("hPa");
					uart_puts("  ");
					uart_puts(bme.humi2str);
					uart_puts("%");

					uart_puts("  ");
					itoa(result_time, measure_time,10);
					uart_puts("measure take = ");
					uart_puts(measure_time);
					uart_puts("ms");
					uart_puts("\n\r");
				}
			}

		}
	}
}



void SysTick_Conf (void)
{
	SysTick_Config(F_PCLK2/8/1000);
	SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;
}

void RCC_Conf(void)
{
  // RCC setting reset
  RCC_DeInit();

  // Turn on HSE
  RCC_HSEConfig(RCC_HSE_ON);

  // Wait up to HSE will be ready
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
	  /*
	   * the introduction of delays is (waitstate) for higher clock rates
	   * is due to the maximum frequency with which it is performed
	   * communication with Flash memory can be 24 MHz
	   */
	  FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

	  // wait for flash memory
	  FLASH_SetLatency(FLASH_Latency_2);

	  // HCLK = SYSCLK
	  RCC_HCLKConfig(RCC_SYSCLK_Div1);

	  // PCLK2 = HCLK
	  RCC_PCLK2Config(RCC_HCLK_Div1);

	  // PCLK1 = HCLK/2
	  RCC_PCLK1Config(RCC_HCLK_Div2);

	  // PLLCLK = 8MHz * 9 = 72 MHz
	  RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

	  // Turn on PLL
	  RCC_PLLCmd(ENABLE);

	  // Wait up to PLL will be ready
	  while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

	  // Select PLL as source of clock
	  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

	  // Wait up to PLL will be the source of clock
	  while(RCC_GetSYSCLKSource() != 0x08);

	  // Turn on W³¹czenie clock signal supervision system
	  //RCC_ClockSecuritySystemCmd(ENABLE);

  }

}


void NVIC_Conf(void)
{

  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
}


void GPIO_Conf(void)
{
	// Set pin PC13 as blinking led
	GPIO_InitTypeDef GPIOInit;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_StructInit(&GPIOInit);

	GPIOInit.GPIO_Pin = GPIO_Pin_13;
	GPIOInit.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIOInit.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOC, &GPIOInit);

}



__attribute__((interrupt)) void SysTick_Handler(void)
{
	static uint16_t counter = 0;
	static uint8_t status = 0;

	if(counter == MEASURE_PERIOD)
	{
		allow_for_measure = source_time;
		flag = 1;
		counter = 0;
	}
	else
	{
		counter++;
	}
	source_time++;

	if(0 == status)
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		status = 1;
	}
	else
	{
		GPIO_SetBits(GPIOC, GPIO_Pin_13);;
		status = 0;
	}
}
