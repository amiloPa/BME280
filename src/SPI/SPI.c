/*
 * SPI.c
 *
 *  Created on: 01.10.2020
 *      Author: Piotr
 */


#include "SPI.h"

#if BMP280_SPI
	void SPI_Conf(void);
	void SELECT (void);
	void DESELECT (void);
#endif

#if BMP280_SPI

	/****************************************************************************/
	/*      Configuration of SPI protocol	        							*/
	/****************************************************************************/
	void SPI_Conf(void)
	{
		GPIO_InitTypeDef  GPIO_InitStructure;
		SPI_InitTypeDef   SPI_InitStructure;

		// Run clocks for peripherals
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA |
								RCC_APB2Periph_SPI1  |
								RCC_APB2Periph_AFIO, ENABLE);


		// PA0 as CS
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		//SCK -> PA5, MISO -> PA6, MOSI -> PA7
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);


		// Configuration SPI
		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
		SPI_InitStructure.SPI_CRCPolynomial = 7;
		SPI_Init(SPI1, &SPI_InitStructure);

		// Turn on SPI
		SPI_Cmd(SPI1, ENABLE);

		SELECT();
		DESELECT();
	}
#endif

	/****************************************************************************/
	/*      Select slave	        											*/
	/****************************************************************************/
#if BMP280_SPI
	void SELECT (void) 		// CS in low state
	{
		GPIO_ResetBits(GPIOA, GPIO_Pin_0);
	}
#endif

	/****************************************************************************/
	/*      Deelect slave	        											*/
	/****************************************************************************/
#if BMP280_SPI
	void DESELECT (void) 	// CS in high state
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_0);
	}
#endif

	/****************************************************************************/
	/*      Send a few data by SPI        										*/
	/****************************************************************************/
#if BMP280_SPI
	void SPI_SendData (uint8_t address, uint8_t *Data, uint8_t size)
	{
		const uint8_t register_mask = 0x7F;
		uint8_t data;

		SELECT();

		for (uint8_t i = 0; i < size; i++)
		{
			data = Data[i];
			while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
			SPI_I2S_SendData(SPI1, register_mask & address);

			while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
			SPI_I2S_SendData(SPI1, data);

			address++;
		}
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

		DESELECT();
	}
#endif

	/****************************************************************************/
	/*      Receiving a few data from external device by SPI      				*/
	/****************************************************************************/
#if BMP280_SPI
	void SPI_ReceiveData (uint8_t address, uint8_t *Data, uint8_t size)
	{
		SELECT();

		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		SPI_I2S_SendData(SPI1, address);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
		SPI_I2S_ReceiveData(SPI1);

		for (uint8_t i = 0; i < size; i++)
		{
			while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
			SPI_I2S_SendData(SPI1, 0);
			while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
			Data[i] = SPI_I2S_ReceiveData(SPI1);
		}

		DESELECT();
	}

#endif
