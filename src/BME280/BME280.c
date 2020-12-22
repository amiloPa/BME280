/*
 * BME280.c
 *
 *  Created on: 18.12.2020
 *      Author: Piotr
 */

#include "BME280.h"


TBMP bmp;
CONF conf_BME280;


void check_boundaries (TBMP *bmp);			// check if read uncompensated values are in boundary MIN and MAX
void soft_reset (void);						// execute sensor reset by software
void get_status (TBMP *bmp);				// read statuses of sensor
uint8_t bme280_compute_meas_time(void);		// measurement time in milliseconds for the active configuration
void pressure_at_sea_level(TBMP *bmp);		// calculating pressure reduced to sea level

void BME280_read_data(uint8_t SLA, uint8_t register_addr,  uint8_t size, uint8_t *Data);	// read data from sensor
void BME280_write_data(uint8_t SLA, uint8_t register_addr, uint8_t size, uint8_t *Data);	// write data to sensor
uint8_t write_configuration_and_check_it (CONF *sensor, TBMP *bmp);							// write configuration and check if saved configuration is equal to set

#if CALCULATION_AVERAGE_TEMP
	void calculation_average_temp(TBMP *bmp);	// calculate average temperature, No of samples to calculations is taken from No_OF_SAMPLES
#endif

/****************************************************************************/
/*      setting function configurations of sensor					        */
/****************************************************************************/
uint8_t BME280_Conf (CONF *sensor, TBMP *bmp)
{
	uint8_t counter = 0;
	uint8_t  result_of_check = 5;
	static uint32_t current_time = 0;


	sensor->filter		= BME280_FILTER_OFF;
	if (!sensor->filter)
	{
		sensor->osrs_p 		= BME280_oversampling_x16; // pressure resolution is 16 + (osrs_p -1) bit if IIR filter is disabled, in this case resolution = 16 (5-1) = 20 bit
		sensor->osrs_t		= BME280_oversampling_x16; // temperature resolution is 16 + (osrs_p -1) bit if IIR filter is disabled, in this case resolution = 16 (5-1) = 20 bit;
	}
	sensor->osrs_h 		= BME280_oversampling_x16;
	sensor->reserved1	= 0;
	sensor->mode 		= BME280_FORCEDMODE;
	sensor->spi3w_en	= 0;
	sensor->reserved2	= 0;
	sensor->t_sb		= BME280_STANDBY_MS_0_5;

	if (current_time == 0)
		{
			soft_reset();				// make a reset to clear previous setting
			current_time = source_time; // get system time
		}

	if(source_time <= (current_time + 3) ) return 3;	//wait 3ms aster software reset

	do
	{
		result_of_check = write_configuration_and_check_it(sensor, bmp);
		counter++;
	}
	while((result_of_check) && (counter < 3));

	return result_of_check;	// if everything is OK return 0
}



/****************************************************************************/
/*     in depend on selected protocol data are saved to sensor		        */
/****************************************************************************/
void BME280_write_data(uint8_t SLA, uint8_t register_addr, uint8_t size, uint8_t *Data)
{
#if BME280_I2C

	int i;
	  const uint8_t* buffer = (uint8_t*)Data;

	  //select device
	  I2C_ADDRES(SLA, register_addr);

	  //sending data from whole buffer
	  for (i = 0; i < size; i++)
	  {
	   I2C_SendData(I2C1, buffer[i]);
	   while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS);
	  }
	  //STOP signal generating
	  I2C_GenerateSTOP(I2C1, ENABLE);

#endif

#if BME280_SPI
//	SPI_SendData(register_addr, Data, size);

	const uint8_t register_mask = 0x7F;
	uint8_t data;

	SELECT();

	for (uint8_t i = 0; i < size; i++)
	{
		data = Data[i];
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		SPI_I2S_SendData(SPI1, register_mask & register_addr);

		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		SPI_I2S_SendData(SPI1, data);

		register_addr++;
	}
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

	DESELECT();
#endif

}

/****************************************************************************/
/*      in depend on selected protocol data are readed from sensor	        */
/****************************************************************************/
void BME280_read_data(uint8_t SLA, uint8_t register_addr,  uint8_t size, uint8_t *Data)
{
#if BME280_I2C

	  int i;
	  uint8_t* buffer = (uint8_t*)Data;

	  //select device
	  I2C_ADDRES(SLA, register_addr);

	  //START signal sending and waiting for response
	  I2C_GenerateSTART(I2C1, ENABLE);
	  while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);

	  //Receiving only one data byte and turn off ack signal
	  I2C_AcknowledgeConfig(I2C1, ENABLE);
	  //Device address sending, setting microcontroller as master in receive mode
	  I2C_Send7bitAddress(I2C1, SLA, I2C_Direction_Receiver);
	  //waitnig for EV6
	  while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != SUCCESS);

	  for (i = 0; i < size - 1; i++)
	  {
	   while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS);
	   buffer[i] = I2C_ReceiveData(I2C1);
	  }
	  //Receiving only one byte
	  I2C_AcknowledgeConfig(I2C1, DISABLE);

	  //STOP signal generating
	  I2C_GenerateSTOP(I2C1, ENABLE);
	  //waiting for signal
	  while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS);
	  //reading data from receiving register
	  buffer[i] = I2C_ReceiveData(I2C1);
#endif


#if BME280_SPI
//	SPI_ReceiveData(register_addr, Data, size );

	SELECT();

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, register_addr);
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
#endif
}


/****************************************************************************/
/*      execute sensor reset by software							        */
/****************************************************************************/
void soft_reset (void)
{
	BME280_write_data(BME280_ADDR, 0xE0, 1, (uint8_t*)BME280_SOFTWARE_RESET);		// write configurations bytes
}




/****************************************************************************/
/*     write configuration and check if saved configuration is equal to set */
/****************************************************************************/
uint8_t write_configuration_and_check_it (CONF *sensor, TBMP *bmp)
{
	uint8_t buf[3];
	uint8_t i;
	uint8_t bt_temp[3];

	BME280_write_data(BME280_ADDR, 0xF2,  1, &sensor->bt[0]);		// write configurations byte for ctrl_hum
	BME280_write_data(BME280_ADDR, 0xF4,  2, &sensor->bt[1]);		// write configurations bytes
	BME280_read_data(BME280_ADDR,  0xF2,  1, &buf[0]);				// read set registers
	BME280_read_data(BME280_ADDR,  0xF4,  2, &buf[1]);				// read set registers
	BME280_read_data(BME280_ADDR,  0x88, 24, bmp->coef.bt);			// read compensation parameters: 0x88 -> 0x9F
	BME280_read_data(BME280_ADDR,  0xA1,  1, &bmp->coef.bt[24]);	// read compensation parameters: 0xA1
	BME280_read_data(BME280_ADDR,  0xE1,  8, &bmp->coef.bt[25]);	// read compensation parameters: 0xE1 -> 0xE8

//	// ----- check if all calibration coefficients are diferent from 0 -----
//	for( i = 0; i < (24/2); i++)
//	{
//		if(bmp->coef.bt2[i] == 0)
//		{
//			bmp->err_conf = calib_reg;
//			return 1;
//		}
//	}

	// ----- check if set configuration registers are that same as readed -----
	if(sensor->mode == BME280_FORCEDMODE)   /* sometimes hapend, that sensor go to sleep mode after single measurement was finished
	 	 	 	 	 	 	 	 	 	 	 * (power mode is equal to forced mode and this situation is decribed in datasheet,
	 	 	 	 	 	 	 	 	 	 	 * chapter: 3.6.2 Forced mode). In this case bytes responsible for select mode
	 	 	 	 	 	 	 	 	 	 	 * in control register 0xF4 are come back to 0 value (sensor go to sleep mode)*/

	{
		for(i = 0; i < 3; i++)
		{
			bt_temp[i] = sensor->bt[i];
			if (i == 1)
			{
				buf[i] &= 0xFE;
				bt_temp[i] &= 0xFE;
			}

			if(buf[i] != bt_temp[i])
			{
				if(bmp->err_conf == calib_reg) 	bmp->err_conf = both;
				else 							bmp->err_conf = config_reg;
				return 2;
			}
		}
	}
	else
	{
		for(i = 0; i < 3; i++)
		{
			if(buf[i] != sensor->bt[i])
			{
				if(bmp->err_conf == calib_reg) 	bmp->err_conf = both;
				else 							bmp->err_conf = config_reg;
				return 2;
			}
		}
	}

	return 0;
}

