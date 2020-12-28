/*
 * BME280.c
 *
 *  Created on: 18.12.2020
 *      Author: Piotr
 */

#include "BME280.h"


BME280 bmp;
CONF conf_BME280;

void check_boundaries (BME280 *bmp);			// check if read uncompensated values are in boundary MIN and MAX
void soft_reset (void);						// execute sensor reset by software
void get_status (BME280 *bmp);				// read statuses of sensor
uint8_t bme280_compute_meas_time(void);		// measurement time in milliseconds for the active configuration
void pressure_at_sea_level(BME280 *bmp);		// calculating pressure reduced to sea level

void BME280_read_data(uint8_t SLA, uint8_t register_addr,  uint8_t size, uint8_t *Data);	// read data from sensor
void BME280_write_data(uint8_t SLA, uint8_t register_addr, uint8_t size, uint8_t *Data);	// write data to sensor
uint8_t write_configuration_and_check_it (CONF *sensor, BME280 *bmp);							// write configuration and check if saved configuration is equal to set

#if CALCULATION_AVERAGE_TEMP
	void calculation_average_temp(BME280 *bmp);	// calculate average temperature, No of samples to calculations is taken from No_OF_SAMPLES
	void calculation_average_humidity(BME280 *bmp);	// calculate average values of temperature or humidity, No of samples to calculations is taken from No_OF_SAMPLES
#endif

/****************************************************************************/
/*      setting function configurations of sensor					        */
/****************************************************************************/
uint8_t BME280_Conf (CONF *sensor, BME280 *bmp)
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
/*      read, check, calculate and prepare string for measured values       */
/****************************************************************************/
uint8_t BME280_ReadTPH(BME280 *bmp)
{
	#if USE_STRING
		uint8_t len;
	#endif

	uint8_t temp[8];
	uint8_t divisor;
	int32_t var1, var2, var3, var4, var5, t_fine;
	uint32_t p;
	uint32_t humidity_max = 102400;
    const int32_t temperature_min = -4000;
    const int32_t temperature_max = 8500;

	var1    = 0;
	var2    = 0;
	p       = 0;
	t_fine  = 0;
	divisor = 0;

	bmp->adc_P = 0;
	bmp->adc_T = 0;
	bmp->adc_H = 0;

	// ----- if occured some error during parameterization sensor don't do any measure -----
	if(bmp->err_conf) return 1;

#if BME280_INCLUDE_STATUS
	// ----- get status of sensor -----
	get_status();

	// ----- if sensor is in measuring or im_update status, wait up to the finishing and after that read measures -----
	if( (bmp.measuring_staus)  || (bmp.im_update_staus)) return 2;

#endif

	BME280_read_data(BME280_ADDR, 0xF7, 8, (uint8_t *)&temp);	// read data register

	bmp->adc_P = (temp[0] << 12) | (temp[1] << 4) | (temp[2] >> 4);
	bmp->adc_T = (temp[3] << 12) | (temp[4] << 4) | (temp[5] >> 4);
	bmp->adc_H = (temp[6] << 8)  |  temp[7];


	// ----- check boundaries -----
	check_boundaries(bmp);

	// ----- if raw values are lower or over the limits, function is intermittent and returning 3  -----
	if ((bmp->err_boundaries_T != 0) || ( bmp->err_boundaries_P != 0) || ( bmp->err_boundaries_H != 0)) return 3;

	/*-----------------------------------------------------------------------*/
	/********************* calculate temperature *****************************/
	/*-----------------------------------------------------------------------*/

    var1 = (int32_t)((bmp->adc_T / 8) - ((int32_t)bmp->coef.dig_T1 * 2));
    var1 = (var1 * ((int32_t)bmp->coef.dig_T2)) / 2048;
    var2 = (int32_t)((bmp->adc_T / 16) - ((int32_t)bmp->coef.dig_T1));
    var2 = (((var2 * var2) / 4096) * ((int32_t)bmp->coef.dig_T3)) / 16384;
    t_fine = var1 + var2;
    bmp->temperature = (t_fine * 5 + 128) / 256;

    if (bmp->temperature < temperature_min)
    {
    	bmp->temperature = temperature_min;
    }
    else if (bmp->temperature > temperature_max)
    {
    	bmp->temperature = temperature_max;
    }

	if(my_abs(bmp->temperature) > 9) 	divisor = 100;
	else 								divisor = 10;


	bmp->t1 = (int32_t)bmp->temperature / (int8_t)divisor;
	bmp->t2 = my_abs((uint32_t)bmp->temperature % (uint8_t)divisor);

	// ----- prepare string with value of temperature -----
#if USE_STRING

	#if CALCULATION_AVERAGE_TEMP


		// ----- calculation of average temperature -----
		calculation_average_temp(bmp);

		itoa(bmp->avearage_temp_cel, &bmp->temp2str[0], 10);
		len = strlen(bmp->temp2str);
		bmp->temp2str[len++] = ',';

		if( bmp->avearage_temp_fract < 10) bmp->temp2str[len++] = '0';
		itoa(bmp->avearage_temp_fract, &bmp->temp2str[len++],10);

	#else

		itoa(bmp->t1, &bmp->temp2str[0], 10);
		len = strlen(bmp->temp2str);
		bmp->temp2str[len++] = ',';

		if( bmp->t2 < 10) bmp->temp2str[len++] = '0';
		itoa(bmp->t2, &bmp->temp2str[len++],10);
	#endif

#endif

	/*-----------------------------------------------------------------------*/
	/********************* calculate pressure ********************************/
	/*-----------------------------------------------------------------------*/

	bmp->compensate_status = 0;
	var1 = 0;
	var2 = 0;
	p 	 = 0;

	var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
	var2 = (((var1 >> 2) * (var1 >> 2)) >> 11 ) * ((int32_t)bmp->coef.dig_P6);
	var2 = var2 + ((var1 * ((int32_t)bmp->coef.dig_P5)) << 1);
	var2 = (var2 >> 2) + (((int32_t)bmp->coef.dig_P4) << 16);
	var1 = (((bmp->coef.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13 )) >> 3) + ((((int32_t)bmp->coef.dig_P2) * var1) >> 1)) >> 18;
	var1 =((((32768 + var1)) * ((int32_t)bmp->coef.dig_P1)) >> 15);

	if (var1 == 0) //if dividing by 0, function is intermittent and returning 4
	{
		bmp->compensate_status = 1;
		return 4;
	}

	p = (((uint32_t)(((int32_t)1048576) - bmp->adc_P) - (var2 >> 12))) * 3125;

	if (p < 0x80000000) p = (p << 1) / ((uint32_t)var1);
	else				p = (p / (uint32_t)var1) * 2;


	var1 = (((int32_t)bmp->coef.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
	var2 = (((int32_t)(p >> 2)) * ((int32_t)bmp->coef.dig_P8)) >> 13;
	p = (uint32_t)((int32_t)p + ((var1 + var2 + bmp->coef.dig_P7) >> 4));


	bmp->preasure = (uint32_t)p;
	bmp->p1 =  (int32_t)bmp->preasure;

	// ----- prepare string with value of pressure -----
#if USE_STRING
	if ((bmp->p1/100) < 1000)
	{
		bmp->pressure2str[0] = ' ';
		itoa(bmp->p1/100, &bmp->pressure2str[1],10);
	}
	else
	{
		itoa(bmp->p1/100, &bmp->pressure2str[0],10);
	}

	bmp->pressure2str[4] = ',';
	itoa(bmp->p1%100, &bmp->pressure2str[5],10);

#endif

	/*-----------------------------------------------------------------------*/
	/********************* calculate humidity ********************************/
	/*-----------------------------------------------------------------------*/

	var1    = 0;
	var2    = 0;
	var3    = 0;
	var4    = 0;
	var5    = 0;

	var1 = t_fine - ((int32_t)76800);
	var2 = (int32_t)(bmp->adc_H * 16384);
	var3 = (int32_t)(((int32_t)bmp->coef.dig_H4) * 1048576);
	var4 = ((int32_t)bmp->coef.dig_H5) * var1;
	var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
	var2 = (var1 * ((int32_t)bmp->coef.dig_H6)) / 1024;
	var3 = (var1 * ((int32_t)bmp->coef.dig_H3)) / 2048;
	var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
	var2 = ((var4 * ((int32_t)bmp->coef.dig_H2)) + 8192) / 16384;
	var3 = var5 * var2;
	var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
	var5 = var3 - ((var4 * ((int32_t)bmp->coef.dig_H1)) / 16);
	var5 = (var5 < 0 ? 0 : var5);
	var5 = (var5 > 419430400 ? 419430400 : var5);
	bmp->humidity = (uint32_t)(var5 / 4096);

	if (bmp->humidity > humidity_max)
	{
		bmp->humidity = humidity_max;
	}

	bmp->humidity *= 100;
	bmp->humidity /= 1024;


	if(my_abs(bmp->humidity/100) > 9) 	divisor = 100;
	else 								divisor = 10;


	bmp->h1 = (int32_t)bmp->humidity / (int8_t)divisor;
	bmp->h2 = my_abs((uint32_t)bmp->humidity % (uint8_t)divisor);

	// ----- prepare string with value of humidity -----
#if USE_STRING

	#if CALCULATION_AVERAGE_HUMIDITY

		// ----- calculation of average humidity -----
		calculation_average_humidity(bmp);

		itoa(bmp->avearage_humidity_cel, &bmp->humi2str[0], 10);
		len = strlen(bmp->humi2str);
		bmp->humi2str[len++] = ',';

		if( bmp->avearage_humidity_fract < 10) bmp->humi2str[len++] = '0';
		itoa(bmp->avearage_humidity_fract, &bmp->humi2str[len++],10);

	#else

		itoa(bmp->h1, &bmp->humi2str[0], 10);
		len = strlen(bmp->humi2str);
		bmp->humi2str[len++] = ',';

		if( bmp->h2 < 10) bmp->humi2str[len++] = '0';
		itoa(bmp->h2, &bmp->humi2str[len++],10);
	#endif

#endif

	// ----- measure and prepare values for the next reading -----
	if(conf_BME280.mode == BME280_FORCEDMODE)
	{
		BME280_write_data(BME280_ADDR, 0xF4, 1, conf_BME280.bt);		// write configurations bytes
	}

	// ----- calculate a preasure sea level -----
	pressure_at_sea_level(bmp);

	return 0;	// if everything is OK return 0
}

/****************************************************************************/
/*      check if read uncompensated values are in boundary MIN and MAX      */
/****************************************************************************/
void check_boundaries (BME280 *bmp)
{
	bmp->err_boundaries_T = 0;
	bmp->err_boundaries_P = 0;
	bmp->err_boundaries_H = 0;

	if (bmp->adc_T == BME280_ST_ADC_MAX_T_P) bmp->err_boundaries_T = Over_limit;
	if (bmp->adc_P == BME280_ST_ADC_MAX_T_P) bmp->err_boundaries_P = Over_limit;
	if (bmp->adc_H == BME280_ST_ADC_MAX_H)   bmp->err_boundaries_H = Over_limit;

}

/****************************************************************************/
/*      read statuses of sensor      										*/
/****************************************************************************/
void get_status (BME280 *bmp)
{
	uint8_t *status = 0;

	BMP280_read_data(BME280_ADDR, 0xF3, 1, status);	// read set register

	bmp->measuring_staus =  *status & BMP280_MEASURING_STATUS;
	bmp->im_update_staus =  *status & BMP280_IM_UPDATE_STATUS;
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
uint8_t write_configuration_and_check_it (CONF *sensor, BME280 *bmp)
{
	uint8_t buf[3];
	uint8_t i;
	uint8_t bt_temp[3];
	uint8_t temp_humidity[7];

	BME280_write_data(BME280_ADDR, 0xF2,  1, &sensor->bt[0]);		// write configurations byte for ctrl_hum
	BME280_write_data(BME280_ADDR, 0xF4,  2, &sensor->bt[1]);		// write configurations bytes
	BME280_read_data(BME280_ADDR,  0xF2,  1, &buf[0]);				// read set registers
	BME280_read_data(BME280_ADDR,  0xF4,  2, &buf[1]);				// read set registers
	BME280_read_data(BME280_ADDR,  0x88, 24, bmp->coef.bt);			// read compensation parameters: 0x88 -> 0x9F
	BME280_read_data(BME280_ADDR,  0xA1,  1, &bmp->coef.bt[24]);	// read compensation parameters: 0xA1

	BME280_read_data(BME280_ADDR,  0xE1,  7, &temp_humidity[0]);	// read compensation parameters: 0xE1 -> 0xE2

//	BME280_read_data(BME280_ADDR,  0xE1,  2, &bmp->coef.bt[25]);	// read compensation parameters: 0xE1 -> 0xE2
//	BME280_read_data(BME280_ADDR,  0xE3,  1, &bmp->coef.bt[27]);	// read compensation parameters: 0xE3
//	BME280_read_data(BME280_ADDR,  0xE4,  5, &bmp->coef.bt[28]);	// read compensation parameters: 0xE4 -> 0xE8


	bmp->coef.dig_H2 = ((uint16_t)temp_humidity[1] << 8 ) | (uint16_t)temp_humidity[0];
	bmp->coef.dig_H3 = temp_humidity[2];
	bmp->coef.dig_H4 = ((uint16_t)temp_humidity[3] << 4) | ((uint16_t)temp_humidity[4] & 0x0F);
	bmp->coef.dig_H5 = ((uint16_t)temp_humidity[5] << 4) | ((uint16_t)temp_humidity[4] >> 4);
	bmp->coef.dig_H6 = temp_humidity[6];


	// ----- check if pressure and temperature calibration coefficients are diferent from 0 -----
	// ----- coefficients of humidity can contain 0 value so that aren't checked -----
	for( i = 0; i < (SIZE_OF_PT_UNION/2); i++)
	{
		if(bmp->coef.bt2[i] == 0)
		{
			bmp->err_conf = calib_reg;
			return 1;
		}
	}

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
				else
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

/****************************************************************************/
/*     Calculate average temperature,										*/
/* 	   No of samples to calculations is taken from No_OF_SAMPLES 			*/
/****************************************************************************/
#if CALCULATION_AVERAGE_TEMP
	void calculation_average_temp(BME280 *bmp)
	{
		int16_t avearage_temp_value = 0;
		static uint8_t i = 1;
		uint8_t k;
		uint8_t avearage_fract_temp;

		if(i <= No_OF_SAMPLES)
		{
			if(bmp->t1 >= 0) bmp->smaples_of_temp[i-1] = 100 * bmp->t1 + bmp->t2;
			else 			 bmp->smaples_of_temp[i-1] = -1 * (100 * my_abs(bmp->t1) + bmp->t2);
		}
		else
		{
			for (k = 1; k < i - 1; k++)
			{
				bmp->smaples_of_temp[k-1] = bmp->smaples_of_temp[k];
			}

			if(bmp->t1 >= 0) bmp->smaples_of_temp[--k] = 100 * bmp->t1 + bmp->t2;
			else 			 bmp->smaples_of_temp[--k] = -1 * (100 * my_abs(bmp->t1) + bmp->t2);
		}

		for (k = 0; k < i - 1; k++)
		{
			avearage_temp_value += bmp->smaples_of_temp[k];
		}

		avearage_temp_value /= (i - 1);

		bmp->avearage_temp_cel = avearage_temp_value / 100 ;
		avearage_fract_temp = (my_abs(avearage_temp_value)) % 100;
		bmp->avearage_temp_fract = avearage_fract_temp;

		if( i <= No_OF_SAMPLES) i++;
	}
#endif

	/****************************************************************************/
	/*     Calculate average humidity,										*/
	/* 	   No of samples to calculations is taken from No_OF_SAMPLES 			*/
	/****************************************************************************/
	#if CALCULATION_AVERAGE_HUMIDITY
		void calculation_average_humidity(BME280 *bmp)
		{
			int16_t avearage_humidity_value = 0;
			static uint8_t i = 1;
			uint8_t k;
			uint8_t avearage_fract_humidity;

			if(i <= No_OF_SAMPLES)
			{
				if(bmp->h1 >= 0) bmp->smaples_of_humidity[i-1] = 100 * bmp->h1 + bmp->h2;
				else 			 bmp->smaples_of_humidity[i-1] = -1 * (100 * my_abs(bmp->h1) + bmp->h2);
			}
			else
			{
				for (k = 1; k < i - 1; k++)
				{
					bmp->smaples_of_humidity[k-1] = bmp->smaples_of_humidity[k];
				}

				if(bmp->h1 >= 0) bmp->smaples_of_humidity[--k] = 100 * bmp->h1 + bmp->h2;
				else 			 bmp->smaples_of_humidity[--k] = -1 * (100 * my_abs(bmp->h1) + bmp->h2);
			}

			for (k = 0; k < i - 1; k++)
			{
				avearage_humidity_value += bmp->smaples_of_humidity[k];
			}

			avearage_humidity_value /= (i - 1);

			bmp->avearage_humidity_cel = avearage_humidity_value / 100 ;
			avearage_fract_humidity = (my_abs(avearage_humidity_value)) % 100;
			bmp->avearage_humidity_fract = avearage_fract_humidity;

			if( i <= No_OF_SAMPLES) i++;
		}
	#endif

	/****************************************************************************/
	/*      calculating pressure reduced to sea level            				*/
	/****************************************************************************/
	void pressure_at_sea_level(BME280 *bmp){

	        uint16_t st_baryczny,tpm,t_sr;
	        uint32_t p0,p_sr;

	        st_baryczny = (800000 * (1000 + 4 * bmp->temperature) / (bmp->preasure));          		// calculation of the baric degree acc. to the pattern of the Babineta
	        p0 = bmp->preasure + (100000 * BME280_ALTITUDE / (st_baryczny));              			// calculation of approximate sea level pressure
	        p_sr = (bmp->preasure + p0) / 2;                     									// calculation of average pressure for layers between sea level and sensor
	        tpm = bmp->temperature + ((6 * BME280_ALTITUDE) / 1000);                           		// calculation of average temperature for layer of air
	        t_sr = (bmp->temperature + tpm) / 2;                                              		// calculation of average temperature for layer between sea level and sensor
	        st_baryczny = (800000 * (1000 + 4 * t_sr) / (p_sr));              						// calculation of more accurate value of baric degree
	        bmp->sea_pressure_redu = bmp->preasure + (100000 * BME280_ALTITUDE / (st_baryczny)); 	// calculation of more accurate value of pressure for sea level
	}
