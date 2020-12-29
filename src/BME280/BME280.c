/*
 * BME280.c
 *
 *  Created on: 18.12.2020
 *      Author: Piotr
 */

#include "BME280.h"


BME280 bme;
CONF conf_BME280;

void check_boundaries (BME280 *bme);																// check if read uncompensated values are in boundary MIN and MAX
void soft_reset (void);																				// execute sensor reset by software
void get_status (BME280 *bme);																		// read statuses of sensor
uint8_t bme280_compute_measure_time(MEASUREMENT_TIME type, CONF *sensor);							// measurement time in milliseconds for the active configuration
void pressure_at_sea_level(BME280 *bme);															// calculating pressure reduced to sea level

void BME280_read_data(uint8_t SLA, uint8_t register_addr,  uint8_t size, uint8_t *Data);			// read data from sensor
void BME280_write_data(uint8_t SLA, uint8_t register_addr, uint8_t size, uint8_t *Data);			// write data to sensor
uint8_t read_compensation_parameter_write_configuration_and_check_it (CONF *sensor, BME280 *bme);	// write configuration and check if saved configuration is equal to set

#if CALCULATION_AVERAGE_TEMP
	void calculation_average_temp(BME280 *bme);	// calculate average temperature, No of samples to calculations is taken from No_OF_SAMPLES
	void calculation_average_humidity(BME280 *bme);	// calculate average values of temperature or humidity, No of samples to calculations is taken from No_OF_SAMPLES
#endif

/****************************************************************************/
/*      setting function configurations of sensor					        */
/****************************************************************************/
uint8_t BME280_Conf (CONF *sensor, BME280 *bme)
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
		result_of_check = read_compensation_parameter_write_configuration_and_check_it(sensor, bme);
		counter++;
	}
	while((result_of_check) && (counter < 3));


	return result_of_check;	// if everything is OK return 0
}


/****************************************************************************/
/*      read, check, calculate and prepare string for measured values       */
/****************************************************************************/
uint8_t BME280_ReadTPH(BME280 *bme)
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

	bme->adc_P = 0;
	bme->adc_T = 0;
	bme->adc_H = 0;

	// ----- if occured some error during parameterization sensor don't do any measure -----
	if(bme->err_conf) return 1;

#if BME280_INCLUDE_STATUS
	// ----- get status of sensor -----
	get_status(bme);

	// ----- if sensor is in measuring or im_update status, wait up to the finishing and after that read measures -----
	if( (bme->measuring_staus)  || (bme->im_update_staus)) return 2;

#endif

	BME280_read_data(BME280_ADDR, 0xF7, 8, (uint8_t *)&temp);	// read data register

	bme->adc_P = (temp[0] << 12) | (temp[1] << 4) | (temp[2] >> 4);
	bme->adc_T = (temp[3] << 12) | (temp[4] << 4) | (temp[5] >> 4);
	bme->adc_H = (temp[6] << 8)  |  temp[7];


	// ----- check boundaries -----
	check_boundaries(bme);

	// ----- if raw values are lower or over the limits, function is intermittent and returning 3  -----
	if ((bme->err_boundaries_T != 0) || ( bme->err_boundaries_P != 0) || ( bme->err_boundaries_H != 0)) return 3;

	/*-----------------------------------------------------------------------*/
	/********************* calculate temperature *****************************/
	/*-----------------------------------------------------------------------*/

    var1 = (int32_t)((bme->adc_T / 8) - ((int32_t)bme->coef.dig_T1 * 2));
    var1 = (var1 * ((int32_t)bme->coef.dig_T2)) / 2048;
    var2 = (int32_t)((bme->adc_T / 16) - ((int32_t)bme->coef.dig_T1));
    var2 = (((var2 * var2) / 4096) * ((int32_t)bme->coef.dig_T3)) / 16384;
    t_fine = var1 + var2;
    bme->temperature = (t_fine * 5 + 128) / 256;

    if (bme->temperature < temperature_min)
    {
    	bme->temperature = temperature_min;
    }
    else if (bme->temperature > temperature_max)
    {
    	bme->temperature = temperature_max;
    }

	if(my_abs(bme->temperature) > 9) 	divisor = 100;
	else 								divisor = 10;


	bme->t1 = (int32_t)bme->temperature / (int8_t)divisor;
	bme->t2 = my_abs((uint32_t)bme->temperature % (uint8_t)divisor);

	// ----- prepare string with value of temperature -----
#if USE_STRING

	#if CALCULATION_AVERAGE_TEMP


		// ----- calculation of average temperature -----
		calculation_average_temp(bme);

		itoa(bme->avearage_temp_cel, &bme->temp2str[0], 10);
		len = strlen(bme->temp2str);
		bme->temp2str[len++] = ',';

		if( bme->avearage_temp_fract < 10) bme->temp2str[len++] = '0';
		itoa(bme->avearage_temp_fract, &bme->temp2str[len++],10);

	#else

		itoa(bme->t1, &bme->temp2str[0], 10);
		len = strlen(bme->temp2str);
		bme->temp2str[len++] = ',';

		if( bme->t2 < 10) bme->temp2str[len++] = '0';
		itoa(bme->t2, &bme->temp2str[len++],10);
	#endif

#endif

	/*-----------------------------------------------------------------------*/
	/********************* calculate pressure ********************************/
	/*-----------------------------------------------------------------------*/

	bme->compensate_status = 0;
	var1 = 0;
	var2 = 0;
	p 	 = 0;

	var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
	var2 = (((var1 >> 2) * (var1 >> 2)) >> 11 ) * ((int32_t)bme->coef.dig_P6);
	var2 = var2 + ((var1 * ((int32_t)bme->coef.dig_P5)) << 1);
	var2 = (var2 >> 2) + (((int32_t)bme->coef.dig_P4) << 16);
	var1 = (((bme->coef.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13 )) >> 3) + ((((int32_t)bme->coef.dig_P2) * var1) >> 1)) >> 18;
	var1 =((((32768 + var1)) * ((int32_t)bme->coef.dig_P1)) >> 15);

	if (var1 == 0) //if dividing by 0, function is intermittent and returning 4
	{
		bme->compensate_status = 1;
		return 4;
	}

	p = (((uint32_t)(((int32_t)1048576) - bme->adc_P) - (var2 >> 12))) * 3125;

	if (p < 0x80000000) p = (p << 1) / ((uint32_t)var1);
	else				p = (p / (uint32_t)var1) * 2;


	var1 = (((int32_t)bme->coef.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
	var2 = (((int32_t)(p >> 2)) * ((int32_t)bme->coef.dig_P8)) >> 13;
	p = (uint32_t)((int32_t)p + ((var1 + var2 + bme->coef.dig_P7) >> 4));


	bme->preasure = (uint32_t)p;
	bme->p1 =  (int32_t)bme->preasure;

	// ----- prepare string with value of pressure -----
#if USE_STRING
	if ((bme->p1/100) < 1000)
	{
		bme->pressure2str[0] = ' ';
		itoa(bme->p1/100, &bme->pressure2str[1],10);
	}
	else
	{
		itoa(bme->p1/100, &bme->pressure2str[0],10);
	}

	bme->pressure2str[4] = ',';
	itoa(bme->p1%100, &bme->pressure2str[5],10);

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
	var2 = (int32_t)(bme->adc_H * 16384);
	var3 = (int32_t)(((int32_t)bme->coef.dig_H4) * 1048576);
	var4 = ((int32_t)bme->coef.dig_H5) * var1;
	var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
	var2 = (var1 * ((int32_t)bme->coef.dig_H6)) / 1024;
	var3 = (var1 * ((int32_t)bme->coef.dig_H3)) / 2048;
	var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
	var2 = ((var4 * ((int32_t)bme->coef.dig_H2)) + 8192) / 16384;
	var3 = var5 * var2;
	var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
	var5 = var3 - ((var4 * ((int32_t)bme->coef.dig_H1)) / 16);
	var5 = (var5 < 0 ? 0 : var5);
	var5 = (var5 > 419430400 ? 419430400 : var5);
	bme->humidity = (uint32_t)(var5 / 4096);

	if (bme->humidity > humidity_max)
	{
		bme->humidity = humidity_max;
	}

	bme->humidity *= 100;
	bme->humidity /= 1024;


	if(my_abs(bme->humidity/100) > 9) 	divisor = 100;
	else 								divisor = 10;


	bme->h1 = (int32_t)bme->humidity / (int8_t)divisor;
	bme->h2 = my_abs((uint32_t)bme->humidity % (uint8_t)divisor);

	// ----- prepare string with value of humidity -----
#if USE_STRING

	#if CALCULATION_AVERAGE_HUMIDITY

		// ----- calculation of average humidity -----
		calculation_average_humidity(bme);

		itoa(bme->avearage_humidity_cel, &bme->humi2str[0], 10);
		len = strlen(bme->humi2str);
		bme->humi2str[len++] = ',';

		if( bme->avearage_humidity_fract < 10) bme->humi2str[len++] = '0';
		itoa(bme->avearage_humidity_fract, &bme->humi2str[len++],10);

	#else

		itoa(bme->h1, &bme->humi2str[0], 10);
		len = strlen(bme->humi2str);
		bme->humi2str[len++] = ',';

		if( bme->h2 < 10) bme->humi2str[len++] = '0';
		itoa(bme->h2, &bme->humi2str[len++],10);
	#endif

#endif

	// ----- measure and prepare values for the next reading -----
	if(conf_BME280.mode == BME280_FORCEDMODE)
	{
		BME280_write_data(BME280_ADDR, 0xF4, 1, &conf_BME280.bt[1]);		// write configurations bytes
	}

	// ----- calculate a preasure sea level -----
	pressure_at_sea_level(bme);

	return 0;	// if everything is OK return 0
}

/****************************************************************************/
/*      check if read uncompensated values are in boundary MIN and MAX      */
/****************************************************************************/
void check_boundaries (BME280 *bme)
{
	bme->err_boundaries_T = 0;
	bme->err_boundaries_P = 0;
	bme->err_boundaries_H = 0;

	if 		(bme->adc_T <= BME280_ST_ADC_MIN_T_P) bme->err_boundaries_T = T_lower_limit;
	else if (bme->adc_T >= BME280_ST_ADC_MAX_T_P) bme->err_boundaries_T = T_over_limit;

	if 		(bme->adc_P <= BME280_ST_ADC_MIN_T_P) bme->err_boundaries_P = P_lower_limit;
	else if (bme->adc_P >= BME280_ST_ADC_MAX_T_P) bme->err_boundaries_P = P_over_limit;

	if      (bme->adc_H <= BME280_ST_ADC_MIN_H)   bme->err_boundaries_H = H_lower_limit;
	else if (bme->adc_H >= BME280_ST_ADC_MAX_H)   bme->err_boundaries_H = H_over_limit;

}

/****************************************************************************/
/*      read statuses of sensor      										*/
/****************************************************************************/
void get_status (BME280 *bme)
{
	uint8_t *status = 0;

	BME280_read_data(BME280_ADDR, 0xF3, 1, status);	// read set register


	bme->measuring_staus =  *status & BMP280_MEASURING_STATUS;
	bme->im_update_staus =  *status & BMP280_IM_UPDATE_STATUS;
}

/****************************************************************************/
/*      measurement time in milliseconds for the active configuration       */
/****************************************************************************/
uint8_t bme280_compute_measure_time(MEASUREMENT_TIME type, CONF *sensor)
{

	uint32_t t_dur = 0, p_dur = 0, h_dur = 0, mesas_time = 0;

	if (type == typical_time)
	{
		if( sensor->osrs_t != BME280_SKIPPED) t_dur = 2000 * (uint8_t)sensor->osrs_t;
		else 								  t_dur = 0;

		if( sensor->osrs_p != BME280_SKIPPED) p_dur = 2000 * (uint8_t)sensor->osrs_p + 500;
		else 								  p_dur = 0;

		if( sensor->osrs_h != BME280_SKIPPED) h_dur = 2000 * (uint8_t)sensor->osrs_h + 500;
		else 								  h_dur = 0;

		mesas_time = 1000 + t_dur + p_dur + h_dur;
	}


	if (type == max_time)
	{
		if( sensor->osrs_t != BME280_SKIPPED) t_dur = 2300 * (uint8_t)sensor->osrs_t;
		else 								  t_dur = 0;

		if( sensor->osrs_p != BME280_SKIPPED) p_dur = 2300 * (uint8_t)sensor->osrs_p + 575;
		else 								  p_dur = 0;

		if( sensor->osrs_h != BME280_SKIPPED) h_dur = 2300 * (uint8_t)sensor->osrs_h + 575;
		else 								  h_dur = 0;

		mesas_time = 1250 * 1 + t_dur + p_dur + h_dur;

	}

	mesas_time += 500;	// Increment the value to next highest integer if greater than 0.5
	mesas_time /= 1000;	// Convert to milliseconds

    return mesas_time;
}

/****************************************************************************/
/*      calculating pressure reduced to sea level            				*/
/****************************************************************************/
void pressure_at_sea_level(BME280 *bme){

        uint16_t st_baryczny,tpm,t_sr;
        uint32_t p0,p_sr;

        st_baryczny = (800000 * (1000 + 4 * bme->temperature) / (bme->preasure));          		// calculation of the baric degree acc. to the pattern of the Babineta
        p0 = bme->preasure + (100000 * BME280_ALTITUDE / (st_baryczny));              			// calculation of approximate sea level pressure
        p_sr = (bme->preasure + p0) / 2;                     									// calculation of average pressure for layers between sea level and sensor
        tpm = bme->temperature + ((6 * BME280_ALTITUDE) / 1000);                           		// calculation of average temperature for layer of air
        t_sr = (bme->temperature + tpm) / 2;                                              		// calculation of average temperature for layer between sea level and sensor
        st_baryczny = (800000 * (1000 + 4 * t_sr) / (p_sr));              						// calculation of more accurate value of baric degree
        bme->sea_pressure_redu = bme->preasure + (100000 * BME280_ALTITUDE / (st_baryczny)); 	// calculation of more accurate value of pressure for sea level
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
uint8_t read_compensation_parameter_write_configuration_and_check_it (CONF *sensor, BME280 *bme)
{
	uint8_t buf[3];
	uint8_t i;
	uint8_t bt_temp[3];
	uint8_t temp_humidity[7];

	BME280_write_data(BME280_ADDR, 0xF2,  1, &sensor->bt[0]);		// write configurations byte for ctrl_hum
	BME280_write_data(BME280_ADDR, 0xF4,  2, &sensor->bt[1]);		// write configurations bytes
	BME280_read_data(BME280_ADDR,  0xF2,  1, &buf[0]);				// read set registers
	BME280_read_data(BME280_ADDR,  0xF4,  2, &buf[1]);				// read set registers
	BME280_read_data(BME280_ADDR,  0x88, 24, bme->coef.bt);			// read compensation parameters: 0x88 -> 0x9F
	BME280_read_data(BME280_ADDR,  0xA1,  1, &bme->coef.bt[24]);	// read compensation parameters: 0xA1

	BME280_read_data(BME280_ADDR,  0xE1,  7, &temp_humidity[0]);	// read compensation parameters: 0xE1 -> 0xE2

	bme->coef.dig_H2 = ((uint16_t)temp_humidity[1] << 8 ) | (uint16_t)temp_humidity[0];
	bme->coef.dig_H3 = temp_humidity[2];
	bme->coef.dig_H4 = ((uint16_t)temp_humidity[3] << 4) | ((uint16_t)temp_humidity[4] & 0x0F);
	bme->coef.dig_H5 = ((uint16_t)temp_humidity[5] << 4) | ((uint16_t)temp_humidity[4] >> 4);
	bme->coef.dig_H6 = temp_humidity[6];


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
				if(bme->err_conf == calib_reg) 	bme->err_conf = both;
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
				if(bme->err_conf == calib_reg) 	bme->err_conf = both;
				else 							bme->err_conf = config_reg;
				return 2;
			}
		}
	}


	// ----- check if pressure and temperature calibration coefficients are diferent from 0 -----
	// ----- coefficients of humidity can contain 0 value so that aren't checked -----
	for( i = 0; i < (SIZE_OF_PT_UNION/2); i++)
	{
		if(bme->coef.bt2[i] == 0)
		{
			bme->err_conf = calib_reg;
			return 1;
		}
	}


	return 0;
}

/****************************************************************************/
/*     Calculate average temperature,										*/
/* 	   No of samples to calculations is taken from No_OF_SAMPLES 			*/
/****************************************************************************/
#if CALCULATION_AVERAGE_TEMP
	void calculation_average_temp(BME280 *bme)
	{
		int32_t avearage_temp_value = 0;
		static uint8_t i = 1;
		uint8_t k;
		uint8_t avearage_fract_temp;

		if(i <= No_OF_SAMPLES)
		{
			if(bme->t1 >= 0) bme->smaples_of_temp[i-1] = 100 * bme->t1 + bme->t2;
			else 			 bme->smaples_of_temp[i-1] = -1 * (100 * my_abs(bme->t1) + bme->t2);
		}
		else
		{
			for (k = 1; k < i - 1; k++)
			{
				bme->smaples_of_temp[k-1] = bme->smaples_of_temp[k];
			}

			if(bme->t1 >= 0) bme->smaples_of_temp[--k] = 100 * bme->t1 + bme->t2;
			else 			 bme->smaples_of_temp[--k] = -1 * (100 * my_abs(bme->t1) + bme->t2);
		}

		for (k = 0; k < i - 1; k++)
		{
			avearage_temp_value += bme->smaples_of_temp[k];
		}

		avearage_temp_value /= (i - 1);

		bme->avearage_temp_cel = avearage_temp_value / 100 ;
		avearage_fract_temp = (my_abs(avearage_temp_value)) % 100;
		bme->avearage_temp_fract = avearage_fract_temp;

		if( i <= No_OF_SAMPLES) i++;
	}
#endif

	/****************************************************************************/
	/*     Calculate average humidity,										*/
	/* 	   No of samples to calculations is taken from No_OF_SAMPLES 			*/
	/****************************************************************************/
	#if CALCULATION_AVERAGE_HUMIDITY
		void calculation_average_humidity(BME280 *bme)
		{
			uint32_t avearage_humidity_value = 0;
			static uint8_t i = 1;
			uint8_t k;
			uint8_t avearage_fract_humidity;

			if(i <= No_OF_SAMPLES)
			{
				bme->smaples_of_humidity[i-1] = 100 * bme->h1 + bme->h2;

			}
			else
			{
				for (k = 1; k < i - 1; k++)
				{
					bme->smaples_of_humidity[k-1] = bme->smaples_of_humidity[k];
				}

				bme->smaples_of_humidity[--k] = 100 * bme->h1 + bme->h2;
			}

			for (k = 0; k < i - 1; k++)
			{
				avearage_humidity_value += bme->smaples_of_humidity[k];
			}

			avearage_humidity_value /= (i - 1);

			bme->avearage_humidity_cel = avearage_humidity_value / 100 ;
			avearage_fract_humidity = (my_abs(avearage_humidity_value)) % 100;
			bme->avearage_humidity_fract = avearage_fract_humidity;

			if( i <= No_OF_SAMPLES) i++;
		}
	#endif


