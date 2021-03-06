/*
 * BME280.h
 *
 *  Created on: 18.12.2020
 *      Author: Piotr
 */

#ifndef BME280_BME280_H_
#define BME280_BME280_H_

#include "stm32f10x.h"
#include "../SPI/SPI.h"
#include "../I2C/I2C.h"
#include <string.h>
#include <stdlib.h>
#include "../COMMON/common_var.h"

// --------------------------------------------------------- //
//select communication protocol
#define BME280_SPI 1
#define BME280_I2C 0

// --------------------------------------------------------- //
#define USE_STRING 1				// allow for preparing of string with temperature and pressure values
#define BME280_INCLUDE_STATUS 0		// allow for waiting up to sensor will be in standby mode (standby time)
#define BME280_ALTITUDE 	205 	// current sensor altitude above sea level at the measurement site [m]

// --------------------------------------------------------- //
#define BME280_ADDR 		0xEC	// Sensor addres -> SDO pin is connected to GND
//#define BME280_ADDR 		0xEE	// Sensor addres -> SDO pin is connected to VCC

// --------------------------------------------------------- //
// Oversampling for registers:
//		osrs_h[2:0] -> addres register 0xF2 bits: 0,1,2;
//		osrs_p[2:0] -> addres register 0xF4 bits: 2,3,4;
//		osrs_t[2:0] -> addres register 0xF4 bits: 5,6,7;
#define BME280_SKIPPED				0	// Oversampling skipped
#define BME280_oversampling_x1		1	// Oversampling x1
#define BME280_oversampling_x2		2	// Oversampling x2
#define BME280_oversampling_x4		3	// Oversampling x4
#define BME280_oversampling_x8		4	// Oversampling x8
#define BME280_oversampling_x16		5	// Oversampling x16

// --------------------------------------------------------- //
// IIR filter ->  filter[2:0]  -> addres register 0xF5 bits: 2, 3, 4
#define BME280_FILTER_OFF	0	// Filter OFF				-> 1 sample to calculate
#define BME280_FILTER_X2 	1	// Filter coefficient 2		-> 2 samples to calculate
#define BME280_FILTER_X4 	2	// Filter coefficient 4		-> 5 samples to calculate
#define BME280_FILTER_X8	3	// Filter coefficient 8		-> 11 samples to calculate
#define BME280_FILTER_X16 	4	// Filter coefficient 16	-> 22 samples to calculate

// --------------------------------------------------------- //
// Mode -> mode[1:0] -> addres register 0xF4 bits: 0,1
#define BME280_SLEEPMODE		0
#define BME280_FORCEDMODE		1
#define BME280_NORMALMODE		3

// --------------------------------------------------------- //
// t_standby time ->  t_sb[2:0] -> addres register 0xF5 bits: 5, 6, 7
#define BME280_STANDBY_MS_0_5	0
#define BME280_STANDBY_MS_62_5	1
#define BME280_STANDBY_MS_125	2
#define BME280_STANDBY_MS_250 	3
#define BME280_STANDBY_MS_500	4
#define BME280_STANDBY_MS_1000	5
#define BME280_STANDBY_MS_10	6
#define BME280_STANDBY_MS_20	7

// --------------------------------------------------------- //
//soft reset ->  reset[7:0] ->	If the value 0x60 is written to the register,
//								the device is reset using the complete power-on-reset procedure
#define BME280_SOFTWARE_RESET 0xB6

// --------------------------------------------------------- //
// definisions of maximum rav values for temperature and pressure and humidity
#define BME280_ST_ADC_MAX_T_P (int32_t)0x800000
#define BME280_ST_ADC_MIN_T_P (int32_t)0x000000
#define BME280_ST_ADC_MAX_H    (int32_t)0x8000
#define BME280_ST_ADC_MIN_H    (int32_t)0x0000


// --------------------------------------------------------- //
//Status registers -> addres register 0xF4:
//		measuring[0] - bit 3
//		in_update[0] - bit 0
#define BMP280_MEASURING_STATUS 0x8
#define BMP280_IM_UPDATE_STATUS 0x1

// --------------------------------------------------------- //
#define SIZE_OF_TCOEF_UNION 33
#define SIZE_OF_PT_UNION 24			//pressure and temperature union

// --------------------------------------------------------- //
// calculation of average values of temperature and humidity
#define CALCULATION_AVERAGE_TEMP 1
#define CALCULATION_AVERAGE_HUMIDITY 1
#define No_OF_SAMPLES 10

// --------------------------------------------------------- //
// 3-wire SPI interface -> spi3w_en[0]  -> addres register 0xF5 bits: 0
#if BME280_SPI
#define BME280_SPI_3_WIRE	1
#endif


// --------------------------------------------------------- //
typedef enum {T_lower_limit = 1, T_over_limit = 2, P_lower_limit = 3, P_over_limit = 4, H_lower_limit = 5, H_over_limit = 6 } ERR_BOUNDARIES;
typedef enum {calib_reg = 1, config_reg = 2, both = 3} ERR_CONF;
typedef enum {typical_time = 1, max_time = 2} MEASUREMENT_TIME;

// --------------------------------------------------------- //
typedef union {
	uint8_t bt[3];
	struct {
		uint8_t osrs_h:3;		// Enabling/disabling the humidity and oversampling humidity
		uint8_t reserved1:5;	// reserved bits
		uint8_t mode:2;			// Measurement modes (Sleep/Forced/Normal mode)
		uint8_t	osrs_p:3;		// Enabling/disabling the measurement and oversampling pressure
		uint8_t	osrs_t:3;		// Enabling/disabling the temperature measurement and oversampling temperature
		uint8_t	spi3w_en:1;		// Enables 3-wire SPI interface when set to �1�.
		uint8_t reserved2:1;	// reserved bit
		uint8_t	filter:3;		// IIR filter
		uint8_t	t_sb:3;			// standby time

	};
}CONF;

extern CONF conf_BME280;

// --------------------------------------------------------- //
typedef union {
	uint8_t  bt[SIZE_OF_TCOEF_UNION];
	uint16_t bt2[SIZE_OF_PT_UNION/2];

	struct {
		uint16_t dig_T1; 	//0x88 - 0x89 		-> [ 0, 1] - index of bt table
		int16_t  dig_T2;	//0x8A - 0x8B 		-> [ 2, 3] - index of bt table
		int16_t  dig_T3;	//0x8C - 0x8D 		-> [ 4, 5] - index of bt table
		uint16_t dig_P1;	//0x8E - 0x8F 		-> [ 6, 7] - index of bt table
		int16_t  dig_P2;	//0x90 - 0x91 		-> [ 8, 9] - index of bt table
		int16_t  dig_P3;	//0x92 - 0x93		-> [10,11] - index of bt table
		int16_t  dig_P4;	//0x94 - 0x95 		-> [12,13] - index of bt table
		int16_t  dig_P5;	//0x96 - 0x97 		-> [14,15] - index of bt table
		int16_t  dig_P6;	//0x98 - 0x99 		-> [16,17] - index of bt table
		int16_t  dig_P7;	//0x9A - 0x9B 		-> [18,19] - index of bt table
		int16_t  dig_P8;	//0x9C - 0x9D 		-> [20,21] - index of bt table
		int16_t  dig_P9;	//0x9E - 0x9F 		-> [22,23] - index of bt table
		//-------------------------------
		uint8_t  dig_H1;	//0xA1		  		->    [24] - index of bt table
		//-------------------------------
		int16_t  dig_H2;	//0xE1 - 0xE2 		-> [25,26] - index of bt table
		uint8_t  dig_H3;	//0xE3		  		->    [27] - index of bt table
		int16_t  dig_H4;	//0xE4 - 0xE5[3:0] 	-> [28,29] - index of bt table
		int16_t  dig_H5;	//0xE5[7:4] - 0xE6 	-> [30,31] - index of bt table
		int8_t   dig_H6;	//0xE7		 		->    [32] - index of bt table


	};
} TCOEF;


typedef struct {
	TCOEF coef;
	uint8_t measuring_staus;	// status of measuring sensor
	uint8_t im_update_staus ;	// status of im update sensor
	int32_t adc_T;				// raw value of temperature
	uint32_t adc_P;				// raw value of pressure
	uint32_t adc_H;				// raw value of humidity
	uint8_t err_conf;			// in configurations registers is some error
	uint8_t compensate_status;	// set "1" if division by zero
	uint8_t err_boundaries_T;	// if raw value of temperature is over limits
	uint8_t err_boundaries_P;	// if raw value of pressure is over limits
	uint8_t err_boundaries_H;	// if raw value of humidity is over limits


	// ----- temperature -----
	int32_t temperature;	// x 0,01 degree
	int8_t 	t1;				// before comma
	uint8_t t2;				// after comma

#if CALCULATION_AVERAGE_TEMP
	int8_t avearage_temp_cel;
	uint8_t avearage_temp_fract;
	int16_t smaples_of_temp[No_OF_SAMPLES];
#endif

#if USE_STRING
	char temp2str[6];		// tepmerature as string
#endif

	// ----- pressure -----
	uint32_t 	preasure;		// value of calculated pressure
	int32_t 	p1;				// before comma
	//int32_t 	p2;				// after comma

	// ----- sea pressure -----
	uint32_t sea_pressure_redu;


#if USE_STRING
	char pressure2str[7];		// pressure as string
#endif

//-----------------------------------------------------------------------
	// ----- humidity -----
	int32_t humidity;		// value of calculated humidity (x 0,01 %)
	int8_t 	h1;				// before comma
	uint8_t h2;				// after comma

#if CALCULATION_AVERAGE_HUMIDITY
	int8_t avearage_humidity_cel;
	uint8_t avearage_humidity_fract;
	int16_t smaples_of_humidity[No_OF_SAMPLES];
#endif

#if USE_STRING
	char humi2str[7];		// humidity as string
#endif

} BME280;

extern BME280 bme;


// --------------------------------------------------------- //
uint8_t BME280_Conf (CONF *sensor, BME280 *bmp);
uint8_t BME280_ReadTPH(BME280 *bmp);

#endif /* BME280_BME280_H_ */
