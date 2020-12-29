This repository consist of library for handling BME280 sensor. It is prepared by suing blue pill (STM32F103C8T6 microprocesor) Library features:

* setting configurations of sensor and checking if saved configuration is equal to set,
* making software reset,
* checking if measured raw values are in min and max boundaries,
* reading current measurement status,
* calculating a measurement time in milliseconds for the active configuration,
* reading and calculating value of temperature, pressure and humidity,
* calculating pressure reduced to sea level,
* communications with sensor by using two protocols: I2C and SPI
* calculating average temperature and humidity,
* auto-preparing strings with calculated temperature, pressure and humidity,
* checking sensor errors, such as: checking if compensation parameter haven't 0 value; checking if saved configuration registers have that same values as we set, checking if the initialization phase was successful
