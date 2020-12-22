################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/BME280/BME280.c 

OBJS += \
./src/BME280/BME280.o 

C_DEPS += \
./src/BME280/BME280.d 


# Each subdirectory must supply rules for building sources it contributes
src/BME280/%.o: ../src/BME280/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -I"E:/STM32_ARM/MY_LIBRARIES/BME280/StdPeriph_Driver/inc" -I"E:/STM32_ARM/MY_LIBRARIES/BME280/inc" -I"E:/STM32_ARM/MY_LIBRARIES/BME280/CMSIS/device" -I"E:/STM32_ARM/MY_LIBRARIES/BME280/CMSIS/core" -I"E:/STM32_ARM/MY_LIBRARIES/BME280/src/COMMON" -I"E:/STM32_ARM/MY_LIBRARIES/BME280/src/I2C" -I"E:/STM32_ARM/MY_LIBRARIES/BME280/src/SPI" -I"E:/STM32_ARM/MY_LIBRARIES/BME280/src/UART" -I"E:/STM32_ARM/MY_LIBRARIES/BME280/src/BME280" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


