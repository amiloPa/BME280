################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../startup/startup_stm32.s 

OBJS += \
./startup/startup_stm32.o 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo $(PWD)
	arm-none-eabi-as -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -I"E:/STM32_ARM/MY_LIBRARIES/BME280/StdPeriph_Driver/inc" -I"E:/STM32_ARM/MY_LIBRARIES/BME280/inc" -I"E:/STM32_ARM/MY_LIBRARIES/BME280/CMSIS/device" -I"E:/STM32_ARM/MY_LIBRARIES/BME280/CMSIS/core" -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


