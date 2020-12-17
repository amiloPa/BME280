################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CMSIS/core/core_cm3.c 

OBJS += \
./CMSIS/core/core_cm3.o 

C_DEPS += \
./CMSIS/core/core_cm3.d 


# Each subdirectory must supply rules for building sources it contributes
CMSIS/core/%.o: ../CMSIS/core/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -DDEBUG -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -I"E:/STM32_ARM/MY_LIBRARIES/BME280/StdPeriph_Driver/inc" -I"E:/STM32_ARM/MY_LIBRARIES/BME280/inc" -I"E:/STM32_ARM/MY_LIBRARIES/BME280/CMSIS/device" -I"E:/STM32_ARM/MY_LIBRARIES/BME280/CMSIS/core" -I"E:/STM32_ARM/MY_LIBRARIES/BME280/src/COMMON" -I"E:/STM32_ARM/MY_LIBRARIES/BME280/src/I2C" -I"E:/STM32_ARM/MY_LIBRARIES/BME280/src/SPI" -I"E:/STM32_ARM/MY_LIBRARIES/BME280/src/UART" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


