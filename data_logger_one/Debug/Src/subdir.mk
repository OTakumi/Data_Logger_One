################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/adxl372_spi.c \
../Src/main.c \
../Src/stm32l0xx_hal_msp.c \
../Src/stm32l0xx_it.c \
../Src/syscalls.c \
../Src/system_stm32l0xx.c 

OBJS += \
./Src/adxl372_spi.o \
./Src/main.o \
./Src/stm32l0xx_hal_msp.o \
./Src/stm32l0xx_it.o \
./Src/syscalls.o \
./Src/system_stm32l0xx.o 

C_DEPS += \
./Src/adxl372_spi.d \
./Src/main.d \
./Src/stm32l0xx_hal_msp.d \
./Src/stm32l0xx_it.d \
./Src/syscalls.d \
./Src/system_stm32l0xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32L051xx -I"C:/Users/takum/Documents/GitHub/Data_Logger_One/data_logger_one/Inc" -I"C:/Users/takum/Documents/GitHub/Data_Logger_One/data_logger_one/Drivers/STM32L0xx_HAL_Driver/Inc" -I"C:/Users/takum/Documents/GitHub/Data_Logger_One/data_logger_one/Drivers/STM32L0xx_HAL_Driver/Inc/Legacy" -I"C:/Users/takum/Documents/GitHub/Data_Logger_One/data_logger_one/Drivers/CMSIS/Device/ST/STM32L0xx/Include" -I"C:/Users/takum/Documents/GitHub/Data_Logger_One/data_logger_one/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


