################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/AS5048A.c \
../Core/Src/BMI270.c \
../Core/Src/DRV8313.c \
../Core/Src/FOC.c \
../Core/Src/FastTrigonometry.c \
../Core/Src/LowPassFilter.c \
../Core/Src/Quaternions.c \
../Core/Src/main.c \
../Core/Src/pid.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/time_utils.c 

OBJS += \
./Core/Src/AS5048A.o \
./Core/Src/BMI270.o \
./Core/Src/DRV8313.o \
./Core/Src/FOC.o \
./Core/Src/FastTrigonometry.o \
./Core/Src/LowPassFilter.o \
./Core/Src/Quaternions.o \
./Core/Src/main.o \
./Core/Src/pid.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/time_utils.o 

C_DEPS += \
./Core/Src/AS5048A.d \
./Core/Src/BMI270.d \
./Core/Src/DRV8313.d \
./Core/Src/FOC.d \
./Core/Src/FastTrigonometry.d \
./Core/Src/LowPassFilter.d \
./Core/Src/Quaternions.d \
./Core/Src/main.d \
./Core/Src/pid.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/time_utils.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/AS5048A.d ./Core/Src/AS5048A.o ./Core/Src/BMI270.d ./Core/Src/BMI270.o ./Core/Src/DRV8313.d ./Core/Src/DRV8313.o ./Core/Src/FOC.d ./Core/Src/FOC.o ./Core/Src/FastTrigonometry.d ./Core/Src/FastTrigonometry.o ./Core/Src/LowPassFilter.d ./Core/Src/LowPassFilter.o ./Core/Src/Quaternions.d ./Core/Src/Quaternions.o ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/time_utils.d ./Core/Src/time_utils.o

.PHONY: clean-Core-2f-Src

