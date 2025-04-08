################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Altitude.c \
../Core/Src/BMP280.c \
../Core/Src/Flight_System.c \
../Core/Src/ICM42688P.c \
../Core/Src/PID.c \
../Core/Src/SensorsCalibration.c \
../Core/Src/Timers.c \
../Core/Src/adc.c \
../Core/Src/crsf.c \
../Core/Src/dShot.c \
../Core/Src/dma.c \
../Core/Src/filters.c \
../Core/Src/gpio.c \
../Core/Src/i2c.c \
../Core/Src/main.c \
../Core/Src/spi.c \
../Core/Src/stm32f7xx_hal_msp.c \
../Core/Src/stm32f7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f7xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/Altitude.o \
./Core/Src/BMP280.o \
./Core/Src/Flight_System.o \
./Core/Src/ICM42688P.o \
./Core/Src/PID.o \
./Core/Src/SensorsCalibration.o \
./Core/Src/Timers.o \
./Core/Src/adc.o \
./Core/Src/crsf.o \
./Core/Src/dShot.o \
./Core/Src/dma.o \
./Core/Src/filters.o \
./Core/Src/gpio.o \
./Core/Src/i2c.o \
./Core/Src/main.o \
./Core/Src/spi.o \
./Core/Src/stm32f7xx_hal_msp.o \
./Core/Src/stm32f7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f7xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/Altitude.d \
./Core/Src/BMP280.d \
./Core/Src/Flight_System.d \
./Core/Src/ICM42688P.d \
./Core/Src/PID.d \
./Core/Src/SensorsCalibration.d \
./Core/Src/Timers.d \
./Core/Src/adc.d \
./Core/Src/crsf.d \
./Core/Src/dShot.d \
./Core/Src/dma.d \
./Core/Src/filters.d \
./Core/Src/gpio.d \
./Core/Src/i2c.d \
./Core/Src/main.d \
./Core/Src/spi.d \
./Core/Src/stm32f7xx_hal_msp.d \
./Core/Src/stm32f7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f7xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F722xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Altitude.cyclo ./Core/Src/Altitude.d ./Core/Src/Altitude.o ./Core/Src/Altitude.su ./Core/Src/BMP280.cyclo ./Core/Src/BMP280.d ./Core/Src/BMP280.o ./Core/Src/BMP280.su ./Core/Src/Flight_System.cyclo ./Core/Src/Flight_System.d ./Core/Src/Flight_System.o ./Core/Src/Flight_System.su ./Core/Src/ICM42688P.cyclo ./Core/Src/ICM42688P.d ./Core/Src/ICM42688P.o ./Core/Src/ICM42688P.su ./Core/Src/PID.cyclo ./Core/Src/PID.d ./Core/Src/PID.o ./Core/Src/PID.su ./Core/Src/SensorsCalibration.cyclo ./Core/Src/SensorsCalibration.d ./Core/Src/SensorsCalibration.o ./Core/Src/SensorsCalibration.su ./Core/Src/Timers.cyclo ./Core/Src/Timers.d ./Core/Src/Timers.o ./Core/Src/Timers.su ./Core/Src/adc.cyclo ./Core/Src/adc.d ./Core/Src/adc.o ./Core/Src/adc.su ./Core/Src/crsf.cyclo ./Core/Src/crsf.d ./Core/Src/crsf.o ./Core/Src/crsf.su ./Core/Src/dShot.cyclo ./Core/Src/dShot.d ./Core/Src/dShot.o ./Core/Src/dShot.su ./Core/Src/dma.cyclo ./Core/Src/dma.d ./Core/Src/dma.o ./Core/Src/dma.su ./Core/Src/filters.cyclo ./Core/Src/filters.d ./Core/Src/filters.o ./Core/Src/filters.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/i2c.cyclo ./Core/Src/i2c.d ./Core/Src/i2c.o ./Core/Src/i2c.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32f7xx_hal_msp.cyclo ./Core/Src/stm32f7xx_hal_msp.d ./Core/Src/stm32f7xx_hal_msp.o ./Core/Src/stm32f7xx_hal_msp.su ./Core/Src/stm32f7xx_it.cyclo ./Core/Src/stm32f7xx_it.d ./Core/Src/stm32f7xx_it.o ./Core/Src/stm32f7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f7xx.cyclo ./Core/Src/system_stm32f7xx.d ./Core/Src/system_stm32f7xx.o ./Core/Src/system_stm32f7xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su

.PHONY: clean-Core-2f-Src

