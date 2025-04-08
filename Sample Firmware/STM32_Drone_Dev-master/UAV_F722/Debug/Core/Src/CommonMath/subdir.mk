################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/CommonMath/crc.c \
../Core/Src/CommonMath/filters.c \
../Core/Src/CommonMath/gps_conversion.c \
../Core/Src/CommonMath/maths.c \
../Core/Src/CommonMath/streambuf.c \
../Core/Src/CommonMath/vector.c 

OBJS += \
./Core/Src/CommonMath/crc.o \
./Core/Src/CommonMath/filters.o \
./Core/Src/CommonMath/gps_conversion.o \
./Core/Src/CommonMath/maths.o \
./Core/Src/CommonMath/streambuf.o \
./Core/Src/CommonMath/vector.o 

C_DEPS += \
./Core/Src/CommonMath/crc.d \
./Core/Src/CommonMath/filters.d \
./Core/Src/CommonMath/gps_conversion.d \
./Core/Src/CommonMath/maths.d \
./Core/Src/CommonMath/streambuf.d \
./Core/Src/CommonMath/vector.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/CommonMath/%.o Core/Src/CommonMath/%.su Core/Src/CommonMath/%.cyclo: ../Core/Src/CommonMath/%.c Core/Src/CommonMath/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F722xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-CommonMath

clean-Core-2f-Src-2f-CommonMath:
	-$(RM) ./Core/Src/CommonMath/crc.cyclo ./Core/Src/CommonMath/crc.d ./Core/Src/CommonMath/crc.o ./Core/Src/CommonMath/crc.su ./Core/Src/CommonMath/filters.cyclo ./Core/Src/CommonMath/filters.d ./Core/Src/CommonMath/filters.o ./Core/Src/CommonMath/filters.su ./Core/Src/CommonMath/gps_conversion.cyclo ./Core/Src/CommonMath/gps_conversion.d ./Core/Src/CommonMath/gps_conversion.o ./Core/Src/CommonMath/gps_conversion.su ./Core/Src/CommonMath/maths.cyclo ./Core/Src/CommonMath/maths.d ./Core/Src/CommonMath/maths.o ./Core/Src/CommonMath/maths.su ./Core/Src/CommonMath/streambuf.cyclo ./Core/Src/CommonMath/streambuf.d ./Core/Src/CommonMath/streambuf.o ./Core/Src/CommonMath/streambuf.su ./Core/Src/CommonMath/vector.cyclo ./Core/Src/CommonMath/vector.d ./Core/Src/CommonMath/vector.o ./Core/Src/CommonMath/vector.su

.PHONY: clean-Core-2f-Src-2f-CommonMath

