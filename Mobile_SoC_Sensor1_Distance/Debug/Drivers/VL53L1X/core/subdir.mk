################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/VL53L1X/core/VL53L1X_api.c \
../Drivers/VL53L1X/core/VL53L1X_calibration.c 

OBJS += \
./Drivers/VL53L1X/core/VL53L1X_api.o \
./Drivers/VL53L1X/core/VL53L1X_calibration.o 

C_DEPS += \
./Drivers/VL53L1X/core/VL53L1X_api.d \
./Drivers/VL53L1X/core/VL53L1X_calibration.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/VL53L1X/core/%.o Drivers/VL53L1X/core/%.su Drivers/VL53L1X/core/%.cyclo: ../Drivers/VL53L1X/core/%.c Drivers/VL53L1X/core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F722xx -c -I"C:/Users/SongYoungBin/Desktop/Soc/Mobile_SoC_Experiments/Mobile_SoC_Sensor1_Distance/Drivers/VL53L1X/core" -I"C:/Users/SongYoungBin/Desktop/Soc/Mobile_SoC_Experiments/Mobile_SoC_Sensor1_Distance/Drivers/VL53L1X/platform" -I../Core/Inc -I"C:/Users/SongYoungBin/Desktop/Soc/Mobile_SoC_Experiments/Mobile_SoC_Sensor1_Distance/Drivers/VL53L1X" -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-VL53L1X-2f-core

clean-Drivers-2f-VL53L1X-2f-core:
	-$(RM) ./Drivers/VL53L1X/core/VL53L1X_api.cyclo ./Drivers/VL53L1X/core/VL53L1X_api.d ./Drivers/VL53L1X/core/VL53L1X_api.o ./Drivers/VL53L1X/core/VL53L1X_api.su ./Drivers/VL53L1X/core/VL53L1X_calibration.cyclo ./Drivers/VL53L1X/core/VL53L1X_calibration.d ./Drivers/VL53L1X/core/VL53L1X_calibration.o ./Drivers/VL53L1X/core/VL53L1X_calibration.su

.PHONY: clean-Drivers-2f-VL53L1X-2f-core

