################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/VL53L1X/platform/vl53l1_platform.c 

OBJS += \
./Drivers/VL53L1X/platform/vl53l1_platform.o 

C_DEPS += \
./Drivers/VL53L1X/platform/vl53l1_platform.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/VL53L1X/platform/%.o Drivers/VL53L1X/platform/%.su Drivers/VL53L1X/platform/%.cyclo: ../Drivers/VL53L1X/platform/%.c Drivers/VL53L1X/platform/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F722xx -c -I"C:/Users/SongYoungBin/Desktop/Soc/Mobile_SoC_Experiments/Mobile_SoC_Sensor1_Distance/Drivers/VL53L1X/core" -I"C:/Users/SongYoungBin/Desktop/Soc/Mobile_SoC_Experiments/Mobile_SoC_Sensor1_Distance/Drivers/VL53L1X/platform" -I../Core/Inc -I"C:/Users/SongYoungBin/Desktop/Soc/Mobile_SoC_Experiments/Mobile_SoC_Sensor1_Distance/Drivers/VL53L1X" -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-VL53L1X-2f-platform

clean-Drivers-2f-VL53L1X-2f-platform:
	-$(RM) ./Drivers/VL53L1X/platform/vl53l1_platform.cyclo ./Drivers/VL53L1X/platform/vl53l1_platform.d ./Drivers/VL53L1X/platform/vl53l1_platform.o ./Drivers/VL53L1X/platform/vl53l1_platform.su

.PHONY: clean-Drivers-2f-VL53L1X-2f-platform

