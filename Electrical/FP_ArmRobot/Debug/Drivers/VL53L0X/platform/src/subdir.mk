################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/VL53L0X/platform/src/vl53l0x_platform.c \
../Drivers/VL53L0X/platform/src/vl53l0x_platform_log.c 

C_DEPS += \
./Drivers/VL53L0X/platform/src/vl53l0x_platform.d \
./Drivers/VL53L0X/platform/src/vl53l0x_platform_log.d 

OBJS += \
./Drivers/VL53L0X/platform/src/vl53l0x_platform.o \
./Drivers/VL53L0X/platform/src/vl53l0x_platform_log.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/VL53L0X/platform/src/%.o Drivers/VL53L0X/platform/src/%.su Drivers/VL53L0X/platform/src/%.cyclo: ../Drivers/VL53L0X/platform/src/%.c Drivers/VL53L0X/platform/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"D:/STM32CubeIDE/workspace_1.14.1/FP_ArmRobot/Drivers/VL53L0X/core/inc" -I"D:/STM32CubeIDE/workspace_1.14.1/FP_ArmRobot/Drivers/VL53L0X/core/src" -I"D:/STM32CubeIDE/workspace_1.14.1/FP_ArmRobot/Drivers/VL53L0X/platform/inc" -I"D:/STM32CubeIDE/workspace_1.14.1/FP_ArmRobot/Drivers/VL53L0X/platform/src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-VL53L0X-2f-platform-2f-src

clean-Drivers-2f-VL53L0X-2f-platform-2f-src:
	-$(RM) ./Drivers/VL53L0X/platform/src/vl53l0x_platform.cyclo ./Drivers/VL53L0X/platform/src/vl53l0x_platform.d ./Drivers/VL53L0X/platform/src/vl53l0x_platform.o ./Drivers/VL53L0X/platform/src/vl53l0x_platform.su ./Drivers/VL53L0X/platform/src/vl53l0x_platform_log.cyclo ./Drivers/VL53L0X/platform/src/vl53l0x_platform_log.d ./Drivers/VL53L0X/platform/src/vl53l0x_platform_log.o ./Drivers/VL53L0X/platform/src/vl53l0x_platform_log.su

.PHONY: clean-Drivers-2f-VL53L0X-2f-platform-2f-src

