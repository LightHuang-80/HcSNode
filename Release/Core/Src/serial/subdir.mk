################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/serial/sw_spi.cpp 

OBJS += \
./Core/Src/serial/sw_spi.o 

CPP_DEPS += \
./Core/Src/serial/sw_spi.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/serial/%.o Core/Src/serial/%.su: ../Core/Src/serial/%.cpp Core/Src/serial/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -DSTM32F405xx -DUSE_HAL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"D:/projects/hcbot/HcSNode/Core/Src/util" -I"D:/projects/hcbot/HcSNode/Core/Src/cia/301" -I"D:/projects/hcbot/HcSNode/Core/Src/cia/board" -I"D:/projects/hcbot/HcSNode/Core/Src/cia/402" -I"D:/projects/hcbot/HcSNode/Core/Src/cia" -I"D:/projects/hcbot/HcSNode/Core/Src/stepper" -I"D:/projects/hcbot/HcSNode/Core/Src/stepper/tmc" -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-serial

clean-Core-2f-Src-2f-serial:
	-$(RM) ./Core/Src/serial/sw_spi.d ./Core/Src/serial/sw_spi.o ./Core/Src/serial/sw_spi.su

.PHONY: clean-Core-2f-Src-2f-serial

