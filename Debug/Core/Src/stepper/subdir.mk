################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/stepper/cmd.c \
../Core/Src/stepper/nstepper.c 

C_DEPS += \
./Core/Src/stepper/cmd.d \
./Core/Src/stepper/nstepper.d 

OBJS += \
./Core/Src/stepper/cmd.o \
./Core/Src/stepper/nstepper.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/stepper/%.o Core/Src/stepper/%.su Core/Src/stepper/%.cyclo: ../Core/Src/stepper/%.c Core/Src/stepper/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F405xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"D:/projects/hcsnode/Core/Src/stepper/tmc" -I"D:/projects/hcsnode/Core/Src/cia/board" -I"D:/projects/hcsnode/Core/Src/cia/402" -I"D:/projects/hcsnode/Core/Src/cia/305" -I"D:/projects/hcsnode/Core/Src/cia/301" -I"D:/projects/hcsnode/Core/Src/cia" -I"D:/projects/hcsnode/Core/Src/util" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-stepper

clean-Core-2f-Src-2f-stepper:
	-$(RM) ./Core/Src/stepper/cmd.cyclo ./Core/Src/stepper/cmd.d ./Core/Src/stepper/cmd.o ./Core/Src/stepper/cmd.su ./Core/Src/stepper/nstepper.cyclo ./Core/Src/stepper/nstepper.d ./Core/Src/stepper/nstepper.o ./Core/Src/stepper/nstepper.su

.PHONY: clean-Core-2f-Src-2f-stepper

