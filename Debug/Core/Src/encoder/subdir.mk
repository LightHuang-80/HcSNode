################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/encoder/as5048a.c \
../Core/Src/encoder/round.c 

C_DEPS += \
./Core/Src/encoder/as5048a.d \
./Core/Src/encoder/round.d 

OBJS += \
./Core/Src/encoder/as5048a.o \
./Core/Src/encoder/round.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/encoder/%.o Core/Src/encoder/%.su Core/Src/encoder/%.cyclo: ../Core/Src/encoder/%.c Core/Src/encoder/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F405xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"D:/projects/hcsnode/Core/Src/stepper/tmc" -I"D:/projects/hcsnode/Core/Src/cia/board" -I"D:/projects/hcsnode/Core/Src/cia/402" -I"D:/projects/hcsnode/Core/Src/cia/305" -I"D:/projects/hcsnode/Core/Src/cia/301" -I"D:/projects/hcsnode/Core/Src/cia" -I"D:/projects/hcsnode/Core/Src/util" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-encoder

clean-Core-2f-Src-2f-encoder:
	-$(RM) ./Core/Src/encoder/as5048a.cyclo ./Core/Src/encoder/as5048a.d ./Core/Src/encoder/as5048a.o ./Core/Src/encoder/as5048a.su ./Core/Src/encoder/round.cyclo ./Core/Src/encoder/round.d ./Core/Src/encoder/round.o ./Core/Src/encoder/round.su

.PHONY: clean-Core-2f-Src-2f-encoder

