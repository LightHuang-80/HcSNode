################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/cia/402/c402.c \
../Core/Src/cia/402/dsposc.c \
../Core/Src/cia/402/dspp.c \
../Core/Src/cia/402/dsstate.c \
../Core/Src/cia/402/home.c 

C_DEPS += \
./Core/Src/cia/402/c402.d \
./Core/Src/cia/402/dsposc.d \
./Core/Src/cia/402/dspp.d \
./Core/Src/cia/402/dsstate.d \
./Core/Src/cia/402/home.d 

OBJS += \
./Core/Src/cia/402/c402.o \
./Core/Src/cia/402/dsposc.o \
./Core/Src/cia/402/dspp.o \
./Core/Src/cia/402/dsstate.o \
./Core/Src/cia/402/home.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/cia/402/%.o Core/Src/cia/402/%.su: ../Core/Src/cia/402/%.c Core/Src/cia/402/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DSTM32F405xx -DUSE_HAL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"D:/projects/hcbot/HcSNode/Core/Src/cia/301" -I"D:/projects/hcbot/HcSNode/Core/Src/cia/402" -I"D:/projects/hcbot/HcSNode/Core/Src/cia/board" -I"D:/projects/hcbot/HcSNode/Core/Src/cia" -I"D:/projects/hcbot/HcSNode/Core/Src/util" -I"D:/projects/hcbot/HcSNode/Core/Src/stepper/tmc" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-cia-2f-402

clean-Core-2f-Src-2f-cia-2f-402:
	-$(RM) ./Core/Src/cia/402/c402.d ./Core/Src/cia/402/c402.o ./Core/Src/cia/402/c402.su ./Core/Src/cia/402/dsposc.d ./Core/Src/cia/402/dsposc.o ./Core/Src/cia/402/dsposc.su ./Core/Src/cia/402/dspp.d ./Core/Src/cia/402/dspp.o ./Core/Src/cia/402/dspp.su ./Core/Src/cia/402/dsstate.d ./Core/Src/cia/402/dsstate.o ./Core/Src/cia/402/dsstate.su ./Core/Src/cia/402/home.d ./Core/Src/cia/402/home.o ./Core/Src/cia/402/home.su

.PHONY: clean-Core-2f-Src-2f-cia-2f-402

