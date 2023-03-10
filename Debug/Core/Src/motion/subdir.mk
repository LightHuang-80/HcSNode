################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/motion/PIDProfile.c \
../Core/Src/motion/nmotion.c \
../Core/Src/motion/planner.c \
../Core/Src/motion/posprofile.c \
../Core/Src/motion/trajectory.c 

C_DEPS += \
./Core/Src/motion/PIDProfile.d \
./Core/Src/motion/nmotion.d \
./Core/Src/motion/planner.d \
./Core/Src/motion/posprofile.d \
./Core/Src/motion/trajectory.d 

OBJS += \
./Core/Src/motion/PIDProfile.o \
./Core/Src/motion/nmotion.o \
./Core/Src/motion/planner.o \
./Core/Src/motion/posprofile.o \
./Core/Src/motion/trajectory.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/motion/%.o Core/Src/motion/%.su: ../Core/Src/motion/%.c Core/Src/motion/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F405xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"D:/projects/hcbot/HcSNode/Core/Src/cia/board" -I"D:/projects/hcbot/HcSNode/Core/Src/cia/402" -I"D:/projects/hcbot/HcSNode/Core/Src/cia/305" -I"D:/projects/hcbot/HcSNode/Core/Src/cia/301" -I"D:/projects/hcbot/HcSNode/Core/Src/cia" -I"D:/projects/hcbot/HcSNode/Core/Src/util" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-motion

clean-Core-2f-Src-2f-motion:
	-$(RM) ./Core/Src/motion/PIDProfile.d ./Core/Src/motion/PIDProfile.o ./Core/Src/motion/PIDProfile.su ./Core/Src/motion/nmotion.d ./Core/Src/motion/nmotion.o ./Core/Src/motion/nmotion.su ./Core/Src/motion/planner.d ./Core/Src/motion/planner.o ./Core/Src/motion/planner.su ./Core/Src/motion/posprofile.d ./Core/Src/motion/posprofile.o ./Core/Src/motion/posprofile.su ./Core/Src/motion/trajectory.d ./Core/Src/motion/trajectory.o ./Core/Src/motion/trajectory.su

.PHONY: clean-Core-2f-Src-2f-motion

