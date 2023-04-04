################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/cia/301/CO_Emergency.c \
../Core/Src/cia/301/CO_HBconsumer.c \
../Core/Src/cia/301/CO_NMT_Heartbeat.c \
../Core/Src/cia/301/CO_PDO.c \
../Core/Src/cia/301/CO_SDOclient.c \
../Core/Src/cia/301/CO_SDOserver.c \
../Core/Src/cia/301/CO_SYNC.c \
../Core/Src/cia/301/CO_TIME.c \
../Core/Src/cia/301/crc16-ccitt.c 

C_DEPS += \
./Core/Src/cia/301/CO_Emergency.d \
./Core/Src/cia/301/CO_HBconsumer.d \
./Core/Src/cia/301/CO_NMT_Heartbeat.d \
./Core/Src/cia/301/CO_PDO.d \
./Core/Src/cia/301/CO_SDOclient.d \
./Core/Src/cia/301/CO_SDOserver.d \
./Core/Src/cia/301/CO_SYNC.d \
./Core/Src/cia/301/CO_TIME.d \
./Core/Src/cia/301/crc16-ccitt.d 

OBJS += \
./Core/Src/cia/301/CO_Emergency.o \
./Core/Src/cia/301/CO_HBconsumer.o \
./Core/Src/cia/301/CO_NMT_Heartbeat.o \
./Core/Src/cia/301/CO_PDO.o \
./Core/Src/cia/301/CO_SDOclient.o \
./Core/Src/cia/301/CO_SDOserver.o \
./Core/Src/cia/301/CO_SYNC.o \
./Core/Src/cia/301/CO_TIME.o \
./Core/Src/cia/301/crc16-ccitt.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/cia/301/%.o Core/Src/cia/301/%.su Core/Src/cia/301/%.cyclo: ../Core/Src/cia/301/%.c Core/Src/cia/301/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32F405xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"D:/projects/hcsnode/Core/Src/stepper/tmc" -I"D:/projects/hcsnode/Core/Src/cia/board" -I"D:/projects/hcsnode/Core/Src/cia/402" -I"D:/projects/hcsnode/Core/Src/cia/305" -I"D:/projects/hcsnode/Core/Src/cia/301" -I"D:/projects/hcsnode/Core/Src/cia" -I"D:/projects/hcsnode/Core/Src/util" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-cia-2f-301

clean-Core-2f-Src-2f-cia-2f-301:
	-$(RM) ./Core/Src/cia/301/CO_Emergency.cyclo ./Core/Src/cia/301/CO_Emergency.d ./Core/Src/cia/301/CO_Emergency.o ./Core/Src/cia/301/CO_Emergency.su ./Core/Src/cia/301/CO_HBconsumer.cyclo ./Core/Src/cia/301/CO_HBconsumer.d ./Core/Src/cia/301/CO_HBconsumer.o ./Core/Src/cia/301/CO_HBconsumer.su ./Core/Src/cia/301/CO_NMT_Heartbeat.cyclo ./Core/Src/cia/301/CO_NMT_Heartbeat.d ./Core/Src/cia/301/CO_NMT_Heartbeat.o ./Core/Src/cia/301/CO_NMT_Heartbeat.su ./Core/Src/cia/301/CO_PDO.cyclo ./Core/Src/cia/301/CO_PDO.d ./Core/Src/cia/301/CO_PDO.o ./Core/Src/cia/301/CO_PDO.su ./Core/Src/cia/301/CO_SDOclient.cyclo ./Core/Src/cia/301/CO_SDOclient.d ./Core/Src/cia/301/CO_SDOclient.o ./Core/Src/cia/301/CO_SDOclient.su ./Core/Src/cia/301/CO_SDOserver.cyclo ./Core/Src/cia/301/CO_SDOserver.d ./Core/Src/cia/301/CO_SDOserver.o ./Core/Src/cia/301/CO_SDOserver.su ./Core/Src/cia/301/CO_SYNC.cyclo ./Core/Src/cia/301/CO_SYNC.d ./Core/Src/cia/301/CO_SYNC.o ./Core/Src/cia/301/CO_SYNC.su ./Core/Src/cia/301/CO_TIME.cyclo ./Core/Src/cia/301/CO_TIME.d ./Core/Src/cia/301/CO_TIME.o ./Core/Src/cia/301/CO_TIME.su ./Core/Src/cia/301/crc16-ccitt.cyclo ./Core/Src/cia/301/crc16-ccitt.d ./Core/Src/cia/301/crc16-ccitt.o ./Core/Src/cia/301/crc16-ccitt.su

.PHONY: clean-Core-2f-Src-2f-cia-2f-301

