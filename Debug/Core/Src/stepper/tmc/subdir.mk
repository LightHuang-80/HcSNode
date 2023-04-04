################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/stepper/tmc/CHOPCONF.cpp \
../Core/Src/stepper/tmc/COOLCONF.cpp \
../Core/Src/stepper/tmc/DRVCONF.cpp \
../Core/Src/stepper/tmc/DRVCTRL.cpp \
../Core/Src/stepper/tmc/DRVSTATUS.cpp \
../Core/Src/stepper/tmc/DRV_CONF.cpp \
../Core/Src/stepper/tmc/DRV_STATUS.cpp \
../Core/Src/stepper/tmc/ENCMODE.cpp \
../Core/Src/stepper/tmc/GCONF.cpp \
../Core/Src/stepper/tmc/IHOLD_IRUN.cpp \
../Core/Src/stepper/tmc/PWMCONF.cpp \
../Core/Src/stepper/tmc/RAMP_STAT.cpp \
../Core/Src/stepper/tmc/SERIAL_SWITCH.cpp \
../Core/Src/stepper/tmc/SGCSCONF.cpp \
../Core/Src/stepper/tmc/SHORT_CONF.cpp \
../Core/Src/stepper/tmc/SMARTEN.cpp \
../Core/Src/stepper/tmc/SW_MODE.cpp \
../Core/Src/stepper/tmc/TMC2130Stepper.cpp \
../Core/Src/stepper/tmc/TMC2160Stepper.cpp \
../Core/Src/stepper/tmc/TMC5130Stepper.cpp \
../Core/Src/stepper/tmc/TMC5160Stepper.cpp \
../Core/Src/stepper/tmc/TMCStepper.cpp 

OBJS += \
./Core/Src/stepper/tmc/CHOPCONF.o \
./Core/Src/stepper/tmc/COOLCONF.o \
./Core/Src/stepper/tmc/DRVCONF.o \
./Core/Src/stepper/tmc/DRVCTRL.o \
./Core/Src/stepper/tmc/DRVSTATUS.o \
./Core/Src/stepper/tmc/DRV_CONF.o \
./Core/Src/stepper/tmc/DRV_STATUS.o \
./Core/Src/stepper/tmc/ENCMODE.o \
./Core/Src/stepper/tmc/GCONF.o \
./Core/Src/stepper/tmc/IHOLD_IRUN.o \
./Core/Src/stepper/tmc/PWMCONF.o \
./Core/Src/stepper/tmc/RAMP_STAT.o \
./Core/Src/stepper/tmc/SERIAL_SWITCH.o \
./Core/Src/stepper/tmc/SGCSCONF.o \
./Core/Src/stepper/tmc/SHORT_CONF.o \
./Core/Src/stepper/tmc/SMARTEN.o \
./Core/Src/stepper/tmc/SW_MODE.o \
./Core/Src/stepper/tmc/TMC2130Stepper.o \
./Core/Src/stepper/tmc/TMC2160Stepper.o \
./Core/Src/stepper/tmc/TMC5130Stepper.o \
./Core/Src/stepper/tmc/TMC5160Stepper.o \
./Core/Src/stepper/tmc/TMCStepper.o 

CPP_DEPS += \
./Core/Src/stepper/tmc/CHOPCONF.d \
./Core/Src/stepper/tmc/COOLCONF.d \
./Core/Src/stepper/tmc/DRVCONF.d \
./Core/Src/stepper/tmc/DRVCTRL.d \
./Core/Src/stepper/tmc/DRVSTATUS.d \
./Core/Src/stepper/tmc/DRV_CONF.d \
./Core/Src/stepper/tmc/DRV_STATUS.d \
./Core/Src/stepper/tmc/ENCMODE.d \
./Core/Src/stepper/tmc/GCONF.d \
./Core/Src/stepper/tmc/IHOLD_IRUN.d \
./Core/Src/stepper/tmc/PWMCONF.d \
./Core/Src/stepper/tmc/RAMP_STAT.d \
./Core/Src/stepper/tmc/SERIAL_SWITCH.d \
./Core/Src/stepper/tmc/SGCSCONF.d \
./Core/Src/stepper/tmc/SHORT_CONF.d \
./Core/Src/stepper/tmc/SMARTEN.d \
./Core/Src/stepper/tmc/SW_MODE.d \
./Core/Src/stepper/tmc/TMC2130Stepper.d \
./Core/Src/stepper/tmc/TMC2160Stepper.d \
./Core/Src/stepper/tmc/TMC5130Stepper.d \
./Core/Src/stepper/tmc/TMC5160Stepper.d \
./Core/Src/stepper/tmc/TMCStepper.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/stepper/tmc/%.o Core/Src/stepper/tmc/%.su Core/Src/stepper/tmc/%.cyclo: ../Core/Src/stepper/tmc/%.cpp Core/Src/stepper/tmc/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DSTM32F405xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"D:/projects/hcsnode/Core/Src/stepper/tmc" -I"D:/projects/hcsnode/Core/Src/util" -I"D:/projects/hcsnode/Core/Src/cia/board" -I"D:/projects/hcsnode/Core/Src/cia/402" -I"D:/projects/hcsnode/Core/Src/cia/305" -I"D:/projects/hcsnode/Core/Src/cia/301" -I"D:/projects/hcsnode/Core/Src/cia" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-stepper-2f-tmc

clean-Core-2f-Src-2f-stepper-2f-tmc:
	-$(RM) ./Core/Src/stepper/tmc/CHOPCONF.cyclo ./Core/Src/stepper/tmc/CHOPCONF.d ./Core/Src/stepper/tmc/CHOPCONF.o ./Core/Src/stepper/tmc/CHOPCONF.su ./Core/Src/stepper/tmc/COOLCONF.cyclo ./Core/Src/stepper/tmc/COOLCONF.d ./Core/Src/stepper/tmc/COOLCONF.o ./Core/Src/stepper/tmc/COOLCONF.su ./Core/Src/stepper/tmc/DRVCONF.cyclo ./Core/Src/stepper/tmc/DRVCONF.d ./Core/Src/stepper/tmc/DRVCONF.o ./Core/Src/stepper/tmc/DRVCONF.su ./Core/Src/stepper/tmc/DRVCTRL.cyclo ./Core/Src/stepper/tmc/DRVCTRL.d ./Core/Src/stepper/tmc/DRVCTRL.o ./Core/Src/stepper/tmc/DRVCTRL.su ./Core/Src/stepper/tmc/DRVSTATUS.cyclo ./Core/Src/stepper/tmc/DRVSTATUS.d ./Core/Src/stepper/tmc/DRVSTATUS.o ./Core/Src/stepper/tmc/DRVSTATUS.su ./Core/Src/stepper/tmc/DRV_CONF.cyclo ./Core/Src/stepper/tmc/DRV_CONF.d ./Core/Src/stepper/tmc/DRV_CONF.o ./Core/Src/stepper/tmc/DRV_CONF.su ./Core/Src/stepper/tmc/DRV_STATUS.cyclo ./Core/Src/stepper/tmc/DRV_STATUS.d ./Core/Src/stepper/tmc/DRV_STATUS.o ./Core/Src/stepper/tmc/DRV_STATUS.su ./Core/Src/stepper/tmc/ENCMODE.cyclo ./Core/Src/stepper/tmc/ENCMODE.d ./Core/Src/stepper/tmc/ENCMODE.o ./Core/Src/stepper/tmc/ENCMODE.su ./Core/Src/stepper/tmc/GCONF.cyclo ./Core/Src/stepper/tmc/GCONF.d ./Core/Src/stepper/tmc/GCONF.o ./Core/Src/stepper/tmc/GCONF.su ./Core/Src/stepper/tmc/IHOLD_IRUN.cyclo ./Core/Src/stepper/tmc/IHOLD_IRUN.d ./Core/Src/stepper/tmc/IHOLD_IRUN.o ./Core/Src/stepper/tmc/IHOLD_IRUN.su ./Core/Src/stepper/tmc/PWMCONF.cyclo ./Core/Src/stepper/tmc/PWMCONF.d ./Core/Src/stepper/tmc/PWMCONF.o ./Core/Src/stepper/tmc/PWMCONF.su ./Core/Src/stepper/tmc/RAMP_STAT.cyclo ./Core/Src/stepper/tmc/RAMP_STAT.d ./Core/Src/stepper/tmc/RAMP_STAT.o ./Core/Src/stepper/tmc/RAMP_STAT.su ./Core/Src/stepper/tmc/SERIAL_SWITCH.cyclo ./Core/Src/stepper/tmc/SERIAL_SWITCH.d ./Core/Src/stepper/tmc/SERIAL_SWITCH.o ./Core/Src/stepper/tmc/SERIAL_SWITCH.su ./Core/Src/stepper/tmc/SGCSCONF.cyclo ./Core/Src/stepper/tmc/SGCSCONF.d ./Core/Src/stepper/tmc/SGCSCONF.o ./Core/Src/stepper/tmc/SGCSCONF.su ./Core/Src/stepper/tmc/SHORT_CONF.cyclo ./Core/Src/stepper/tmc/SHORT_CONF.d ./Core/Src/stepper/tmc/SHORT_CONF.o ./Core/Src/stepper/tmc/SHORT_CONF.su ./Core/Src/stepper/tmc/SMARTEN.cyclo ./Core/Src/stepper/tmc/SMARTEN.d ./Core/Src/stepper/tmc/SMARTEN.o ./Core/Src/stepper/tmc/SMARTEN.su ./Core/Src/stepper/tmc/SW_MODE.cyclo ./Core/Src/stepper/tmc/SW_MODE.d ./Core/Src/stepper/tmc/SW_MODE.o ./Core/Src/stepper/tmc/SW_MODE.su ./Core/Src/stepper/tmc/TMC2130Stepper.cyclo ./Core/Src/stepper/tmc/TMC2130Stepper.d ./Core/Src/stepper/tmc/TMC2130Stepper.o ./Core/Src/stepper/tmc/TMC2130Stepper.su ./Core/Src/stepper/tmc/TMC2160Stepper.cyclo ./Core/Src/stepper/tmc/TMC2160Stepper.d ./Core/Src/stepper/tmc/TMC2160Stepper.o ./Core/Src/stepper/tmc/TMC2160Stepper.su ./Core/Src/stepper/tmc/TMC5130Stepper.cyclo ./Core/Src/stepper/tmc/TMC5130Stepper.d ./Core/Src/stepper/tmc/TMC5130Stepper.o ./Core/Src/stepper/tmc/TMC5130Stepper.su ./Core/Src/stepper/tmc/TMC5160Stepper.cyclo ./Core/Src/stepper/tmc/TMC5160Stepper.d ./Core/Src/stepper/tmc/TMC5160Stepper.o ./Core/Src/stepper/tmc/TMC5160Stepper.su ./Core/Src/stepper/tmc/TMCStepper.cyclo ./Core/Src/stepper/tmc/TMCStepper.d ./Core/Src/stepper/tmc/TMCStepper.o ./Core/Src/stepper/tmc/TMCStepper.su

.PHONY: clean-Core-2f-Src-2f-stepper-2f-tmc

