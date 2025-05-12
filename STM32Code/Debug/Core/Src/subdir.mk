################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/AS5600.c \
../Core/Src/main.c \
../Core/Src/prop.c \
../Core/Src/propeller_control.c \
../Core/Src/ring_buffer.c \
../Core/Src/rudder_control.c \
../Core/Src/servo_controls.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c \
../Core/Src/wit_c_sdk.c 

OBJS += \
./Core/Src/AS5600.o \
./Core/Src/main.o \
./Core/Src/prop.o \
./Core/Src/propeller_control.o \
./Core/Src/ring_buffer.o \
./Core/Src/rudder_control.o \
./Core/Src/servo_controls.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o \
./Core/Src/wit_c_sdk.o 

C_DEPS += \
./Core/Src/AS5600.d \
./Core/Src/main.d \
./Core/Src/prop.d \
./Core/Src/propeller_control.d \
./Core/Src/ring_buffer.d \
./Core/Src/rudder_control.d \
./Core/Src/servo_controls.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d \
./Core/Src/wit_c_sdk.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H723xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/AS5600.cyclo ./Core/Src/AS5600.d ./Core/Src/AS5600.o ./Core/Src/AS5600.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/prop.cyclo ./Core/Src/prop.d ./Core/Src/prop.o ./Core/Src/prop.su ./Core/Src/propeller_control.cyclo ./Core/Src/propeller_control.d ./Core/Src/propeller_control.o ./Core/Src/propeller_control.su ./Core/Src/ring_buffer.cyclo ./Core/Src/ring_buffer.d ./Core/Src/ring_buffer.o ./Core/Src/ring_buffer.su ./Core/Src/rudder_control.cyclo ./Core/Src/rudder_control.d ./Core/Src/rudder_control.o ./Core/Src/rudder_control.su ./Core/Src/servo_controls.cyclo ./Core/Src/servo_controls.d ./Core/Src/servo_controls.o ./Core/Src/servo_controls.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su ./Core/Src/wit_c_sdk.cyclo ./Core/Src/wit_c_sdk.d ./Core/Src/wit_c_sdk.o ./Core/Src/wit_c_sdk.su

.PHONY: clean-Core-2f-Src

