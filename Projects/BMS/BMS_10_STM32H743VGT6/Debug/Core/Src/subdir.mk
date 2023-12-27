################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/BMS_Communication_Protocol.c \
../Core/Src/BMS_Power_Control.c \
../Core/Src/Event_Log.c \
../Core/Src/INA229.c \
../Core/Src/LTC6813.c \
../Core/Src/LTC6813_BMS.c \
../Core/Src/LTC681x.c \
../Core/Src/LT_SPI.c \
../Core/Src/STM32_EEPROM_SPI.c \
../Core/Src/bms_hardware.c \
../Core/Src/main.c \
../Core/Src/stm32_tm1637.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c \
../Core/Src/w25qxx.c 

OBJS += \
./Core/Src/BMS_Communication_Protocol.o \
./Core/Src/BMS_Power_Control.o \
./Core/Src/Event_Log.o \
./Core/Src/INA229.o \
./Core/Src/LTC6813.o \
./Core/Src/LTC6813_BMS.o \
./Core/Src/LTC681x.o \
./Core/Src/LT_SPI.o \
./Core/Src/STM32_EEPROM_SPI.o \
./Core/Src/bms_hardware.o \
./Core/Src/main.o \
./Core/Src/stm32_tm1637.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o \
./Core/Src/w25qxx.o 

C_DEPS += \
./Core/Src/BMS_Communication_Protocol.d \
./Core/Src/BMS_Power_Control.d \
./Core/Src/Event_Log.d \
./Core/Src/INA229.d \
./Core/Src/LTC6813.d \
./Core/Src/LTC6813_BMS.d \
./Core/Src/LTC681x.d \
./Core/Src/LT_SPI.d \
./Core/Src/STM32_EEPROM_SPI.d \
./Core/Src/bms_hardware.d \
./Core/Src/main.d \
./Core/Src/stm32_tm1637.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d \
./Core/Src/w25qxx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/BMS_Communication_Protocol.cyclo ./Core/Src/BMS_Communication_Protocol.d ./Core/Src/BMS_Communication_Protocol.o ./Core/Src/BMS_Communication_Protocol.su ./Core/Src/BMS_Power_Control.cyclo ./Core/Src/BMS_Power_Control.d ./Core/Src/BMS_Power_Control.o ./Core/Src/BMS_Power_Control.su ./Core/Src/Event_Log.cyclo ./Core/Src/Event_Log.d ./Core/Src/Event_Log.o ./Core/Src/Event_Log.su ./Core/Src/INA229.cyclo ./Core/Src/INA229.d ./Core/Src/INA229.o ./Core/Src/INA229.su ./Core/Src/LTC6813.cyclo ./Core/Src/LTC6813.d ./Core/Src/LTC6813.o ./Core/Src/LTC6813.su ./Core/Src/LTC6813_BMS.cyclo ./Core/Src/LTC6813_BMS.d ./Core/Src/LTC6813_BMS.o ./Core/Src/LTC6813_BMS.su ./Core/Src/LTC681x.cyclo ./Core/Src/LTC681x.d ./Core/Src/LTC681x.o ./Core/Src/LTC681x.su ./Core/Src/LT_SPI.cyclo ./Core/Src/LT_SPI.d ./Core/Src/LT_SPI.o ./Core/Src/LT_SPI.su ./Core/Src/STM32_EEPROM_SPI.cyclo ./Core/Src/STM32_EEPROM_SPI.d ./Core/Src/STM32_EEPROM_SPI.o ./Core/Src/STM32_EEPROM_SPI.su ./Core/Src/bms_hardware.cyclo ./Core/Src/bms_hardware.d ./Core/Src/bms_hardware.o ./Core/Src/bms_hardware.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32_tm1637.cyclo ./Core/Src/stm32_tm1637.d ./Core/Src/stm32_tm1637.o ./Core/Src/stm32_tm1637.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su ./Core/Src/w25qxx.cyclo ./Core/Src/w25qxx.d ./Core/Src/w25qxx.o ./Core/Src/w25qxx.su

.PHONY: clean-Core-2f-Src

