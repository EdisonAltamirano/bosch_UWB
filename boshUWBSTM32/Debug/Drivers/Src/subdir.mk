################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/gpio.c \
../Drivers/Src/main.c \
../Drivers/Src/spi.c \
../Drivers/Src/stm32h7xx_hal_msp.c \
../Drivers/Src/stm32h7xx_hal_timebase_tim.c \
../Drivers/Src/stm32h7xx_it.c \
../Drivers/Src/syscalls.c \
../Drivers/Src/sysmem.c \
../Drivers/Src/system_stm32h7xx.c \
../Drivers/Src/udp_server.c \
../Drivers/Src/uwb_udp_protocol.c 

OBJS += \
./Drivers/Src/gpio.o \
./Drivers/Src/main.o \
./Drivers/Src/spi.o \
./Drivers/Src/stm32h7xx_hal_msp.o \
./Drivers/Src/stm32h7xx_hal_timebase_tim.o \
./Drivers/Src/stm32h7xx_it.o \
./Drivers/Src/syscalls.o \
./Drivers/Src/sysmem.o \
./Drivers/Src/system_stm32h7xx.o \
./Drivers/Src/udp_server.o \
./Drivers/Src/uwb_udp_protocol.o 

C_DEPS += \
./Drivers/Src/gpio.d \
./Drivers/Src/main.d \
./Drivers/Src/spi.d \
./Drivers/Src/stm32h7xx_hal_msp.d \
./Drivers/Src/stm32h7xx_hal_timebase_tim.d \
./Drivers/Src/stm32h7xx_it.d \
./Drivers/Src/syscalls.d \
./Drivers/Src/sysmem.d \
./Drivers/Src/system_stm32h7xx.d \
./Drivers/Src/udp_server.d \
./Drivers/Src/uwb_udp_protocol.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o Drivers/Src/%.su Drivers/Src/%.cyclo: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32H7xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"/home/edison/boschUWBSTM32/LWIP" -I"/home/edison/boschUWBSTM32/LWIP/App" -I"/home/edison/boschUWBSTM32/LWIP/Target" -I"/home/edison/boschUWBSTM32/Middlewares/Third_Party/LwIP/src/include" -I"/home/edison/boschUWBSTM32/Middlewares/Third_Party/LwIP/system" -I"/home/edison/boschUWBSTM32/Drivers/BSP/Components/lan8742" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/gpio.cyclo ./Drivers/Src/gpio.d ./Drivers/Src/gpio.o ./Drivers/Src/gpio.su ./Drivers/Src/main.cyclo ./Drivers/Src/main.d ./Drivers/Src/main.o ./Drivers/Src/main.su ./Drivers/Src/spi.cyclo ./Drivers/Src/spi.d ./Drivers/Src/spi.o ./Drivers/Src/spi.su ./Drivers/Src/stm32h7xx_hal_msp.cyclo ./Drivers/Src/stm32h7xx_hal_msp.d ./Drivers/Src/stm32h7xx_hal_msp.o ./Drivers/Src/stm32h7xx_hal_msp.su ./Drivers/Src/stm32h7xx_hal_timebase_tim.cyclo ./Drivers/Src/stm32h7xx_hal_timebase_tim.d ./Drivers/Src/stm32h7xx_hal_timebase_tim.o ./Drivers/Src/stm32h7xx_hal_timebase_tim.su ./Drivers/Src/stm32h7xx_it.cyclo ./Drivers/Src/stm32h7xx_it.d ./Drivers/Src/stm32h7xx_it.o ./Drivers/Src/stm32h7xx_it.su ./Drivers/Src/syscalls.cyclo ./Drivers/Src/syscalls.d ./Drivers/Src/syscalls.o ./Drivers/Src/syscalls.su ./Drivers/Src/sysmem.cyclo ./Drivers/Src/sysmem.d ./Drivers/Src/sysmem.o ./Drivers/Src/sysmem.su ./Drivers/Src/system_stm32h7xx.cyclo ./Drivers/Src/system_stm32h7xx.d ./Drivers/Src/system_stm32h7xx.o ./Drivers/Src/system_stm32h7xx.su ./Drivers/Src/udp_server.cyclo ./Drivers/Src/udp_server.d ./Drivers/Src/udp_server.o ./Drivers/Src/udp_server.su ./Drivers/Src/uwb_udp_protocol.cyclo ./Drivers/Src/uwb_udp_protocol.d ./Drivers/Src/uwb_udp_protocol.o ./Drivers/Src/uwb_udp_protocol.su

.PHONY: clean-Drivers-2f-Src

