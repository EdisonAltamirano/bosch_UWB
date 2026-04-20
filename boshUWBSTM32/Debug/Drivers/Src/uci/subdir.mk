################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/uci/uci_commands.c \
../Drivers/Src/uci/uci_core.c \
../Drivers/Src/uci/uci_radar.c \
../Drivers/Src/uci/uci_sr250.c \
../Drivers/Src/uci/uci_transport.c 

OBJS += \
./Drivers/Src/uci/uci_commands.o \
./Drivers/Src/uci/uci_core.o \
./Drivers/Src/uci/uci_radar.o \
./Drivers/Src/uci/uci_sr250.o \
./Drivers/Src/uci/uci_transport.o 

C_DEPS += \
./Drivers/Src/uci/uci_commands.d \
./Drivers/Src/uci/uci_core.d \
./Drivers/Src/uci/uci_radar.d \
./Drivers/Src/uci/uci_sr250.d \
./Drivers/Src/uci/uci_transport.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/uci/%.o Drivers/Src/uci/%.su Drivers/Src/uci/%.cyclo: ../Drivers/Src/uci/%.c Drivers/Src/uci/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32H7xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"/home/edison/boschUWBSTM32/LWIP" -I"/home/edison/boschUWBSTM32/LWIP/App" -I"/home/edison/boschUWBSTM32/LWIP/Target" -I"/home/edison/boschUWBSTM32/Middlewares/Third_Party/LwIP/src/include" -I"/home/edison/boschUWBSTM32/Middlewares/Third_Party/LwIP/system" -I"/home/edison/boschUWBSTM32/Drivers/BSP/Components/lan8742" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Src-2f-uci

clean-Drivers-2f-Src-2f-uci:
	-$(RM) ./Drivers/Src/uci/uci_commands.cyclo ./Drivers/Src/uci/uci_commands.d ./Drivers/Src/uci/uci_commands.o ./Drivers/Src/uci/uci_commands.su ./Drivers/Src/uci/uci_core.cyclo ./Drivers/Src/uci/uci_core.d ./Drivers/Src/uci/uci_core.o ./Drivers/Src/uci/uci_core.su ./Drivers/Src/uci/uci_radar.cyclo ./Drivers/Src/uci/uci_radar.d ./Drivers/Src/uci/uci_radar.o ./Drivers/Src/uci/uci_radar.su ./Drivers/Src/uci/uci_sr250.cyclo ./Drivers/Src/uci/uci_sr250.d ./Drivers/Src/uci/uci_sr250.o ./Drivers/Src/uci/uci_sr250.su ./Drivers/Src/uci/uci_transport.cyclo ./Drivers/Src/uci/uci_transport.d ./Drivers/Src/uci/uci_transport.o ./Drivers/Src/uci/uci_transport.su

.PHONY: clean-Drivers-2f-Src-2f-uci

