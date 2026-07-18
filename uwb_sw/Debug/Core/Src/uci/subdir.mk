################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/uci/uci_commands.c \
../Core/Src/uci/uci_core.c \
../Core/Src/uci/uci_radar.c \
../Core/Src/uci/uci_sr250.c \
../Core/Src/uci/uci_transport.c 

OBJS += \
./Core/Src/uci/uci_commands.o \
./Core/Src/uci/uci_core.o \
./Core/Src/uci/uci_radar.o \
./Core/Src/uci/uci_sr250.o \
./Core/Src/uci/uci_transport.o 

C_DEPS += \
./Core/Src/uci/uci_commands.d \
./Core/Src/uci/uci_core.d \
./Core/Src/uci/uci_radar.d \
./Core/Src/uci/uci_sr250.d \
./Core/Src/uci/uci_transport.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/uci/%.o Core/Src/uci/%.su Core/Src/uci/%.cyclo: ../Core/Src/uci/%.c Core/Src/uci/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Core/Inc/uci -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32H7xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Usuario/Documents/summer/bosch_UWB/uwb_sw/LWIP" -I"C:/Users/Usuario/Documents/summer/bosch_UWB/uwb_sw/LWIP/App" -I"C:/Users/Usuario/Documents/summer/bosch_UWB/uwb_sw/LWIP/Target" -I"C:/Users/Usuario/Documents/summer/bosch_UWB/uwb_sw/Middlewares/Third_Party/LwIP/src/include" -I"C:/Users/Usuario/Documents/summer/bosch_UWB/uwb_sw/Middlewares/Third_Party/LwIP/system" -I"C:/Users/Usuario/Documents/summer/bosch_UWB/uwb_sw/Drivers/BSP/Components/lan8742" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-uci

clean-Core-2f-Src-2f-uci:
	-$(RM) ./Core/Src/uci/uci_commands.cyclo ./Core/Src/uci/uci_commands.d ./Core/Src/uci/uci_commands.o ./Core/Src/uci/uci_commands.su ./Core/Src/uci/uci_core.cyclo ./Core/Src/uci/uci_core.d ./Core/Src/uci/uci_core.o ./Core/Src/uci/uci_core.su ./Core/Src/uci/uci_radar.cyclo ./Core/Src/uci/uci_radar.d ./Core/Src/uci/uci_radar.o ./Core/Src/uci/uci_radar.su ./Core/Src/uci/uci_sr250.cyclo ./Core/Src/uci/uci_sr250.d ./Core/Src/uci/uci_sr250.o ./Core/Src/uci/uci_sr250.su ./Core/Src/uci/uci_transport.cyclo ./Core/Src/uci/uci_transport.d ./Core/Src/uci/uci_transport.o ./Core/Src/uci/uci_transport.su

.PHONY: clean-Core-2f-Src-2f-uci

