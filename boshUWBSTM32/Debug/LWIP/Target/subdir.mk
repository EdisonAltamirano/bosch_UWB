################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../LWIP/Target/ethernetif.c 

OBJS += \
./LWIP/Target/ethernetif.o 

C_DEPS += \
./LWIP/Target/ethernetif.d 


# Each subdirectory must supply rules for building sources it contributes
LWIP/Target/%.o LWIP/Target/%.su LWIP/Target/%.cyclo: ../LWIP/Target/%.c LWIP/Target/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32H7xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Usuario/Documents/Stanford/Arbabian/quarter3/bosch/new/bosch_UWB/boshUWBSTM32/LWIP" -I"C:/Users/Usuario/Documents/Stanford/Arbabian/quarter3/bosch/new/bosch_UWB/boshUWBSTM32/LWIP/App" -I"C:/Users/Usuario/Documents/Stanford/Arbabian/quarter3/bosch/new/bosch_UWB/boshUWBSTM32/LWIP/Target" -I"C:/Users/Usuario/Documents/Stanford/Arbabian/quarter3/bosch/new/bosch_UWB/boshUWBSTM32/Middlewares/Third_Party/LwIP/src/include" -I"C:/Users/Usuario/Documents/Stanford/Arbabian/quarter3/bosch/new/bosch_UWB/boshUWBSTM32/Middlewares/Third_Party/LwIP/system" -I"C:/Users/Usuario/Documents/Stanford/Arbabian/quarter3/bosch/new/bosch_UWB/boshUWBSTM32/Drivers/BSP/Components/lan8742" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-LWIP-2f-Target

clean-LWIP-2f-Target:
	-$(RM) ./LWIP/Target/ethernetif.cyclo ./LWIP/Target/ethernetif.d ./LWIP/Target/ethernetif.o ./LWIP/Target/ethernetif.su

.PHONY: clean-LWIP-2f-Target

