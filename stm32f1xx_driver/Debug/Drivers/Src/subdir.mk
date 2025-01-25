################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/stm32f1xx_gpio.c \
../Drivers/Src/stm32f1xx_usart.c 

OBJS += \
./Drivers/Src/stm32f1xx_gpio.o \
./Drivers/Src/stm32f1xx_usart.o 

C_DEPS += \
./Drivers/Src/stm32f1xx_gpio.d \
./Drivers/Src/stm32f1xx_usart.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o Drivers/Src/%.su Drivers/Src/%.cyclo: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103RBTx -DSTM32 -DSTM32F1 -DNUCLEO_F103RB -c -I../Inc -I"D:/Workspace/stm32/my-hal-driver/stm32f1xx_driver/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/stm32f1xx_gpio.cyclo ./Drivers/Src/stm32f1xx_gpio.d ./Drivers/Src/stm32f1xx_gpio.o ./Drivers/Src/stm32f1xx_gpio.su ./Drivers/Src/stm32f1xx_usart.cyclo ./Drivers/Src/stm32f1xx_usart.d ./Drivers/Src/stm32f1xx_usart.o ./Drivers/Src/stm32f1xx_usart.su

.PHONY: clean-Drivers-2f-Src

