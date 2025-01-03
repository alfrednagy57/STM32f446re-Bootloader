################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BL/Src/BL.c 

OBJS += \
./BL/Src/BL.o 

C_DEPS += \
./BL/Src/BL.d 


# Each subdirectory must supply rules for building sources it contributes
BL/Src/%.o BL/Src/%.su BL/Src/%.cyclo: ../BL/Src/%.c BL/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I"D:/ARM_COURSE/Workspace/BootLoader/CODE/BL" -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/wolfSSL_wolfSSL_wolfSSL/wolfssl/ -I"D:/ARM_COURSE/Workspace/BootLoader/CODE/BL" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-BL-2f-Src

clean-BL-2f-Src:
	-$(RM) ./BL/Src/BL.cyclo ./BL/Src/BL.d ./BL/Src/BL.o ./BL/Src/BL.su

.PHONY: clean-BL-2f-Src

