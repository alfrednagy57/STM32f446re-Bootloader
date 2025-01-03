################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../CRYPTO/Src/Crypto_Program.c \
../CRYPTO/Src/Crypto_Prv.c 

OBJS += \
./CRYPTO/Src/Crypto_Program.o \
./CRYPTO/Src/Crypto_Prv.o 

C_DEPS += \
./CRYPTO/Src/Crypto_Program.d \
./CRYPTO/Src/Crypto_Prv.d 


# Each subdirectory must supply rules for building sources it contributes
CRYPTO/Src/%.o CRYPTO/Src/%.su CRYPTO/Src/%.cyclo: ../CRYPTO/Src/%.c CRYPTO/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I"D:/ARM_COURSE/Workspace/BootLoader/BL" -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/wolfSSL_wolfSSL_wolfSSL/wolfssl/ -I../wolfSSL -I"D:/ARM_COURSE/Workspace/BootLoader/BL" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-CRYPTO-2f-Src

clean-CRYPTO-2f-Src:
	-$(RM) ./CRYPTO/Src/Crypto_Program.cyclo ./CRYPTO/Src/Crypto_Program.d ./CRYPTO/Src/Crypto_Program.o ./CRYPTO/Src/Crypto_Program.su ./CRYPTO/Src/Crypto_Prv.cyclo ./CRYPTO/Src/Crypto_Prv.d ./CRYPTO/Src/Crypto_Prv.o ./CRYPTO/Src/Crypto_Prv.su

.PHONY: clean-CRYPTO-2f-Src

