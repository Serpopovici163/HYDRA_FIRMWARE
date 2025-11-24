################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
D:/HYDRA/1_HYDRA_FIRMWARE/Common/Src/hydra_comm.c \
D:/HYDRA/1_HYDRA_FIRMWARE/Common/Src/hydra_health.c \
D:/HYDRA/1_HYDRA_FIRMWARE/Common/Src/hydra_identity.c \
D:/HYDRA/1_HYDRA_FIRMWARE/Common/Src/hydra_state_mgr.c \
D:/HYDRA/1_HYDRA_FIRMWARE/Common/Src/hydra_watchdog.c 

OBJS += \
./Common/Src/hydra_comm.o \
./Common/Src/hydra_health.o \
./Common/Src/hydra_identity.o \
./Common/Src/hydra_state_mgr.o \
./Common/Src/hydra_watchdog.o 

C_DEPS += \
./Common/Src/hydra_comm.d \
./Common/Src/hydra_health.d \
./Common/Src/hydra_identity.d \
./Common/Src/hydra_state_mgr.d \
./Common/Src/hydra_watchdog.d 


# Each subdirectory must supply rules for building sources it contributes
Common/Src/hydra_comm.o: D:/HYDRA/1_HYDRA_FIRMWARE/Common/Src/hydra_comm.c Common/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G473xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Common/Src/hydra_health.o: D:/HYDRA/1_HYDRA_FIRMWARE/Common/Src/hydra_health.c Common/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G473xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Common/Src/hydra_identity.o: D:/HYDRA/1_HYDRA_FIRMWARE/Common/Src/hydra_identity.c Common/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G473xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Common/Src/hydra_state_mgr.o: D:/HYDRA/1_HYDRA_FIRMWARE/Common/Src/hydra_state_mgr.c Common/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G473xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Common/Src/hydra_watchdog.o: D:/HYDRA/1_HYDRA_FIRMWARE/Common/Src/hydra_watchdog.c Common/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G473xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Common-2f-Src

clean-Common-2f-Src:
	-$(RM) ./Common/Src/hydra_comm.cyclo ./Common/Src/hydra_comm.d ./Common/Src/hydra_comm.o ./Common/Src/hydra_comm.su ./Common/Src/hydra_health.cyclo ./Common/Src/hydra_health.d ./Common/Src/hydra_health.o ./Common/Src/hydra_health.su ./Common/Src/hydra_identity.cyclo ./Common/Src/hydra_identity.d ./Common/Src/hydra_identity.o ./Common/Src/hydra_identity.su ./Common/Src/hydra_state_mgr.cyclo ./Common/Src/hydra_state_mgr.d ./Common/Src/hydra_state_mgr.o ./Common/Src/hydra_state_mgr.su ./Common/Src/hydra_watchdog.cyclo ./Common/Src/hydra_watchdog.d ./Common/Src/hydra_watchdog.o ./Common/Src/hydra_watchdog.su

.PHONY: clean-Common-2f-Src

