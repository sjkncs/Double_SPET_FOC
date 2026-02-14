################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/calib_platform_m1.c \
../Core/Src/calib_platform_m2.c \
../Core/Src/curr_loop_autocalib.c \
../Core/Src/flash_params.c \
../Core/Src/foc_adapt.c \
../Core/Src/kth71xx.c \
../Core/Src/main.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c \
../Core/Src/vofa_engine.c \
../Core/Src/zero_calib.c 

OBJS += \
./Core/Src/calib_platform_m1.o \
./Core/Src/calib_platform_m2.o \
./Core/Src/curr_loop_autocalib.o \
./Core/Src/flash_params.o \
./Core/Src/foc_adapt.o \
./Core/Src/kth71xx.o \
./Core/Src/main.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o \
./Core/Src/vofa_engine.o \
./Core/Src/zero_calib.o 

C_DEPS += \
./Core/Src/calib_platform_m1.d \
./Core/Src/calib_platform_m2.d \
./Core/Src/curr_loop_autocalib.d \
./Core/Src/flash_params.d \
./Core/Src/foc_adapt.d \
./Core/Src/kth71xx.d \
./Core/Src/main.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d \
./Core/Src/vofa_engine.d \
./Core/Src/zero_calib.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/calib_platform_m1.cyclo ./Core/Src/calib_platform_m1.d ./Core/Src/calib_platform_m1.o ./Core/Src/calib_platform_m1.su ./Core/Src/calib_platform_m2.cyclo ./Core/Src/calib_platform_m2.d ./Core/Src/calib_platform_m2.o ./Core/Src/calib_platform_m2.su ./Core/Src/curr_loop_autocalib.cyclo ./Core/Src/curr_loop_autocalib.d ./Core/Src/curr_loop_autocalib.o ./Core/Src/curr_loop_autocalib.su ./Core/Src/flash_params.cyclo ./Core/Src/flash_params.d ./Core/Src/flash_params.o ./Core/Src/flash_params.su ./Core/Src/foc_adapt.cyclo ./Core/Src/foc_adapt.d ./Core/Src/foc_adapt.o ./Core/Src/foc_adapt.su ./Core/Src/kth71xx.cyclo ./Core/Src/kth71xx.d ./Core/Src/kth71xx.o ./Core/Src/kth71xx.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32g4xx_hal_msp.cyclo ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.cyclo ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.cyclo ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su ./Core/Src/vofa_engine.cyclo ./Core/Src/vofa_engine.d ./Core/Src/vofa_engine.o ./Core/Src/vofa_engine.su ./Core/Src/zero_calib.cyclo ./Core/Src/zero_calib.d ./Core/Src/zero_calib.o ./Core/Src/zero_calib.su

.PHONY: clean-Core-2f-Src

