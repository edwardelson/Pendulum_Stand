################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../system/src/FreeRTOS/cmsis_os.c \
../system/src/FreeRTOS/croutine.c \
../system/src/FreeRTOS/event_groups.c \
../system/src/FreeRTOS/heap_2.c \
../system/src/FreeRTOS/list.c \
../system/src/FreeRTOS/port.c \
../system/src/FreeRTOS/queue.c \
../system/src/FreeRTOS/tasks.c \
../system/src/FreeRTOS/timers.c 

OBJS += \
./system/src/FreeRTOS/cmsis_os.o \
./system/src/FreeRTOS/croutine.o \
./system/src/FreeRTOS/event_groups.o \
./system/src/FreeRTOS/heap_2.o \
./system/src/FreeRTOS/list.o \
./system/src/FreeRTOS/port.o \
./system/src/FreeRTOS/queue.o \
./system/src/FreeRTOS/tasks.o \
./system/src/FreeRTOS/timers.o 

C_DEPS += \
./system/src/FreeRTOS/cmsis_os.d \
./system/src/FreeRTOS/croutine.d \
./system/src/FreeRTOS/event_groups.d \
./system/src/FreeRTOS/heap_2.d \
./system/src/FreeRTOS/list.d \
./system/src/FreeRTOS/port.d \
./system/src/FreeRTOS/queue.d \
./system/src/FreeRTOS/tasks.d \
./system/src/FreeRTOS/timers.d 


# Each subdirectory must supply rules for building sources it contributes
system/src/FreeRTOS/%.o: ../system/src/FreeRTOS/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DSTM32F411xE -DTRACE -I"../include" -I"../system/include/FreeRTOS" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f4xx" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


