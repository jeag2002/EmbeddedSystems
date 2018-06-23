################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../example/src/cr_startup_lpc175x_6x.c \
../example/src/fifo.c \
../example/src/practica.c \
../example/src/sysinit.c \
../example/src/uart.c 

OBJS += \
./example/src/cr_startup_lpc175x_6x.o \
./example/src/fifo.o \
./example/src/practica.o \
./example/src/sysinit.o \
./example/src/uart.o 

C_DEPS += \
./example/src/cr_startup_lpc175x_6x.d \
./example/src/fifo.d \
./example/src/practica.d \
./example/src/sysinit.d \
./example/src/uart.d 


# Each subdirectory must supply rules for building sources it contributes
example/src/%.o: ../example/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -DDEBUG -D__CODE_RED -D__USE_LPCOPEN -DCORE_M3 -D__NEWLIB__ -I"C:\workspaces\workspresso-nxp\lpc_chip_175x_6x\inc" -I"C:\workspaces\workspresso-nxp\lpc_board_nxp_lpcxpresso_1769\inc" -I"C:\workspaces\workspresso-nxp\PracticaV4\example\inc" -I"C:\workspaces\workspresso-nxp\PracticaV4\freertos\inc" -Og -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m3 -mthumb -D__NEWLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


