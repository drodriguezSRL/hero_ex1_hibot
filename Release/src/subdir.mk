################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/adc.c \
../src/commandInterface.c \
../src/control.c \
../src/crc.c \
../src/encoder.c \
../src/led.c \
../src/main.c \
../src/motor_driver.c \
../src/stm32f4xx_it.c \
../src/system_stm32f4xx.c \
../src/test.c \
../src/time.c \
../src/usart.c 

S_UPPER_SRCS += \
../src/startup_stm32f4xx.S 

OBJS += \
./src/adc.o \
./src/commandInterface.o \
./src/control.o \
./src/crc.o \
./src/encoder.o \
./src/led.o \
./src/main.o \
./src/motor_driver.o \
./src/startup_stm32f4xx.o \
./src/stm32f4xx_it.o \
./src/system_stm32f4xx.o \
./src/test.o \
./src/time.o \
./src/usart.o 

S_UPPER_DEPS += \
./src/startup_stm32f4xx.d 

C_DEPS += \
./src/adc.d \
./src/commandInterface.d \
./src/control.d \
./src/crc.d \
./src/encoder.d \
./src/led.d \
./src/main.d \
./src/motor_driver.d \
./src/stm32f4xx_it.d \
./src/system_stm32f4xx.d \
./src/test.d \
./src/time.d \
./src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -DUSE_STDPERIPH_DRIVER -DSTM32F4XX -DUSE_STM32F4_DISCOVERY -I"C:\Users\rover\Documents\HERO\3_ex1_hibot_sandbox\Libraries\CMSIS\Include" -I"C:\Users\rover\Documents\HERO\3_ex1_hibot_sandbox\Libraries\Device\STM32F4xx\Include" -I"C:\Users\rover\Documents\HERO\3_ex1_hibot_sandbox\inc" -I"C:\Users\rover\Documents\HERO\3_ex1_hibot_sandbox\Libraries\STM32F4xx_StdPeriph_Driver\inc" -std=gnu11 -Wmissing-prototypes -Wstrict-prototypes -Wbad-function-cast -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.S
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM GNU Assembler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections  -g -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


