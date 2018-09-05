################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../codec/fsl_codec_common.c \
../codec/fsl_sgtl5000.c 

OBJS += \
./codec/fsl_codec_common.o \
./codec/fsl_sgtl5000.o 

C_DEPS += \
./codec/fsl_codec_common.d \
./codec/fsl_sgtl5000.d 


# Each subdirectory must supply rules for building sources it contributes
codec/%.o: ../codec/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -D__REDLIB__ -DCPU_MK22FN512VDC12_cm4 -DCPU_MK22FN512VDC12 -DSDK_DEBUGCONSOLE=0 -DCR_INTEGER_PRINTF -D_DEBUG=1 -DUSB_STACK_BM -DUSING_SAI -DSDK_I2C_BASED_COMPONENT_USED=1 -DBOARD_USE_CODEC=1 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -I../board -I../source -I../ -I../usb/device/source/khci -I../usb/include -I../osa -I../drivers -I../CMSIS -I../utilities -I../startup -I../board/src -I../codec -I../sources -I../usb/device/class/audio -I../usb/device/class -I../usb/device/source -I../usb/device/include -O0 -fno-common -g -Wall -c  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


