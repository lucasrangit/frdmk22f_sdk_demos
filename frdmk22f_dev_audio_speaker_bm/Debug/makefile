################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include utilities/subdir.mk
-include usb/device/source/khci/subdir.mk
-include usb/device/source/subdir.mk
-include usb/device/class/audio/subdir.mk
-include usb/device/class/subdir.mk
-include startup/subdir.mk
-include sources/subdir.mk
-include source/subdir.mk
-include osa/subdir.mk
-include drivers/subdir.mk
-include codec/subdir.mk
-include board/src/subdir.mk
-include board/subdir.mk
-include CMSIS/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: twrk22f120m_dev_audio_speaker_bm.axf

# Tool invocations
twrk22f120m_dev_audio_speaker_bm.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -Xlinker -Map="twrk22f120m_dev_audio_speaker_bm.map" -Xlinker --gc-sections -Xlinker -print-memory-usage -Xlinker --sort-section=alignment -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -T twrk22f120m_dev_audio_speaker_bm_Debug.ld -o "twrk22f120m_dev_audio_speaker_bm.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) twrk22f120m_dev_audio_speaker_bm.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "twrk22f120m_dev_audio_speaker_bm.axf"; # arm-none-eabi-objcopy -v -O binary "twrk22f120m_dev_audio_speaker_bm.axf" "twrk22f120m_dev_audio_speaker_bm.bin" ; # checksum -p MK22FN512xxx12 -d "twrk22f120m_dev_audio_speaker_bm.bin";
	-@echo ' '

.PHONY: all clean dependents post-build

-include ../makefile.targets
