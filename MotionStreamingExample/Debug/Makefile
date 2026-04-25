################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=
LINKER_SCRIPT_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS :=  \
../Device_Startup/


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../Device_Startup/startup_same53.c \
../MotionStreaming.cpp


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
Device_Startup/startup_same53.o \
MotionStreaming.o

OBJS_AS_ARGS +=  \
Device_Startup/startup_same53.o \
MotionStreaming.o

C_DEPS +=  \
Device_Startup/startup_same53.d \
MotionStreaming.d

C_DEPS_AS_ARGS +=  \
Device_Startup/startup_same53.d \
MotionStreaming.d

OUTPUT_FILE_PATH +=MotionStreaming.elf

OUTPUT_FILE_PATH_AS_ARGS +=MotionStreaming.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+=  \
Z:/EnhancedClearCoreLibrary/libClearCore/Debug/libClearCore.a \
Z:/EnhancedClearCoreLibrary/LwIP/Debug/libLwIP.a

LINKER_SCRIPT_DEP+=  \
../Device_Startup/flash_without_bootloader.ld \
../Device_Startup/flash_with_bootloader.ld \
../Device_Startup/same53n19a_flash.ld \
../Device_Startup/same53n19a_sram.ld


# AVR32/GNU C Compiler
Device_Startup/startup_same53.o: ../Device_Startup/startup_same53.c
	@echo Building file: $<
	@echo Invoking: ARM/GNU C Compiler : 6.3.1
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-gcc.exe$(QUOTE)  -x c -mthumb -D__SAME53N19A__ -DDEBUG  -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\SAME53_DFP\1.1.118\include" -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\arm\CMSIS\4.5.0\CMSIS\Include" -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\arm\CMSIS\5.4.0\CMSIS\Core\Include"  -O3 -ffunction-sections -fdata-sections -g3 -Wall -mcpu=cortex-m4 -c -std=gnu99 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	

./MotionStreaming.o: .././MotionStreaming.cpp
	@echo Building file: $<
	@echo Invoking: ARM/GNU C Compiler : 6.3.1
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-g++.exe$(QUOTE) -mthumb -D__SAME53N19A__ -DDEBUG  -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\arm\CMSIS\4.5.0\CMSIS\Include" -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\SAME53_DFP\1.1.118\include" -I"../../libClearCore/inc" -I"../../LwIP/LwIP/src/include" -I"../../LwIP/LwIP/port/include" -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\arm\CMSIS\5.4.0\CMSIS\Core\Include"  -O3 -ffunction-sections -fno-rtti -fno-exceptions -g2 -Wall -mcpu=cortex-m4 -c -mfloat-abi=hard -mfpu=fpv4-sp-d16 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	




# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP) $(LINKER_SCRIPT_DEP)
	@echo Building target: $@
	@echo Invoking: ARM/GNU Linker : 6.3.1
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-g++.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -mthumb -Wl,-Map="MotionStreaming.map" --specs=rdimon.specs -Wl,--start-group -lm -larm_cortexM4lf_math -lClearCore -lLwIP  -Wl,--end-group -L"../../Device_Startup" -L"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\arm\CMSIS\4.5.0\CMSIS\Lib\GCC" -L"..\\Device_Startup" -L"Z:\EnhancedClearCoreLibrary\libClearCore\Debug" -L"Z:\EnhancedClearCoreLibrary\LwIP\Debug"  -Wl,--gc-sections -mcpu=cortex-m4 -Tflash_with_bootloader.ld -mfloat-abi=hard -mfpu=fpv4-sp-d16  
	@echo Finished building target: $@
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-objcopy.exe" -O binary "MotionStreaming.elf" "MotionStreaming.bin"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature  "MotionStreaming.elf" "MotionStreaming.hex"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-objcopy.exe" -j .eeprom --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0 --no-change-warnings -O binary "MotionStreaming.elf" "MotionStreaming.eep" || exit 0
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-objdump.exe" -h -S "MotionStreaming.elf" > "MotionStreaming.lss"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature  "MotionStreaming.elf" "MotionStreaming.srec"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin\arm-none-eabi-size.exe" "MotionStreaming.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "MotionStreaming.elf" "MotionStreaming.a" "MotionStreaming.hex" "MotionStreaming.bin" "MotionStreaming.lss" "MotionStreaming.eep" "MotionStreaming.map" "MotionStreaming.srec"
	