SHELL := cmd.exe
QUOTE := "

TOOLCHAIN := C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\arm\arm-gnu-toolchain\bin
PACKS := C:\Program Files (x86)\Atmel\Studio\7.0\Packs
CC := $(QUOTE)$(TOOLCHAIN)\arm-none-eabi-gcc.exe$(QUOTE)
CXX := $(QUOTE)$(TOOLCHAIN)\arm-none-eabi-g++.exe$(QUOTE)
OBJCOPY := $(QUOTE)$(TOOLCHAIN)\arm-none-eabi-objcopy.exe$(QUOTE)
SIZE := $(QUOTE)$(TOOLCHAIN)\arm-none-eabi-size.exe$(QUOTE)

ROOT := ../../..
LDSCRIPT := $(ROOT)/Depreciated/MotionStreamingExample/Device_Startup/flash_with_bootloader.ld

CXXFLAGS := -mthumb -D__SAME53N19A__ -DDEBUG \
	-I"$(PACKS)\arm\CMSIS\4.5.0\CMSIS\Include" \
	-I"$(PACKS)\atmel\SAME53_DFP\1.1.118\include" \
	-I"$(PACKS)\arm\CMSIS\5.4.0\CMSIS\Core\Include" \
	-I"$(ROOT)/libClearCore/inc" \
	-I"$(ROOT)/LwIP/LwIP/src/include" \
	-I"$(ROOT)/LwIP/LwIP/port/include" \
	-I.. -I../third_party/jsmn \
	-O3 -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions \
	-g2 -Wall -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16

CFLAGS := -x c -mthumb -D__SAME53N19A__ -DDEBUG -std=gnu99 \
	-I"$(PACKS)\atmel\SAME53_DFP\1.1.118\include" \
	-I"$(PACKS)\arm\CMSIS\4.5.0\CMSIS\Include" \
	-I"$(PACKS)\arm\CMSIS\5.4.0\CMSIS\Core\Include" \
	-O3 -ffunction-sections -fdata-sections -g2 -Wall \
	-mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16

LDFLAGS := -mthumb -Wl,-Map=ClearCoreROS.map --specs=rdimon.specs \
	-Wl,--start-group -lm -larm_cortexM4lf_math -lClearCore -lLwIP -Wl,--end-group \
	-L"$(PACKS)\arm\CMSIS\4.5.0\CMSIS\Lib\GCC" \
	-L"$(ROOT)/libClearCore/Debug" \
	-L"$(ROOT)/LwIP/Debug" \
	-Wl,--gc-sections -mcpu=cortex-m4 \
	-T$(LDSCRIPT) -mfloat-abi=hard -mfpu=fpv4-sp-d16

OBJS := Device_Startup/startup_same53.o ClearCoreROS.o MotionCore.o Session.o Transport.o

all: ClearCoreROS.elf

ClearCoreROS.elf: $(OBJS)
	$(CXX) -o $@ $(OBJS) $(LDFLAGS)
	$(OBJCOPY) -O binary $@ ClearCoreROS.bin
	$(SIZE) $@

Device_Startup/startup_same53.o: $(ROOT)/Depreciated/MotionStreamingExample/Device_Startup/startup_same53.c
	if not exist Device_Startup mkdir Device_Startup
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: ../%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	-del /Q *.o *.d *.elf *.bin *.hex *.map *.lss *.eep *.srec 2>nul
	-del /Q Device_Startup\*.o Device_Startup\*.d 2>nul

.PHONY: all clean
