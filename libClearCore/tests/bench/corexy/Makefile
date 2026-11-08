# CoreXY bench test firmware (ClearCore). Builds against ../../../Release/libClearCore.a
# and ../../../../LwIP/Release/libLwIP.a, with the Atmel Studio 7 toolchain (same flags as an
# Atmel Studio app project).
#
#   make            corexy_bench.bin and corexy_bench.uf2
#   make clean

TC     = C:/Program Files (x86)/Atmel/Studio/7.0/toolchain/arm/arm-gnu-toolchain/bin
PACKS  = C:/Program Files (x86)/Atmel/Studio/7.0/Packs
CXX    = "$(TC)/arm-none-eabi-g++.exe"
CC     = "$(TC)/arm-none-eabi-gcc.exe"
OBJCOPY = "$(TC)/arm-none-eabi-objcopy.exe"
SIZE   = "$(TC)/arm-none-eabi-size.exe"
UF2    = ../../../../Tools/uf2-builder/Release/uf2-builder.exe

ROOT   = ../../../..
START  = $(ROOT)/ProjectTemplate/Device_Startup
ARCH   = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
DEFS   = -D__SAME53N19A__ -DNDEBUG
INC    = -I"$(PACKS)/arm/CMSIS/4.5.0/CMSIS/Include" -I"$(PACKS)/atmel/SAME53_DFP/1.1.118/include" \
         -I"$(PACKS)/arm/CMSIS/5.4.0/CMSIS/Core/Include" -I$(ROOT)/libClearCore/inc \
         -I$(ROOT)/LwIP/LwIP/src/include -I$(ROOT)/LwIP/LwIP/port/include
CXXFLAGS = $(ARCH) $(DEFS) $(INC) -O3 -ffunction-sections -fno-rtti -fno-exceptions -Wall -Wextra
CFLAGS   = $(ARCH) $(DEFS) $(INC) -O3 -ffunction-sections -fdata-sections -Wall
LDFLAGS  = $(ARCH) -Wl,-Map=corexy_bench.map --specs=rdimon.specs -Wl,--gc-sections \
           -T$(START)/flash_with_bootloader.ld -L$(START) \
           -L$(ROOT)/libClearCore/Release -L$(ROOT)/LwIP/Release \
           -L"$(PACKS)/arm/CMSIS/4.5.0/CMSIS/Lib/GCC"
LIBS     = -Wl,--start-group -lm -larm_cortexM4lf_math -lClearCore -lLwIP -Wl,--end-group

all: corexy_bench.uf2

main.o: main.cpp $(ROOT)/libClearCore/Release/libClearCore.a
	$(CXX) $(CXXFLAGS) -c -o $@ $<

startup_same53.o: $(START)/startup_same53.c
	$(CC) $(CFLAGS) -c -o $@ $<

corexy_bench.elf: main.o startup_same53.o $(ROOT)/libClearCore/Release/libClearCore.a
	$(CXX) -o $@ main.o startup_same53.o $(LDFLAGS) $(LIBS)
	$(SIZE) $@

corexy_bench.bin: corexy_bench.elf
	$(OBJCOPY) -O binary $< $@

corexy_bench.uf2: corexy_bench.bin
	$(UF2) $< $@

clean:
	rm -f *.o *.elf *.bin *.uf2 *.map

.PHONY: all clean
