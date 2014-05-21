# Authored by Farrell Farahbod, last revised on 2014-05-20
# This file is released into the public domain

EXECUTABLE=firmware.elf
BIN_IMAGE=firmware.bin

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

CFLAGS  = -g -O2
CFLAGS += -mlittle-endian -mcpu=cortex-m0  -mthumb --specs=nano.specs
CFLAGS += -Wl,--gc-sections

# to run from FLASH
CFLAGS+=-Wl,-T,stm/stm32f0.ld

# libs
CFLAGS+=-I../../stm32f0-discovery-basic-template/Libraries/STM32F0xx_StdPeriph_Driver/inc
CFLAGS+=-I../../stm32f0-discovery-basic-template/Libraries/CMSIS/Device/ST/STM32F0xx/Include
CFLAGS+=-I../../stm32f0-discovery-basic-template/Libraries/CMSIS/Include
CFLAGS+=-I../../gcc-arm-none-eabi-4_8-2013q4/arm-none-eabi/include

# linker stuff
# reminder: these MUST be listed AFTER the input files (*.c) in the command line
LDFLAGS  = -L../../stm32f0-discovery-basic-template/Libraries -lstm32f0
LDFLAGS += -L../../gcc-arm-none-eabi-4_8-2013q4/arm-none-eabi/lib/armv6-m -lc -lm

all: $(BIN_IMAGE)

$(BIN_IMAGE): $(EXECUTABLE)
	$(OBJCOPY) -O binary $^ $@

$(EXECUTABLE): *.c f0lib/*.c stm/system_stm32f0xx.c stm/startup_stm32f0xx.s
	$(CC) $(CFLAGS) $^ $(LDFLAGS) -o $@
	
install:
#	st-flash write $(BIN_IMAGE) 0x08000000
	openocd -f /usr/local/share/openocd/scripts/board/stm32f0discovery.cfg -f stm32f0-openocd.cfg -c "stm_flash $(BIN_IMAGE)" -c shutdown

clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(BIN_IMAGE)

.PHONY: all clean debug
