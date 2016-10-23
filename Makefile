TARGET = main
DST_DL=/media/`whoami`/GR-SAKURA
DST_RPI=/media/pi/GR-SAKURA

#### Setup ####
TOOLCHAIN	= rx-elf
LINKER_FILE	= ./gr-sakura.ld
#LINKER_FILE	= ./rx_gr-sakura.ld
BUILDDIR	= build

CC		= $(TOOLCHAIN)-gcc
CP		= $(TOOLCHAIN)-objcopy
SIZE	= $(TOOLCHAIN)-size

CFLAGS	+= -O0 -ffunction-sections -fdata-sections
LFLAGS	= -Wl,--gc-sections
CPFLAGS	= -Obinary
##AFLAGS :=-Wall -I"$(GNU_PATH)rx-elf/include" -I. -I"$(GNU_PATH)lib/gcc/rx-elf/$(GCC_VERSION)/include" -I"$(GNU_PATH)rx-elf/include/c++/$(GCC_VERSION)/" -I"$(GNU_PATH)rx-elf/include/c++/$(GCC_VERSION)/rx-elf/64-bit-double/" -ffunction-sections -fno-function-cse -fsigned-char -fdata-sections -DTESTING=1 -DGRSAKURA -DARDUINO=100 -DCPPAPP -D__RX_LITTLE_ENDIAN__=1 -D__T4__ -O2 -flto -mlittle-endian-data -mcpu=rx600 -m64bit-doubles

SRCC   = $(wildcard src/*.c)
OBJC   = $(SRCC:%.c=%.o)
SRCA   = $(wildcard src/*.asm)
OBJA   = $(SRCA:%.asm=%.o)
OBJ    = $(OBJC) $(OBJA)

.PHONY: clean install
#### Rules ####
all: $(TARGET).bin

%.o: %.asm
	$(CC) $(AFLAGS) $(CCINC) -c -x assembler-with-cpp $< -o $@

##%.o: %.asm
##	$(CC) -c $(CFLAGS) $< -o $@

%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

$(TARGET).elf: $(OBJ)
	$(CC) -T $(LINKER_FILE) $(LFLAGS) $(CFLAGS) -o $(TARGET).elf $(OBJ)

$(TARGET).bin: $(TARGET).elf
	$(CP) $(CPFLAGS) $(TARGET).elf $(TARGET).bin
	$(SIZE) $(TARGET).elf

install: $(TARGET).elf
	cp $(TARGET).bin $(DST_DL)
##pumount $(DST_DL)

rpi_install: $(TARGET).elf
	scp $(TARGET).bin pi@192.168.1.30:$(DST_RPI)
#	scp $(TARGET).bin pi@192.168.1.40:$(DST_RPI)
#	scp $(TARGET).bin pi@192.168.1.31:$(DST_RPI)
##pumount $(DST_DL)

clean:
	rm -f *.elf *.bin src/*.o
