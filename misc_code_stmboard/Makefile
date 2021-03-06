# put your *.o targets here, make should handle the rest!

SRCS = main.c stm32f4xx_it.c system_stm32f4xx.c

STLINK = ../stlink

# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)

PROJ_NAME=main

# that's it, no need to change anything below this line!

###################################################
BUILDDIR = build

CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump

OBJCOPY=arm-none-eabi-objcopy

DEFS = -DUSE_STDPERIPT_DRIVER -DSTM32F4xx -DHSE_VALUE=8000000
STARTUP = ../

CFLAGS  = -g -O2 -Wall -Tstm32_flash.ld 
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

###################################################

vpath %.c src
vpath %.a lib

ROOT=$(shell pwd)

CFLAGS += -Iinc -Ilib -Ilib/inc 
CFLAGS += -Ilib/inc/core -Ilib/inc/peripherals 

SRCS += lib/startup_stm32f4xx.s # add startup file to build

OBJS = $(SRCS:.c=.o)

###################################################

.PHONY: lib proj

all: lib proj

lib:
	$(MAKE) -C lib

proj: 	$(PROJ_NAME).elf

burn:
	$(STLINK)/st-flash write $(BUILDDIR)/$(PROJ_NAME).bin 0x8000000


$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $(BUILDDIR)/$@ -Llib -lstm32f4
	$(OBJCOPY) -O ihex $(BUILDDIR)/$(PROJ_NAME).elf $(BUILDDIR)/$(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(BUILDDIR)/$(PROJ_NAME).elf $(BUILDDIR)/$(PROJ_NAME).bin

clean:
	$(MAKE) -C lib clean
	rm -f $(BUILDIR)/$(PROJ_NAME).elf
	rm -f $(BUILDDIR)/$(PROJ_NAME).hex
	rm -f $(BUILDDIR)/$(PROJ_NAME).bin
