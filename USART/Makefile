#########################################################################
PROJ_NAME=main

LIBPATH=../Libraries

SRC = main.c stm32f4_discovery.c system_stm32f4xx.c stm32f4xx_it.c

##Startup File
STARTUP = startup_stm32f4xx.s

#########################################################################

##GCC ARM FILES
CC=arm-none-eabi-gcc
LD=arm-none-eabi-ld 
#LD=arm-none-eabi-gcc
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump


##Dependancies and include paths

DEFS = -DUSE_STDPERIPH_DRIVER -DSTM32F4XX -DMANGUSTA_DISCOVERY -DUSE_USB_OTG_FS -DHSE_VALUE=8000000

MCU = cortex-m4
MCFLAGS = -mcpu=$(MCU) -mthumb -mlittle-endian -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb-interwork -std=c99
STM32_INCLUDES = -I$(LIBPATH)/CMSIS/ST/STM32F4xx/Include/ \
	-I$(LIBPATH)/CMSIS/Include/ \
	-I$(LIBPATH)/STM32F4xx_StdPeriph_Driver/inc/ \
	-I$(LIBPATH)/STM32_USB_Device_Library/Class/hid/inc \
	-I$(LIBPATH)/STM32_USB_Device_Library/Core/inc/ \
	-I$(LIBPATH)/STM32_USB_OTG_Driver/inc/

##Optimizations - keep this at -0s
OPTIMIZE       = -Os

##CFLAGS
CFLAGS	= $(MCFLAGS)  $(OPTIMIZE)  $(DEFS) -I./ -I./ $(STM32_INCLUDES)  -Wl,-T,stm32_flash.ld
AFLAGS	= $(MCFLAGS) 
#-mapcs-float use float regs. small increase in code size


##USB OTG Libraries
STM32_USB_OTG_SRC = $(LIBPATH)/STM32_USB_OTG_Driver/src/usb_dcd_int.c \
	$(LIBPATH)/STM32_USB_OTG_Driver/src/usb_core.c \
	$(LIBPATH)/STM32_USB_OTG_Driver/src/usb_dcd.c \

STM32_USB_DEVICE_SRC = $(LIBPATH)/STM32_USB_Device_Library/Class/hid/src/usbd_hid_core.c \
	$(LIBPATH)/STM32_USB_Device_Library/Core/src/usbd_req.c \
	$(LIBPATH)/STM32_USB_Device_Library/Core/src/usbd_core.c \
	$(LIBPATH)/STM32_USB_Device_Library/Core/src/usbd_ioreq.c


##Standard Periph Libraries
SRC += \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/misc.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_adc.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_can.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_crc.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_aes.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_des.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_tdes.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dac.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dbgmcu.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dcmi.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_exti.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_flash.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_fsmc.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash_md5.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash_sha1.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_iwdg.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_pwr.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rng.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rtc.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_sdio.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_syscfg.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c \
  $(LIBPATH)/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_wwdg.c	


OBJDIR = .
OBJ = $(SRC:%.c=$(OBJDIR)/%.o) 
OBJ += Startup.o

####################################################################

.PHONY: proj

all: proj

burn:
	st-flash write $(PROJ_NAME).bin 0x8000000

proj: $(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRC) $(STARTUP)
	$(CC) $(CFLAGS) $^ -lm -lc -lnosys -o $@
	$(CP) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

clean:
	rm -f Startup.lst  $(PROJ_NAME)  $(PROJ_NAME).lst $(OBJ) $(AUTOGEN)  $(PROJ_NAME).out $(PROJ_NAME).map \
	 $(PROJ_NAME).dmp  $(PROJ_NAME).elf $(PROJ_NAME).bin
