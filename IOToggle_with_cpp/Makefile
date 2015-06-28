#########################################################################
PROJ_NAME=main

BUILDDIR=build

LIBPATH=../Libraries

SRC =  stm32f4_discovery.c system_stm32f4xx.c
#main.c

##Startup File
STARTUP = lib/startup_stm32f4xx.s

#########################################################################

##GCC ARM FILES
CC=arm-none-eabi-gcc
CXX=arm-none-eabi-g++
LD=arm-none-eabi-ld 
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump

vpath %.c src
vpath %.c lib

##Dependancies and include paths

DEFS = -DUSE_STDPERIPH_DRIVER -DSTM32F40_41xxx -DUSE_STM324xG_EVAL -DMANGUSTA_DISCOVERY -DUSE_USB_OTG_FS -DHSE_VALUE=8000000

MCU = cortex-m4
MCFLAGS = -mcpu=$(MCU) -mthumb -mlittle-endian -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb-interwork 
STM32_INCLUDES = -I$(LIBPATH)/CMSIS/Device/ST/STM32F4xx/Include/ \
	-I$(LIBPATH)/CMSIS/Include/ \
	-I$(LIBPATH)/STM32F4xx_StdPeriph_Driver/inc/ \
	-I$(LIBPATH)/STM32_USB_Device_Library/Class/hid/inc \
	-I$(LIBPATH)/STM32_USB_Device_Library/Core/inc/ \
	-I$(LIBPATH)/STM32_USB_OTG_Driver/inc/ \
	-I$(LIBPATH)/STM32_EVAL/STM3240_41_G_EVAL/ \
	-I$(LIBPATH)/STM32_EVAL/Common/ \
	-Iinc/

##Optimizations - keep this at -0s
OPTIMIZE       = -Os

##CFLAGS
CFLAGS	=  -c $(OPTIMIZE) $(MCFLAGS) $(DEFS) -I./ -I./ $(STM32_INCLUDES)   -std=c99
CXXFLAGS = -c $(OPTIMIZE) $(MCFLAGS) $(DEFS) -I./ -I./ $(STM32_INCLUDES)   --specs=nosys.specs -std=c++11
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

OBJ =  startup_stm32f4xx.o   stm32f4xx_fsmc.o  stm32f4xx_usart.o stm32f4_discovery.o   stm32f4xx_gpio.o       stm32f4xx_wwdg.o stm32f4xx_adc.o stm32f4xx_hash_md5.o   system_stm32f4xx.o stm32f4xx_can.o       stm32f4xx_hash.o stm32f4xx_crc.o       stm32f4xx_hash_sha1.o stm32f4xx_cryp_aes.o  stm32f4xx_i2c.o stm32f4xx_cryp_des.o  stm32f4xx_iwdg.o
#Startup.o

OBJ += stm32f4xx_cryp.o stm32f4xx_pwr.o stm32f4xx_cryp_tdes.o  stm32f4xx_rcc.o stm32f4xx_dac.o stm32f4xx_rng.o stm32f4xx_dbgmcu.o     stm32f4xx_rtc.o stm32f4xx_dcmi.o stm32f4xx_sdio.o stm32f4xx_dma.o stm32f4xx_spi.o misc.o stm32f4xx_exti.o stm32f4xx_syscfg.o stm32f4xx_flash.o stm32f4xx_tim.o



####################################################################

.PHONY: proj

all: proj

burn:
	st-flash write $(BUILDDIR)/$(PROJ_NAME).bin 0x8000000

proj: $(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRC) $(STARTUP)
	
	$(CC) $(CFLAGS) $^ -lm -lc -lnosys $(PROJ_NAME).o
	#$(CC) $(CFLAGS) $^ -lm -lc -lnosys -o $(BUILDDIR)/$@
	
	$(CXX)  $(CXXFLAGS) src/main.cpp test.cpp test.o
	
	$(CXX) test.o $(PROJ_NAME).o $(OBJ) $(MCFLAGS) -Wl,-T,lib/stm32_flash.ld --specs=nosys.specs  -o $(BUILDDIR)/$@
	$(CP) -O binary $(BUILDDIR)/$(PROJ_NAME).elf $(BUILDDIR)/$(PROJ_NAME).bin
	

clean:
	#rm -f Startup.lst  $(BUILDDIR)/$(PROJ_NAME)  $(BUILDDIR)/$(PROJ_NAME).lst $(BUILDDIR)/$(OBJ) $(AUTOGEN)  $(BUILDDIR)/$(PROJ_NAME).out $(BUILDDIR)/$(PROJ_NAME).map \
	 #$(BUILDDIR)/$(PROJ_NAME).dmp  $(BUILDDIR)/$(PROJ_NAME).elf $(BUILDDIR)/$(PROJ_NAME).bin
