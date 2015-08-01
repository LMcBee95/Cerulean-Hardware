#########################################################################

#Name of the .cpp file that contains the the main function
PROJ_NAME=main

#Names of folders located in the directory containing the makefile
BUILDDIR=build
OBJDIR = object
CXXDIR = CPP_src
CDIR = C_src

LIBPATH=../Libraries

#C_FILES is the variable that stores the path to each .c file in the C_src folder 
C_FILES = $(wildcard $(CDIR)/*.c)

#CXX_FILES is the variable that stores the path to each .cpp file in the CPP_src folder
CXX_FILES = $(wildcard $(CXXDIR)/*.cpp)

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

vpath %.c C_src
vpath %.c lib
vpath %.cpp CPP_src 
vpath %.o object

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
@CFLAGS	=  -c $(OPTIMIZE) $(MCFLAGS) $(DEFS) -I./ -I./ $(STM32_INCLUDES)   -std=c99
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
#C_LIB_SRC = stm32f4_discovery.c system_stm32f4xx.c
#C_LIB_SRC += \
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

	
#C_LIB_OBJ is the variable that stores the path to every precompiled .c library needed for the stm32f40_41xxx MCU
C_LIB_OBJ =  object/startup_stm32f4xx.o  object/stm32f4xx_fsmc.o  object/stm32f4xx_usart.o object/stm32f4_discovery.o  object/stm32f4xx_gpio.o       object/stm32f4xx_wwdg.o object/stm32f4xx_adc.o object/stm32f4xx_hash_md5.o   object/system_stm32f4xx.o object/stm32f4xx_can.o       object/stm32f4xx_hash.o object/stm32f4xx_crc.o       object/stm32f4xx_hash_sha1.o object/stm32f4xx_cryp_aes.o  object/stm32f4xx_i2c.o object/stm32f4xx_cryp_des.o  object/stm32f4xx_iwdg.o
C_LIB_OBJ += object/stm32f4xx_cryp.o object/stm32f4xx_pwr.o object/stm32f4xx_cryp_tdes.o  object/stm32f4xx_rcc.o object/stm32f4xx_dac.o object/stm32f4xx_rng.o object/stm32f4xx_dbgmcu.o    object/stm32f4xx_rtc.o object/stm32f4xx_dcmi.o object/stm32f4xx_sdio.o object/stm32f4xx_dma.o object/stm32f4xx_spi.o object/misc.o object/stm32f4xx_exti.o object/stm32f4xx_syscfg.o object/stm32f4xx_flash.o object/stm32f4xx_tim.o

#CXX_OBJ is the variable name for all of the .cpp object files located in the main dirrectery during the compiling process
CXX_OBJ:=$(CXX_FILES:$(CXXDIR)/%.cpp=%.o)

#C_OBJ is the variable name for all of the .c object files located in the main dirrectery during the compiling process
C_OBJ:=$(C_FILES:$(C_DIR)/%.c=%.o)

####################################################################

.PHONY: proj

all: proj

#Uploads a .bin file to a stm32f4 discovery board
burn:
	st-flash write $(BUILDDIR)/$(PROJ_NAME).bin 0x8000000

dfu:
	dfu-util -a 0 --dfuse-address 0x08000000 -D $(BUILDDIR)/$(PROJ_NAME).bin

proj: $(PROJ_NAME).elf

$(PROJ_NAME).elf: $(CXX_FILES)
	
	@#$(CC) $(CFLAGS) $(C_LIB_SRC) -lm -lc -lnosysmake 
	
	@#This compiles C_SRC files if you ever need to
	@#Uncomment this if you want to compile your own .c files in the C_src folder
	@#$(CC) $(CFLAGS) $(C_FILES) -lm -lc -lnosysmake 
	
	@#Creates the object files from the .cpp files
	$(CXX)  $(CXXFLAGS) $(CXX_FILES)
	#CPP code compiled
	
	@#Links all of the object files together using the g++ compiler
	@$(CXX) $(CXX_OBJ) $(C_OBJ) $(C_LIB_OBJ) $(MCFLAGS) -Wl,-T,lib/stm32_flash.ld --specs=nosys.specs  -o $(BUILDDIR)/$@
	#All files linked
	@$(CP) -O binary $(BUILDDIR)/$(PROJ_NAME).elf $(BUILDDIR)/$(PROJ_NAME).bin
	
	@rm -f $(CXX_OBJ) $(C_OBJ)
	
	#Jolly Good. Everything is in tip top shape.

clean:
	rm -f Startup.lst  $(BUILDDIR)/$(PROJ_NAME)  $(BUILDDIR)/$(PROJ_NAME).lst $(AUTOGEN)  $(BUILDDIR)/$(PROJ_NAME).out $(BUILDDIR)/$(PROJ_NAME).map \
	$(BUILDDIR)/$(PROJ_NAME).dmp  $(BUILDDIR)/$(PROJ_NAME).elf $(BUILDDIR)/$(PROJ_NAME).bin
