X7-hardware
===========

# Latest update 
In this new setup I have included all the necessary files to program the IOToggle program to teh stm32F4 Discovery board. 

## What are we using
This setup is based on the STM32F4xx_StdPeriph_Driver as well as the excellent Linux port of STLink by [Texane](https://github.com/texane/stlink) 

## How to get setup on Linux or Mac: (Windows instructions coming soon!)
1. Download GCC-ARM for your system from here https://launchpad.net/gcc-arm-embedded/+download
2. Setup the path on your computer to the location of the downloaded GCC-ARM
3. Clone this directory to your computer
4. Clone the STLink program from the link above into a directory of your choosing. 
5. Follow the instructions on the STLink github to compile the program 
6. Edit the Makefile variable "STLINK" to point to the STLink directory 
7. cd into IOToggle and run "make"
8. Plug in your discovery board via the mini-usb connector and run "make burn"
9. Viola! Your Discovery board will now be running the simple IOToggle program.

## Other examples
There are some GPIO and USART examples [here.](https://github.com/devthrash/STM32F4-workarea) 
Note the makefile for these will not work yet, I'm going to be modifying it. 


## Tasks for the hardware programming team 

- [] write your own programs for the discovery board 
- [] enable serial communication from the discovery board
- [] talk to ATTinys 


### Who has our STM32F4 Boards 
- Eric Colter  
- Martin Tuskevicius  
- Luke McBee  
- Jack Cottom  
- Erik with K?  


### Still useful but outdatd links

How to setup envirns

Windows with GCC and openOCD (not IAR) : http://www.angstromsandalgorithms.com/free-eclipse-arm-gcc-openocd-toolchain-for-windows-part-2-arm-gcc/

Linux: http://eliaselectronics.com/stm32f4-tutorials/setting-up-the-stm32f4-arm-development-toolchain/

and OSX : http://spin.atomicobject.com/2013/08/10/arm-cortex-m-toolchain/


Things to do

### Main board

serial (uart)  
motor control i2c  
led gpio  
7 seg display  
manipulator control (servo)  
lasers! (stepper motor)  
other distance measurement  
camera muxes  
alge collection (suction motor)  
pressure sensor  
accelerometer   
gyroscope  
valve turning thingy  
volt meter?  
flow sensor  
analog to digital converters  




### Motor driver board

generate pwm  
temperature sensor?  
current sensor?  
voltage sensor?  
i2c interface  
  


