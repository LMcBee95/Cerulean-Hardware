# Latest update 
In this new setup I have included all the necessary files to program the IOToggle program to the stm32F4 Discovery board. Also added links to other examples and updated instructions to get things working 

## What are we using
This setup is based on the STM32F4xx_StdPeriph_Driver as well as the excellent Linux port of STLink by [Texane](https://github.com/texane/stlink) 

## How to get setup on Linux or Mac: 
1. Download GCC-ARM for your system from here https://launchpad.net/gcc-arm-embedded/+download
2. Setup the path on your computer to the location of the downloaded GCC-ARM
3. Clone this directory to your computer
4. Clone the STLink program from the link above into a directory of your choosing. 
5. Follow the instructions on the STLink github to compile the program 
6. Add the STLink directory to your path
7. cd into IOToggle
8. run "make"
9. Plug in your discovery board via the mini-usb connector and run "make burn"
10. Viola! Your Discovery board will now be running the simple IOToggle program.

## How to get setup on Windows: 
1. Download GCC-ARM for your system from here [here](https://launchpad.net/gcc-arm-embedded/+download)
2. Setup the path on your computer to the location of the downloaded GCC-ARM
4. Download the Make Binaries for your computer from [here](http://gnuwin32.sourceforge.net/packages/make.htm)
5. Setup the path on your computer to the location of the download Make binaries. (Note you can also put these in the same directory as GCC-ARM)
6. Download the STLink program binaries from [here](http://www.emb4fun.de/archive/stlink/index.html)
7. Do the same as step 5 except for the STLink binaries.
8. Clone this directory to your computer
9. cd into IOToggle and run "make"
10. Plug in your discovery board via the mini-usb connector and run "make burn"
11. Viola! Your Discovery board will now be running the simple IOToggle program.


## Other examples
There are some GPIO and USART examples [here.](https://github.com/devthrash/STM32F4-workarea) 
Note the makefile for these will not work yet, I'm going to be modifying it. 

## Git How To
Please see the GitCommands.markdown file for basics on how to use git for this project.

### Directory Structure
- **Examples**: Contains examples from ST for the Discovery board.
- **IOToggle**: One specific example with a working makefile.
- **Libraries**: Contains the STM Standard Periphial libraries we are using.
- **MotorController**: Code for the Motor Controller boards, to be compiled with the Arduino IDE currently.
- **Demo**: The demo code initinally on the Discovery board. Makefile needs to be modified to work.
- **mainboard_old**: Old code from previous years being made to work with a makefile. Still lots of work to do on this.

### Tasks for the hardware programming team 

- write your own programs for the discovery board 
- enable serial communication from the discovery board
- talk to ATTinys 


### Who has our STM32F4 Boards 
- Eric Colter  
- Martin Tuskevicius  
- Luke McBee  
- Jack Cottom  
- Erik with K?  


#### Still useful but outdatd links

How to setup envirns

Windows with GCC and openOCD (not IAR) : http://www.angstromsandalgorithms.com/free-eclipse-arm-gcc-openocd-toolchain-for-windows-part-2-arm-gcc/

Linux: http://eliaselectronics.com/stm32f4-tutorials/setting-up-the-stm32f4-arm-development-toolchain/

and OSX : http://spin.atomicobject.com/2013/08/10/arm-cortex-m-toolchain/


Things to do

#### Main board

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




#### Motor driver board

generate pwm  
temperature sensor?  
current sensor?  
voltage sensor?  
i2c interface  
  


