To include the stepper driver library into your project

1. Copy stepper.c from the lib folder to your project's lib folder

2. Copy stepper.h from the inc folder to your project's inc folder

3. Add the following line in your main.c
	#incude "stepper.h"

4. Add the following text to the end of line 8 of the Makefile
	stepper.c
(You can look at the makefile for this project as an example)

5. Have a nice day