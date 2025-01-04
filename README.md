# Dji_motor
Dji 3508 and 2006 control module code, through the annotation, call macro definition for motor selection and contro
By operating Dji_init.h, the creation and use of a single motor structure can be realized, and the choice of 2006 or 3508 can be made according to Dji_init.c, as well as the initialization of the control mode and parameters
This control code adopts the speed position nesting mode
Solved the problem that the Hall feedback is not 0 when DJI is initially powered on
You only need to put the callbacks in the code into your own can callback function, and then call the initialization function to use this code normally
