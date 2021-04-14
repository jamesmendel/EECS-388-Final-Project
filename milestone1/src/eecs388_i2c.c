#include <stdio.h>
#include <stdint.h>
#include "eecs388_lib.h"
#include "metal/i2c.h"

struct metal_i2c *i2c;

#define bufWriteSize 5
#define bufReadSize  1

uint8_t bufWrite[bufWriteSize];
uint8_t bufRead[bufReadSize];

// The entire setup sequence
void set_up_I2C(){
    uint8_t oldMode;
    uint8_t newMode;
    _Bool success;

    bufWrite[0] = PCA9685_MODE1;
    bufWrite[1] = MODE1_RESTART;
    printf("%d\n", bufWrite[0]);
    
    i2c = metal_i2c_get_device(0);

    if(i2c == NULL)
    {
        printf("Connection Unsuccessful\n");
    }
    else
    {
        printf("Connection Successful\n");
    }
    
    // Setup Sequence
    metal_i2c_init(i2c, I2C_BAUDRATE, METAL_I2C_MASTER);
    success = metal_i2c_write(i2c, PCA9685_I2C_ADDRESS, 2, bufWrite, METAL_I2C_STOP_DISABLE);   // reset
    delay(100);
    printf("resetting PCA9685 control 1\n");

    // Initial Read of control 1
    bufWrite[0] = PCA9685_MODE1;    // Address
    success = metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 1, bufRead, 1);            // initial read
    printf("Read success: %d and control value is: %d\n", success, bufWrite[0]);
    
    // Configuring Control 1
    oldMode = bufRead[0];
    newMode = (oldMode & ~MODE1_RESTART) | MODE1_SLEEP;
    printf("sleep setting is %d\n", newMode);
    bufWrite[0] = PCA9685_MODE1;    // address
    bufWrite[1] = newMode;          // writing to register
    success = metal_i2c_write(i2c, PCA9685_I2C_ADDRESS, 2, bufWrite, METAL_I2C_STOP_DISABLE);  // sleep
    bufWrite[0] = PCA9685_PRESCALE; // Setting PWM prescale
    bufWrite[1] = 0x79;
    success = metal_i2c_write(i2c, PCA9685_I2C_ADDRESS, 2, bufWrite, METAL_I2C_STOP_DISABLE);  // sets prescale
    bufWrite[0] = PCA9685_MODE1;
    bufWrite[1] = 0x01 | MODE1_AI | MODE1_RESTART;
    printf("on setting is %d\n", bufWrite[1]);
    success = metal_i2c_write(i2c, PCA9685_I2C_ADDRESS, 2, bufWrite, METAL_I2C_STOP_DISABLE);  // awake
    delay(100);
    printf("Setting the control register\n");
    bufWrite[0] = PCA9685_MODE1;
    success = metal_i2c_transfer(i2c, PCA9685_I2C_ADDRESS, bufWrite, 1, bufRead, 1);           // initial read
    printf("Set register is %d\n", bufRead[0]);

} 

/*  
    Defining the breakup function

    Task 1: breaking bigNum into two 8-bit numbers

    Implement the following function that recieves a 12 bit number
    from 0 to 4096 and breaks it up into two 8-bit numbers.

    ex: Breakup decimal number 2106 into two bytes

    uint8_t variable1;
    uint8_t variable2;
    breakup(2106, &variable1, &variable2);

    // variable1 has low  8 bits of 2106 (0000 0110)
    // variable2 has high 8 bits of 2106 (0010 0001) 
*/

void breakup(int bigNum, uint8_t* low, uint8_t* high){
    /*
        Write Task 1 code here
    */
}

/*
    Writing with i2c

    -Task 2: using bufWrite, bufRead, and
    and metal_i2c_write(), implement the funcion made
    below.

    ex:
    uint8_t variable1;
    uint8_t variable2;
    breakup(2106, &variable1, &variable2);
    write(PCA9685_LED1_ON_L, 0, 0, variable1, variable2);
*/

void write(int LED, uint8_t ON_L, uint8_t ON_H, uint8_t OFF_L, uint8_t OFF_H){
    /*
        Write Task 2 code here
    */
}

/*
    Changing Steering Heading

    Task 3: using getServoCycle(), breakup(), and write(),
    implement the function defined below to control the servo
    with the passed in argument "angle" ranging from -45 to 45.

    Use the getServoCycle function to get the value to 
    breakup.

    ex: 
    int valToBreak = getServoCycle(45);
    // sets valToBreak to 355

    note: the motor's speed controller is configured for 
    LED0 and the servo for LED1. 

    ex: 
    steering(0);   // driving angle forward
    steering(-45); // driving left
    steering(45);  // driving right
*/

void steering(int angle){
    /*
        Write Task 3 code here
    */
}

int main()
{
    set_up_I2C();

    /*
        Partially Controlling the PCA9685

        In order to demo, add function calls here 
        to turn the wheels. You should call the steering
        function with sufficient delays between calls to
        make clear the angle being written.
    */
    
}
