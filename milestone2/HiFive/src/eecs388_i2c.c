#include <stdio.h>
#include <stdint.h>
#include "eecs388_lib.h"
#include "metal/i2c.h"

#define DEBUG true

struct metal_i2c *i2c;

#define bufWriteSize 5
#define bufReadSize  1

uint8_t bufWrite[bufWriteSize];
uint8_t bufRead[bufReadSize];

volatile int g_angle;

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

void breakup(int bigNum, uint8_t* low, uint8_t* high){
    *low  = bigNum & 0xFF;    // bitmask first 8 bits only
    *high = bigNum >> 8;      // shift high bits 8 to the right
    return;
}

void write(uint8_t LED, uint8_t ON_L, uint8_t ON_H, uint8_t OFF_L, uint8_t OFF_H){
    bufWrite[0] = LED;
    bufWrite[1] = ON_L;
    bufWrite[2] = ON_H;
    bufWrite[3] = OFF_L;
    bufWrite[4] = OFF_H;
    
    metal_i2c_write(i2c, PCA9685_I2C_ADDRESS, 5, bufWrite, METAL_I2C_STOP_ENABLE);
    
    // #ifdef DEBUG
    //     printf("WRITE BUFFER:\n");
    //     for(uint8_t i = 0; i<5; i++){
    //         printf("\tBuffer[%i]: %x\n",i, bufWrite[i]);
    //     }
    // #endif
    return;
}

void steering(int angle){
    uint8_t ang_off_l, ang_off_h;

    uint16_t ang_off_cycles = getServoCycle(angle);
    breakup(ang_off_cycles, &ang_off_l, &ang_off_h);    //Breakup into bytes
    write(PCA9685_LED1_ON_L, 0x00, 0x00, ang_off_l, ang_off_h); //Write to I2C controller
    
    #ifdef DEBUG
        printf("STEERING ANGLE (degrees)  : %i\n", g_angle);
        printf("ANGLE ON DURATION (cycles): %i\n", ang_off_cycles);
    #endif
    
    return;
}

/*
    Motor config/stop. 

    Task 1: using the breakup(..) and write(..) function, implement the 
    funcion defined below. This function configures the motor by 
    writing a value of 280 to the motor.

    Usage: when calling stopMotor for calibration, include
    a 2 second delay for proper configuration. 

    Note: the motor's speed controller is configured for 
    LED0 and the servo for LED1. 

    Example Use: stopMotor(); -> sets LED0_Off to 280 
*/
void stopMotor()
{
    uint8_t stop_l, stop_h;
    breakup(280, &stop_l, &stop_h);
    write(PCA9685_LED0_ON_L, 0x00, 0x00, stop_l, stop_h);
}

/*
    Motor Forward

    Task 2: using breakup(..) and write(..), implement 
    the function defined below to Drive the motor 
    forward. The given speedFlag will alter the 
    motor speed as follows:
    
    speedFlag = 1 -> value to breakup = 303 
    speedFlag = 2 -> value to breakup = 305
    speedFlag = 3 -> value to breakup = 307

    Note: the motor's speed controller is configured for 
    LED0 and the servo for LED1. 

    Example Use: driveForward(3); -> sets LED0_Off to 307 
*/

void driveForward(uint8_t speedFlag)
{
    uint8_t speed_l, speed_h;
    uint16_t val_to_write = 303;

    if (speedFlag <= 3 && speedFlag > 0)
        val_to_write += (speedFlag - 1) * 2;
    breakup(val_to_write, &speed_l, &speed_h);

    write(PCA9685_LED0_ON_L, 0x00, 0x00, speed_l, speed_h);
}

void raspberrypi_int_handler(int devid)
{
    // Message from pi
	char pi_msg[24] = "\0";
    char* key = "angle:";
    int length = 6;  //length of key
	int ret;
    
    // read from uart[devid] up to 24 chars and store in pi_msg
    ser_readline(devid, 24, pi_msg);
    ser_printline(0, pi_msg);
    // printf("PI_MESSAGE: %s\n", pi_msg);
    //printf("PI_MESSAGE:");
    // for(int i = 0; i < 24; i++){
    //     printf("%c",pi_msg[i]);
    // }
    ret = strncmp(pi_msg, key, length);
    if (ret == 0)   // only change steering angle if msg matches key
        sscanf(pi_msg+length, "%d", &g_angle); // update g_angle to the int value of pi_msg 
}

int main()
{
    // Initialize I2C
    set_up_I2C();

    // initialize UART channels
    ser_setup(1); // uart1 (raspberry pi)
    ser_setup(0); // uart0 (hifive)

    // Initialize global angle
    g_angle = 0;

    // 1. Calibrate Motor
    // 2. Delay
    // 3. Stop Motor
    // 4. Begin main loop
    
    printf("Ready for calibration...\n");
    stopMotor();
    delay(2500);
    printf("Calibration window closed. Initializing.\n");
    driveForward(2);
    delay(2000);
    stopMotor();

    while (1) {

        if(ser_isready(0) & 0b10){
            raspberrypi_int_handler(0);
            steering(g_angle);
        }


    }
    
}
