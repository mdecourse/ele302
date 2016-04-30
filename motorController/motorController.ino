// imported library code from Progressive Automations

// include the SPI library (used to control H-Bridge Motor Controllers):
// Serial Peripheral Interface (SPI)
#include <SPI.h>

// Set Directional Pins for controlling motor direction
#define DIR_M1 2
#define DIR_M2 3
#define DIR_M3 4
#define DIR_M4 7

// Set PWM Pins for the motors
#define PWM_M1 9
#define PWM_M2 10    
#define PWM_M3 5
#define PWM_M4 6

// Set Slave Pins for the motors
#define SS_M1 11
#define SS_M2 12
#define SS_M3 13
#define SS_M4 14

// Enable for all 4 motors
#define ENABLE_MOTORS 8

int pwm1, pwm2, pwm3, pwm4;
boolean dir1, dir2, dir3, dir4;

// initialization code
void setup() {
    unsigned int configWord;

    // Setup pins:
    pinMode(SS_M1, OUTPUT); 
    pinMode(SS_M2, OUTPUT); 
    pinMode(SS_M3, OUTPUT); 
    pinMode(SS_M4, OUTPUT); 

    // LOW = enabled, HIGH = disabled
    digitalWrite(SS_M1, LOW);
    digitalWrite(SS_M2, LOW);  
    digitalWrite(SS_M3, LOW);  
    digitalWrite(SS_M4, LOW);  

    // Directional pins
    pinMode(DIR_M1, OUTPUT);
    pinMode(DIR_M2, OUTPUT);
    pinMode(DIR_M3, OUTPUT);
    pinMode(DIR_M4, OUTPUT);

    // PWM pins
    pinMode(PWM_M1, OUTPUT);  
    pinMode(PWM_M2, OUTPUT);      
    pinMode(PWM_M3, OUTPUT);  
    pinMode(PWM_M4, OUTPUT);      

    // LOW = enabled, HIGH = disabled
    digitalWrite(PWM_M1, LOW);
    digitalWrite(PWM_M2, LOW);
    digitalWrite(PWM_M3, LOW);
    digitalWrite(PWM_M4, LOW);

    // Enable all 4 motors
    pinMode(ENABLE_MOTORS, OUTPUT); 

    // LOW = enabled, HIGH = disabled
    digitalWrite(ENABLE_MOTORS, HIGH);  

  /* /******* Set up L9958 chips *********
    ' L9958 Config Register
    ' Bit
    '0 - RES
    '1 - DR - reset
    '2 - CL_1 - curr limit
    '3 - CL_2 - curr_limit
    '4 - RES
    '5 - RES
    '6 - RES
    '7 - RES
    '8 - VSR - voltage slew rate (1 enables slew limit, 0 disables)
    '9 - ISR - current slew rate (1 enables slew limit, 0 disables)
    '10 - ISR_DIS - current slew disable
    '11 - OL_ON - open load enable
    '12 - RES
    '13 - RES
    '14 - 0 - always zero
    '15 - 0 - always zero
    */  // set to max current limit and disable ISR slew limiting */
    configWord = 0b0000010000001100;

    SPI.begin();
    SPI.setBitOrder(LSBFIRST);
    // clock pol = low, phase = high??
    SPI.setDataMode(SPI_MODE1);

    // Motor 1
    digitalWrite(SS_M1, LOW);
    SPI.transfer(lowByte(configWord));
    SPI.transfer(highByte(configWord));
    digitalWrite(SS_M1, HIGH);

    // Motor 2
    digitalWrite(SS_M2, LOW);
    SPI.transfer(lowByte(configWord));
    SPI.transfer(highByte(configWord));
    digitalWrite(SS_M2, HIGH);

    // Motor 3
    digitalWrite(SS_M3, LOW);
    SPI.transfer(lowByte(configWord));
    SPI.transfer(highByte(configWord));
    digitalWrite(SS_M3, HIGH);

    // Motor 4
    digitalWrite(SS_M4, LOW);
    SPI.transfer(lowByte(configWord));
    SPI.transfer(highByte(configWord));
    digitalWrite(SS_M4, HIGH);

    //Set initial actuator settings to pull at 0 speed for safety
    dir1 = 0; dir2 = 0; dir3 = 0; dir4 = 0; // Set direction
    pwm1 = 0; pwm2 = 0; pwm3 = 0; pwm4 = 0; // Set speed (0-255)

    // LOW = enabled, HIGH = disabled
    digitalWrite(ENABLE_MOTORS, LOW);

} 

// Process to repeat continuously in loop
void loop() {

    //set direction and speed (sample values below)
    dir1 = 0;
    pwm1 = 255; 
    digitalWrite(DIR_M1, dir1);
    analogWrite(PWM_M1, pwm1); // write to pins

    dir2 = 1;
    pwm2 = 255;
    digitalWrite(DIR_M2, dir2);
    analogWrite(PWM_M2, pwm2);

    dir3 = 0;
    pwm3 = 255;
    digitalWrite(DIR_M3, dir3);
    analogWrite(PWM_M3, pwm3);

    dir4 = 0;
    pwm4 = 128;
    digitalWrite(DIR_M4, dir4);
    analogWrite(PWM_M4, pwm4);

    // apply a delay to wait for the motors to set
    delay(5000);

}
