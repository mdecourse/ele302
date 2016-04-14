// For the ArduinoUNO
#include <SoftwareSerial.h>
#include <serLCD.h>

// For the Sensor Chip
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"

<<<<<<< HEAD
#define AHRS true         // set to false for basic data read
#define SerialDebug true   // set to true to get Serial output for debugging

=======
#define LED_PIN 13
>>>>>>> origin/master

// Attach the serial display's RX line to digital pin 2
SoftwareSerial mySerial(3,2); // pin 2 = TX, pin 3 = RX (unused)

MPU9250 mpu;

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed = 13; // Set up pin 13 led for toggling

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output

<<<<<<< HEAD
uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0, sumCount = 0; // used to control display output rate
float pitch, yaw, roll;

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
=======
bool blinkState = false;
>>>>>>> origin/master

void setup()
{
	// Arduino Setup
	mySerial.begin(9600); // set up serial port for 9600 baud
	delay(500); // wait for display to boot up

    Serial.begin(9600); 

	 // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize device
    mySerial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    mySerial.println("Testing device connections...");
    mySerial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");

    // configure Arduino LED for
    pinMode(myLed, OUTPUT);

    Wire.begin();

    // Set up the interrupt pin, its set as active high, push-pull
    pinMode(intPin, INPUT);
    digitalWrite(intPin, LOW);
    pinMode(myLed, OUTPUT);
    digitalWrite(myLed, HIGH);

    // Read the WHO_AM_I register, this is a good test of communication
    byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
    mySerial.write(254)
    mySerial.write(128)
    mySerial.write("I AM");
    mySerial.write(c, HEX);  
    mySerial.write(254)
    mySerial.write(192)
    mySerial.write("I Should Be");
    mySerial.write(0x71, HEX); 

if (c == 0x71) // WHO_AM_I should always be 0x68
  {  
    Serial.println("MPU9250 is online...");
    
    MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
    Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");
 
    calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers

    display.clearDisplay();

    // clear display
    mySerial.write(254)
    mySerial.write(128) 
    mySerial.write("                "); 
    mySerial.write("                ");

    mySerial.write(254)
    mySerial.write(128)
    mySerial.write("MPU9250 bias");
    mySerial.write(254)
    mySerial.write(192)
    mySerial.write(" x   y   z  ");

    // clear display
    mySerial.write(254)
    mySerial.write(128) 
    mySerial.write("                "); 
    mySerial.write("                ");

    // display acceleration biasses
    mySerial.write(254)
    mySerial.write(128)
    mySerial.write((int)(1000*accelBias[0]));
    mySerial.write((int)(1000*accelBias[1]));
    mySerial.write((int)(1000*accelBias[2]));
    mySerial.write("mg");
    delay(2000)

    // clear display
    mySerial.write(254)
    mySerial.write(128) 
    mySerial.write("                "); 
    mySerial.write("                ");

    // display gyro biasses 
    mySerial.write(254)
    mySerial.write(128)
    mySerial.write(gyroBias[0]));
    mySerial.write(gyroBias[1]));
    mySerial.write(gyroBias[2]));
    mySerial.write("o/s");
    delay(2000) 
  
    display.display();
    delay(1000); 
  
    initMPU9250(); 
    Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
  
    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    byte d = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for AK8963
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
    display.clearDisplay();
    display.setCursor(20,0); display.print("AK8963");
    display.setCursor(0,10); display.print("I AM");
    display.setCursor(0,20); display.print(d, HEX);  
    display.setCursor(0,30); display.print("I Should Be");
    display.setCursor(0,40); display.print(0x48, HEX);  
    display.display();
    delay(1000); 
  
    // Get magnetometer calibration from AK8963 ROM
    initAK8963(magCalibration); Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer
  
  if(SerialDebug) {
    //  Serial.println("Calibration values: ");
    Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
    Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
    Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);
  }
  
    display.clearDisplay();
    display.setCursor(20,0); display.print("AK8963");
    display.setCursor(0,10); display.print("ASAX "); display.setCursor(50,10); display.print(magCalibration[0], 2);
    display.setCursor(0,20); display.print("ASAY "); display.setCursor(50,20); display.print(magCalibration[1], 2);
    display.setCursor(0,30); display.print("ASAZ "); display.setCursor(50,30); display.print(magCalibration[2], 2);
    display.display();
    delay(1000);  
  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
}


}

void loop()
{



	// // Arduino Setup
	// mySerial.write(254); // move cursor to beginning of first line
	// mySerial.write(128);
	// mySerial.write("                "); // clear display
	// mySerial.write("                ");
	// mySerial.write(254); // move cursor to beginning of first line
	// mySerial.write(128);
	// mySerial.write("Hello, world!");
	// while(1); // wait forever


}