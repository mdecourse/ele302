// For the ArduinoUNO
#include <SoftwareSerial.h>
#include <serLCD.h>

// For the Sensor Chip
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"

#define LED_PIN 13

// Attach the serial display's RX line to digital pin 2
SoftwareSerial mySerial(3,2); // pin 2 = TX, pin 3 = RX (unused)

MPU9250 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

bool blinkState = false;

void setup()
{
	// Arduino Setup
	mySerial.begin(9600); // set up serial port for 9600 baud
	delay(500); // wait for display to boot up

	 // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize device
    mySerial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    mySerial.println("Testing device connections...");
    mySerial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);




}

void loop()
{
	 // read raw accel/gyro measurements from device
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    // display tab-separated accel/gyro x/y/z values
    mySerial.print("a/g/m:\t");
    mySerial.print(ax); mySerial.print("\t");
    mySerial.print(ay); mySerial.print("\t");
    mySerial.print(az); mySerial.print("\t");
    mySerial.print(gx); mySerial.print("\t");
    mySerial.print(gy); mySerial.print("\t");
    mySerial.print(gz); mySerial.print("\t");
    mySerial.print(mx); mySerial.print("\t");
    mySerial.print(my); mySerial.print("\t");
    mySerial.println(mz);

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

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