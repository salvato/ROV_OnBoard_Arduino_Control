/*
  Serial Event by
  Tom Igoe - 9 May 2011

 Servo position by
 Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 
 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob

 sudo usermod -a -G tty yourUserName
 sudo usermod -a -G dialout yourUserName

 Gabriele Salvato
 
*/

// Servo SAMWA SM-394 pinout
// Pin 1 Signal (PWM)
// Pin 2 GND
// Pin 3 +5 V

#include <Servo.h>
#include <Wire.h>
#include "MS5837.h"

void    serialEvent();
void    executeCommand(String inputString);

enum commands {
  AreYouThere   = 70,
  InflateValve  = 71,
  DeflateValve  = 72,
  RightForward  = 73,
  RightReverse  = 74,
  LeftForward   = 75,
  LeftReverse   = 76,
  RightSpeed    = 77,
  LeftSpeed     = 78,
  FrontThruster = 79,
  BackThruster  = 80,
  GetDepth      = 81
};
char command;

char ACK  = 6;
char NACK = 21;
char EOS  = char(127);

// Serial port communicates on digital pins 0 (RX) and 1 (TX) as well as with the computer via USB. 
// Thus, since we use these functions, we cannot also use pins 0 and 1 for digital input or output. 

// SPI on Nano: 10 (SS), 11 (MOSI), 12 (MISO), 13 (SCK)

// I2C on Nano: A4 (SDA) and A5 (SCL)

// The Nano has 8 analog inputs, each of which provide 10 bits of resolution.
// By default they measure from ground to 5 volts, though is it possible to
// change the upper end of their range using the analogReference() function.
// Analog pins 6 and 7 cannot be used as digital pins.

// The Arduino has its own set default values for the modulation frequency for PWM pins.
// For pins 3, 9, 10, 11 it is approximately 488 Hz.
// For pins 5 and 6, it is about 977 Hz.
// These values are for a stock Arduino running at 16MHz.

// You can change these frequencies easily by writing new values to the appropriate timer register.
// For example, to change the frequency of timer 2, which controls pins 9 and 10, to 3906 Hz
// you would set its register like so:
// TCCR1B = TCCR1B & 0b11111000 | 0x02;

// The PWM outputs generated on pins 5 and 6 will have higher-than-expected duty cycles.
// This is because of interactions with the millis() and delay() functions, which share the same
// internal timer used to generate those PWM outputs.

// This will be noticed mostly on low duty-cycle settings (e.g 0 - 10) and may result in a value
// of 0 not fully turning off the output on pins 5 and 6.

// Serial pins
int SerialTX             = 0;
int SerialRX             = 1;

// Motor control pins
int leftMotorForwardPin  = 2;
int leftMotorReversePin  = 4;
int rightMotorForwardPin = 7;
int rightMotorReversePin = 8;

int FrontThrusterPin     = 9; // PWM Pins on Nano: 3, 5, 6, 9, 10, and 11
int BackThrusterPin      = 10;// PWM Out
int leftMotorSpeedPin    = 3; // PWM Out
int rightMotorSpeedPin   = 5; // PWM Out
int inValvePin           = 13;
int outValvePin          = 12;

// Pins used: 0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13, A4, A5

Servo            F_Thruster;           // create servo object to control the Front Thruster
Servo            B_Thruster;           // create servo object to control the Back Thruster

int minThruster = 1100; // In microseconds
int maxThruster = 1900; // In microseconds

String           inputString = "";        // a string to hold incoming data
boolean          stringComplete = false;  // whether the string is complete
long             baudRate = 115200;

MS5837           depthSensor;

int seaWaterDensity   = 1029;             // Kg/m^3
int freshWaterDensity =  997;             // Kg/m^3

void 
setup() {
  // attach(pin, min, max) - Attaches to a pin setting min and max values in microseconds
  // default min is 544, max is 2400
  F_Thruster.attach(FrontThrusterPin, minThruster, maxThruster); // Attaches the Front Thruster on pin 9 to the servo object
  F_Thruster.writeMicroseconds(1500);                            // Sets the initialization ESC value.
  B_Thruster.attach(BackThrusterPin, minThruster, maxThruster);  // Attaches the Back Thruster on pin 10 to the servo object
  B_Thruster.writeMicroseconds(1500);                            // Sets the initialization ESC value.

  // Pins to control the motors
  pinMode(leftMotorForwardPin,   OUTPUT);
  pinMode(leftMotorReversePin,   OUTPUT);
  pinMode(rightMotorForwardPin,  OUTPUT);
  pinMode(rightMotorReversePin,  OUTPUT);

  pinMode(leftMotorSpeedPin,     OUTPUT);
  pinMode(rightMotorSpeedPin,    OUTPUT);
  
  // Pins to control the valves
  pinMode(inValvePin,            OUTPUT);
  pinMode(outValvePin,           OUTPUT);

  // turn off the Motors
  digitalWrite(leftMotorForwardPin,  LOW);
  digitalWrite(leftMotorReversePin,  LOW);
  digitalWrite(rightMotorForwardPin, LOW);
  digitalWrite(rightMotorReversePin, LOW);

  analogWrite(leftMotorSpeedPin,  0);
  analogWrite(rightMotorSpeedPin, 0);

  // turn off the Valves 
  digitalWrite(inValvePin,  LOW); 
  digitalWrite(outValvePin, LOW);

  Serial.begin(baudRate, SERIAL_8N1); // initialize serial: 8 data bits, no parity, one stop bit.
  inputString.reserve(200);           // reserve 200 bytes for the inputString:

  // I2c devices (just depth sensor for now)
  Wire.begin();
  depthSensor.init();
  depthSensor.setFluidDensity(seaWaterDensity);
  
  delay(1000);                        // Wait for the ESC to initialize
}


// SerialEvent occurs whenever a new data comes in the hardware serial RX.
// This routine is run between each time loop() runs, so using delay inside loop can delay response.
// Multiple bytes of data may be available.
void 
serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == EOS) {
      stringComplete = true;
    }
  }
}


void 
loop() {
  serialEvent(); //call the function
  if (stringComplete) {
    //Serial.println(inputString);// print the string when a newline arrives:
    executeCommand(inputString);
    inputString = "";// clear the string:
    stringComplete = false;
  }
}


void
executeCommand(String inputString) {
  command = inputString.charAt(0);
  int value = inputString.charAt(1);// Get the value
  
  if(command == char(FrontThruster)) {   // Up Down Servo position 
    value = map(value, -10, 10, minThruster, maxThruster); // scale it to use it with the ESC
    F_Thruster.write(value);                            // sets the servo position according to the scaled value 
    Serial.print(ACK);
  } 
  
  if(command == char(BackThruster)) {// Up Down Servo position 
    value = map(value, -10, 10, minThruster, maxThruster); // scale it to use it with the ESC
    B_Thruster.write(value);                            // sets the servo position according to the scaled value 
    Serial.print(ACK);
  } 
  
  else if(command == char(LeftSpeed)) { // Left Motor Speed
    value = map(value, 0, 10, 0, 255); // scale to 0 - 255 
    analogWrite(leftMotorSpeedPin, value);
    Serial.print(ACK);
  } 
  
  else if(command == char(LeftForward)) { // Left Motor Forward
    if(value)
      digitalWrite(leftMotorForwardPin, HIGH);
    else
      digitalWrite(leftMotorForwardPin, LOW);
    Serial.print(ACK);
  } 
  
  else if(command == char(LeftReverse)) { // Left Motor Reverse
    if(value)
      digitalWrite(leftMotorReversePin, HIGH);
    else
      digitalWrite(leftMotorReversePin, LOW);
    Serial.print(ACK);
  } 
  
  else if(command == char(RightSpeed)) { // Right Motor Speed
    value = map(value, 0, 10, 0, 255); // scale to 0 - 255 
    analogWrite(rightMotorSpeedPin, value);
    Serial.print(ACK);
  } 
  
  else if(command == char(RightForward)) { // Right Motor Forward
    if(value)
      digitalWrite(rightMotorForwardPin, HIGH);
    else
      digitalWrite(rightMotorForwardPin, LOW);
    Serial.print(ACK);
  } 
  
  else if(command == char(RightReverse)) { // Right Motor Reverse
    if(value)
      digitalWrite(rightMotorReversePin, HIGH);
    else
      digitalWrite(rightMotorReversePin, LOW);
    Serial.print(ACK);
  } 
  
  else if(command == char(InflateValve)) { // Inflate Valve
    if(value)
      digitalWrite(inValvePin, HIGH);
    else
      digitalWrite(inValvePin, LOW);
    Serial.print(ACK);
  } 
  
  else if(command == char(DeflateValve)) { // Deflate Valve
    if(value)
      digitalWrite(outValvePin, HIGH);
    else
      digitalWrite(outValvePin, LOW);
    Serial.print(ACK);
  }

  else if(command == char(GetDepth)) {// Get ROV Depth
    depthSensor.read();// The read from I2C takes up for 40 ms, so use sparingly if possible.
    int depth = int(depthSensor.depth()*100.0);// depth expressed in cm
    Serial.print(ACK);
    for(int i=0; i<4; i++) {
      Serial.print(char(depth & 0x0F));
      depth = depth >> 4;
    }
  }
  
  else if(command == char(AreYouThere)) {// Are you ? 
    Serial.print(ACK);
  } 

  else {
    Serial.print(NACK);
  }
  
}


