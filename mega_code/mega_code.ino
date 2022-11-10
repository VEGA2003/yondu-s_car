/*
   example code for car project M5 with the FX IMU sensor
   v0.1: initial setup
   v0.2: errors removed
   v0.3: repair of accelerometer readout
*/

//configurable items
#define SERIALPORT Serial3  //define debug output. Serial is via USB, Serial3 is bluetooth
#define BAUDRATE 9600       //9600 for Serial3
#define GEARING 34          //47, 31 motor gearing. Change to your motor
#define PPR 48              //Pulses per revolution. See rotary encode spec

//libraries used
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <Encoder.h>

//encoder declarations. Do not change!
// Motor encoder external interrupt pins (no other pins allowed!) Pins 18 and 19 are only available on the Arduino Mega
#define ENCA_L 19  // Encoder A input         Yellow
#define ENCB_L 2   // Encoder B input         Green
#define ENCA_R 18  // Encoder A input         Yellow
#define ENCB_R 3   // Encoder B input         Green

Encoder encoderL(ENCA_L, ENCB_L);
Encoder encoderR(ENCB_R, ENCA_R);  //due to wrong wiring

float cpr = GEARING * PPR;
long positionLeft;
long positionRight;
float angle = 0;
float time = millis();
float now = 0;
float td = 0;
float FFT;
int count = 10;
int raw = 0;

int inPin = 22;
int d;

unsigned long tNow;
unsigned long tPrev;
long prevPositionLeft;
long prevPositionRight;

//servo declarations
Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards


// motor pins. Do not change
#define PWMA 5   // Motor A PWM control     Orange
#define AIN2 7   // Motor A input 2         Brown
#define AIN1 4   // Motor A input 1         Green
#define BIN1 10  // Motor B input 1         Yellow
#define BIN2 8   // Motor B input 2         Purple
#define PWMB 6   // Motor B PWM control     White


/* Assign a unique ID to this sensor at the same time */
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

/* Assign a unique ID to this sensor at the same time */
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);

//global variables
int pos = 99;  // variable to store the servo position
int speedL = 50;
int speedR = 50;
int startPos = 99;  //neutral posistion servo in degrees
float turns = 0;

void setup() {  //leave as is

  SERIALPORT.begin(BAUDRATE);
  while (!SERIALPORT) {
    // wait for SERIALPORT to start
  }
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object

  initMotors();  //function to setup motor
  initIMU();     //init IMU. Change ranges in this function
  pinMode(15, INPUT_PULLUP);

  //set steering in neutral position and wait 2 sec
  myservo.write(startPos);
  delay(2000);

  //init for rotary speeds
  tPrev = micros();
  prevPositionLeft = encoderL.read();
  ;
  prevPositionRight = encoderR.read();
  time = millis();

  Serial2.begin(9600);

  pinMode(inPin, INPUT);
}

void loop() {

  attachInterrupt(digitalPinToInterrupt(15), break_ISR, LOW);

  if (Serial2.available()) {
    FFT = Serial2.parseInt();  //Read the serial data and store in var
                               // average
    for (int i = 0; i < count; i++) raw += FFT;
    raw = raw / count;
  }

  sensors_event_t a, m, g;

  /* Get a new sensor event */
  gyro.getEvent(&g);
  accelmag.getEvent(&a, &m);

  float gyrox = g.gyro.x;
  float gyroy = g.gyro.y;
  float gyroz = g.gyro.z;


  //read rotary sensor position
  positionLeft = encoderL.read();
  positionRight = encoderR.read();


  d = digitalRead(inPin);

  // drive forward
  if (raw == 1.00 && d == 1) {
    speedL = 50;
    speedR = speedL - 1;
    pos = 99;
    forwardA(abs(speedL));
    forwardB(abs(speedR));
  }

  // turn right
  else if (raw == 2.00 && d == 1) {
    speedR = 0;
    speedL = 100;
    pos = 150;
    forwardA(abs(speedL));
    forwardB(abs(speedR));
  }

  // turn left
  else if (raw == 3.00 && d == 1) {
    speedL = 0;
    speedR = 100;
    pos = 40;
    forwardA(abs(speedL));
    forwardB(abs(speedR));
  }
  // drive forward
  else if (raw == 4.00 && d == 1) {
    speedL = 50;
    speedR = speedL - 1;
    pos = 99;
    forwardA(abs(speedL));
    forwardB(abs(speedR));
  }

  // turn right
  else if (raw == 5.00 && d == 1) {
    speedR = 0;
    speedL = 100;
    pos = 120;
    forwardA(abs(speedL));
    forwardB(abs(speedR));
  }

  // turn left
  else if (raw == 6.00 && d == 1) {
    speedL = 0;
    speedR = 100;
    pos = 70;
    forwardA(abs(speedL));
    forwardB(abs(speedR));

  } else {
    brakeA();
    brakeB();
  }

  // set steering position
  myservo.write(pos);  // ranging 0-180 degrees

  //calculation of rotation speed of motor
  tNow = micros();                                 //timestap of calculation
  float dT = (tNow - tPrev) / 1e6;                 //calculate time difference since last measurement in seconds
  long dPosL = positionLeft - prevPositionLeft;    //number of pulses since last measurement
  long dPosR = positionRight - prevPositionRight;  //dito

  float VelocityL = dPosL / dT;  //actual calculation in pulses per second
  float VelocityR = dPosR / dT;

  //store variable for next run
  tPrev = tNow;
  prevPositionLeft = positionLeft;
  prevPositionRight = positionRight;

  delay(50);
}

void break_ISR() {
  brakeA();
  brakeB();
}

//functions
void forwardA(uint16_t pwm) {  //addapted to wiring
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, pwm);
}
void forwardB(uint16_t pwm) {  //addapted to wiring
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, pwm);
}
void reverseA(uint16_t pwm) {  //addapted to wiring
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, pwm);
}
void reverseB(uint16_t pwm) {  //addapted to wiring
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, pwm);
}
void brakeA() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}
void brakeB() {
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void initMotors() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
}
void initIMU() {
  SERIALPORT.println("Gyroscope Test");
  SERIALPORT.println("");
  /* Initialise the sensor */
  if (!gyro.begin()) {
    /* There was a problem detecting the FXAS21002C ... check your connections
    */
    SERIALPORT.println("Ooops, no FXAS21002C detected ... Check your wiring!");
    while (1)
      ;
  }
  SERIALPORT.println("FXOS8700 Test");
  SERIALPORT.println("");
  /* Initialise the sensor */
  if (!accelmag.begin()) {
    /* There was a problem detecting the FXOS8700 ... check your connections */
    SERIALPORT.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while (1)
      ;
  }
}
