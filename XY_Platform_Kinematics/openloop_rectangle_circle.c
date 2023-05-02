//Motor Code For Rectangle and Circle - Also Replaced Delay() With Millis()



#include "PinChangeInterrupt.h"

#define LEFT 13 // PCINT pin
#define RIGHT 12 // PCINT pin
#define BOTTOM 11 // PCINT pin
#define TOP 10 // PCINT pin

// Motor Speed and Direction Pin Setup
#define E1 5 // Left Motor - Speed
#define M1 4 // Left Motor - Direction

#define E2 6 // Right Motor - Speed
#define M2 7 // Right Motor - Direction

int E_value = 80;
int conversionFactor = 0;
int timerCount = 0;
bool hit_Limit_Switch = false;
bool activateLimitSwitch = true;

unsigned long startMilliseconds;
unsigned long currentMilliseconds;
unsigned long periodElapsed = currentMilliseconds - startMilliseconds;  //Time elapsed in milliseconds

void setup() {
  Serial.begin(9600);

  pinMode(TOP, INPUT);
  pinMode(BOTTOM, INPUT);
  pinMode(RIGHT, INPUT);
  pinMode(LEFT, INPUT);

  analogWrite(E1, 80);
  analogWrite(E2, 80);

  attachPCINT(digitalPinToPCINT(TOP), limitSwitch, RISING);
  attachPCINT(digitalPinToPCINT(BOTTOM), limitSwitch, RISING);
  attachPCINT(digitalPinToPCINT(RIGHT), limitSwitch, RISING);
  attachPCINT(digitalPinToPCINT(LEFT), limitSwitch, RISING);

  getConversionFactor(E_value);

  draw_OL_Square(50, 30); // Input = (H, W) in mm

  draw_OL_Circle(10); // Input = (D) in mm
}

void loop() {

}

void limitSwitch() // handle pin change interrupt for D8 to D13 here
{
  Serial.print("INT ");
  hit_Limit_Switch = true;
  if (activateLimitSwitch == true) {
    Serial.println("STOP");
    analogWrite(E1, 0);
    analogWrite(E2, 0);
  }
}

void getConversionFactor(int E) {
  conversionFactor = 85;
}


void draw_OL_Square(int H, int W) {

  Serial.print("draw_OL_Square ");

  int H_sec = miliMeter2miliSec(H);
  int W_sec = miliMeter2miliSec(W);

  Up();
  currentMilliseconds = millis();

  while (periodElapsed <= H_sec) {
    currentMilliseconds = millis();
    periodElapsed = currentMilliseconds - startMilliseconds;
  }

  analogWrite(E1, 0); analogWrite(E2, 0);
  Right();
  currentMilliseconds = millis();
  while (periodElapsed <= W_sec) {
    currentMilliseconds = millis();
    periodElapsed = currentMilliseconds - startMilliseconds;
  }

  analogWrite(E1, 0); analogWrite(E2, 0);
  Down();
  currentMilliseconds = millis();
  while (periodElapsed <= H_sec) {
    currentMilliseconds = millis();
    periodElapsed = currentMilliseconds - startMilliseconds;
  }
  analogWrite(E1, 0); analogWrite(E2, 0);
  Left();
  currentMilliseconds = millis();
  while (periodElapsed <= W_sec) {
    currentMilliseconds = millis();
    periodElapsed = currentMilliseconds - startMilliseconds;
  }
  analogWrite(E1, 0); analogWrite(E2, 0);
}


int miliMeter2miliSec(int miliMeter) {
  return (5000 / conversionFactor) * miliMeter;
}


void draw_OL_Circle(int D) {
  Serial.print("draw_OL_Circle ");

  int D_sec = miliMeter2miliSec(D);
  Left();
  currentMilliseconds = millis();
  while (periodElapsed <= (D_sec / 2)) {
    currentMilliseconds = millis();
    periodElapsed = currentMilliseconds - startMilliseconds;
  }
  analogWrite(E1, 0); analogWrite(E2, 0);

  Up();
  currentMilliseconds = millis();
  while (periodElapsed <= (D_sec / 4)) {
    currentMilliseconds = millis();
    periodElapsed = currentMilliseconds - startMilliseconds;
  }
  analogWrite(E1, 0); analogWrite(E2, 0);

  Dg_RU_StartTime();
  currentMilliseconds = millis();
  analogWrite(E1, 0);
  analogWrite(E2, E_value);
  periodElapsed = currentMilliseconds - startMilliseconds;
  while (periodElapsed <= D_sec) {
    E_value = E_value -= 5;      //decrements the motor speed by 5 everytime the elapsed time have not passed
    analogWrite(E1, 0);
    analogWrite(E2, E_value);
    currentMilliseconds = millis();
    periodElapsed = currentMilliseconds - startMilliseconds;
  }
  analogWrite(E1, 0); analogWrite(E2, 0);

  Right();
  currentMilliseconds = millis();
  while (periodElapsed <= (D_sec / 4)) {
    currentMilliseconds = millis();
    periodElapsed = currentMilliseconds - startMilliseconds;
  }
  analogWrite(E1, 0); analogWrite(E2, 0);


  Dg_RD_StartTime();
  currentMilliseconds = millis();
  analogWrite(E1, E_value);
  analogWrite(E2, 0);
  periodElapsed = currentMilliseconds - startMilliseconds;
  while (periodElapsed <= D_sec) {
    E_value = E_value -= 5;      //decrements the motor speed by 5 everytime the elapsed time have not passed
    analogWrite(E1, E_value);
    analogWrite(E2, 0);
    currentMilliseconds = millis();
    periodElapsed = currentMilliseconds - startMilliseconds;
  }
  analogWrite(E1, 0); analogWrite(E2, 0);

    Down();
  currentMilliseconds = millis();
  while (periodElapsed <= (D_sec / 4)) {
    currentMilliseconds = millis();
    periodElapsed = currentMilliseconds - startMilliseconds;
  }
  analogWrite(E1, 0); analogWrite(E2, 0);

  Dg_LD_StartTime(); 
    currentMilliseconds = millis();
     analogWrite(E1, 0); 
  analogWrite(E2, E_value); 
  periodElapsed = currentMilliseconds - startMilliseconds;
      while (periodElapsed <= D_sec) {
          E_value = E_value -= 5;      //decrements the motor speed by 5 everytime the elapsed time have not passed
  analogWrite(E1, 0); 
  analogWrite(E2, E_value); 
        currentMilliseconds = millis();
        periodElapsed = currentMilliseconds - startMilliseconds;
      }
  analogWrite(E1, 0); analogWrite(E2, 0);


    Left();
  currentMilliseconds = millis();
  while (periodElapsed <= (D_sec / 4)) {
    currentMilliseconds = millis();
    periodElapsed = currentMilliseconds - startMilliseconds;
  }
  analogWrite(E1, 0); analogWrite(E2, 0);


  Dg_LU_StartTime(); 
    currentMilliseconds = millis();
     analogWrite(E1, E_value); 
  analogWrite(E2, 0); 
  periodElapsed = currentMilliseconds - startMilliseconds;
      while (periodElapsed <= D_sec) {
          E_value = E_value -= 5;      //decrements the motor speed by 5 everytime the elapsed time have not passed
  analogWrite(E1, E_value); 
  analogWrite(E2, 0); 
        currentMilliseconds = millis();
        periodElapsed = currentMilliseconds - startMilliseconds;
      }
  analogWrite(E1, 0); analogWrite(E2, 0);
   
}


// Simple Motor Movement
void Left () {
  Serial.println("Left");
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  startMilliseconds = millis();  //Save the initial time when the timer starts
}
void Right () {
  Serial.println("Right");
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  startMilliseconds = millis();  //Save the initial time when the timer starts
}
void Up () {
  Serial.println("Up");
  digitalWrite(M1, LOW);
  digitalWrite(M2, HIGH);
  startMilliseconds = millis();  //Save the initial time when the timer starts
}
void Down () {
  Serial.println("Down");
  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);
  startMilliseconds = millis();  //Save the initial time when the timer starts
}


// Motor diagonal movements
void Dg_RU_StartTime () {
  Serial.println("Dg_RU_StartTime");
  digitalWrite(M1, 0);
  digitalWrite(M2, HIGH);
  startMilliseconds = millis();  //Save the initial time when the timer starts
}

void Dg_RD_StartTime  () {
  Serial.println("Dg_RD_StartTime");
  digitalWrite(M1, HIGH);
  digitalWrite(M2, 0);
  startMilliseconds = millis();  //Save the initial time when the timer starts
}

void Dg_LU_StartTime  () {
  Serial.println("Dg_LU_StartTime");
  digitalWrite(M1, LOW);
  digitalWrite(M2, 0);
  startMilliseconds = millis();  //Save the initial time when the timer starts
}

void Dg_LD_StartTime () {
  Serial.println("Dg_LD_StartTime");
  digitalWrite(M1, 0);
  digitalWrite(M2, LOW);
  startMilliseconds = millis();  //Save the initial time when the timer starts
}


Alice’s Version of Drawing Circle
#include "PinChangeInterrupt.h"

#define LEFT 13 // PCINT pin
#define RIGHT 12 // PCINT pin
#define BOTTOM 11 // PCINT pin
#define TOP 10 // PCINT pin

// Motor Speed and Direction Pin Setup
#define E1 5 // Left Motor - Speed
#define M1 4 // Left Motor - Direction

#define E2 6 // right Motor - Speed
#define M2 7 // right Motor - Direction


// Conversion Parameters
#define motor_power 80
#define conversionFactor 85

// Cicle Parameters
#define Radius 50 // radius of the circle in mm

// Limit Switch Variables
bool hit_Limit_Switch = false;
bool activateLimitSwitch = true;

void setup() {
  Serial.begin(9600);

  pinMode(TOP, INPUT);
  pinMode(BOTTOM, INPUT);
  pinMode(RIGHT, INPUT);
  pinMode(LEFT, INPUT);

  attachPCINT(digitalPinToPCINT(TOP), limitSwitch, RISING);
  attachPCINT(digitalPinToPCINT(BOTTOM), limitSwitch, RISING);
  attachPCINT(digitalPinToPCINT(RIGHT), limitSwitch, RISING);
  attachPCINT(digitalPinToPCINT(LEFT), limitSwitch, RISING);

  draw_OL_Circle(Radius);

}


int miliMeter2miliSec(int miliMeter) {
  return (5000 / conversionFactor) * miliMeter;
}


// Drawing Open Loop Circle
void draw_OL_Circle (int R) {

  int longerLegnth_miliSec = miliMeter2miliSec(R * sqrt(0.5));
  int shorterLength_miliSec = miliMeter2miliSec(R - R * sqrt(0.5));

  int mili_second_tracker = 0;
  int motor_power_tracker = 0;
  int longerLength_incrementing_value = motor_power / longerLegnth_miliSec;
  int shorterLength_incrementing_value = motor_power / shorterLength_miliSec;

  Up ();  analogWrite(E1, motor_power); analogWrite(E2, motor_power);

  // ------------------ Top Left Quadrant ------------------
  while (mili_second_tracker < shorterLength_miliSec) {
    motor_power_tracker = motor_power_tracker - shorterLength_incrementing_value;
    analogWrite(E1, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis() * 100; // increment every 100 miliseconds
  }
  mili_second_tracker = 0; motor_power_tracker = 0; // rESET Tracker variables
  Right ();
  while (mili_second_tracker < longerLegnth_miliSec) {
    motor_power_tracker = motor_power_tracker + shorterLength_incrementing_value;
    analogWrite(E1, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis() * 100;
  }
  mili_second_tracker = 0; motor_power_tracker = 0;


  // ------------------ Top right Quadrant ------------------
  while (mili_second_tracker < longerLegnth_miliSec) {
    motor_power_tracker = motor_power_tracker - shorterLength_incrementing_value;
    analogWrite(E2, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis() * 100; // increment every 100 miliseconds
  }
  mili_second_tracker = 0; motor_power_tracker = 0;
  Down ();
  while (mili_second_tracker < shorterLength_miliSec) {
    motor_power_tracker = motor_power_tracker + shorterLength_incrementing_value;
    analogWrite(E2, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis() * 100;
  }
  mili_second_tracker = 0; motor_power_tracker = 0;

  // ------------------ Bottom right Quadrant ------------------
  while (mili_second_tracker < longerLegnth_miliSec) {
    motor_power_tracker = motor_power_tracker - shorterLength_incrementing_value;
    analogWrite(E1, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis() * 100; // increment every 100 miliseconds
  }
  mili_second_tracker = 0; motor_power_tracker = 0;
  Left ();
  while (mili_second_tracker < shorterLength_miliSec) {
    motor_power_tracker = motor_power_tracker + shorterLength_incrementing_value;
    analogWrite(E1, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis() * 100;
  }
  mili_second_tracker = 0; motor_power_tracker = 0;


  // ------------------ Bottom Left Quadrant ------------------
  while (mili_second_tracker < shorterLength_miliSec) {
    motor_power_tracker = motor_power_tracker - shorterLength_incrementing_value;
    analogWrite(E2, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis() * 100; // increment every 100 miliseconds
  }
  mili_second_tracker = 0; motor_power_tracker = 0;
  Left ();
  while (mili_second_tracker < longerLegnth_miliSec) {
    motor_power_tracker = motor_power_tracker + shorterLength_incrementing_value;
    analogWrite(E2, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis() * 100;
  }
  mili_second_tracker = 0; motor_power_tracker = 0;
  
}



// Simple Motor Movement
void Left () {
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
}
void Right () {
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
}
void Up () {
  digitalWrite(M1, LOW);
  digitalWrite(M2, HIGH);
}
void Down () {
  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);
}


// Motor diagonal movements
void Dg_RU () {
  digitalWrite(M1, 0);
  digitalWrite(M2, HIGH);
}
void Dg_RD () {
  digitalWrite(M1, HIGH);
  digitalWrite(M2, 0);
}
void Dg_LU () {
  digitalWrite(M1, LOW);
  digitalWrite(M2, 0);
}
void Dg_LD () {
  digitalWrite(M1, 0);
  digitalWrite(M2, LOW);
}


void limitSwitch() // handle pin change interrupt for D8 to D13 here
{
  hit_Limit_Switch = true;
  if (activateLimitSwitch == true) {
    analogWrite(E1, 0);
    analogWrite(E2, 0);
  }
}

void loop (){}



Fixed a little
#include "PinChangeInterrupt.h"

#define LEFT 13 // PCINT pin
#define RIGHT 12 // PCINT pin
#define BOTTOM 11 // PCINT pin
#define TOP 10 // PCINT pin

// Motor Speed and Direction Pin Setup
#define E1 5 // Left Motor - Speed
#define M1 4 // Left Motor - Direction

#define E2 6 // right Motor - Speed
#define M2 7 // right Motor - Direction


// Conversion Parameters
#define motor_power 100
#define conversionFactor 85

// Cicle Parameters
#define Radius 50 // radius of the circle in mm

// Limit Switch Variables
bool hit_Limit_Switch = false;
bool activateLimitSwitch = true;

void setup() {
  Serial.begin(9600);

  pinMode(TOP, INPUT);
  pinMode(BOTTOM, INPUT);
  pinMode(RIGHT, INPUT);
  pinMode(LEFT, INPUT);

  attachPCINT(digitalPinToPCINT(TOP), limitSwitch, RISING);
  attachPCINT(digitalPinToPCINT(BOTTOM), limitSwitch, RISING);
  attachPCINT(digitalPinToPCINT(RIGHT), limitSwitch, RISING);
  attachPCINT(digitalPinToPCINT(LEFT), limitSwitch, RISING);
  analogWrite(E1, motor_power); analogWrite(E2, motor_power);


  draw_OL_Circle(Radius);

}


int miliMeter2miliSec(int miliMeter) {
  return (5000 / conversionFactor) * miliMeter;
}


// Drawing Open Loop Circle
void draw_OL_Circle (int R) {

  int longerLegnth_miliSec = miliMeter2miliSec(R * sqrt(0.5))*100;
  int shorterLength_miliSec = miliMeter2miliSec(R - R * sqrt(0.5))*100;

  int mili_second_tracker = 0;
  int motor_power_tracker = 0;
  int longerLength_incrementing_value = motor_power / longerLegnth_miliSec;
  int shorterLength_incrementing_value = motor_power / shorterLength_miliSec;

  Up ();  analogWrite(E1, motor_power); analogWrite(E2, motor_power);

  
  Serial.println(mili_second_tracker);
  Serial.println(shorterLength_miliSec);
  // ------------------ Top Left Quadrant ------------------
  while (mili_second_tracker < shorterLength_miliSec) {
    motor_power_tracker = motor_power_tracker - shorterLength_incrementing_value;
    analogWrite(E1, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis();
  Serial.println(mili_second_tracker);
  }
  mili_second_tracker = 0; motor_power_tracker = 0; // rESET Tracker variables
  Right ();
  while (mili_second_tracker < longerLegnth_miliSec) {
    motor_power_tracker = motor_power_tracker + shorterLength_incrementing_value;
    analogWrite(E1, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis();
  }
  mili_second_tracker = 0; motor_power_tracker = 0;


  // ------------------ Top right Quadrant ------------------
  while (mili_second_tracker < longerLegnth_miliSec) {
    motor_power_tracker = motor_power_tracker - shorterLength_incrementing_value;
    analogWrite(E2, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis(); // increment every 100 miliseconds
  }
  mili_second_tracker = 0; motor_power_tracker = 0;
  Down ();
  while (mili_second_tracker < shorterLength_miliSec) {
    motor_power_tracker = motor_power_tracker + shorterLength_incrementing_value;
    analogWrite(E2, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis();
  }
  mili_second_tracker = 0; motor_power_tracker = 0;

  // ------------------ Bottom right Quadrant ------------------
  while (mili_second_tracker < longerLegnth_miliSec) {
    motor_power_tracker = motor_power_tracker - shorterLength_incrementing_value;
    analogWrite(E1, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis(); 
  }
  mili_second_tracker = 0; motor_power_tracker = 0;
  Left ();
  while (mili_second_tracker < shorterLength_miliSec) {
    motor_power_tracker = motor_power_tracker + shorterLength_incrementing_value;
    analogWrite(E1, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis();
  }
  mili_second_tracker = 0; motor_power_tracker = 0;


  // ------------------ Bottom Left Quadrant ------------------
  while (mili_second_tracker < shorterLength_miliSec) {
    motor_power_tracker = motor_power_tracker - shorterLength_incrementing_value;
    analogWrite(E2, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis();
  }
  mili_second_tracker = 0; motor_power_tracker = 0;
  Left ();
  while (mili_second_tracker < longerLegnth_miliSec) {
    motor_power_tracker = motor_power_tracker + shorterLength_incrementing_value;
    analogWrite(E2, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis();
  }
  mili_second_tracker = 0; motor_power_tracker = 0;
  
}



// Simple Motor Movement
void Left () {
  Serial.println("Left");

  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
}
void Right () {
  Serial.println("Right");

  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
}
void Up () {
  Serial.println("Up");

  digitalWrite(M1, LOW);
  digitalWrite(M2, HIGH);
}
void Down () {
  Serial.println("Down");

  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);
}


// Motor diagonal movements
void Dg_RU () {
  digitalWrite(M1, 0);
  digitalWrite(M2, HIGH);
}
void Dg_RD () {
  digitalWrite(M1, HIGH);
  digitalWrite(M2, 0);
}
void Dg_LU () {
  digitalWrite(M1, LOW);
  digitalWrite(M2, 0);
}
void Dg_LD () {
  digitalWrite(M1, 0);
  digitalWrite(M2, LOW);
}


void limitSwitch() // handle pin change interrupt for D8 to D13 here
{
  hit_Limit_Switch = true;
  if (activateLimitSwitch == true) {
    analogWrite(E1, 0);
    analogWrite(E2, 0);
    while(true){   }
  }
}

void loop (){}



Fixed millis()
#include "PinChangeInterrupt.h"

#define LEFT 13 // PCINT pin
#define RIGHT 12 // PCINT pin
#define BOTTOM 11 // PCINT pin
#define TOP 10 // PCINT pin

// Motor Speed and Direction Pin Setup
#define E1 5 // Left Motor - Speed
#define M1 4 // Left Motor - Direction

#define E2 6 // right Motor - Speed
#define M2 7 // right Motor - Direction


// Conversion Parameters
#define motor_power 100
#define conversionFactor 85

// Cicle Parameters
#define Radius 50 // radius of the circle in mm

// Limit Switch Variables
bool hit_Limit_Switch = false;
bool activateLimitSwitch = true;

void setup() {
  Serial.begin(9600);

  pinMode(TOP, INPUT);
  pinMode(BOTTOM, INPUT);
  pinMode(RIGHT, INPUT);
  pinMode(LEFT, INPUT);

  attachPCINT(digitalPinToPCINT(TOP), limitSwitch, RISING);
  attachPCINT(digitalPinToPCINT(BOTTOM), limitSwitch, RISING);
  attachPCINT(digitalPinToPCINT(RIGHT), limitSwitch, RISING);
  attachPCINT(digitalPinToPCINT(LEFT), limitSwitch, RISING);
  //Down(); analogWrite(E1, motor_power); analogWrite(E2, motor_power);

  draw_OL_Circle(Radius);

}


int miliMeter2miliSec(int miliMeter) {
  return (5000 / conversionFactor) * miliMeter;
}


// Drawing Open Loop Circle
void draw_OL_Circle (int R) {

  int longerLegnth_miliSec = miliMeter2miliSec(R * sqrt(0.5));
  int shorterLength_miliSec = miliMeter2miliSec(R - R * sqrt(0.5));

  unsigned long mili_second_tracker = 0;
  int motor_power_tracker = 0;
  int longerLength_incrementing_value = motor_power / (longerLegnth_miliSec/100);
  int shorterLength_incrementing_value = motor_power / (shorterLength_miliSec/100);

  Serial.println(mili_second_tracker);
  Serial.println(shorterLength_miliSec);
  Serial.println(shorterLength_incrementing_value);
  Up ();  analogWrite(E1, motor_power); analogWrite(E2, motor_power);


  // ------------------ Harded Coded Top Left Quadrant ------------------
  mili_second_tracker = millis();
  motor_power_tracker = motor_power;
  while (mili_second_tracker < 5000) {
    motor_power_tracker = motor_power_tracker - motor_power/5000;
    analogWrite(E2, motor_power_tracker);
    mili_second_tracker = millis();
  Serial.print(mili_second_tracker);
  Serial.print(" - ");
  Serial.println(motor_power_tracker);
  }
  mili_second_tracker = 0; motor_power_tracker = 0; // rESET Tracker variables
  Right ();
  while (mili_second_tracker < longerLegnth_miliSec) {
    motor_power_tracker = motor_power_tracker + shorterLength_incrementing_value;
    analogWrite(E2, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis();
  }
  mili_second_tracker = 0; motor_power_tracker = 0;

  
  // ------------------ Top Left Quadrant ------------------
  mili_second_tracker = millis();
  while (mili_second_tracker < shorterLength_miliSec) {
    motor_power_tracker = motor_power_tracker - shorterLength_incrementing_value;
    analogWrite(E1, motor_power_tracker);
    mili_second_tracker = millis();
  Serial.println(mili_second_tracker);
  }
  mili_second_tracker = 0; motor_power_tracker = 0; // rESET Tracker variables
  Right ();
  while (mili_second_tracker < longerLegnth_miliSec) {
    motor_power_tracker = motor_power_tracker + shorterLength_incrementing_value;
    analogWrite(E1, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis();
  }
  mili_second_tracker = 0; motor_power_tracker = 0;


  // ------------------ Top right Quadrant ------------------
  while (mili_second_tracker < longerLegnth_miliSec) {
    motor_power_tracker = motor_power_tracker - shorterLength_incrementing_value;
    analogWrite(E2, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis(); // increment every 100 miliseconds
  }
  mili_second_tracker = 0; motor_power_tracker = 0;
  Down ();
  while (mili_second_tracker < shorterLength_miliSec) {
    motor_power_tracker = motor_power_tracker + shorterLength_incrementing_value;
    analogWrite(E2, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis();
  }
  mili_second_tracker = 0; motor_power_tracker = 0;

  // ------------------ Bottom right Quadrant ------------------
  while (mili_second_tracker < longerLegnth_miliSec) {
    motor_power_tracker = motor_power_tracker - shorterLength_incrementing_value;
    analogWrite(E1, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis(); 
  }
  mili_second_tracker = 0; motor_power_tracker = 0;
  Left ();
  while (mili_second_tracker < shorterLength_miliSec) {
    motor_power_tracker = motor_power_tracker + shorterLength_incrementing_value;
    analogWrite(E1, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis();
  }
  mili_second_tracker = 0; motor_power_tracker = 0;


  // ------------------ Bottom Left Quadrant ------------------
  while (mili_second_tracker < shorterLength_miliSec) {
    motor_power_tracker = motor_power_tracker - shorterLength_incrementing_value;
    analogWrite(E2, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis();
  }
  mili_second_tracker = 0; motor_power_tracker = 0;
  Left ();
  while (mili_second_tracker < longerLegnth_miliSec) {
    motor_power_tracker = motor_power_tracker + shorterLength_incrementing_value;
    analogWrite(E2, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis();
  }
  mili_second_tracker = 0; motor_power_tracker = 0;
  
}



// Simple Motor Movement
void Left () {
  Serial.println("Left");

  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
}
void Right () {
  Serial.println("Right");

  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
}
void Up () {
  Serial.println("Up");

  digitalWrite(M1, LOW);
  digitalWrite(M2, HIGH);
}
void Down () {
  Serial.println("Down");

  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);
}


// Motor diagonal movements
void Dg_RU () {
  digitalWrite(M1, 0);
  digitalWrite(M2, HIGH);
}
void Dg_RD () {
  digitalWrite(M1, HIGH);
  digitalWrite(M2, 0);
}
void Dg_LU () {
  digitalWrite(M1, LOW);
  digitalWrite(M2, 0);
}
void Dg_LD () {
  digitalWrite(M1, 0);
  digitalWrite(M2, LOW);
}


void limitSwitch() // handle pin change interrupt for D8 to D13 here
{
  hit_Limit_Switch = true;
  if (activateLimitSwitch == true) {
    analogWrite(E1, 0);
    analogWrite(E2, 0);
    while(true){   }
  }
}

void loop (){}



Slanted Circle:

#include "PinChangeInterrupt.h"

#define LEFT 13 // PCfloat pin
#define RIGHT 12 // PCfloat pin
#define BOTTOM 11 // PCfloat pin
#define TOP 10 // PCfloat pin

// Motor Speed and Direction Pin Setup
#define E1 5 // Left Motor - Speed
#define M1 4 // Left Motor - Direction

#define E2 6 // right Motor - Speed
#define M2 7 // right Motor - Direction


// Conversion Parameters
#define motor_power 100.0
#define conversionFactor 85.0

// Cicle Parameters
#define Radius 50.0 // radius of the circle in mm

// Limit Switch Variables
bool hit_Limit_Switch = false;
bool activateLimitSwitch = true;

void setup() {
  Serial.begin(9600);

  pinMode(TOP, INPUT);
  pinMode(BOTTOM, INPUT);
  pinMode(RIGHT, INPUT);
  pinMode(LEFT, INPUT);

  attachPCINT(digitalPinToPCINT(TOP), limitSwitch, RISING);
  attachPCINT(digitalPinToPCINT(BOTTOM), limitSwitch, RISING);
  attachPCINT(digitalPinToPCINT(RIGHT), limitSwitch, RISING);
  attachPCINT(digitalPinToPCINT(LEFT), limitSwitch, RISING);
  //Down(); analogWrite(E1, 150); analogWrite(E2, 150);
//Up(); analogWrite(E1, 0); analogWrite(E2, 150); delay(100000);
  draw_OL_Circle(Radius);

}


float miliMeter2miliSec(float miliMeter) {
  return (5000 / conversionFactor) * miliMeter;
}


// Drawing Open Loop Circle
void draw_OL_Circle (float R) {

  float long_ms = miliMeter2miliSec(R * sqrt(0.5));
  float short_ms = miliMeter2miliSec(R - R * sqrt(0.5));

  unsigned long mili_second_tracker = 0;
  float motor_power_tracker = 0;
  float long_inc_val = motor_power / (long_ms /100);
  float short_inc_val = motor_power / (short_ms /100);

  Serial.println(mili_second_tracker);
  Serial.println(short_ms );
  Serial.println(short_inc_val );
  Up ();  analogWrite(E1, motor_power); analogWrite(E2, motor_power);


  // ------------------ Harded Coded Top Left Quadrant ------------------
  mili_second_tracker = millis();
  motor_power_tracker = motor_power;
  while (mili_second_tracker < 3000) {
    motor_power_tracker = motor_power_tracker - 0.5;
    analogWrite(E1, motor_power_tracker);
    mili_second_tracker = millis();
  Serial.print(mili_second_tracker);
  Serial.print(" - ");
  Serial.println(motor_power_tracker);
  }
  mili_second_tracker = 0; motor_power_tracker = 0; // rESET Tracker variables
  Right ();
  while (mili_second_tracker < long_ms ) {
    motor_power_tracker = motor_power_tracker + short_inc_val ;
    analogWrite(E1, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis();
  }
  mili_second_tracker = 0; motor_power_tracker = 0;

  
//  // ------------------ Top Left Quadrant ------------------
//  mili_second_tracker = millis();
//  while (mili_second_tracker < short_ms ) {
//    motor_power_tracker = motor_power_tracker - short_inc_val ;
//    analogWrite(E1, motor_power_tracker);
//    mili_second_tracker = millis();
//  Serial.println(mili_second_tracker);
//  }
//  mili_second_tracker = 0; motor_power_tracker = 0; // rESET Tracker variables
//  Right ();
//  while (mili_second_tracker < long_ms ) {
//    motor_power_tracker = motor_power_tracker + short_inc_val ;
//    analogWrite(E1, motor_power_tracker);
//    mili_second_tracker = mili_second_tracker + millis();
//  }
//  mili_second_tracker = 0; motor_power_tracker = 0;


  // ------------------ Top right Quadrant ------------------
  while (mili_second_tracker < long_ms ) {
    motor_power_tracker = motor_power_tracker - short_inc_val ;
    analogWrite(E2, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis(); // increment every 100 miliseconds
  }
  mili_second_tracker = 0; motor_power_tracker = 0;
  Down ();
  while (mili_second_tracker < short_ms ) {
    motor_power_tracker = motor_power_tracker + short_inc_val ;
    analogWrite(E2, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis();
  }
  mili_second_tracker = 0; motor_power_tracker = 0;

  // ------------------ Bottom right Quadrant ------------------
  while (mili_second_tracker < long_ms ) {
    motor_power_tracker = motor_power_tracker - short_inc_val ;
    analogWrite(E1, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis(); 
  }
  mili_second_tracker = 0; motor_power_tracker = 0;
  Left ();
  while (mili_second_tracker < short_ms ) {
    motor_power_tracker = motor_power_tracker + short_inc_val ;
    analogWrite(E1, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis();
  }
  mili_second_tracker = 0; motor_power_tracker = 0;


  // ------------------ Bottom Left Quadrant ------------------
  while (mili_second_tracker < short_ms ) {
    motor_power_tracker = motor_power_tracker - short_inc_val ;
    analogWrite(E2, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis();
  }
  mili_second_tracker = 0; motor_power_tracker = 0;
  Left ();
  while (mili_second_tracker < long_ms ) {
    motor_power_tracker = motor_power_tracker + short_inc_val ;
    analogWrite(E2, motor_power_tracker);
    mili_second_tracker = mili_second_tracker + millis();
  }
  mili_second_tracker = 0; motor_power_tracker = 0;
  
}



// Simple Motor Movement
void Left () {
  Serial.println("Left");

  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
}
void Right () {
  Serial.println("Right");

  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
}
void Up () {
  Serial.println("Up");

  digitalWrite(M1, LOW);
  digitalWrite(M2, HIGH);
}
void Down () {
  Serial.println("Down");

  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);
}


// Motor diagonal movements
void Dg_RU () {
  digitalWrite(M1, 0);
  digitalWrite(M2, HIGH);
}
void Dg_RD () {
  digitalWrite(M1, HIGH);
  digitalWrite(M2, 0);
}
void Dg_LU () {
  digitalWrite(M1, LOW);
  digitalWrite(M2, 0);
}
void Dg_LD () {
  digitalWrite(M1, 0);
  digitalWrite(M2, LOW);
}


void limitSwitch() // handle pin change interrupt for D8 to D13 here
{
  hit_Limit_Switch = true;
  if (activateLimitSwitch == true) {
    analogWrite(E1, 0);
    analogWrite(E2, 0);
    while(true){   }
  }
}

void loop (){}





