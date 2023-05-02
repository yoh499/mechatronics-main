//Drawing A Rectangle Using Timer/Counter2 


#include <avr/interrupt.h>
#include <Arduino.h>


//Initialise value of the next set of x, y coordinates to 0
int timerCount = 0;
long timeToMove = 0.0;
bool isDrawing = true;
int distanceY = 40;
int distanceX = 50;


// Motor Speed and Direction Pin Setup
#define E1 5 // Left Motor - Speed
#define M1 4 // Left Motor - Direction

#define E2 6 // Right Motor - Speed
#define M2 7 // Right Motor - Direction


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


void setup() {

  cli();//stop interrupts

  //Set timer2 interrupt
  TCCR2B = 0;//Set timer2 to 0
  TCNT2  = 0;//Initialise counter value to 0

  //Enable CTC mode

  TCCR0B |= (1 << WGM02);
  TCCR2B |= (1 << WGM01);

  //Set compare match register to count every 1 ms
  //Set CS22 bits for 64 prescaler

  TCCR2B |= (1 << CS22);
  OCR2A = 250;// = 1/ ((16*10^6) / (64)) * 250 = 1 ms

  //Enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  sei();//allow interrupts
}
ISR(TIMER2_COMPA_vect) { //timer2 interrupt
  //delay (200);
  if (timerCount < OCR2A) {
    timerCount++;
  }
}


void loop() {

  if (isDrawing == true) {
    timeToMove = (distanceY * 5000) / 84.5;
    while (timerCount <= timeToMove) { // 40 mm upwards
      Up();
    }
    timerCount = 0;
    analogWrite(E1, 0); // Left Motor STOP
    analogWrite(E2, 0); // Right Motor STOP
    timeToMove = (distanceX * 5000) / 84.5;

    while (timerCount <= timeToMove) { // 50 mm length
      Right();
    }
    timerCount = 0;
    analogWrite(E1, 0); // Left Motor STOP
    analogWrite(E2, 0); // Right Motor STOP
    timeToMove = (distanceY * 5000) / 84.5;
    
    while (timerCount <= timeToMove) { // 40 mm downwards
      Down();
    }
    timerCount = 0;
    analogWrite(E1, 0); // Left Motor STOP
    analogWrite(E2, 0); // Right Motor STOP
    timeToMove = (distanceX * 5000) / 84.5;

    while (timerCount <= timeToMove) { //50 mm length
      Left();
    }
    timerCount = 0;
    analogWrite(E1, 0); // Left Motor STOP
    analogWrite(E2, 0); // Right Motor STOP

    isDrawing = false;

  }
}



Combined with limit switch and calculation 

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

// mm -> milisec



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


  //Set timer2 interrupt
  TCCR2B = 0;//Set timer2 to 0
  TCNT2  = 0;//Initialise counter value to 0

  //Enable CTC mode

  TCCR0B |= (1 << WGM02);
  TCCR2B |= (1 << WGM01);

  //Set compare match register to count every 1 ms
  //Set CS22 bits for 64 prescaler

  TCCR2B |= (1 << CS22);
  OCR2A = 250;// = 1/ ((16*10^6) / (64)) * 250 = 1 ms

  //Enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);



}

ISR(TIMER2_COMPA_vect) { //timer2 interrupt
  //delay (200);
  if (timerCount < OCR2A) {
    timerCount++;
  }
}

void getConversionFactor(int E) {
  conversionFactor = 85;
}


void draw_OL_Square(int H, int W) {

  
  Serial.print("draw_OL_Square ");

  int H_sec = miliMeter2miliSec(H);
  int W_sec = miliMeter2miliSec(W);
int randomVariable = 0;
  Up();
  while (timerCount <= H_sec) {
    randomVariable++;
  }
  analogWrite(E1, 0); analogWrite(E2, 0);
  Right();
  timerCount = 0;
  analogWrite(E1, E_value); analogWrite(E2, E_value);

  while (timerCount <= W_sec) {
    randomVariable++;
  }
  analogWrite(E1, 0); analogWrite(E2, 0);
  Down();
  timerCount = 0;
  analogWrite(E1, E_value); analogWrite(E2, E_value);
  while (timerCount <= H_sec) {
    randomVariable++;
  }
  analogWrite(E1, 0); analogWrite(E2, 0);
  Left();
  timerCount = 0;
  analogWrite(E1, E_value); analogWrite(E2, E_value);
  while (timerCount <= W_sec) {
    randomVariable++;
  }

  analogWrite(E1, 0); analogWrite(E2, 0);
}



int miliMeter2miliSec(int miliMeter) {
  return (5000 / conversionFactor) * miliMeter;
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




// Bring Pencil to (0,0) Reset Coordinate
void Rest() {

  Serial.println("Rest");
  Down();
  while (hit_Limit_Switch == false) {
    delay(10);
  }
  Serial.println("while done");
  activateLimitSwitch = false;
  delay(5000);
  Serial.println("delay");
  Up();
  analogWrite(E1, 80);
  analogWrite(E2, 80);
  delay(3000);
  analogWrite(E1, 0);
  analogWrite(E2, 0);
  delay(3000);

  // RESET
  activateLimitSwitch = true;
  hit_Limit_Switch = false;

  Left();
  analogWrite(E1, 80);
  analogWrite(E2, 80);
  while (hit_Limit_Switch == false) {
    delay(10);
  }
  activateLimitSwitch = false;
  delay(5000);
  Right();
  analogWrite(E1, 80);
  analogWrite(E2, 80);
  delay(3000);
  analogWrite(E1, 0);
  analogWrite(E2, 0);

  // RESET
  activateLimitSwitch = true;
  hit_Limit_Switch = false;


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



Using millis

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

// mm -> milisec



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

  
//Rest();
getConversionFactor(E_value);

draw_OL_Square(50, 30); // Input = (H, W) in mm

}


void getConversionFactor(int E) {
  conversionFactor = 85;
}


void draw_OL_Square(int H, int W) {

  Serial.print("draw_OL_Square ");

  int H_sec = miliMeter2miliSec(H);
  int W_sec = miliMeter2miliSec(W);

  Up(); delay(H_sec);
  
  analogWrite(E1, 0); analogWrite(E2, 0);
  Right();
  analogWrite(E1, E_value); analogWrite(E2, E_value);
  delay(W_sec);
  
  analogWrite(E1, 0); analogWrite(E2, 0);
  Down();
  analogWrite(E1, E_value); analogWrite(E2, E_value);
  delay(H_sec);
  
  analogWrite(E1, 0); analogWrite(E2, 0);
  Left();
  analogWrite(E1, E_value); analogWrite(E2, E_value);
  delay(W_sec);

  analogWrite(E1, 0); analogWrite(E2, 0);
}



int miliMeter2miliSec(int miliMeter) {
  return (5000 / conversionFactor) * miliMeter;
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




// Bring Pencil to (0,0) Reset Coordinate
void Rest() {

  Serial.println("Rest");
  Down();
  while (hit_Limit_Switch == false) {
    delay(10);
  }
  Serial.println("while done");
  activateLimitSwitch = false;
  delay(5000);
  Serial.println("delay");
  Up();
  analogWrite(E1, 80);
  analogWrite(E2, 80);
  delay(3000);
  analogWrite(E1, 0);
  analogWrite(E2, 0);
  delay(3000);

  // RESET
  activateLimitSwitch = true;
  hit_Limit_Switch = false;

  Left();
  analogWrite(E1, 80);
  analogWrite(E2, 80);
  while (hit_Limit_Switch == false) {
    delay(10);
  }
  activateLimitSwitch = false;
  delay(5000);
  Right();
  analogWrite(E1, 80);
  analogWrite(E2, 80);
  delay(3000);
  analogWrite(E1, 0);
  analogWrite(E2, 0);

  // RESET
  activateLimitSwitch = true;
  hit_Limit_Switch = false;


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





