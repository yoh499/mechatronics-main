//-----------------------------------------
#include "math.h"


// IR-PT Sensor Pair
#define g0 A1 //LSB
#define g1 A2
#define g2 A3
#define g3 A4
#define g4 A5 //MSB


// Initialising Binary scale array
int Initial_Binary[5] = { -1, -1, -1, -1, -1};
int Current_Binary[5] = { -1, -1, -1, -1, -1};


// Initialising State value
int Initial_State = 0;
int Current_State = 0;


// Initialising Grey scale array
int Initial[5] = { -1, -1, -1, -1, -1};
int Previous[5] = { -1, -1, -1, -1, -1};
int Current[5] = { -1, -1, -1, -1, -1};


// Tracking number of times any of the sensors detect a change in colour
int stateTracker = 0;


// Storing 
float Angle;
float Direction; // CW = 1; CCW = -1 


float Resolution = 360 / pow(2,5); // 11.25 degrees for this project
//-----------------------------------------






int finish = 0; //finish indicator
int rep = 1;   //Repetition indicator




float deg = 45; // Rotation degree
float s = 0;  //Encoder counts
int sm1 = 0;  //Built-in channel 1
int sm2 = 0;  //Built-in channel 2
int r = 0;    //indicator for reading builtin encoder to avoid the reading redundancy
float er;     //Proportional error for PI controller
float eri;    //Integral error for PI controller








int t = 0;  //time in ms
int t0 = 0; //memory for time in ms












void setup() {


  Serial.begin(250000);                                                 //Baud rate of communication


  Serial.println("Enter the desired rotation in degree.");


  while (Serial.available() == 0)                                       //Obtaining data from user
  {
    //Wait for user input
  }


  deg = Serial.readString().toFloat(); //Reading the Input string from Serial port.
  if (deg < 0)
  {
    analogWrite(3, 255);                                                  //change the direction of rotation by applying voltage to pin 3 of arduino
  }
  deg = abs(deg);


  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);


}




float kp = .6 * 90 / deg;                     //proportional gain of PI
float ki = .02;                               //integral gain of PI












void loop() {
  // put your main code here, to run repeatedly:


// Reset Variable
  stateTracker = 0;


// Read the initial Greyscale. One threshold.
  Previous[0] =  AnalogToDigital(analogRead(g0));
  Previous[1] =  AnalogToDigital(analogRead(g1));
  Previous[2] =  AnalogToDigital(analogRead(g2));
  Previous[3] =  AnalogToDigital(analogRead(g3));
  Previous[4] =  AnalogToDigital(analogRead(g4));
  Initial[0] = Previous[0];
  Initial[1] = Previous[1];
  Initial[2] = Previous[2];
  Initial[3] = Previous[3];
  Initial[4] = Previous[4];
  






  t = millis();               //reading time
  t0 = t;                     //saving the current time in memory
  while (t < t0 + 4000 && rep <= 10)                                  //let the code to ran for 4 seconds each with repetitions of 10
  {


    if (t % 10 == 0)                                  //PI controller that runs every 10ms
    {
      if (s < deg * 114 * 2 / 360)
      {
        er = deg - s * 360 / 228;
        eri = eri + er;
        analogWrite(6, kp * er + ki * eri);
      }


      if (s >= deg * 228 / 360)
      {
        analogWrite(6, 0);
        eri = 0;
      }
      delay(1);




      // Continuously read the Grey scale. Two thresholds with previous reading as reference
      Current[0] =  ChangingLevel(analogRead(g0), Previous[0]);
      Current[1] =  ChangingLevel(analogRead(g1), Previous[1]);
      Current[2] =  ChangingLevel(analogRead(g2), Previous[2]);
      Current[3] =  ChangingLevel(analogRead(g3), Previous[3]);
      Current[4] =  ChangingLevel(analogRead(g4), Previous[4]);




      // Check if any of the IR-PT sensor pairs experienced a change in colour
      if (Previous[0] != Current[0] ||
      Previous[1] != Current[1] ||
      Previous[2] != Current[2] ||
      Previous[3] != Current[3] ||
      Previous[4] != Current[4] ) {
        // If so: increment the state tracking variable and update the Grey scale reading
        stateTracker++;
        Previous[0] = Current[0];
        Previous[1] = Current[1];
        Previous[2] = Current[2];
        Previous[3] = Current[3];
        Previous[4] = Current[4];
      }
    }






    sm1 = digitalRead(7);         //reading channel 1
    sm2 = digitalRead(8);         //reading channel 2








    if (sm1 != sm2 && r == 0) {                                      //counting the number changes for both chanels
      s = s + 1;
      r = 1;                                                         // this indicator wont let this condition, (sm1 != sm2), to be counted until the next condition, (sm1 == sm2), happens
    }
    if (sm1 == sm2 && r == 1) {
      s = s + 1;
      r = 0;                                                         // this indicator won't let this condition, (sm1 == sm2), to be counted until the next condition, (sm1 != sm2), happens
    }








    t = millis();         //updating time
    finish = 1;           //changing finish indicator


  }


  // Get the final angle and direction
  GetAngle();
  GetDirection();


  if (finish == 1) {                             //this part of the code is for displaying the result
    delay(500);                              //half second delay
    rep = rep + 1;                           // increasing the repetition indicator
 
    Serial.print("shaft displacement from optical absolute sensor: ");
    Serial.println(Angle * Direction);


    Serial.print("Shaft displacement from motor's builtin encoder: ");
    Serial.println(s * 360 / 228);                                      //every full Revolution of the shaft is associated with 228 counts of builtin
    //encoder so to turn it to degree we can use this formula (s * 360 / 228), "s" is the number of  built-in encoder counts


    float Error = Angle - s * 360 / 228;
    Serial.print("Error :");
    Serial.println(Error);                                              //displaying error
    Serial.println();
    s = 0;
    finish = 0;
  }
  analogWrite(6, 0);                                                        //turning off the motor
}




void GetAngle() {
  Angle = stateTracker * Resolution;
}




int ChangingLevel (int sig, int prevLevel) {
  if (sig > 400) {
    return 1;
  } else if (sig < 300) {
    return 0;
  }
  return prevLevel;
}




int AnalogToDigital (int sig) {
  if (sig > 450) {
    return 1;
  }
  return 0;
}


void ConvertToBinary() {
  Initial_Binary[4] = Initial[4];
  Initial_Binary[3] = Initial[4] ^ Initial[3];
  Initial_Binary[2] = Initial_Binary[3] ^ Initial[2];
  Initial_Binary[1] = Initial_Binary[2] ^ Initial[1];
  Initial_Binary[0] = Initial_Binary[1] ^ Initial[0];


  Current_Binary[4] = Current[4];
  Current_Binary[3] = Current[4] ^ Current[3];
  Current_Binary[2] = Current_Binary[3] ^ Current[2];
  Current_Binary[1] = Current_Binary[2] ^ Current[1];
  Current_Binary[0] = Current_Binary[1] ^ Current[0];
}




void getState () {
Initial_State = 0;
Current_State = 0;
  for (int i = 0; i < 5; i++) {
    float dummyVariable = (float(Initial_Binary[i]) * pow(2, i));
    Initial_State = dummyVariable + Initial_State;
  }
  for (int i = 0; i < 5; i++) {
    float dummyVariable = (float(Current_Binary[i]) * pow(2, i));
    Current_State = dummyVariable + Current_State;
  }
}




void GetDirection() {
  ConvertToBinary();
  getState ();


  float sign = Initial_State - Current_State;
  float difference = abs(sign);


  if (difference > 20) {
    if (sign < 0) { //Current > Reference
      Direction = -1; //CCW
    } else {
      Direction = 1; //CW
    }
  } else {
    if (sign < 0) { //Current > Reference
      Direction = 1; //CW
    } else {
      Direction = -1; //CCW
    }
  }
}










