#include "PinChangeInterrupt.h"
 
// Limit switch
#define LEFT 13 // PCINT pin
#define RIGHT 12 // PCINT pin
#define BOTTOM 11 // PCINT pin
#define TOP 10 // PCINT pin
 
// Motor speed and direction for PWM
#define E1 5 // Left Motor - Speed 
#define M1 4 // Left Motor - Direction (HIGH CC, LOW CW)
#define E2 6 // Right Motor - Speed
#define M2 7 // Right Motor - Direction (HIGH CC, LOW CW)
 
// Motor encoders
#define L_Enc_A 3 // Purple
#define L_Enc_B 2 // Grey
#define R_Enc_A 18 // White
#define R_Enc_B 19 // Black
 
// Circle measurements
#define xC  40.0
#define yC  0
#define D  80.0
 
// Rectangle measurements
#define xR 0
#define yR 0
#define W  60
#define L  90
 
// PID tuning values for rectangles
float kp = 0.05;
float ki = 0.0002;
float kd = 0.0001; //0.0001
 
// Motor power
int MotorPower = 150;
int basePower = 80;
 
// Initialise encoder counter variable
volatile int L_count = 0;
volatile int R_count = 0;
 
// bool hit_Limit_Switch = false;
volatile bool safe = true;
volatile bool isGoing = true;
 
void setup() {
  // Limit switch
  pinMode(TOP, INPUT);
  pinMode(BOTTOM, INPUT);
  pinMode(RIGHT, INPUT);
  pinMode(LEFT, INPUT);
  attachPCINT(digitalPinToPCINT(TOP), INT_Button, RISING);
  attachPCINT(digitalPinToPCINT(BOTTOM), INT_Button, RISING);
  attachPCINT(digitalPinToPCINT(RIGHT), INT_Button, RISING);
  attachPCINT(digitalPinToPCINT(LEFT), INT_Button, RISING);
 
  // Encoder
  pinMode(L_Enc_A, INPUT);
  pinMode(L_Enc_B, INPUT);
  pinMode(R_Enc_A, INPUT);
  pinMode(R_Enc_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(L_Enc_A), read_Left_Encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(R_Enc_A), read_Right_Encoder, RISING);
 
//  Right();
//   delay(2000);     
//Up();
//     delay(2000);
//// 
//  Reset();
//  delay(500);
//  Stop();
//  delay(5000);
//  
}
 
void loop() {
  // Only once and if it is safe
  while (isGoing && safe) {
    //Go to bottom left corner
 // Reset();
  //delay(500);
//  Up();
//     delay(2000);
//  Reset();
//  delay(500);
//  Stop();
//  delay(5000);
  
    L_count = 0;
    R_count = 0;
 
    // Go to starting position of the rectangle
    MoveTo(xR, yR + L);
    Stop();
    delay(300);
    MoveTo(xR + W, yR + L);
    Stop();
    delay(300);
    MoveTo(xR + W, yR);
    Stop();
    delay(300);
    MoveTo(xR, yR);
    Stop();
    delay(300);
 
    basePower = 200;
    MoveTo(100, 50);
    delay(300);


 //New PID values for circle
    kp = 0.0005;//0.005; //0.005
    kd = 0.08;//0.008; //0.008
    ki = 0.01;//0.08; //0.001
    basePower = 200;
 
    L_count = 0;
    R_count = 0;
 
    float angle_delta = (float)(PI) / 360.0;
    for (float angle = 0.0; angle <= 2.0 * PI; angle += angle_delta)
 
    {
      float xx = xC - (D / 2) * cos (angle);
      float yy = yC + (D / 2) * sin (angle);
      MoveTo(xx, yy);
    }
 
    delay(200);
    Stop();
    isGoing = false;
  }
}
 
void MoveTo(float xTarget, float yTarget) {
  // Initialising variables
  float currT = 0;
  float prevT = 0;
  float deltaT;
  float L_error = 0;
  float R_error = 0;
  float L_error_prev = 0;
  float R_error_prev = 0;
  float L_pwr = 0;
  float R_pwr = 0;
  float L_eintegral = 0;
  float R_eintegral = 0;
 
  // Calculating the desired change in A and B
  float A = xTarget + yTarget;
  float B = xTarget - yTarget;
 
  // Calculating the desired encoder count
  float L_count_desired = mm_to_counts(A) * -1;
  float R_count_desired = mm_to_counts(B) * -1;
 
  bool isSteadyState = false;
 
  while (!isSteadyState && safe) {
    // Calculating the time difference
    currT = micros();
    deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
    prevT = currT;
 
    // Calculating the error
    L_error = L_count_desired - L_count;
    R_error = R_count_desired - R_count;
 
    // Calculating the derivative term
    float L_dedt = (L_error - L_error_prev) / (deltaT);
    float R_dedt = (R_error - R_error_prev) / (deltaT);
 
    // Calculating the integral term
    L_eintegral = L_eintegral + L_error * deltaT;
    R_eintegral = R_eintegral + R_error * deltaT;
 
    // Calculating the control signal
    float L_u = kp * L_error + kd * L_dedt + ki * L_eintegral;
    float R_u = kp * R_error + kd * R_dedt + ki * R_eintegral;
 
    // Calculating the power
    L_pwr = fabs(L_u) + basePower;
    R_pwr = fabs(R_u) + basePower;
 
    // Saturating the power
    if ( L_pwr > 255 ) {
      L_pwr = 255;
    }
    if ( R_pwr > 255 ) {
      R_pwr = 255;
    }
 
    // Sending power to the motors
    setMotor(L_error, L_pwr, 1);
    setMotor(R_error, R_pwr, 2);
 
    // Storing the previous error
    L_error_prev = L_error;
    R_error_prev = R_error;
 
    // Checking if steady state has been reached
    if ((fabs(L_error) < 100) && (fabs(R_error) < 100)) {
      isSteadyState = true;
    }
  }
}
 
void setMotor(int error, int power, int LR) {
  // Control the left motor
  if (LR == 2) {
    if (error < 0) {
      digitalWrite(M2, HIGH);
    } else {
      digitalWrite(M2, LOW);
    }
    analogWrite(E2, power);
  } else {
    if (error < 0) {
      digitalWrite(M1, HIGH);
    } else {
      digitalWrite(M1, LOW);
    }
    analogWrite(E1, power);
  }
 
}
 
float mm_to_counts (float mm) {
  float counts = (mm * 2064.0) / (14.0 * PI);
  return counts;
}
 
void read_Left_Encoder() {
  int b = digitalRead(L_Enc_B);
  if (b > 0) {
    L_count++;
  }
  else {
    L_count--;
  }
}
 
void read_Right_Encoder() {
  int b = digitalRead(R_Enc_B);
  if (b > 0) {
    R_count++;
  }
  else {
    R_count--;
  }
}
 
// If the limit switch is pressed set stop motors and indicate it is not safe to move
void INT_Button()
{
  safe = false;
  Stop();
}
 
// Bring Pencil to (0,0) Reset Coordinate
void Reset() {
  // Down --> stop (2) --> up (1) --> stop (2) --> left --> stop (2) --> right (1)
  // Serial.println("Reset");
  // Serial.println("Going down until limit switch hit");
  while (safe) {
    Down();
    delay(10);
  }
  // Serial.println("Finished going down stopping for 3 seconds");
  delay(700);
  // Serial.println("Going up for 2 seconds");
  Up();
  delay(700);
  Stop();
  // Serial.println("Finished going up stopping for 3 second");
  delay(700);
  safe = true;
  // Serial.println("Going left until limit switch hit");
  while (safe) {
    Left();
    delay(10);
  }
  // Serial.println("Finished going left stopping for 3 seconds");
  delay(700);
  // Serial.println("Going right for 2 seconds");
  Right();
  delay(700);
  Stop();
  safe = true;
 
  L_count = 0;
  R_count = 0;
}
 
void Left () {
  digitalWrite(M1, LOW); // CW
  digitalWrite(M2, LOW); // CW
  analogWrite(E1, MotorPower);
  analogWrite(E2, MotorPower);
}
void Right () {
  digitalWrite(M1, HIGH); // CC
  digitalWrite(M2, HIGH); // CC
  analogWrite(E1, MotorPower);
  analogWrite(E2, MotorPower);
}
void Down () {
  digitalWrite(M1, LOW); // CW
  digitalWrite(M2, HIGH); // CC
  analogWrite(E1, MotorPower);
  analogWrite(E2, MotorPower);
}
void Up () {
  digitalWrite(M1, HIGH); // CC
  digitalWrite(M2, LOW); // CW
  analogWrite(E1, MotorPower);
  analogWrite(E2, MotorPower);
}
 
void Stop() {
  analogWrite(E1, 0);
  analogWrite(E2, 0);
}
 
 
 
 
 
 
 
 
 
 
 


