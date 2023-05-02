#define rpmSensor A1
#define dirSensor A0

int rpmLevel;
int dirLevel;
int prevRpmLevel = 0;
int prevDirLevel = 0;
int startTime = 0;
int finishTime = 0;
float deltaT = 0.00;
float rpm = 0.00;
int dir = 0;
double rpmSum = 0.0;
float rpmAverage = 0.0;
int rpmCounter = 0;

int b = 0; //reading the time for main loop to be run for 15s
int c = 0; //memory for the time in mainloop

float s = 0; //built-in encoder counts
float s_2;   //built-in encoder counts for RPM calculation for PI controler

float rpmm;  //rpm obtained each 5s from built-in encoder

int s1 = 0;  //built-in encoder chanel one outpot
int s2 = 0;  //built-in encoder chanel two outpot
int r = 0;   //repetition indicator for reading counts of bult-in encoder
int s2m = 0; //memory of built-in encoder chanel two outpot
int directionm = 0; //indicator for direction read by built-in encoder
int dirm;          //indicator for direction read by built-in encode
int RPM;           //Commanded RPM

int exitt = 0;     //mainloop exit condition





float ctrl;      //PI controller outpot
float kp = .4;   //proportional gain of PI controller
float ki = .01;  //integral gain of PI controller
float eri;       //integral of error of PI controller

int repc = 1;    //repetition condition of PI controller
int t0;          //memory of time for the Purpose of displaying the results
int repeat = 0;  //repeat indicator to only let the memory of time for the Purpose of displaying the results be updated once


void setup() {
  // put your setup code here, to run on
  Serial.begin(250000);                  //Baud rate of communication

  Serial.println("Enter the desired RPM.");

  while (Serial.available() == 0)
  {
    //Wait for user input
  }

  RPM = Serial.readString().toFloat(); //Reading the Input string from Serial port.
  if (RPM < 0)
  {
    analogWrite(3, 255);                //changing the direction of motor's rotation
  }
  RPM = abs(RPM);

  pinMode(rpmSensor, INPUT);
  pinMode(dirSensor, INPUT);

}








void loop() {



  b = millis();  //reading time
  c = b;         //storing the current time



  while ((b >= c) && (b <= (c + 15500)) && exitt == 0) //let the main loop to be run for 15s
  {




    if (b % 13 == 0 && repc == 1)             //PI controller
    {
      eri = ki * (RPM - rpmm) + eri;
      ctrl = 50 + kp * (RPM - rpmm) + eri;
      analogWrite(6, ctrl);
      repc = 0;
    }
    if (b % 13 == 1)
    {
      repc = 1;
    }

    calculatingSpeed();






    s1 = digitalRead(7);         //reading Chanel 1 of builtin encoder
    s2 = digitalRead(8);         //reading Chanel 2 of builtin encoder
    if (s1 != s2 && r == 0)
    {
      s = s + 1;  //counters for rpm that displyed every 5s
      s_2 = s_2 + 1; //counters for rpm that used in PI contoller
      r = 1;      // this indicator wont let this condition, (s1 != s2), to be counted until the next condition, (s1 == s2), happens
    }

    if (s1 == s2 && r == 1)
    {
      s = s + 1;                                            //counters for rpm that displyed every 5s
      s_2 = s_2 + 1;                                        //counters for rpm that used in PI contoller
      r = 0;                                                // this indicator wont let this condition, (sm1 == sm2), to be counted until the next condition, (sm1 != sm2), happens
    }




    b = millis();                                           //updating time
    if (b % 100 <= 1 && repeat == 0)
    {
      t0 = b;                                               //storing the current time once
      repeat = 1;
    }


    if (b % 100 == 0)
    {
      //Serial.print("time in ms: ");
      //Serial.print(b - t0);

      //Serial.print("  spontaneous speed from builtin encoder:  ");
      rpmm = (s_2 / (2 * 114)) * 600;                       //formulation for rpm in each 100ms for PI controller
      //Serial.println(rpmm);
      s_2 = 0;                                              //reseting the counters of PI controller rpm meter

      // Averaging the rpm every 100ms
      rpmCounter++;
      rpmSum = rpmSum + rpm;
      rpmAverage = rpmSum / rpmCounter;

      if ((b - t0) % 5000 == 0)
      {
        Serial.println();
        Serial.print("RPM from builtin encoder: ");
        Serial.println((s / (228)) * 12);                     //formula for rpm in each 5s

        Serial.print("RPM from optical quadrature encoder: ");
        Serial.println(rpmAverage);

        Serial.print("Error: ");
        Serial.println(rpmAverage - ((s / (228)) * 12));

        Serial.print("direction read by motor's sensor: ");
        if (dirm == 0) {
          Serial.print("CW");
        }
        else {
          Serial.print("CCW");
        }
        Serial.print("  ,   ");

        // Printing the direction read by the sensor
        Serial.print("direction read by sensor:  ");
        if (dir > 0) {
          Serial.println("CW");
        } else if (dir < 0) {
          Serial.println("CCW");
        }
        Serial.println();

        s = 0;
        directionm = 0;

        // Clearing the average variables after each 5s
        rpmSum = 0.0;
        rpmCounter = 0;
      }
      delay(1);
    }





    if ((s1 == HIGH) && (s2 == HIGH) && (s2m == LOW))       //reading the direction of motor by cheaking which chanel follows which
    {
      directionm = directionm + 1;
    }

    if ((s1 == LOW) && (s2 == LOW) && (s2m == HIGH))
    {
      directionm = directionm + 1;
    }



    s2m = s2;                                               //memory of the previous builtin encoder chanel 2



    if (directionm > 100)
    {
      dirm = 0;
    }
    if (directionm < 20)
    {
      dirm = 1;
    }



    b = millis();                                           //updating time

  }
  analogWrite(6, 0);                                      //turning off the motor
  exitt = 1;                                              //changing the exit condition to prevent the motor to run after 15s
}

// This function calculates the speed of the motor shaft
void calculatingSpeed() {

  // Getting the 2 sensor levels
  rpmLevel = Level(analogRead(rpmSensor), prevRpmLevel);
  dirLevel = Level(analogRead(dirSensor), prevDirLevel);

  // When the levels change (ie: one stripe has passed) calculate the speed
  if (prevRpmLevel != rpmLevel) {
    finishTime = millis();
    deltaT = (finishTime - startTime) / 1000.0;
    startTime = millis();
    rpm = 60.0 / (16.0 * deltaT);
  }

  // Determine direction of the motor shaft on the rising edge of the rpm sensor
  if ((prevRpmLevel == 0) && (rpmLevel == 1)) {
    // If the direction sensor is low then increment the direction (+ve = CW)
    if (dirLevel == 0) {
      dir++;

      // If the direction sensor is high then decrement the direction (-ve = CCW)
    } else {
      dir--;
    }
  }

  // Storing the levels
  prevRpmLevel = rpmLevel;
  prevDirLevel = dirLevel;
}

// This function returns the level (white = 1 or black = 0) detected by the sensor
int Level (int sig, int prevLevel) {
  // If the signal is above the minimum high threshold set the level as 1 (white)
  if (sig > 500) {
    return 1;

    // If the signal is below the maximum low threshold set the level as 0 (black)
  } else if (sig < 350) {
    return 0;
  }

  // If the signal is between the thresholds return the previous level
  return prevLevel;
}


