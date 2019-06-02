/***  ME 362 Hydrodynamics Lab (with LCD screen) ***

  - Reads and maps a potentiometer to control PWM and speed
    of the motor. 
  - Calculates RPM by counting the number of pulses from the motor encoder in
    durations of 50ms, then prints the average RPM over 10 readings on the LCD.

  The circuit:
  - Motor M2 and M1 connect to +- terminal on motor controller, respectively.
  - VCC and GND on motor encoder connect to 5V and GND on Arduino. 
  - C2 on encoder connects to digital pin 2 on Arduino.
  - E2 and M2 on motor controller connect to digital pins 6 and 7.
  - VD and VS both connect to positive power supply, sharing the same GND
  - 10k pot. to A0
  - LCD screen: rs = 8, en = 9, d4 = 10, d5 = 11, d6 = 12, d7 = 13;

 Henry Wu
 Feb 22, 2019
*/

#include <LiquidCrystal.h>

#define inputA 2                      //hall sensor
#define M2 7                          //motor dir. control
#define E2 6                          //motor pwm control
#define in A0

int pwm; 

int analogIn;

unsigned short pulseCount;   //14 per small shaft rev., 700 per main shaft rev.

bool nowState;                        //current state of hall sensor
bool lastState;                       //stores last hall sensor state to detect change

unsigned long timeA;                  //first point in time to calculate rpm
byte sampleDuration = 50;             //time between two smaple points (ms)
short tempPulseCount;                 //number of pulse during the period
float rpm;

float averageSum;                     //take average to smooth out data
byte averageCount;                    //count if 10 readings have been read
float averageRpm;

const int rs = 8, en = 9, d4 = 10, d5 = 11, d6 = 12, d7 = 13;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  
  pinMode(inputA, INPUT);
  pinMode(in, INPUT);
  pinMode(M2, OUTPUT);
  digitalWrite(M2, LOW);
  
  pulseCount = 0;
  averageCount = 1;

  lastState = digitalRead(inputA);                        //read current state of hall sensor
}

void loop() {
  //SET MOTOR SPEED

  /*
  if(Serial.available() > 0){                             //get input for pwm
    pwm = Serial.parseInt();
    analogWrite(E2, pwm);
  }
  */

  analogIn = analogRead(in);
  pwm = map(analogIn, 0, 1023, 0, 255);
  analogWrite(E2, pwm);
  

  // READ HALL PULSES
  nowState = digitalRead(inputA);                         //get current state

  if (nowState != lastState)                              //if current and last state is different, a change is registered
    pulseCount++;

  if (pulseCount > 700)                                   //700 pulses are counted, one main shaft rotation++
    pulseCount = 0;                                       //reset pulse count

  // RPM CALC
  if (millis() > timeA + sampleDuration) {                      //second point in time is reached
    tempPulseCount = pulseCount - tempPulseCount;               //how many pulses in interval
    
    if (tempPulseCount >= 0)                                    //filters out bad readings after resetting at 700 
      rpm = (tempPulseCount / (float)sampleDuration) * 85.714;
      
    // RESET TIME AND PULSE COUNT
    timeA = millis();
    tempPulseCount = pulseCount;

    // AVERAGE RPM CALC
    if (averageCount <= 10) {
      averageSum = averageSum + rpm;
      averageCount++;
    }
    else {
      lcd.clear();
      
     //Serial.print("\nPWM: ");
     //Serial.println(pwm);

      lcd.home();
      lcd.print("PWM=");
      lcd.print(pwm);
      
      averageRpm = averageSum / 10;
      
      //Serial.print("RPM: ");
      //Serial.println(averageRpm);

      lcd.setCursor(0, 1);
      lcd.print("RPM=");
      lcd.print(averageRpm);
      
      averageSum = 0;
      averageCount = 1;
    }
  }
  lastState = nowState;   // current state is now last state
}
 
