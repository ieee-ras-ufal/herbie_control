#include <Keyboard.h>
#include <LiquidCrystal.h>
#include <PID_v1.h>

#define LM_ENCODER_A 3 
#define LM_ENCODER_B 4
#define R 0.035 // Wheel radius in meters
#define N 397.2 // Encoder PPR (Pulse Per Rotation) defined on datasheet

/* Set Arduino Pins */
//motor_A
int IN1 = 6;
int IN2 = 5;
int pwmA = 10;

/* PID instance */
double setpoint, input, output;
double kp = 120;
double ki = 50;
double kd = 0;
double velocity = 0;

PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

//Auxiliar Variables
double Ts = 0.1;
volatile unsigned long leftCount = 0;

void setup() {
  setpoint = 1;
  input = 0;
  leftCount = 0;
  
  pinMode(LM_ENCODER_A, INPUT);
  pinMode(LM_ENCODER_B, INPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(pwmA,OUTPUT);

  myPID.SetOutputLimits(0,255);
  myPID.SetMode(AUTOMATIC);
  
  attachInterrupt(1, leftEncoderEvent, CHANGE);
  
  Serial.begin(9600);
}
 
void loop(){
  // Moves the wheels clockwise
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  
  /* Velocity calculus */ 
  // V = (2piR(n/N))/Ts
  // R is the wheel radiuns in meters
  // n is the rotation count in Ts seconds
  // N is the encoder pulse per rotation (PPR - defined on datasheet)
  
  velocity = 2*PI*R*leftCount/(N*Ts);
  input = velocity;
  myPID.Compute();
  analogWrite(pwmA, output);
  
  leftCount = 0;
  
  //Display output PWM result and current Velocity
  Serial.print("Output:" );
  Serial.println(output);
  Serial.print("Velocity: ");
  Serial.println(velocity);
  Serial.println();
  
  delay(Ts*1000);
}


// Left encoder event for the interrupt call
void leftEncoderEvent() {
  if (digitalRead(LM_ENCODER_A) == HIGH) {
    if (digitalRead(LM_ENCODER_B) == LOW) {
      leftCount++;
    } else {
      leftCount--;
    }
  } else {
    if (digitalRead(LM_ENCODER_B) == LOW) {
      leftCount--;
    } else {
      leftCount++;
    }
  }
}
