#include <SoftwareSerial.h>
#include <Servo.h>
#include <avr/io.h>
#include <math.h>




Servo left_front_motor;   // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;    // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;   // create servo object to control Vex Motor Controller 29
Servo right_front_motor;  // create servo object to control Vex Motor Controller 29

Servo Pivot;

#define PivotPin 21

#define L1 7.6    //Lengths
#define L2 8.875  //Lengths


//motor pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;


//global variables
long duration = 0;  // variable for the duration of sound wave travel
int distance = 0;   // variable for the distance measurement


byte serialRead = 0;  //for control serial communication
int signalADC = 0;    // the read out signal in 0-1023 corresponding to 0-5v


//wireless communication variables
#define INTERNAL_LED 13
#define BLUETOOTH_RX 10   // Serial Data input pin
#define BLUETOOTH_TX 11   // Serial Data output pin
#define STARTUP_DELAY 10  // Seconds
#define LOOP_DELAY 2      // miliseconds
#define SAMPLE_DELAY 10   // miliseconds


#define OUTPUTMONITOR 0  // USB Serial Port
#define OUTPUTPLOTTER 0
#define OUTPUTBLUETOOTHMONITOR 1  // Bluetooth Serial Port
SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);


//ultrasonic global variables
#define echoPin A4  // attach pin A4 Arduino to pin Echo of HC-SR04
#define trigPin A5  //attach pin A5 Arduino to pin Trig of HC-SR04
#define fanPin 14    //attach pin 0 to fan




void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  BluetoothSerial.begin(9600);

  pinMode(trigPin, OUTPUT);  // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT);   // Sets the echoPin as an INPUT
  pinMode(INTERNAL_LED, OUTPUT);
  pinMode(fanPin, OUTPUT);

  Pivot.attach(PivotPin, 560, 2610);

  enable_motors();
}


int measuredistanceUltrasonic(void) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);  // Reads the echoPin, returns the sound wave travel time in microseconds
  distance = duration * 0.034 / 2;    // Calculating the distance and dividing by 2 (distance to and from)


  return distance;
}





void driveMaxDistance() {

  int angleOfMaxDistance = FindMaxDistanceAngle(360);
  openLoopTurn(angleOfMaxDistance);
  openLoopDrive(measuredistanceUltrasonic() / 2);  //might need work if cant do int/2
}







int FindMaxDistanceAngle(int angle) {
  int adjustFactor = 10;
  int motorPower = 200;
  int maxDistanceTime = 0;
  int maxDistanceObserved = 0;
  int currentDistance;




  if (angle < 0) {
    motorPower = -200;
  }


  int turningTimeMs = abs(angle) * adjustFactor;


  //enable motors?


  for (int i = 0; i < turningTimeMs; i++) {

    left_front_motor.writeMicroseconds(saturate(1500 + motorPower, 850, 2150));
    right_front_motor.writeMicroseconds(saturate(1500 - motorPower, 850, 2150));
    left_rear_motor.writeMicroseconds(saturate(1500 + motorPower, 850, 2150));
    right_rear_motor.writeMicroseconds(saturate(1500 - motorPower, 850, 2150));

    currentDistance = measuredistanceUltrasonic();

    if (currentDistance > maxDistanceObserved) {
      maxDistanceObserved = currentDistance;
      maxDistanceTime = i;
    }

    delay(1);
  }

  return maxDistanceTime / adjustFactor;
}






void openLoopDrive(int distance) {

  int adjustFactor = 10;
  int motorPower = 200;


  if (distance < 0) {
    motorPower = -200;
  }


  int turningTimeMs = abs(distance) * adjustFactor;


  enable_motors();

  for (int i = 0; i < turningTimeMs; i++) {


    left_front_motor.writeMicroseconds(saturate(1500 + motorPower, 850, 2150));
    right_front_motor.writeMicroseconds(saturate(1500 + motorPower, 850, 2150));
    left_rear_motor.writeMicroseconds(saturate(1500 + motorPower, 850, 2150));
    right_rear_motor.writeMicroseconds(saturate(1500 + motorPower, 850, 2150));


    delay(1);
  }

  disable_motors();
}



void openLoopTurn(int angle) {
  int adjustFactor = 10;
  int motorPower = 200;


  if (angle < 0) {
    motorPower = -200;
  }


  int turningTimeMs = abs(angle) * adjustFactor;

  enable_motors();

  for (int i = 0; i < turningTimeMs; i++) {


    left_front_motor.writeMicroseconds(saturate(1500 + motorPower, 850, 2150));
    right_front_motor.writeMicroseconds(saturate(1500 + motorPower, 850, 2150));
    left_rear_motor.writeMicroseconds(saturate(1500 + motorPower, 850, 2150));
    right_rear_motor.writeMicroseconds(saturate(1500 + motorPower, 850, 2150));

    delay(1);
  }
  disable_motors();
}




double saturate(double u, double min_val, double max_val) {
  return fmin(fmax(u, min_val), max_val);
}




void enable_motors() {
  left_front_motor.attach(left_front);    // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);      // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);    // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_front_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}




void disable_motors() {
  left_front_motor.attach(1500);   // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(1500);    // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(1500);   // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_front_motor.attach(1500);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}




void loop() {
  // put your main code here, to run repeatedly:



  // Serial.println(millis());
  // Pivot.write(0);
  //   delay(2000);
  // Serial.println(millis());
 digitalWrite(fanPin, HIGH);
 Pivot.write(90);
 delay(500);
 Pivot.write(135);
 delay(500);
 Pivot.write(90);
 delay(500);
 Pivot.write(45);
 delay(500);
 Pivot.write(90);
 delay(500);

  //int angle = FindMaxDistanceAngle(360);
  //Serial.println(angle);
  // openLoopTurn(-90);
  // delay(2000);
  // Serial.println("ecksdee");
  // openLoopTurn(90);
  // openLoopTurn(180);



}
