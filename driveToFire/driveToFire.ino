#include <SoftwareSerial.h>
#include <Servo.h>
#include <avr/io.h>


//gyro globals
int sensorPin = A3;               //define the pin that gyro is connected
int sensorValue = 0;              // read out value of sensor
float T = 50;                     //Time period
float gyroSupplyVoltage = 5;      // supply voltage for gyro
float gyroZeroVoltage = 0;        // the value of voltage when gyro is zero
float gyroSensitivity = 0.007;    // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 2;      // because of gyro drifting, defining rotation angular velocity less than this value will not be ignored
float gyroRate = 0;               // read out value of sensor in voltage
volatile float currentAngle = 180;  // current angle calculated by angular velocity integral on
byte serialReadGyro = 0;          // for serial print control

volatile unsigned int gyro_timer = 0;  //timer variable


Servo left_front_motor;   // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;    // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;   // create servo object to control Vex Motor Controller 29
Servo right_front_motor;  // create servo object to control Vex Motor Controller 29
Servo Pivot;              // create servo object to control pivot servo
//motor pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;
const byte PivotPin = 21;


volatile float targetPos = 90;   // target position for the servo
volatile float currentPos = 90;   // current position of the servo
bool left, right, middle = false;

void sweep() {
  if (middle) {
    targetPos = 45;
  } else if (right) {
    targetPos = 135;
  } else if (left) {
    targetPos = 90;
  }

  if (currentPos != targetPos) {
    if (targetPos > currentPos) {
      currentPos = min(currentPos + 2, targetPos);  // increment position
    } else {
      currentPos = max(currentPos - 2, targetPos);  // decrement position
    }
    Pivot.write(currentPos);  // move servo to new position

  }
  else {
    left = (currentPos == 135);
    right = (currentPos == 45);
    middle = (currentPos == 90);
  }
  //Serial.println(currentPos);
}

int count;
ISR(TIMER2_COMPA_vect) {
  count++;
  if(count >= 200){
    measureIRs();
  }
  if(count >= 200){
    measurePTs();
    count = 0;
  }
  if(count%50 == 0){
    measure_gyro_angle();
  }
  if(count%25 == 0){
    sweep();
  }
}

double saturate(double u, double min_val, double max_val) {
  return fmin(fmax(u, min_val), max_val);
}





#define phototransistor_right_inner A10
#define phototransistor_right_outer A11
#define phototransistor_left_outer A12 
#define phototransistor_left_inner A13

int FIRE_FLAG = 0;
int fireThreshold = 4;

void measurePTs(){
  FIRE_FLAG = 0;
  
  int leftInner = analogRead(phototransistor_left_inner);
  int leftOuter = analogRead(phototransistor_left_outer);
  int rightInner = analogRead(phototransistor_right_inner);
  int rightOuter = analogRead(phototransistor_right_outer);

  if((leftInner > fireThreshold)||(leftOuter > fireThreshold) || (rightInner > fireThreshold)|| (rightOuter > fireThreshold)){
    FIRE_FLAG = 1;
  }
}


  #define PinIRLF A6
  #define PinIRLS A7
  #define PinIRRS A8 
  #define PinIRRF A9

//obstacle flags
  int IRLF_FLAG = 0;
  int IRLS_FLAG = 0;
  int IRRS_FLAG = 0;
  int IRRF_FLAG = 0;

  int IRLF = 0;
  int IRLS = 0;
  int IRRS = 0;
  int IRRF = 0;
  int OBSTACLE_FLAG = 0;
  
void measureIRs(){
  IRLF_FLAG = 0;
  IRLS_FLAG = 0;
  IRRS_FLAG = 0;
  IRRF_FLAG = 0;
  OBSTACLE_FLAG = 0;
  
  IRLF = analogRead(PinIRLF*function);
  IRLS = analogRead(PinIRLS*function);
  IRRS = analogRead(PinIRRS*function);
  IRRF = analogRead(PinIRRF*function);
  
  if(IRLF < 10){
    IRLF_FLAG = 1;
    OBSTACLE_FLAG = 1;
  }
  if(IRLS < 10){
    IRLS_FLAG = 1;
    OBSTACLE_FLAG = 1;
  }

  if(IRRS < 10){
    IRRS_FLAG = 1;
    OBSTACLE_FLAG = 1;
  }

  if(IRRF < 10){
    IRRF_FLAG = 1;
    OBSTACLE_FLAG = 1;
  }
}



void measure_gyro_angle() {  //Function calculates gyroscope direction in degrees

  gyroRate = (analogRead(sensorPin) * gyroSupplyVoltage) / 1023;  // convert the 0-1023 signal to 0-5v
  gyroRate -= (gyroZeroVoltage / 1023 * 5);                       // find the voltage offset the value of voltage when gyro is zero (still)
  float angularVelocity = gyroRate / gyroSensitivity;             // read out voltage divided the gyro sensitivity to calculate the angular velocity

  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) {  // if the angular velocity is less than the threshold, ignore it
    float angleChange = angularVelocity / (1000.00 / (T));                              // we are running a loop in T. one second will run (1000/T).
    currentAngle += angleChange;
  }

  if (currentAngle < 0) {  // keep the angle between 0-360
    currentAngle += 360;
  } else if (currentAngle > 359) {
    currentAngle -= 360;
  }

  //Serial.println(currentAngle);
}




void sweep() {
  if (middle) {
    targetPos = 45;
  } else if (right) {
    targetPos = 135;
  } else if (left) {
    targetPos = 90;
  }

  if (currentPos != targetPos) {
    if (targetPos > currentPos) {
      currentPos = min(currentPos + 1, targetPos);  // increment position
    } else {
      currentPos = max(currentPos - 1, targetPos);  // decrement position
    }
    Pivot.write(currentPos);  // move servo to new position

  }
  else {
    left = (currentPos == 135);
    right = (currentPos == 45);
    middle = (currentPos == 90);
  }
  Serial.println(currentPos);
}









//PT RELATED CODE BELOW THIS LINE//////////////////////////////////////////////////////////////////////////////

int fanPin = 14;
int fanOn() {
  digitalWrite(fanPin, HIGH);
  delay(5000);
  digitalWrite(fanPin, LOW);
}

int motorPower = 200;
int startMotors() {
  left_front_motor.writeMicroseconds(saturate(1500 + motorPower, 850, 2150));
  right_front_motor.writeMicroseconds(saturate(1500 - motorPower, 850, 2150));
  left_rear_motor.writeMicroseconds(saturate(1500 + motorPower, 850, 2150));
  right_rear_motor.writeMicroseconds(saturate(1500 - motorPower, 850, 2150));
}

int stopMotors() {
  left_front_motor.writeMicroseconds(1500);
  right_front_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
}



int PTpinIR = A10;
int PTpinOR = A11;
int PTpinOL = A12;
int PTpinIL = A13;
int sweepFlag = 1;

int narrowServoSearch() {
  int numSearches = 100;
  int maxBrightness = 0;
  int lastServoFireAngle = 90; //default fire direction
  

  Pivot.write(120);
  //servo(60);
  sweepFlag = 1;
  int x = 0;
  for (x = 1; x < numSearches; x++) {

    //servo(60 + x); //SWEEP SERVO CODE GOES HERE
    //sweep();


    if ((analogRead(PTpinIR) + analogRead(PTpinOR)) > maxBrightness) {
      maxBrightness = (analogRead(PTpinIR) + analogRead(PTpinOR));
      lastServoFireAngle = currentPos;                              //GLOBAL VARS?
    }
  }
  sweepFlag = 0;

  Pivot.write(lastServoFireAngle);
  //servo(lastServoFireAngle);

  return lastServoFireAngle; //may not need
}

int lastServoFireAngle = 0;

void driveToFire() {
  int fireDistance = 10;
  int fireFound = 0;
  
  turn(currentAngle -180 + currentPos - 90);
  servo(90);

  narrowServoSearch();

  turn(currentAngle -180 + currentPos - 90);
  servo(90);

  startMotors();

  while (fireFound == 0) {
    narrowServoSearch();

    if (OBSTACLE_FLAG) {
      break;
    }

    if ((lastServoFireAngle - 90) > 10) {
      openLoopAdjustCourse(currentAngle -180 + currentPos - 90);
    }

    if ((lastServoFireAngle - 90) > 25) {
      stopMotors();
      //driveToFire();
    }

    if (fireDistance < 10) {
      fireFound = 1;
      narrowServoSearch(); //SHOULD THIS BE MOED TO FireFight() Function (state D)
      fanOn();
    }
  }
}
