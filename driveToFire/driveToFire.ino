#include <SoftwareSerial.h>
#include <Servo.h>
#include <avr/io.h>

int fanPin = 14;
int IRpin = A13;

volatile unsigned int gyro_timer = 0;  //timer variable
float T = 50;                     //Time period
float gyroSupplyVoltage = 5;      // supply voltage for gyro

void setup() {
  pinMode(14, OUTPUT);
  pinMode(IRpin, INPUT);
  Serial.begin(9600);



  //motor pins
  const byte left_front = 46;
  const byte left_rear = 47;
  const byte right_rear = 50;
  const byte right_front = 51;
  const byte PivotPin = 21;
  Servo Pivot;              // create servo object to control pivot servo


  Pivot.attach(PivotPin, 450, 2560);
  TCNT2 = 0;                // zero timer
  OCR2A = 249;              // Set CTC compare value with a prescaler of 64 = 100ms
  TCCR2A |= (1 << WGM21);   // Configure timer 2 for CTC mode
  TCCR2B |= (1 << CS22);    // Start timer at Fcpu/64
  TIMSK2 |= (1 << OCIE2A);  // Enable CTC interrupt


  sei();  // Enable global interrupts
}

void loop() {
  /*
    digitalWrite(14,HIGH);
    delay(500);
    digitalWrite(14,LOW);
    delay(500);
  */
  Serial.println(analogRead(IRpin));
  delay(100);
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


volatile unsigned int servo_timer = 0;  //timer variable

ISR(TIMER2_COMPA_vect) {
  gyro_timer++;
  if (gyro_timer >= T) {
    measure_gyro_angle();
    gyro_timer = 0;
  }

  servo_timer++;
  if (servo_timer >= 25) {
    sweep();
    servo_timer = 0;
  }
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
    right_front_motor.writeMicroseconds(saturate(1500 - motorPower, 850, 2150));
    left_rear_motor.writeMicroseconds(saturate(1500 + motorPower, 850, 2150));
    right_rear_motor.writeMicroseconds(saturate(1500 - motorPower, 850, 2150));


    delay(1);
  }

  //disable_motors();
}












int sweep = 0;
volatile float targetPos = 90;   // target position for the servo
volatile float currentPos = 90;   // current position of the servo
bool left, right, middle = false;

void sweep() {
  if (sweep) {
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
}


int fanOn() {
  digitalWrite(fanPin, HIGH);
  delay(5000);
  digitalWrite(fanPin, LOW);
}




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




int narrowServoSearch() {
  numSearches = 100;
  maxBrightness = 0;
  lastServoFireAngle = 90; //default fire direction


  Pivot.write(120);
  //servo(60);
  sweep = 1;
  for (x = 1; x < numSearches; x++) {

    //servo(60 + x); //SWEEP SERVO CODE GOES HERE
    //sweep();


    if ((analogRead(IRpin1) + analogRead(IRpin2)) > maxBrightness) {
      maxBrightness = (analogRead(IRpin1) + analogRead(IRpin2));
      lastServoFireAngle = servoAngle;                              //GLOBAL VARS?
    }
  }
  sweep = 0;

  Pivot.write(lastServoFireAngle);
  //servo(lastServoFireAngle);

  return lastServoFireAngle; //may not need
}

void driveToFire() {
  turn(currentHeading + ServoAngle - 90);
  servo(90);

  narrowServoSearch();

  turn(currentHeading + ServoAngle - 90);
  servo(90);

  startMotors();

  while (fireFound == 0) {
    narrowServoSearch();

    if (obstacleFlag) {
      break;
    }

    if ((lastServoFireAngle - 90) > 10) {
      openLoopAdjustCourse(currentHeading + ServoAngle - 90);
    }

    if ((lastServoFireAngle - 90) > 25) {
      stopMotors();
      //driveToFire();
    }

    if (fireDistance < 10) {
      fireFound = 1;
      narrowServoSearch();
      fanOn();
    }
  }
}
