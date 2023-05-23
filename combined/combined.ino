#include <SoftwareSerial.h>
#include <Servo.h>
#include <avr/io.h>
#include <math.h>

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

byte serialRead = 0;  //for control serial communication
int signalADC = 0;    // the read out signal in 0-1023 corresponding to 0-5v


//ultrasonic global variables
#define echoPin A4  // attach pin A4 Arduino to pin Echo of HC-SR04
#define trigPin A5  //attach pin A5 Arduino to pin Trig of HC-SR04
#define fanPin 14    //attach pin 0 to fan
long duration = 0;  // variable for the duration of sound wave travel
int distance = 0;   // variable for the distance measurement



//length variables for kinematics
#define L1 7.6
#define L2 8.875



//Phototransistor related globals
#define phototransistor_right_inner A10
#define phototransistor_right_outer A11
#define phototransistor_left_outer A12 
#define phototransistor_left_inner A13

int FIRE_FLAG = 0;
int fireThreshold = 14;


//IR RELATED GLOBALS
  #define PinIRLF A6
  #define PinIRLS A7
  #define PinIRRS A8 
  #define PinIRRF A9

  int IRLF_FLAG = 0;
  int IRLS_FLAG = 0;
  int IRRS_FLAG = 0;
  int IRRF_FLAG = 0;
  int SONAR_FLAG = 0;

  int IRLF = 0;
  int IRLS = 0;
  int IRRS = 0;
  int IRRF = 0;
  int SONAR = 0;
  int OBSTACLE_FLAG = 0;

//Movement related variables
volatile float error_theta = 0;
float kp_theta = 5; //  float kp_theta = 1.75, ki_theta = 0.15, kd_theta = 0.40;
#define LEFT -1
#define RIGHT 1


void setup(){

  BluetoothSerial.begin(9600);

  pinMode(trigPin, OUTPUT);  // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT);   // Sets the echoPin as an INPUT

  pinMode(sensorPin, INPUT);  // Sets the gyroscope pin as INPUT
  Pivot.attach(PivotPin, 450, 2560);

  
  pinMode(INTERNAL_LED, OUTPUT);
  pinMode(fanPin, OUTPUT);

  Pivot.attach(PivotPin, 560, 2610);

  cli();

  
  Serial.begin(9600);
  Serial.println("Calibrating, keep gyroscope still.");
  Serial.println("Getting gyro zero voltage");

  float sum = 0;                 // variables for gyroscope calibration
  for (int j = 0; j < 100; j++)  // read 100 values of voltage when gyro is at still, to calculate the zero-drift
  {
    sensorValue = analogRead(sensorPin);
    sum += sensorValue;
    delay(5);
  }
  gyroZeroVoltage = sum / 100;  // average the sum as the zero drifting



  
  TCNT2 = 0;                // zero timer
  OCR2A = 249;              // Set CTC compare value with a prescaler of 64 = 100ms
  TCCR2A |= (1 << WGM21);   // Configure timer 2 for CTC mode
  TCCR2B |= (1 << CS22);    // Start timer at Fcpu/64
  TIMSK2 |= (1 << OCIE2A);  // Enable CTC interrupt
  sei();  // Enable global interrupts


  pinMode(phototransistor_right_inner, INPUT);
  pinMode(phototransistor_left_inner, INPUT);
  pinMode(phototransistor_right_outer, INPUT);
  pinMode(phototransistor_left_outer, INPUT);


  enable_motors();
  delay(1000);
}

//////////////////////SERIAL RELATED CODE/////////////////////////////////////////////////////////////////////////////////////////

void serialOutputMonitor(int32_t Value1, int32_t Value2, int32_t Value3) {
  String Delimiter = ", ";

  Serial.print(Value1, DEC);
  Serial.print(Delimiter);
  Serial.print(Value2, DEC);
  Serial.print(Delimiter);
  Serial.println(Value3, DEC);
}

void serialOutput(int32_t Value1, int32_t Value2, int32_t Value3) {  //only for plotting - to turn on change OUTPUTMONITOR = 1
  if (OUTPUTMONITOR) {
    serialOutputMonitor(Value1, Value2, Value3);
  }

  if (OUTPUTBLUETOOTHMONITOR) {
    bluetoothSerialOutputMonitor(Value1, Value2);
  }
}

void bluetoothSerialOutputMonitor(double Value1, double Value2) {
  String Delimiter = ", ";

  BluetoothSerial.print(Value1, DEC);
  BluetoothSerial.print(Delimiter);
  BluetoothSerial.print(Value2, DEC);
  BluetoothSerial.println(Delimiter);
}



//////////////////////INTERRUPT RELATED FUNCTIONS///////////////////////////////////////////////////////////////////////////////////////////////////
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


  
void measureIRs(){
  IRLF_FLAG = 0;
  IRLS_FLAG = 0;
  IRRS_FLAG = 0;
  IRRF_FLAG = 0;
  SONAR_FLAG = 0;
  OBSTACLE_FLAG = 0;

  SONAR = measure_distance_ultrasonic;
  IRLF = analogRead(PinIRLF);
  IRLS = analogRead(PinIRLS);
  IRRS = analogRead(PinIRRS);
  IRRF = analogRead(PinIRRF);


  if(IRLF > 220){
    IRLF_FLAG = 8; //NOW NON BOOLEAN BE CAREFUL WHEN HANDLING
    OBSTACLE_FLAG = 1;
  }
  if(IRLS > 600){
    IRLS_FLAG = 1;
    OBSTACLE_FLAG = 1;
  }

  if(IRRS > 600){
    IRRS_FLAG = 2;
    OBSTACLE_FLAG = 1;
  }

  if(IRRF > 220){
    IRRF_FLAG = 4;
    OBSTACLE_FLAG = 1;
  }

   if(SONAR < 7){
    SONAR_FLAG = 1;
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


//////////////////////SONAR RELATED CODE/////////////////////////////////////////////////
int measure_distance_ultrasonic(void) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);  // Reads the echoPin, returns the sound wave travel time in microseconds
  distance = duration * 0.034 / 2;    // Calculating the distance and dividing by 2 (distance to and from)


  return distance;
}





///////////////////MOVEMENT RELATED CODE////////////////////////////////////////////////////

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



double saturate(double u, double min_val, double max_val) {
  return fmin(fmax(u, min_val), max_val);
}



void findMax(int turnPower, int maxSearches) {
  int numSearches = 0;
  int MaxDist = 0;
  int foundMax = 0;
  int maxAngle = 0;
  float findWallAngle = 0;

  int dist = 0;

  //get the initial distance
  enable_motors();//VECTORISED MOTOR CONTROL IS DISABLING MOTORS

  //search for corner to the left
  //Serial.println("Turning left 1");
  left_front_motor.writeMicroseconds(saturate((1500 - turnPower), 750, 2250));
  left_rear_motor.writeMicroseconds(saturate((1500 - turnPower), 750, 2250));
  right_rear_motor.writeMicroseconds(saturate((1500 - turnPower), 750, 2250));
  right_front_motor.writeMicroseconds(saturate((1500 - turnPower), 750, 2250));

  //continue to turn while not found max distance
  while (foundMax == 0) {

   
   // if(OBSTACLE_FLAG == 1 || FIRE_FLAG == 1){break;}

    dist = measure_distance_ultrasonic();
    //Serial.println("current dist:   ");
    //Serial.print(dist);
    //find the largest distance
    if (dist > MaxDist) {
      MaxDist = dist;
      //Serial.println("/////////////////////////////////////////////////////////New max dist:   ");
      //Serial.print(MaxDist);
      
      maxAngle = currentAngle;
      // Serial.print(currentAngle);
      //Serial.println(maxAngle);
    }

    //increment search variable until max searches done
    if (numSearches < maxSearches) {
      numSearches++;
    } else {
      foundMax = 1;
    }

    delay(10);  //search at 100 hz
  }

   //shutdown motors
//disable_motors();
       

  //find angle turned
  findWallAngle = maxAngle - currentAngle;
  if (currentAngle > maxAngle) {
    findWallAngle += 360;
  }
        Serial.println(findWallAngle);

currentAngle = 180;
  while (abs((findWallAngle / 2) - (currentAngle - 180)) > 5) {

    if (OBSTACLE_FLAG == 1 || FIRE_FLAG == 1) { break; }
    //Serial.println(currentAngle);
    vectorised_motor_inputs(0, 0, (findWallAngle / 2 - (currentAngle - 180)) * 0.8);
  }

  currentAngle = 180;
  while (abs((findWallAngle / 2) - (currentAngle - 180)) > 5) {

    if (OBSTACLE_FLAG == 1 || FIRE_FLAG == 1) { break; }
    //Serial.println(currentAngle);
    vectorised_motor_inputs(0, 0, (findWallAngle / 2 - (currentAngle - 180)) * 0.8);
  }

  disable_motors();
  delay(5000);


}



void vectorised_motor_inputs(double x, double y, double theta) {
  double motor_speed[4] = { 0, 0, 0, 0 };
  double max_abs_value = fmax(fabs(x), fmax(fabs(y), fabs(theta)));

  if (max_abs_value >= 650.0) {
    double scale_factor = 650.0 / max_abs_value;
    x = x * scale_factor;
    y = y * scale_factor;
    theta = theta * scale_factor;
  }

  motor_speed[0] = (x + y + theta * (L1 + L2));  //FL
  motor_speed[1] = (x - y - theta * (L1 + L2));  //FR
  motor_speed[2] = (x - y + theta * (L1 + L2));  //BL
  motor_speed[3] = (x + y - theta * (L1 + L2));  //BR

  left_front_motor.writeMicroseconds(saturate(1500 + motor_speed[0], 850, 2150));
  right_front_motor.writeMicroseconds(saturate(1500 - motor_speed[1], 850, 2150));
  left_rear_motor.writeMicroseconds(saturate(1500 + motor_speed[2], 850, 2150));
  right_rear_motor.writeMicroseconds(saturate(1500 - motor_speed[3], 850, 2150));
}


void straight() {
  //open loop drive forward with closed loop angle 
  error_theta = currentAngle - 180;

  //Motor kinematics control
  vectorised_motor_inputs(175, 0, kp_theta * error_theta);
} 


void strafe(int direction) {
  //Left: -1, Right:1
  error_theta = -(currentAngle - 180);

  //Motor kinematics control
  vectorised_motor_inputs(0, direction * 175, kp_theta * error_theta);
}




void turnAngle(int angle) {
  currentAngle = 180;
  float Kp = 0.8;
  //2-(angle/100);
  Serial.print("99999999999999999999999999999999999999999999999999999999999999999999999999999999");
  while (abs((angle) - (currentAngle - 180)) > 10) {

    if (OBSTACLE_FLAG == 1 || FIRE_FLAG == 1) { break; }
    Serial.println(currentAngle);
    vectorised_motor_inputs(0, 0, (angle - (currentAngle - 180)) * 0.8);
  }
  disable_motors();
}





//////////////////////SERVO RELATED CODE//////////////////////////////////////////////////
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




///////////////////OBSTACLE AVOIDANCE FUNCTIONS/////////////////////////////////////////////
void obstacleAvoid(void){
  int sum = IRLS_FLAG + IRRS_FLAG + IRRF_FLAG + IRLF_FLAG;
Serial.println(sum);
  
  if(SONAR_FLAG){
      if( (sum == 2 ||(sum == 4) || (sum == 10) || (sum == 6) )){
          //strafe left
           Serial.println("strafe left sonar"); 
          strafe(-1);    
      }else if((sum == 1) ||  (sum == 5) || (sum == 8) || (sum == 9) ){
          //strafe right
          Serial.println("strafe right sonar");
          strafe(1);     
      }else if((sum == 3) ||  (sum == 7) || (sum == 11) ){
          //escape 
           Serial.println("escape sonar");
      }else if(sum == 0){
        //strafe min dist
        Serial.println("strafe min dist sonar");
      }else if((sum == 12) || (sum == 13) || (sum == 14) || (sum == 15)){
        //go to max dist
        Serial.println("find max dist sonar");
      }

      //NEED LOGIN FOR CONTINUE
    
//    if(!IRLS_FLAG && ((IRLF_FLAG + IRRF_FLAG) == 1)){
//      //strafe left
//      strafe(-1);
//      Serial.println("strafe left sonar");
//    }else if(!IRRS_FLAG && ((IRLF_FLAG + IRRF_FLAG) == 1)){
//      //strafe right
//      strafe(1);
//      Serial.println("strafe right sonar");
//    }else if((IRLS_FLAG && IRRS_FLAG) && (IRLF_FLAG || IRRF_FLAG)){
//      //escape
//            Serial.println("escape sonar");
//    }else if(IRLF_FLAG && IRRF_FLAG){
//      //find max dist
//            Serial.println("find max dist sonar 1");
//    }else{
//      //escape THIS IS NOT MOST EFFICIENT PATH
//          Serial.println("find max dist sonar 2");
//    }

    
  }
  
  if(!SONAR_FLAG){
      if((sum == 4) ||  (sum == 6) || (sum == 10) || (sum == 14) ){
          //strafe left
          Serial.println("strafe left no sonar");
          strafe(-1);     
      }else if((sum == 5) ||  (sum == 8) || (sum == 9) || (sum == 13) ){
          //strafe right
          Serial.println("strafe right no sonar");
          strafe(1);     
      }else if((sum == 7) ||  (sum == 11) || (sum == 15) ){
          //escape 
           Serial.println("escpae no sonar");
      }else if(sum == 12){
        //strafe min dist
        Serial.println("min dist no sonar");
      }
//NEED LOGIN FOR CONTINUE


    
//    if(!IRLS_FLAG && (IRRF_FLAG || IRLF_FLAG)){
//      //STRAFE LEFT
//      strafe(-1);
//      Serial.println("strafe left no sonar");
//    }else if(!IRRS_FLAG && (IRRF_FLAG || IRLF_FLAG)){
//      //STRAFE RIGHT
//      strafe(1);
//      Serial.println("strafe right no sonar");
//    }else if(IRLS_FLAG && IRRS_FLAG && (IRLF_FLAG || IRRF_FLAG)){
//      //ESCAPE
//      Serial.println("escpae no sonar");
//    }else if(IRLS_FLAG && IRRS_FLAG && !(IRLF_FLAG || IRRF_FLAG)){
//      //SRAFE MIN DIST
//      Serial.println("min dist no sonar");
//    }else if(!IRLF_FLAG && !IRRF_FLAG){
//      //DRIVE FORWARD OPEN LOOP A BIT
//      Serial.println("forward no sonar");
//    }
  }
}



///////////////////////PT RELATED CODE BELOW THIS LINE/////////////////////////////////////////////////////////////////////////////////////////
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
  
  //turn(currentAngle -180 + currentPos - 90);
  //servo(90);
  Pivot.write(90);

  narrowServoSearch();

  //turn(currentAngle -180 + currentPos - 90);
  //servo(90);
  Pivot.write(90);

  startMotors();

  while (fireFound == 0) {
    narrowServoSearch();

    if (OBSTACLE_FLAG) {
      break;
    }

    if ((lastServoFireAngle - 90) > 10) {
      //openLoopAdjustCourse(currentAngle -180 + currentPos - 90);
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



void loop(){
  //strafe(-1);
  //findMax(120, 450);
  //turnAngle(120);
  obstacleAvoid();
  //delay(10000);
}
