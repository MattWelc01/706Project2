#include <SoftwareSerial.h>
#include <Servo.h>
#include <avr/io.h>

Servo left_front_motor;   // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;    // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;   // create servo object to control Vex Motor Controller 29
Servo right_front_motor;  // create servo object to control Vex Motor Controller 29
Servo Pivot;              // create servo object to control pivot servo

//wireless communication variables
#define INTERNAL_LED 13
#define BLUETOOTH_RX 10   // Serial Data input pin
#define BLUETOOTH_TX 11   // Serial Data output pin
#define STARTUP_DELAY 10  // Seconds
#define LOOP_DELAY 2      // miliseconds
#define SAMPLE_DELAY 10   // miliseconds
#define OUTPUTMONITOR 0   // USB Serial Port
#define OUTPUTPLOTTER 0
#define OUTPUTBLUETOOTHMONITOR 1  // Bluetooth Serial Port
byte serialRead = 0;              //for control serial communication
SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

//motor pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;
const byte PivotPin = 21;

//length variables for kinematics
#define L1 7.6
#define L2 8.875

//global variables
volatile unsigned int gyro_timer = 0;  //timer variable

//ultrasonic global variables
#define echoPin A4           // attach pin A4 Arduino to pin Echo of HC-SR04
#define trigPin A5           // attach pin A5 Arduino to pin Trig of HC-SR04
#define fanPin 14            // attach pin 0 to fan
volatile long duration = 0;  // variable for the duration of sound wave travel
volatile int distance = 0;   // variable for the distance measurement

//phototransistor global variables
#define phototransistor_1 A11
#define phototransistor_2 A12
#define phototransistor_3 A13
#define phototransistor_4 A14
int moving_average[4] = { 0, 0, 0, 0 };
int phototransistor_values[4] = { 0, 0, 0, 0 };

//gyro globals
int sensorPin = A3;               //define the pin that gyro is connected
int sensorValue = 0;              // read out value of sensor
float T = 50;                     //Time period
float gyroSupplyVoltage = 5;      // supply voltage for gyro
float gyroZeroVoltage = 0;        // the value of voltage when gyro is zero
float gyroSensitivity = 0.007;    // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 2;      // because of gyro drifting, defining rotation angular velocity less than this value will not be ignored
float gyroRate = 0;               // read out value of sensor in voltage
volatile float currentAngle = 0;  // current angle calculated by angular velocity integral on
byte serialReadGyro = 0;          // for serial print control

void setup() {
  delay(2000);

  pinMode(trigPin, OUTPUT);   // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT);    // Sets the echoPin as an INPUT
  pinMode(sensorPin, INPUT);  // Sets the gyroscope pin as INPUT
  Pivot.attach(PivotPin, 450, 2560);

  cli();  // Disable interrupts
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

  enable_motors();
  sei();  // Enable global interrupts

  delay(1000);
}


////////////////////////////////////////////*************NON FUNCTIONAL CODE********/////////////////////////////////////////////////////////////////

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

/////////////////////////////////////////////*************FUNCTION CODE********/////////////////////////////////////////////////////////////////

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
  //Serial.println(currentPos);
}

int measure_distance_ultrasonic(void) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);  // Reads the echoPin, returns the sound wave travel time in microseconds
  distance = duration * 17 / 1000;    // 0.034 / 2 = 17/1000 Calculating the distance and dividing by 2 (distance to and from)

  return distance;
}

void readPhototransistors() {
  phototransistor_values[0] = analogRead(phototransistor_1);
  phototransistor_values[1] = analogRead(phototransistor_2);
  phototransistor_values[2] = analogRead(phototransistor_3);
  phototransistor_values[3] = analogRead(phototransistor_4);
}


volatile double cumulative_error_theta = 0, derivative_error_theta = 0, last_error_theta = 0, power_error_theta = 0;
unsigned long elapsedTime = 0, lastTime = 0, elapsed_PID_time = 0;
unsigned long last_update_time = 0;  // timer for PID control
volatile float error_theta = 0;
float kp_theta = 1.2, ki_theta = 0.15, kd_theta = 0.4;  //  float kp_theta = 1.75, ki_theta = 0.15, kd_theta = 0.40;

void forward() {
  //open loop drive forward with closed loop angle 
  error_theta = currentAngle - 180;

  // P CONTROLLER
  power_error_theta = (kp_theta * error_theta) + (ki_theta * cumulative_error_theta) + (kd_theta * derivative_error_theta);

  //Motor kinematics control
  vectorised_motor_inputs(200, 0, power_error_theta);

  // Updating derivative and integral terms every 20ms
  elapsed_PID_time = millis() - last_update_time;
  if (elapsed_PID_time >= 40) {

    derivative_error_theta = (error_theta - last_error_theta) / elapsed_PID_time;
    power_error_theta += kd_theta * derivative_error_theta;
    last_error_theta = error_theta;

    cumulative_error_theta += error_theta;
    cumulative_error_theta = saturate(cumulative_error_theta, -5, 5);

    last_update_time = millis();
  }
}

void strafe(int direction) {
  //Left: -1, Right:1
  error_theta = currentAngle - 180;

  // P CONTROLLER
  power_error_theta = (kp_theta * error_theta) + (ki_theta * cumulative_error_theta) + (kd_theta * derivative_error_theta);

  //Motor kinematics control
  vectorised_motor_inputs(0, direction * 200, power_error_theta);

  // Updating derivative and integral terms every 20ms
  elapsed_PID_time = millis() - last_update_time;
  if (elapsed_PID_time >= 40) {

    derivative_error_theta = (error_theta - last_error_theta) / elapsed_PID_time;
    power_error_theta += kd_theta * derivative_error_theta;
    last_error_theta = error_theta;

    cumulative_error_theta += error_theta;
    cumulative_error_theta = saturate(cumulative_error_theta, -5, 5);

    last_update_time = millis();
  }
}

void reset_variables(void){
  cumulative_error_theta = 0, derivative_error_theta = 0, last_error_theta = 0, power_error_theta = 0;
  elapsedTime = 0, lastTime = 0, elapsed_PID_time = 0;
  last_update_time = 0;
  error_theta = 0;
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

  motor_speed[0] = (x + y - theta * (L1 + L2));  //FL
  motor_speed[1] = (x - y + theta * (L1 + L2));  //FR
  motor_speed[2] = (x - y - theta * (L1 + L2));  //BL
  motor_speed[3] = (x + y + theta * (L1 + L2));  //BR

  left_front_motor.writeMicroseconds(saturate(1500 + motor_speed[0], 850, 2150));
  right_front_motor.writeMicroseconds(saturate(1500 - motor_speed[1], 850, 2150));
  left_rear_motor.writeMicroseconds(saturate(1500 + motor_speed[2], 850, 2150));
  right_rear_motor.writeMicroseconds(saturate(1500 - motor_speed[3], 850, 2150));
}


void loop() {
 //digitalWrite(fanPin, HIGH);
//  Pivot.write(0);
//  delay(1500);
//  Pivot.write(180);
//  delay(1500);
//  Pivot.write(90);
//  delay(500);
//  Pivot.write(45);
//  delay(500);

//state go forward 
  enable_motors();

 while(1){
  measure_distance_ultrasonic();
  Serial.println(distance);
 }
}

//NOTES
/*
  //phototransistors
  phototransistor_reading phototransistor(void);

  //infared sensors
  infared_distance IR_distance(void);

  //find_max()
  distance find_dist_sonar(void);
  void turn_angle(double angle);
  current_angle servo_rotate(double abs_angle);

  //fire_search()
  void drive_dist_sonar(double distance);
  current_angle servo_rotate(double abs_angle);

  //drive_to_fire
  current_angle servo_rotate(double abs_angle);
  void turn_angle(double angle);
  void drive_phototransistor(double phototransistor_reading);

  //fire_fight
  void put_out_fire(void);

  //obstacle_avoid
  void strafe(void);
  current_angle servo_rotate(double abs_angle);
  void drive_open_loop(void);
*/