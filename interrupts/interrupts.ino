#define phototransistor_right_inner A10
#define phototransistor_right_outer A11
#define phototransistor_left_outer A12 
#define phototransistor_left_inner A13


void setup() {
 
  Serial.begin(9600);

  cli();
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




#define phototransistor_right_inner A10
#define phototransistor_right_outer A11
#define phototransistor_left_outer A12 
#define phototransistor_left_inner A13

int fireFlag = 0;
int fireThreshold = 4;

void measurePTs(){
  fireFlag = 0;
  
  int leftInner = analogRead(phototransistor_left_inner);
  int leftOuter = analogRead(phototransistor_left_outer);
  int rightInner = analogRead(phototransistor_right_inner);
  int rightOuter = analogRead(phototransistor_right_outer);

  if((leftInner > fireThreshold)||(leftOuter > fireThreshold) || (rightInner > fireThreshold)|| (rightOuter > fireThreshold)){
    fireFlag = 1;
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
void measureIRs(){
  IRLF_FLAG = 0;
  IRLS_FLAG = 0;
  IRRS_FLAG = 0;
  IRRF_FLAG = 0;
  
  IRLF = analogRead(PinIRLF*function);
  IRLS = analogRead(PinIRLS*function);
  IRRS = analogRead(PinIRRS*function);
  IRRF = analogRead(PinIRRF*function);
  
  if(IRLF < 10){
    IRLF_FLAG = 1;
  }
  if(IRLS < 10){
    IRLS_FLAG = 1;
  }

  if(IRRS < 10){
    IRRS_FLAG = 1;
  }

  if(IRRF < 10){
    IRRF_FLAG = 1;
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




void loop(){
  Serial.println(count); //check runs at 1000hz
}
