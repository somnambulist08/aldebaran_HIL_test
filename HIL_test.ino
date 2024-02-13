#define RPM 750
#define frame_sec 0.1

#define P -0.0006
#define I -0.0006

#define v_max 160
#define alti_min 0
#define alti_max 1500

#define MOTOR_STEPS 200
#define MICROSTEPS 8


int step_pin = 9;
int dir_pin = 8;

//int alti_pin = A0;
//int v_pin = A1;

int flap_pin = A0;

unsigned long frame_micros = frame_sec * 1000000;
float pps = (RPM / 60) * MOTOR_STEPS * MICROSTEPS;
unsigned long pwmicros = 1000000 / (pps);

int currentStep = 0;
float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int microStepsFromFlapAngle(float angle) {
  angle -= 0.046121116014511f;
  float A = 103.6259f - (32.53459696f * cos(angle) + sqrt(6814.5025f - pow(43.4f + 32.53459696f * sin(angle), 2)));
  float microSteps = ((float)MICROSTEPS) * A / 2.54f * 360.0f / 1.8f;
  return (int)microSteps;
}

float getDesired(float time) {
  return 480.0f * powf(M_E, -time * 3.0f / 10.0f) + 1143.0f;
}
float predictAltitude(float height, float velocity) {
  return height + velocity * velocity / 2 / 9.81;
}

float integratorState = 0;
float getControl(float desired, float predicted, float dt) {
  float err = desired - predicted;
  float control = P * err;
  integratorState += I * err * dt;

  control += PI / 12;
  if (control > PI / 2) control = PI / 2;
  if (control < 0) control = 0;
  return control;
}
unsigned long timeAtBurnout = 0;
void setup() {
  // put your setup code here, to run once:
  pinMode(step_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  pinMode(flap_pin, OUTPUT);
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.println("Setup Done...");
}
int started = 0;
void loop() {
  while(Serial1.available()<=0){
  //  while(Serial.available()<=0){

  }
  //String alti_string = Serial1.readStringUntil('\n');
//  String alti_string = Serial.readStringUntil('\n');
  float inAltitude = 0;//alti_string.toFloat();                                   //step 1: make some room for a number
  Serial1.readBytes((char *)&inAltitude, sizeof(float));  //step 2: pretend tshe number is not a number
                                                          // Serial.println(String(inAltitude)); //step 3: profit
  while(Serial1.available()<=0){
  //  while(Serial.available()<=0){

  }
  //String vel_string = Serial1.readStringUntil('\n');
  //String vel_string = Serial.readStringUntil('\n');

  float inVelocity = 0;//vel_string.toFloat();                                   //step 1: make some room for a number
  Serial1.readBytes((char *)&inVelocity, sizeof(float));  //step 2: pretend tshe number is not a number
                                                        // Serial.println(String(inAltitude)); //step 3: profit
  
  
  if (!started) {
    started = 1;
    timeAtBurnout = micros();
  }
  unsigned long t_now = micros();
  unsigned long timeSinceBurnout = micros() - timeAtBurnout;
  if (inVelocity > 0) {
    float ang = getControl(getDesired(timeSinceBurnout), predictAltitude(inAltitude, inVelocity), timeSinceBurnout - t_now);
    Serial.print("altitude in: ");
    Serial.println(inAltitude);
    Serial.print("vel in: ");
    Serial.println(inVelocity);
    int steps = microStepsFromFlapAngle(ang);
    int moveSteps = steps - currentStep;

    //Serial.print("Current Steps to Motor: ");
    //Serial.println(currentStep);
    unsigned long prev = 0;
    int direction = -1;
    if (moveSteps <= 0) {
      direction = 0;
    } else {
      direction = 1;
    }
    digitalWrite(dir_pin, direction);
    unsigned long pw_on = pwmicros / 2;
    while ((micros() - t_now) < frame_micros) {

      unsigned long current = micros();
      if ((current - prev >= pwmicros) && moveSteps != 0) {
        prev = current;
        digitalWrite(step_pin, HIGH);
        delayMicroseconds(pw_on);
        digitalWrite(step_pin, LOW);
        delayMicroseconds(pwmicros - pw_on);
        if (direction) {
          moveSteps = moveSteps + 1;
          currentStep = currentStep + 1;
        } else {
          moveSteps = moveSteps + 1;
          currentStep = currentStep - 1;
        }
      }
    }
    Serial.println("Time's up");
    moveSteps = 0;
  } else {
    Serial.print("Final Altitude: ");
    Serial.println(inAltitude);
    Serial.println("v=0");
  }
  int flap_pos = analogRead(flap_pin);
  Serial.println(flap_pos);
  if (flap_pos < 10){
    Serial1.println("000"+String(flap_pos));
    //Serial.println("000"+String(flap_pos));
  }
  else if(flap_pos <100){
    Serial1.println("00"+String(flap_pos));
    //Serial.println("00"+String(flap_pos));
  }
  else if (flap_pos<1000){
    Serial1.println("0"+String(flap_pos));
    //Serial.println("0"+String(flap_pos));
  }
  else{
    Serial1.println(flap_pos);
    //Serial.println(flap_pos);
  }
  //size_t written = Serial1.write((char *)&flap_pin, sizeof(char));
  //Serial.write((char *)&flap_pin, sizeof(uint16_t));
  //Serial.println(written);
  //Serial.println(flap_pos);
} 
