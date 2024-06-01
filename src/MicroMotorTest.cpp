
#include <Arduino.h>

int LED_BUILTIN = 2;
int encoder_left_a = 4;
int encoder_left_b = 16;
volatile int encoder_left_a_state = 0;
volatile int encoder_left_a_prev_state = 0;
volatile int encoder_left_b_state = 0;
volatile int encoder_left_b_prev_state = 0;
bool encoder_left_a_changed = false;
bool encoder_left_b_changed = false;

int encoder_right_a = 5;
int encoder_right_b = 18;
volatile int encoder_right_a_state = 0;
volatile int encoder_right_a_prev_state = 0;
volatile int encoder_right_b_state = 0;
volatile int encoder_right_b_prev_state = 0;
bool encoder_right_a_changed = false;
bool encoder_right_b_changed = false;

bool is_forwards = false;
bool go_forwards = true;

long previous_millis = 0;
long interval = 500;  // interval at which to change motor direction (milliseconds)

int motor_control_left_a = 22;
int motor_control_left_b = 23;

int motor_control_right_a = 19;
int motor_control_right_b = 21;


//const int PWM_CHANNEL_A = 0;    // ESP32 has 16 channels which can generate 16 independent waveforms
//const int PWM_CHANNEL_B = 0;    // ESP32 has 16 channels which can generate 16 independent waveforms
const int PWM_FREQ = 500;     // Recall that Arduino Uno is ~490 Hz. Official ESP32 example uses 5,000Hz
const int PWM_RESOLUTION = 8; // We'll use same resolution as Uno (8 bits, 0-255) but ESP32 can go up to 16 bits

// The max duty cycle value based on PWM resolution (will be 255 if resolution is 8 bits)
const int MAX_DUTY_CYCLE = (int)(pow(2, PWM_RESOLUTION) - 1); 

volatile long pulse_count_left_a = 0;
volatile long pulse_count_left_b = 0;
long previous_pulse_count_left_a = 0;
long previous_pulse_count_left_b = 0;
int revolutions_left = 0;

volatile long pulse_count_right_a = 0;
volatile long pulse_count_right_b = 0;
long previous_pulse_count_right_a = 0;
long previous_pulse_count_right_b = 0;
int revolutions_right = 0;
// 7 CPR
// 210 reduction ratio
// 2940 pulses-per-rev (pulse = STATE CHANGE) [7*210*2]
const int PULSES_PER_REV = 2940;

float rpm_left = 0;
float rpm_right = 0;


void IRAM_ATTR isrEncoderLeftAChange(){
  encoder_left_a_state = digitalRead(encoder_left_a);
  pulse_count_left_a++;
}


void IRAM_ATTR isrEncoderLeftBChange(){
  encoder_left_b_state = digitalRead(encoder_left_b);
  pulse_count_left_b++;
}



void IRAM_ATTR isrEncoderRightAChange(){
  encoder_right_a_state = digitalRead(encoder_right_a);
  pulse_count_right_a++;
}



void IRAM_ATTR isrEncoderRightBChange(){
  encoder_right_b_state = digitalRead(encoder_right_b);
  pulse_count_right_b++;
}



void motorForwards(uint8_t dutyCycle){
  // go forwards
  ledcDetach(motor_control_left_b);
  ledcAttach(motor_control_left_a, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(motor_control_left_a, dutyCycle);

  ledcDetach(motor_control_right_a);
  ledcAttach(motor_control_right_b, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(motor_control_right_b, dutyCycle);
  
}



void motorBackwards(uint8_t dutyCycle){
  // go backwards
  ledcDetach(motor_control_left_a);
  ledcAttach(motor_control_left_b, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(motor_control_left_b, dutyCycle);

  ledcDetach(motor_control_right_b);
  ledcAttach(motor_control_right_a, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(motor_control_right_a, dutyCycle);
}



void motorChangeDirection(){
  if(go_forwards){
    go_forwards = false;
    motorBackwards(50);
  } else {
    go_forwards = true;
    motorForwards(50);
  }
}



void motorTest(){
  // go forwards
  ledcDetach(motor_control_left_b);
  ledcAttach(motor_control_left_a, PWM_FREQ, PWM_RESOLUTION);

  // fade up PWM on given channel
  for(int dutyCycle = 0; dutyCycle <= MAX_DUTY_CYCLE; dutyCycle++){   
    ledcWrite(motor_control_left_a, dutyCycle);
    Serial.print(" Duty cycle set to ");
    Serial.println(dutyCycle);
    delay(100);
  }

  // turn off channel A
  ledcWrite(motor_control_left_a, 0);

  // go backwards
  ledcDetach(motor_control_left_a);
  ledcAttach(motor_control_left_b, PWM_FREQ, PWM_RESOLUTION);
  
  // fade up PWM on given channel
  for(int dutyCycle = 0; dutyCycle <= MAX_DUTY_CYCLE; dutyCycle++){   
    ledcWrite(motor_control_left_b, dutyCycle);
    Serial.print(" Duty cycle set to ");
    Serial.println(dutyCycle);
    delay(100);
  }

  // turn off channel B
  ledcWrite(motor_control_left_b, 0);
}



void motorStop(){
  if(go_forwards){
    ledcWrite(motor_control_left_a, 0);
    ledcWrite(motor_control_right_b, 0);
  } else {
    ledcWrite(motor_control_left_b, 0);
    ledcWrite(motor_control_right_a, 0);
  }
}



void setup() {

  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(encoder_left_a, INPUT);
  pinMode(encoder_left_b, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_left_a), isrEncoderLeftAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_left_b), isrEncoderLeftBChange, CHANGE);

  pinMode(encoder_right_a, INPUT);
  pinMode(encoder_right_b, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_right_a), isrEncoderRightAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_right_b), isrEncoderRightBChange, CHANGE);

  // Sets up a channel (0-15), a PWM duty cycle frequency, and a PWM resolution (1 - 16 bits) 
  // ledcSetup(uint8_t channel, double freq, uint8_t resolution_bits);
  //ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  //ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);

  // ledcAttachPin(uint8_t pin, uint8_t channel);
  ledcAttach(motor_control_left_a, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(motor_control_left_b, PWM_FREQ, PWM_RESOLUTION);
  //Set both channels to 0
  ledcWrite(motor_control_left_a, 0);
  ledcWrite(motor_control_left_b, 0);

  ledcAttach(motor_control_right_a, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(motor_control_right_b, PWM_FREQ, PWM_RESOLUTION);
  //Set both channels to 0
  ledcWrite(motor_control_right_a, 0);
  ledcWrite(motor_control_right_b, 0);

  // set initial forwards to 50/255 duty cycle
  motorForwards(50);

}



void loop() {

  unsigned long current_millis = millis();
  
  if(current_millis - previous_millis > interval) {
    
    unsigned long time_difference_millis = current_millis - previous_millis;
    
    // calculate current rpm for left motor;
    float pulse_difference_left_a = pulse_count_left_a - previous_pulse_count_left_a;
    previous_pulse_count_left_a = pulse_count_left_a;
    previous_pulse_count_left_b = pulse_count_left_b;
    rpm_left = ((pulse_difference_left_a/PULSES_PER_REV) / (float(time_difference_millis)/1000)) * 60;

    // calculate current rpm for right motor;
    float pulse_difference_right_a = pulse_count_right_a - previous_pulse_count_right_a;
    previous_pulse_count_right_a = pulse_count_right_a;
    previous_pulse_count_right_b = pulse_count_right_b;
    rpm_right = ((pulse_difference_right_a/PULSES_PER_REV) / (float(time_difference_millis)/1000)) * 60;

    previous_millis = current_millis;    
  }


  if(pulse_count_left_a % PULSES_PER_REV == 0 || pulse_count_left_b % PULSES_PER_REV == 0){
    revolutions_left+=1;
    Serial.print("A Pulse Count ");
    Serial.print(pulse_count_left_a);
    Serial.print("     B Pulse Count ");
    Serial.print(pulse_count_left_b);
    Serial.print("     Revolutions ");
    Serial.print(revolutions_left);
    Serial.print("     RPM Left ");
    Serial.print(rpm_left);
    Serial.print("     RPM Right ");
    Serial.println(rpm_right);

    if(revolutions_left%4 == 0){
      motorChangeDirection();
    }
    if(revolutions_left%20 == 0){
      motorForwards(255);
    } else if (revolutions_left%10 == 0) {
      motorForwards(50);
    }
    //pulse_count_left_a = 0;
    //pulse_count_left_b = 0;
  }


  
  // check which direction left motor is moving. Is assumed that right will be same direction. Direction defined as robot direction
  if(encoder_left_a_prev_state != encoder_left_a_state){
    noInterrupts();
    if(encoder_left_a_state != encoder_left_b_state){
      digitalWrite(LED_BUILTIN, 1);
      is_forwards = true;
    } else {
      digitalWrite(LED_BUILTIN, 0);
      is_forwards = false;
    }

  }
 
  if(encoder_left_b_prev_state != encoder_left_b_state){
    noInterrupts();
    if(encoder_left_b_state == encoder_left_a_state){
      digitalWrite(LED_BUILTIN, 1);
      is_forwards = true;
    } else {
      digitalWrite(LED_BUILTIN, 0);
      is_forwards = false;
    }
  }


  encoder_left_a_prev_state = encoder_left_a_state;
  encoder_left_b_prev_state = encoder_left_b_state;
  encoder_right_a_prev_state = encoder_right_a_state;
  encoder_right_b_prev_state = encoder_right_b_state;
  interrupts();
 
}
