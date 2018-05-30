
#include <Encoder.h>
#include <Servo.h>

#include "serial_parser.h"

// Teensy 3.x / Teensy LC have the LED on pin 13
#define LED_PIN 13
#define PHASE_A 2
#define PHASE_B 3
#define STEER_PIN 5
#define THROTTLE_PIN 6
#define US_PER_S 1000000
#define RATE 50 //Hz
#define PULSES_PER_M 2000 //put the real value here
#define BRAKE_DELAY 50 // in milliseconds
#define BACK_SONAR_PWM_PIN 9

#define RC_THR_PIN 7
#define RC_STR_PIN 8


volatile int last_thr_pwm_rise;
volatile int thr_pwm;
volatile int last_str_pwm_rise;
volatile int str_pwm;



void throttle_PWM_isr()
{
  // If the signal is rising, then capture the time
  if (digitalRead(RC_THR_PIN) == HIGH)
  {
    last_thr_pwm_rise = micros();
  }
  else
  {
    thr_pwm = micros() - last_thr_pwm_rise;
  }
}


void steering_PWM_isr()
{
  // If the signal is rising, then capture the time
  if (digitalRead(RC_STR_PIN) == HIGH)
  {
    last_str_pwm_rise = micros();
  }
  else
  {
    str_pwm = micros() - last_str_pwm_rise;
  }
}



// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder enc(PHASE_A, PHASE_B);
//   avoid using pins with LEDs attached

// create servo objects for the steering and throttle
Servo steering;
Servo throttle;

// Create an IntervalTimer object for
IntervalTimer encTimer;

// a place to store encoder values
volatile int pos  = 0;
volatile int diff  = 0;

float dist_front = 0.0f;

float thr = 0.0f;
float steer = 0.0f;

// loop timing
long long prevTime;


void setup() {
  // initialize the digital pin as an output.
  pinMode(LED_PIN, OUTPUT);

//  pinMode(STEER_PIN, OUTPUT);
//  pinMode(THROTTLE_PIN, OUTPUT);
  Serial.begin(115200);
  Serial2.begin(9600); // back sonar
  encTimer.begin(readEnc, US_PER_S/RATE);  // Read the encoder at the specified rate

  // assign servo pins
  steering.attach(STEER_PIN);
  throttle.attach(THROTTLE_PIN);

  // initialize servos to neutral position
  steering.write(90);
  throttle.write(90);
  prevTime = (long long)micros();

  // Configure the RC inputs
  attachInterrupt(digitalPinToInterrupt(RC_THR_PIN), throttle_PWM_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_STR_PIN), steering_PWM_isr, CHANGE);
}

// callback to save the current encoder values
// this is called from an interval timer
void readEnc() {
  long newEnc;
  newEnc = enc.read();
  diff = newEnc - pos;
  pos = newEnc;
}



enum Thr_State{s_forward, s_brake, s_wait, s_rev};

void setServos(float steer, float thr) {
  static Thr_State state = s_forward;
  static long rev_time = 0;
  // write() takes angle values between 0 and 180 (stupid)
  steering.write(steer*90 + 90);
  // we need a state machine for throttle to circumvent
  // the stupid "braking" feature of the esc
  switch(state) {
    case s_forward:
      if(thr >= 0.0f) {
        throttle.write(thr*90 + 90);
      } else {
        throttle.write(-0.1*90 + 90);
        rev_time = millis();
        state = s_brake;
        Serial.println("brake");
      }
      break;
    case s_brake:
      if(millis() - rev_time > BRAKE_DELAY) {
        throttle.write(90);
        rev_time = millis();
        state = s_wait;
        Serial.println("wait");
      }
      break;
    case s_wait:
      if(millis() - rev_time > BRAKE_DELAY) {
        state = s_rev;
        Serial.println("reverse");
      }
      break;
    case s_rev:
      throttle.write(thr*90 + 90);
      if(thr > 0.0f) {
        state = s_forward;
        Serial.println("forward");
      }
      break;
    default:
      break;
  }
}

message_t out_message;

// Callback for when we get a command message
void command_callback()
{
  thr = in_message.dat.cmd.thr;
  steer = in_message.dat.cmd.steer;
}

// callback for when we get a sensor message
void sensor_callback()
{
  // Don't do anything here
}


float readSonar()
{
  pinMode(BACK_SONAR_PWM_PIN,INPUT);
  float pulse = pulseIn(BACK_SONAR_PWM_PIN,HIGH);
  float inches = pulse/147; // 147 microseconds per inch
  float dist = inches*0.0254; // convert to meters
  return dist;
}

void loop() {
  int pos_copy;
  int diff_copy;

  // Create a local copy of position and velocity while in a lock
  noInterrupts();
  pos_copy = pos;
  diff_copy = diff;
  interrupts();

  // convert the pulses into SI units (m and m/s)
  float dist = (float)pos_copy/PULSES_PER_M;
  float vel = (float)diff_copy/PULSES_PER_M*RATE;
  float dist_back = readSonar();

  // Create a sensor message to send
  out_message.type = TYPE_SENSOR;
  out_message.len = sizeof(sens_t);
  out_message.dat.sens.rc_thr =  thr_pwm;
  out_message.dat.sens.rc_steer =  str_pwm;
  out_message.dat.sens.distance = dist;  // distance traveled in (m)
  out_message.dat.sens.velocity = vel; // current estimate of velocity (m/s)
  out_message.dat.sens.sonar = dist_back; // distance read by sonar (m)
  uint8_t crc = calcChecksum(&out_message);

  // send the sensor message
  Serial.write(START_BYTE);
  Serial.write(out_message.type);
  Serial.write(out_message.len);
  Serial.write(out_message.dat.buf, sizeof(sens_t));
  Serial.write(crc);

  //delay(100);
  if(diff_copy != 0) {
    digitalWrite(LED_PIN, HIGH);   // set the LED on
  } else {
    digitalWrite(LED_PIN, LOW);   // set the LED off
  }
  
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  while (Serial.available()) {
    parseMsg(Serial.read());
  }

  setServos(steer, thr);

  // calculate how long all of our stuff took
  long long now = (long long)micros();
  long long dur = now - prevTime;
  //Serial.print((int)dur);

  // if the loop took less time than our desired period,
  // sleep until the next iteration should happen
  // this could be more graceful, but I don't care right now
  if(dur > 0 && dur < US_PER_S/RATE) {
    delayMicroseconds(US_PER_S/RATE - dur);
    //Serial.print("sleeping");
  }
  prevTime = (long long)micros();
}

