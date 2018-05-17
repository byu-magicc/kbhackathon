
#include <Encoder.h>
#include <Servo.h>

// Teensy 3.x / Teensy LC have the LED on pin 13
#define LED_PIN 13
#define PHASE_A 2
#define PHASE_B 3
#define STEER_PIN 5
#define THROTTLE_PIN 6
#define US_PER_S 1000000
#define RATE 50 //Hz
#define PULSES_PER_M 2000 //put the real value here


// the setup() method runs once, when the sketch starts


// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder enc(PHASE_A, PHASE_B);
//   avoid using pins with LEDs attached

// create servo objects for the steering and throttle
Servo steering;
Servo throttle;

// Create an IntervalTimer object 
IntervalTimer encTimer;

// a place to store encoder values
volatile int pos  = 0;
volatile int diff  = 0;

// loop timing
long long prevTime;



void setup() {
  // initialize the digital pin as an output.
  pinMode(LED_PIN, OUTPUT);
//  pinMode(STEER_PIN, OUTPUT);
//  pinMode(THROTTLE_PIN, OUTPUT);
  Serial.begin(115200);
  encTimer.begin(readEnc, US_PER_S/RATE);  // Read the encoder at the specified rate
  // assign servo pins
  steering.attach(STEER_PIN);
  throttle.attach(THROTTLE_PIN);
  // initialize servos to neutral position
  steering.write(90);
  throttle.write(90);
  prevTime = (long long)micros();
}

// callback to save the current encoder values
// this is called from an interval timer
void readEnc() {
  long newEnc;
  newEnc = enc.read();
  diff = newEnc - pos;
  pos = newEnc;
}

// this creates a byte-wise xor checksum to be sent with the
// data packet for error checking
char createChecksum(void* a_p, void* b_p) {
  char chk = 0x00;
  int a = *(int*)a_p;
  int b = *(int*)b_p;
  chk ^= (char)((a & 0xff000000) >> 24);
  chk ^= (char)((a & 0x00ff0000) >> 16);
  chk ^= (char)((a & 0x0000ff00) >>  8);
  chk ^= (char)(a & 0x000000ff);
  chk ^= (char)((b & 0xff000000) >> 24);
  chk ^= (char)((b & 0x00ff0000) >> 16);
  chk ^= (char)((b & 0x0000ff00) >>  8);
  chk ^= (char)(b & 0x000000ff);
  return chk;
}

void setServos(float steer, float thr) {
  Serial.print(thr);
  Serial.print(", ");
  Serial.print(steer);
  Serial.println();
  // write() takes angle values between 0 and 180 (stupid)
  steering.write(steer*90 + 90);
  throttle.write(thr*90 + 90);
}

enum State{s_idle, s_start, s_reset, s_c1, s_c2, s_c3};

void parseCmd(char data) {
  static State state = s_idle;
  static String val_str;
  static float thr = 0.0f;
  static float steer = 0.0f;
  switch(state) {
    case s_idle:
      if(data == '<') state = s_start;
      break;
    case s_start:
      switch(data) {
        case 'r':
          state = s_reset;
          break;
        case 'c':
          state = s_c1;
          break;
        default:
          state = s_idle;
          break;
      }
      break;
    case s_reset:
      if(data == '>') enc.write(0);
      state = s_idle;
      break;
    case s_c1:
      if(data == ':') {
        thr = 0.0f;
        steer = 0.0f;
        val_str = String("");
        state = s_c2;
      } else {
        state = s_idle;
      }
      break;
    case s_c2:
      if(data >= '-' && data <= '9' && data != '/') {
        val_str += data;
      } else if(data == ',' && val_str.length() > 0) {
        steer = val_str.toFloat();
        val_str = String("");
        state = s_c3;
      } else {
        state = s_idle;
      }
      break;
    case s_c3:
      if(data >= '-' && data <= '9' && data != '/') {
        val_str += data;
      } else if(data == '>' && val_str.length() > 0) {
        thr = val_str.toFloat();
        setServos(thr, steer);
        val_str = String("");
        state = s_idle;
      } else {
        state = s_idle;
      }
      break;
    default:
      state = s_idle;
  }
  
  
}

void loop() {
  int pos_copy;
  int diff_copy;
  noInterrupts();
  pos_copy = pos;
  diff_copy = diff;
  interrupts();
  // convert the pulses into SI units (m and m/s)
  float dist = (float)pos_copy/PULSES_PER_M;
  float vel = (float)diff_copy/PULSES_PER_M*RATE;
  // create a checksum by doing a byte-wise xor of the data
  char checksum = createChecksum(&dist, &vel);
//  Serial.print("[");
//  Serial.print(dist, 3);
//  Serial.print(",");
//  Serial.print(vel, 3);
//  Serial.print(",");
//  if(checksum < 100) Serial.print("0");
//  if(checksum < 10) Serial.print("0");
//  Serial.print((int)checksum);
//  //printf("%03d", checksum);
//  Serial.print("]");
//  Serial.println();
  //delay(100);
  if(diff_copy != 0) {
    digitalWrite(LED_PIN, HIGH);   // set the LED on
  } else {
    digitalWrite(LED_PIN, LOW);   // set the LED off
  }
  
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  while (Serial.available()) {
    parseCmd(Serial.read());
  }

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

