
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
#define PULSES_PER_M 10180 //10180
#define BRAKE_DELAY 50 // in milliseconds
#define BACK_SONAR_PIN 9

#define SERVO_MIN 850
#define SERVO_MAX 2150
#define SERVO_RANGE 45    //this limits steering range on the servo so it doesn't bind
#define SAFETY_DEADZONE_MIN 1410
#define SAFETY_DEADZONE_MAX 1590
#define SAFETY_DELAY 2000
#define SAFETY_DROPOUT_TIMEOUT 100   //milliseconds
#define COMMAND_TIMEOUT 500  //milliseconds

#define MAX_VEL 3.0 //m/s

#define RC_THR_PIN 8
#define RC_STR_PIN 7


volatile int last_thr_pwm_rise;
volatile int thr_pwm;
volatile int last_str_pwm_rise;
volatile int str_pwm;
volatile int last_sonar_pwm_rise;
volatile int sonar_pwm;
volatile bool rc_override;

float sat(float val, float max, float min) {
  if(val > max) val = max;
  if(val < min) val = min;
  return val;
}

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


void sonar_PWM_isr()
{
  // If the signal is rising, then capture the time
  if (digitalRead(BACK_SONAR_PIN) == HIGH)
  {
    last_sonar_pwm_rise = micros();
  }
  else
  {
    sonar_pwm = micros() - last_sonar_pwm_rise;
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

// Create an IntervalTimer object
IntervalTimer encTimer;

// a place to store encoder values
volatile int pos  = 0;
volatile int diff  = 0;

float dist_front = 0.0f;
float dist_back = 0.0f;

float thr = 0.0f;
float steer = 0.0f;
long last_command_time;


// loop timing
long long prevTime;


// Choose the UART or USB device to use:
//   - USB   --> Serial
//   - UARTx --> Serialx
auto serial = Serial;


void setup() {
  
  // initialize the digital pin as an output.
  pinMode(LED_PIN, OUTPUT);
//  pinMode(BACK_SONAR_PIN,INPUT);
//  pinMode(STEER_PIN, OUTPUT);
//  pinMode(THROTTLE_PIN, OUTPUT);
  serial.begin(115200);
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

  // Configure sonar PWM input
  attachInterrupt(digitalPinToInterrupt(BACK_SONAR_PIN), sonar_PWM_isr, CHANGE);
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
char createChecksum(void* a_p, void* b_p, void* c_p, void* d_p, void* e_p, void* f_p) {
  char chk = 0x00;
  int a = *(int*)a_p;
  int b = *(int*)b_p;
  int c = *(int*)c_p;
  int d = *(int*)d_p;
  int e = *(int*)e_p;
  int f = *(int*)f_p;
  chk ^= (char)((a & 0xff000000) >> 24);
  chk ^= (char)((a & 0x00ff0000) >> 16);
  chk ^= (char)((a & 0x0000ff00) >>  8);
  chk ^= (char)(a & 0x000000ff);
  chk ^= (char)((b & 0xff000000) >> 24);
  chk ^= (char)((b & 0x00ff0000) >> 16);
  chk ^= (char)((b & 0x0000ff00) >>  8);
  chk ^= (char)(b & 0x000000ff);
  chk ^= (char)((c & 0xff000000) >> 24);
  chk ^= (char)((c & 0x00ff0000) >> 16);
  chk ^= (char)((c & 0x0000ff00) >>  8);
  chk ^= (char)(c & 0x000000ff);
  chk ^= (char)((d & 0xff000000) >> 24);
  chk ^= (char)((d & 0x00ff0000) >> 16);
  chk ^= (char)((d & 0x0000ff00) >>  8);
  chk ^= (char)(d & 0x000000ff);
  chk ^= (char)((e & 0xff000000) >> 24);
  chk ^= (char)((e & 0x00ff0000) >> 16);
  chk ^= (char)((e & 0x0000ff00) >>  8);
  chk ^= (char)(e & 0x000000ff);
  chk ^= (char)((f & 0xff000000) >> 24);
  chk ^= (char)((f & 0x00ff0000) >> 16);
  chk ^= (char)((f & 0x0000ff00) >>  8);
  chk ^= (char)(f & 0x000000ff);
  return chk;
}

bool check_safety_override(int rc_thr, int rc_str) {
  static long override_time = 0;
  static long no_override_time = 0;
  long now = millis();
  static bool override = true;
  if(override_time == 0) {
    override_time = now;
  }
  if(no_override_time == 0) {
    no_override_time = now;
  }



  // if no safety pilot connected or if input from pilot
  // or if we have received input recently
  if(rc_thr == 0 || rc_str == 0 ||
     rc_thr < SAFETY_DEADZONE_MIN || rc_thr > SAFETY_DEADZONE_MAX ||
     rc_str < SAFETY_DEADZONE_MIN || rc_str > SAFETY_DEADZONE_MAX) {
    override_time = now;
  } else {
    no_override_time = now;
  }
  if(now - no_override_time > SAFETY_DROPOUT_TIMEOUT) {
    override = true;
  }
  if(now - override_time > SAFETY_DELAY) {
    override = false;
  }
  return override;
}

enum Thr_State{s_forward, s_brake, s_wait, s_rev};

void setServos(float steer, float thr) {
  static Thr_State state = s_forward;
  static long rev_time = 0;
  // write() takes angle values between 0 and 180 (stupid)
  steering.write(steer*SERVO_RANGE + 90);
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
      }
      break;
    case s_brake:
      if(millis() - rev_time > BRAKE_DELAY) {
        throttle.write(90);
        rev_time = millis();
        state = s_wait;
      }
      break;
    case s_wait:
      if(millis() - rev_time > BRAKE_DELAY) {
        state = s_rev;
      }
      break;
    case s_rev:
      throttle.write(thr*90 + 90);
      if(thr > 0.0f) {
        state = s_forward;
      }
      break;
    default:
      break;
  }
}

enum Com_State{s_idle, s_start, s_reset, s_c1, s_c2, s_c3};

void parseCmd(char data) {
  static Com_State state = s_idle;
  static String val_str;
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
        steer = sat(val_str.toFloat(), 1.0f, -1.0f);
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
        thr = sat(val_str.toFloat(), 1.0f, -1.0f);
        //serial.println(thr);
        val_str = String("");
        last_command_time = millis();
        state = s_idle;
      } else {
        steer = 0.0;
        thr = 0.0;
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
  int rc_thr_copy;
  int rc_str_copy;
  static bool toggle1 = false;
  static bool toggle2 = false;

  noInterrupts();
  pos_copy = pos;
  if(pos == 0) {
    diff = 0;
  }
  diff_copy = diff;
  rc_thr_copy = thr_pwm;
  rc_str_copy = str_pwm;
  dist_back = sonar_pwm/147.0*0.0254; // 147 microseconds per inch converted to meters
  interrupts();


  // convert the pulses into SI units (m and m/s)
  float dist = (float)pos_copy/PULSES_PER_M;
  float vel = (float)diff_copy/PULSES_PER_M*RATE;

  // create a checksum by doing a byte-wise xor of the data
  bool rc_override_copy = rc_override;
  char checksum = createChecksum(&dist, &vel, &dist_back, &rc_thr_copy, &rc_str_copy, &rc_override_copy);
  serial.print("[");
  serial.print(dist, 3);
  serial.print(",");
  serial.print(vel, 3);
  serial.print(",");
  serial.print(dist_back, 3);
  serial.print(",");
  serial.print(rc_str_copy);
  serial.print(",");
  serial.print(rc_thr_copy);
  serial.print(",");
  serial.print(rc_override_copy ? "1" : "0");
  serial.print(",");
  if(checksum < 100) serial.print("0");
  if(checksum < 10) serial.print("0");
  serial.print((int)checksum);
  //printf("%03d", checksum);
  serial.print("]");
  serial.println();
  //delay(100);
  if(diff_copy != 0) {
    digitalWrite(LED_PIN, HIGH);   // set the LED on
  } else {
    digitalWrite(LED_PIN, LOW);   // set the LED off
  }

  // if a character is sent from the serial monitor,
  // reset both back to zero.
  while (serial.available()) {
    parseCmd(serial.read());
  }

  // Check for safety pilot input
  // if no input act normal
  // else pass through pilot commands and don't go back to normal until
  // no input for a couple seconds
  rc_override = check_safety_override(rc_thr_copy, rc_str_copy);
  if(rc_override) {
    if(rc_thr_copy > SERVO_MIN || rc_thr_copy < SERVO_MAX ||
       rc_str_copy > SERVO_MIN || rc_str_copy < SERVO_MAX) {
      throttle.write((rc_thr_copy - 1500)/500.*SERVO_RANGE + 90);
      steering.write((rc_str_copy - 1500)/500.*SERVO_RANGE + 90);
    } else {
      steer = 0.0;
      thr = 0.0;
      setServos(steer, thr);
    }
    digitalWrite(LED_PIN, toggle2);   // set the LED blink to indicate override
    toggle1 = !toggle1;
    if(toggle1) toggle2 = !toggle2;
    steer = 0.0;
    thr = 0.0;
  } else {
    long now = millis();
    if(now - last_command_time > COMMAND_TIMEOUT) {
      steer = 0.0;
      thr = 0.0;
    }
    setServos(steer, thr);
  }

  // This is the Governator. Do not disable, or you will be Terminated!
  if(abs(vel) > MAX_VEL) {
    // slow down fool!
    throttle.write(90);
  }

  // calculate how long all of our stuff took
  long long now = (long long)micros();
  long long dur = now - prevTime;
  //serial.print((int)dur);

  // if the loop took less time than our desired period,
  // sleep until the next iteration should happen
  // this could be more graceful, but I don't care right now
  if(dur > 0 && dur < US_PER_S/RATE) {
    delayMicroseconds(US_PER_S/RATE - dur);
    //serial.print("sleeping");
  }
  prevTime = (long long)micros();
}
