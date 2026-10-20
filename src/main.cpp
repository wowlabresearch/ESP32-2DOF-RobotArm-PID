/* MoRo 2025-2 @ Hongik Univ.
   ESP32-S3 DevKitC-1 firmware for the p5.js 2-DOF Robot Arm GUI (gui/sketch.js)
   ===
   Protocol (GUI -> ESP32), one line per GUI frame (~60 Hz):
       "U:<us1>,<us2>\n"       servo pulse width in microseconds (PULSE_MIN..PULSE_MAX), ~0.1 deg resolution
       "A:<cmd1>,<cmd2>\n"     legacy: servo angle 0..180 (integer degrees)
   - servo1 = theta1          (J1_OFFSET = 0)
   - servo2 = 90 + theta2     (J2_OFFSET = 90, servo 90deg = kinematic 0deg)
   The dynamics simulation and the PID / impedance controllers run in the GUI. This firmware only
   drives the servos. Between two received commands it interpolates linearly at 1 kHz over the
   measured command interval, so the motion is smooth instead of stepping at the GUI frame rate
   (this adds about one command interval of latency).
     * "Follow Sim" ON  -> GUI sends the simulated angle -> arm moves like the simulated response
     * "Follow Sim" OFF -> GUI sends the target pose      -> servo goes directly to the target
*/

#include <Arduino.h>
#include <ESP32Servo.h>

// ---- Hardware settings ----
const int  SERVO1_PIN = 4;      // J1 (shoulder) signal pin
const int  SERVO2_PIN = 5;      // J2 (elbow) signal pin
const long BAUD       = 115200; // must match the GUI's serial baud rate (UART port)
const int  PULSE_MIN  = 500;    // us at 0 deg   (tune for your servo; must match the GUI)
const int  PULSE_MAX  = 2400;   // us at 180 deg (tune for your servo; must match the GUI)
const int  SERVO_HZ   = 50;     // PWM frame rate. Analog servos (SG90 etc.): 50. Many digital servos accept 100-300

// Interpolation
const uint32_t INTERP_PERIOD_US = 1000;  // output update period (1 kHz)
const uint32_t SPAN_MIN_US      = 5000;  // clamp for the measured command interval
const uint32_t SPAN_MAX_US      = 100000;

// Initial pose = GUI's centerEE(): theta1 = 0, theta2 = 0  ->  servo (0 deg, 90 deg)
const float INIT_US1 = PULSE_MIN;
const float INIT_US2 = (PULSE_MIN + PULSE_MAX) / 2.0f;

Servo servo1, servo2;

// Linear interpolation from `from` to `to` over `span` microseconds, started at `t0`
struct Interp {
  float from, to, out;
  int written;  // last value sent to the servo (-1 = none)
};
Interp ch1 = {INIT_US1, INIT_US1, INIT_US1, -1};
Interp ch2 = {INIT_US2, INIT_US2, INIT_US2, -1};
uint32_t segStart = 0, segSpan = SPAN_MAX_US, lastCmdAt = 0, lastTick = 0;
bool haveCmd = false;

// One line buffer per serial port (native USB CDC + UART)
struct LineReader {
  char buf[32];
  uint8_t len = 0;
};
LineReader usbReader, uartReader;

float degToUs(float deg) {
  return PULSE_MIN + constrain(deg, 0.0f, 180.0f) / 180.0f * (PULSE_MAX - PULSE_MIN);
}

// New target: start a segment from the current output toward it, lasting one command interval
void setTarget(float us1, float us2) {
  uint32_t now = micros();
  segSpan = haveCmd ? constrain(now - lastCmdAt, SPAN_MIN_US, SPAN_MAX_US) : SPAN_MIN_US;
  lastCmdAt = now;
  haveCmd = true;
  segStart = now;
  ch1.from = ch1.out; ch1.to = constrain(us1, (float)PULSE_MIN, (float)PULSE_MAX);
  ch2.from = ch2.out; ch2.to = constrain(us2, (float)PULSE_MIN, (float)PULSE_MAX);
}

void writeChannel(Servo &s, Interp &c, float a) {
  c.out = c.from + (c.to - c.from) * a;
  int us = (int)lroundf(c.out);
  if (us != c.written) { s.writeMicroseconds(us); c.written = us; }
}

void updateOutputs() {
  float a = (float)(micros() - segStart) / segSpan;
  if (a > 1.0f) a = 1.0f;
  writeChannel(servo1, ch1, a);
  writeChannel(servo2, ch2, a);
}

// Parse one line: "U:1500,1500" or "A:123,45"
void handleLine(const char *s) {
  if (s[1] != ':') return;
  int c1, c2;
  if (sscanf(s + 2, "%d,%d", &c1, &c2) != 2) return;
  if (s[0] == 'U')      setTarget(c1, c2);
  else if (s[0] == 'A') setTarget(degToUs(c1), degToUs(c2));
}

void pollStream(Stream &port, LineReader &r) {
  while (port.available()) {
    char ch = (char)port.read();
    if (ch == '\n' || ch == '\r') {
      if (r.len > 0) {
        r.buf[r.len] = '\0';
        handleLine(r.buf);
        r.len = 0;
      }
    } else if (r.len < sizeof(r.buf) - 1) {
      r.buf[r.len++] = ch;
    } else {
      r.len = 0;  // overflow: drop garbage line
    }
  }
}

void setup() {
  Serial.begin(BAUD);   // native USB CDC (baud is ignored)
  Serial0.begin(BAUD);  // UART port

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servo1.setPeriodHertz(SERVO_HZ);
  servo2.setPeriodHertz(SERVO_HZ);
  servo1.attach(SERVO1_PIN, PULSE_MIN, PULSE_MAX);
  servo2.attach(SERVO2_PIN, PULSE_MIN, PULSE_MAX);

  updateOutputs();

  Serial.println("MoRo arm ready");
  Serial0.println("MoRo arm ready");
}

void loop() {
  pollStream(Serial, usbReader);
  pollStream(Serial0, uartReader);

  uint32_t now = micros();
  if (now - lastTick >= INTERP_PERIOD_US) {
    lastTick = now;
    updateOutputs();
  }
}
