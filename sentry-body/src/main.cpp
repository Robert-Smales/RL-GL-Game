#include <AccelStepper.h>

AccelStepper pan(AccelStepper::DRIVER, 14, 12); // STEP, DIR pins for pan
AccelStepper tilt(AccelStepper::DRIVER, 2, 15); // STEP, DIR pins for tilt

const int MAX_ABS_SPEED = 4000; // steps/s, match Pi script

void clampSetSpeed(AccelStepper& m, int s) {
  if (s >  MAX_ABS_SPEED) s =  MAX_ABS_SPEED;
  if (s < -MAX_ABS_SPEED) s = -MAX_ABS_SPEED;
  m.setSpeed(s);
}

void setup() {
  Serial.begin(115200);
  pan.setMaxSpeed(MAX_ABS_SPEED);
  tilt.setMaxSpeed(MAX_ABS_SPEED);
}

void loop() {
  // Read lines like: "SPD <panSpeed> <tiltSpeed>\n"
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      int spPan, spTilt;
      if (sscanf(buf.c_str(), "SPD %d %d", &spPan, &spTilt) == 2) {
        clampSetSpeed(pan,  spPan);
        clampSetSpeed(tilt, spTilt);
      }
      buf = "";
    } else if (buf.length() < 40) {
      buf += c;
    }
  }

  pan.runSpeed();
  tilt.runSpeed();
}
