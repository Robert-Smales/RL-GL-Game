// MAIN BODY of the Pointer, This includes all actuator exectution (making the robot move)
// Takes all the inputs given through the serial connection and makes them 
// 
//

#include <AccelStepper.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN 27
#define NUM_LEDS 60

Adafruit_NeoPixel ring(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800); // Initialising the ring variable
AccelStepper pan(AccelStepper::DRIVER, 14, 12); // STEP, DIR pins for pan
AccelStepper tilt(AccelStepper::DRIVER, 2, 15); // STEP, DIR pins for tilt

const int MAX_ABS_SPEED = 4000; // steps/s, match Pi script

void clampSetSpeed(AccelStepper& m, int s) {
  if (s >  MAX_ABS_SPEED) s =  MAX_ABS_SPEED;
  if (s < -MAX_ABS_SPEED) s = -MAX_ABS_SPEED;
  m.setSpeed(s);
}

void setColour (int r, int g, int b) {
  for (int i = 0; i < NUM_LEDS; i++) {
    ring.setPixelColor(i, ring.Color(r, g, b));
  }
  ring.show();
}

bool gotoMode = false;

//setting the base values for all the variables that need it
void setup() {
  Serial.begin(115200);
  pan.setMaxSpeed(MAX_ABS_SPEED);
  tilt.setMaxSpeed(MAX_ABS_SPEED);
  pan.setAcceleration(2000);
  tilt.setAcceleration(2000);
  pinMode(32, OUTPUT);
  digitalWrite(32, LOW);
  ring.begin();
  ring.show();
  setColour(0, 0, 0);
}

// main loop that turns the commands from the serial connection into actions through the pins on the arduino
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "L1") {
      digitalWrite(32, HIGH);
    } 
    else if (cmd == "L0") {
      digitalWrite(32, LOW);
    } 
    else if (cmd.startsWith("SPD")) {
      int spPan, spTilt;
      if (sscanf(cmd.c_str(), "SPD %d %d", &spPan, &spTilt) == 2) {
        gotoMode = false;
        clampSetSpeed(pan, spPan);
        clampSetSpeed(tilt, spTilt);
      }
    }
    else if (cmd.startsWith("GOTO_REL")) {
      long dpan, dtilt;
      if (sscanf(cmd.c_str(), "GOTO_REL %ld %ld", &dpan, &dtilt) == 2) {
        pan.move(dpan);
        tilt.move(dtilt);
        gotoMode = true;
      }
    }
    else if (cmd == "ZERO") {
      pan.setCurrentPosition(0);
      tilt.setCurrentPosition(0);
      pan.setSpeed(0);
      tilt.setSpeed(0);
      gotoMode = false;
    }
    else if (cmd == "HOME") {
      pan.moveTo(0);
      tilt.moveTo(0);
      gotoMode = true;
    }
    else if (cmd == "LED_RED") {
      setColour(128, 0, 0);
    }
    else if (cmd == "LED_GREEN") {
      setColour(0, 128, 0);
    }
    else if (cmd == "LED_OFF") {
      setColour(0, 0, 0);
    }
    else if (cmd == "S1") {
      digitalWrite(4, HIGH);
    } 
    else if (cmd == "S0") {
      digitalWrite(4, LOW);
    }
  }
  if (gotoMode) {
    bool panMoving = pan.run();
    bool tiltMoving = tilt.run();
    if (!panMoving && !tiltMoving) {
      gotoMode = false;
    }
  } else {
    pan.runSpeed();
    tilt.runSpeed();
  }
}
