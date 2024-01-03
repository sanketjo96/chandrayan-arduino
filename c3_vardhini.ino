#include <AccelStepper.h>
#include <FastLED.h>

// Define motor interface type
#define motorInterfaceType 1

// global constants
const int MAX_SPEED = 12000;
const int HOME_SPEED = 1000;
const int NORMAL_SPEED = 500;
long TOTAL_X_STEPS = -29000;

const int NUM_LEDS = 300;
const int ROCKET_SMOKE_TIME = 5000;
const int ROCKET_TO_EATH_ORBIT_TIME = 2000;
const int EARTH_ORBIT_TIME = 5000;
const int MOON_ORBIT_TIME = 5000;
const int ORBITOR_TO_MOON_TIME = 2000;
const int MOON_STAY_TIME = 2000;

// global flags
bool isOrbiting = true;
bool isReversing = false;


// Define X motor 
const int xStepPin = 2;
const int xDirPin = 5;
const int xHomePin = 9;
long initialHomingX=-1;


// Define Y motor 
const int yStepPin = 3;
const int yDirPin = 6;
const int yHomePin = 10;
long initialHomingY=-1;

const int LedDataPin = A1;
const int orbitorPin = A0;
const int smokePin = A2;
const int MasterPin = A3;

// Creates an instance
AccelStepper stepperX(motorInterfaceType, xStepPin, xDirPin);
AccelStepper stepperY(motorInterfaceType, yStepPin, yDirPin);
CRGB leds[NUM_LEDS];

void initFlags () {
  isOrbiting = true;
  isReversing = false;
  long initialHomingX=-1;
  initialHomingY = -1;
}

void setup() {  
  Serial.begin(9600);
  pinMode(xHomePin, INPUT_PULLUP);
  pinMode(yHomePin, INPUT_PULLUP);
  pinMode(orbitorPin, OUTPUT);
  pinMode(smokePin, OUTPUT);
  
  digitalWrite(LedDataPin, LOW);
  digitalWrite(LedDataPin, LOW);
  FastLED.addLeds<WS2811, LedDataPin, RGB>(leds, NUM_LEDS);

  motorHoming(stepperX, xHomePin, initialHomingX);
  motorHoming(stepperY, yHomePin, initialHomingY);
}

void loop() {
  if (!isReversing) {
    if (stepperX.distanceToGo() != 0) {
      stepperX.runSpeedToPosition();
    } else {
      if (isOrbiting) {
        applyEarthOrbit();
        applyMoonOrbit();
        isOrbiting = false;
      } else {
        if (stepperY.distanceToGo() != 0) {
          stepperY.runSpeedToPosition();
        } else {
            delay(MOON_STAY_TIME);
            isReversing = true;
        }
      }
    }
  } else {
    stepperX.moveTo(0);
    stepperX.setMaxSpeed(MAX_SPEED);
    stepperX.setSpeed(NORMAL_SPEED);
    stepperX.runSpeedToPosition();

    stepperY.moveTo(0);
    stepperY.setMaxSpeed(MAX_SPEED);
    stepperY.setSpeed(NORMAL_SPEED);
    stepperY.runSpeedToPosition();

    if (stepperY.distanceToGo() == 0 && stepperX.distanceToGo() == 0) {
      initFlags();
    }
  }
}

void motorHoming(AccelStepper &motor, int homePin, int homeFlag) {
  Serial.print("Motor is homing...\n");
  delay(500);

  while(digitalRead(homePin)) {
    motor.moveTo(homeFlag);
    motor.setMaxSpeed(MAX_SPEED);
    motor.setSpeed(HOME_SPEED);
    motor.runSpeedToPosition();
    homeFlag++;
  }

  homeFlag = 1;
  while(!digitalRead(homePin)) {
    motor.moveTo(homeFlag);
    motor.setMaxSpeed(MAX_SPEED);
    motor.setSpeed(HOME_SPEED);
    motor.runSpeedToPosition();
    homeFlag--;
  }

  motor.setCurrentPosition(0);
  motor.moveTo(TOTAL_X_STEPS);
  motor.setMaxSpeed(MAX_SPEED);
  motor.setSpeed(NORMAL_SPEED);

  Serial.print("Motor is done...\n");
  delay(1000);
}

void applySmoke() {
  digitalWrite(smokePin, HIGH);
  delay(ROCKET_SMOKE_TIME);
  digitalWrite(smokePin, LOW);
}

void applyEarthOrbit () {
  showLedStrip(51, 255, 255, 100, 10);
}

void applyMoonOrbit () {
  digitalWrite(orbitorPin, HIGH);
  delay(MOON_ORBIT_TIME);
  digitalWrite(orbitorPin, LOW);
  delay(ORBITOR_TO_MOON_TIME);
}

void applyRocketSectionLights() {

}

void applyLanderSectionLights() {

}

void showLedStrip(int R, int G, int B, int sleepMiliSec, int brightness) {
  FastLED.setBrightness(brightness);
   for(int whiteLed = 0; whiteLed < NUM_LEDS; whiteLed = whiteLed + 1) {
      leds[whiteLed] =  CRGB(R,  G,  B);
      FastLED.show();
      if (sleepMiliSec) {
        delay(sleepMiliSec);
      }
   }
}