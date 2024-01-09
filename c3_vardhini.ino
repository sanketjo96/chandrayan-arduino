#include <AccelStepper.h>
#include <FastLED.h>

// Define
#define SECOND 1000UL
#define MINUTE (SECOND * 60UL)
#define HOUR (MINUTE * 60UL)
#define motorInterfaceType 1

// Global constants
const int MAX_SPEED = 12000;
const int HOME_SPEED = 1000;
const int NORMAL_SPEED = 500;
long TOTAL_X_STEPS = -28000;

const int NUM_LEDS = 300;
const unsigned long ROCKET_LIGHT_TIME = 10 * SECOND;
const unsigned long ROCKET_SMOKE_TIME = 10 * SECOND;
const unsigned long ROCKET_TO_EATH_ORBIT_TIME = 2 * SECOND;
const unsigned long EARTH_ORBIT_TIME = 5 * SECOND;
const unsigned long MOON_ORBIT_TIME = 1 * MINUTE;
const unsigned long ORBITOR_TO_MOON_TIME = 2 * SECOND;
const unsigned long MOON_STAY_TIME = 2 * SECOND;

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

// Define relay
const int rocketLightPin = A3;
const int smokePin = A2;
const int LedDataPin = A1;
const int orbitorPin = A0;
const int moonLightPin = 13;

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
  pinMode(rocketLightPin, OUTPUT);
  pinMode(moonLightPin, OUTPUT);
  
  digitalWrite(LedDataPin, HIGH);
  digitalWrite(orbitorPin, HIGH);
  digitalWrite(smokePin, HIGH);
  digitalWrite(rocketLightPin, HIGH);
  digitalWrite(moonLightPin, HIGH);
  digitalWrite(LedDataPin, LOW);

  FastLED.addLeds<WS2811, LedDataPin, RGB>(leds, NUM_LEDS);
  motorHoming(stepperX, xHomePin, initialHomingX);
  motorHoming(stepperY, yHomePin, initialHomingY);
}

void loop() {
  if (!isReversing) {
    if (stepperX.distanceToGo() != 0) {
      digitalWrite(smokePin, LOW);
      applyRocketSectionLights();
      stepperX.runSpeedToPosition();
    } else {
      if (isOrbiting) {
        digitalWrite(smokePin, HIGH);
        digitalWrite(rocketLightPin, HIGH);
        applyEarthOrbit();
        applyMoonOrbit();
        isOrbiting = false;
      } else {
        if (stepperY.distanceToGo() != 0) {  
          applyLanderSectionLights();        
          stepperY.runSpeedToPosition();
        } else {
            delay(MOON_STAY_TIME);
            digitalWrite(moonLightPin, HIGH);
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

void applyEarthOrbit () {
  showLedStrip(51, 255, 255, 100, 10);
}

void applyMoonOrbit () {
  digitalWrite(LedDataPin, LOW);

  digitalWrite(orbitorPin, LOW);
  delay(MOON_ORBIT_TIME);
  digitalWrite(orbitorPin, HIGH);
  delay(ORBITOR_TO_MOON_TIME);
}

void applyRocketSectionLights() {
  digitalWrite(rocketLightPin, LOW);
}

void applySmoke() {
  digitalWrite(smokePin, LOW);
  delay(ROCKET_SMOKE_TIME);
  digitalWrite(smokePin, HIGH);
}

void applyLanderSectionLights() {
  digitalWrite(moonLightPin, LOW);
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