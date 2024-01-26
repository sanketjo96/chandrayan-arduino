#include <AccelStepper.h>
#include <FastLED.h>
#include "ezButton.h";

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
long TOTAL_Y_STEPS = -30000;

const int isPressed = LOW;
const int isReleased = HIGH;

const int NUM_LEDS = 300;
const int THRUSTER1 = 64;
const int THRUSTER2 = 146;
const int THRUSTER3 = 246;

const unsigned long ROCKET_SMOKE_TILL_STEPS = 6000;
const unsigned long ROCKET_TO_EATH_ORBIT_PAUSE = 2 * SECOND;
const unsigned long MOON_ORBIT_TIME = 60 * SECOND;
const unsigned long ORBITOR_TO_MOON_TIME = 10 * SECOND;
const unsigned long MOON_STAY_TIME = 120 * SECOND;

// global flags
bool isOrbiting = true;
bool isReversing = false;
bool isTriggered = false;


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

// Define relay pins
const int rocketLightPin = A3;
const int smokePin = A0;
const int moonOrbitorPin = A1;
const int moonLightPin = A2;

// Define other pins
const int masterPin = 4;
const int earthOrbitPin = 13;

// Creates an instance
AccelStepper stepperX(motorInterfaceType, xStepPin, xDirPin);
AccelStepper stepperY(motorInterfaceType, yStepPin, yDirPin);
CRGB leds[NUM_LEDS];

ezButton masterSwitch(masterPin);

void initFlags () {
  isTriggered = false;
  isOrbiting = true;
  isReversing = false;
  initialHomingX=-1;
  initialHomingY = -1;
}

void initPins () {
  digitalWrite(rocketLightPin, HIGH);
  digitalWrite(smokePin, HIGH);
  digitalWrite(moonOrbitorPin, HIGH);
  digitalWrite(moonLightPin, HIGH);
}

void setup() {  
  Serial.begin(9600);
  //pinMode(masterPin, INPUT_PULLUP);
  pinMode(xHomePin, INPUT_PULLUP);
  pinMode(yHomePin, INPUT_PULLUP);
  pinMode(moonOrbitorPin, OUTPUT);
  pinMode(smokePin, OUTPUT);
  pinMode(rocketLightPin, OUTPUT);
  pinMode(moonLightPin, OUTPUT);

  masterSwitch.setDebounceTime(50);
  
  initPins();

  FastLED.addLeds<WS2811, earthOrbitPin, RGB>(leds, NUM_LEDS);
  clearLEDStrip();

  motorHoming(stepperX, xHomePin, initialHomingX, TOTAL_X_STEPS, 1);
  motorHoming(stepperY, yHomePin, initialHomingY, TOTAL_Y_STEPS, 2);
}

void loop() {
  masterSwitch.loop();
  int masterSwitchState = masterSwitch.getState();

  if (isTriggered) {
    if (!isReversing) {
      if (stepperX.distanceToGo() != 0) {
        digitalWrite(rocketLightPin, LOW);
        applySmoke(stepperX.distanceToGo());
        stepperX.runSpeedToPosition();
      } else {
        if (isOrbiting) {
          Serial.print("Earth orbit\n");
          digitalWrite(rocketLightPin, HIGH);
          digitalWrite(moonLightPin, LOW);
          applyEarthOrbit();
          digitalWrite(moonLightPin, HIGH);
          Serial.print("Moon orbit\n");
          applyMoonOrbit();
          isOrbiting = false;
          Serial.print("Start Lander\n");
        } else {
          if (stepperY.distanceToGo() != 0) {  
            digitalWrite(moonLightPin, LOW);     
            stepperY.runSpeedToPosition();
          } else {
              delay(MOON_STAY_TIME);
              digitalWrite(moonLightPin, HIGH);
              isReversing = true;
              Serial.print("Reverse\n");
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
        isTriggered = false;
        motorInit(stepperX, TOTAL_X_STEPS);
        motorInit(stepperY, TOTAL_Y_STEPS);
        Serial.print("Back to Zero\n");
      }
    }
  } else {
    if (masterSwitchState == isPressed) {
      initFlags();
      isTriggered = true;
      Serial.print("\nTriggered Rocket\n");
    }
  }
}

void motorInit(AccelStepper &motor, long totalSteps) {
  motor.setCurrentPosition(0);
  motor.moveTo(totalSteps);
  motor.setMaxSpeed(MAX_SPEED);
  motor.setSpeed(NORMAL_SPEED);
}

void motorHoming(AccelStepper &motor, int homePin, int homeFlag, long totalSteps, int index) {
  Serial.print((String)"\nMotor is homing: " + index);
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
  motor.moveTo(totalSteps);
  motor.setMaxSpeed(MAX_SPEED);
  motor.setSpeed(NORMAL_SPEED);

  Serial.print((String)"\nMotor homing done: " + index);
  delay(1000);
}

void applySmoke(long distanceToCover) {
  if (distanceToCover == TOTAL_X_STEPS) {
    delay(2000);
    digitalWrite(smokePin, LOW);
    delay(5000);
  }

  if (distanceToCover == (TOTAL_X_STEPS + ROCKET_SMOKE_TILL_STEPS)) {
    digitalWrite(smokePin, HIGH);
  }
}

void applyMoonOrbit () {
  clearLEDStrip();
  digitalWrite(moonOrbitorPin, LOW);
  delay(MOON_ORBIT_TIME);
  digitalWrite(moonOrbitorPin, HIGH);
  // delay(ORBITOR_TO_MOON_TIME);
}

void applyEarthOrbit () {
  showLedStrip(255);
}

void showLedStrip(int brightness) {
  FastLED.setBrightness(brightness);
  int time = 300;
  for(int i = 0; i < (NUM_LEDS - 1); i = i + 1) {
    leds[i] =  CRGB(0, 255, 255);
    leds[i + 1] =  CRGB(0, 255, 0);
    FastLED.show();

    time = fireThrusters(i, time);
    delay(time);
    
    leds[i] =  CRGB(0, 0, 0);
    leds[i + 1] =  CRGB(0, 0, 0);
  }
}

void thrusterFireLight(int index) {
  leds[index + 1] =  CRGB(255, 0, 0);
  FastLED.show();
  delay(120);
}

int fireThrusters(int i, int startTime) {
  int time = startTime;
  if (i == THRUSTER1) { 
    thrusterFireLight(i);
    time = 200;
  } else if (i == THRUSTER2) {
    thrusterFireLight(i);
    time = 100;
  } else if (i == THRUSTER3) {
    thrusterFireLight(i);
    time = 60;
  }
  return time;
}

void clearLEDStrip() {
  FastLED.clear();
  FastLED.show();
}