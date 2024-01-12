#include <Servo.h>

// Define
#define SECOND 1000UL
#define MINUTE (SECOND * 60UL)
#define HOUR (MINUTE * 60UL)

const unsigned long SERVO_STEP_DELAY = 20;
const unsigned long TRACING_LED_DELAY = SERVO_STEP_DELAY;

const unsigned long MOON_STAY_TIME = 30 * SECOND;
const unsigned long DOOR_OPEN_DELAY = 10 * SECOND;
const unsigned long DOOR_CLOSE_DELAY = 2 * SECOND;


long starttime;
long endtime ;

const int irSensorPin = A0;           // IR sensor output pin (digital)
const int servoPin = 9;              // Servo motor control pin
int led = 8;
Servo doorServo;                     // Create a servo object

const int obstacleThreshold = LOW;   // IR sensor output for detecting an obstacle
const int clearPathThreshold = HIGH; // IR sensor output for clear path

const int servoStepDelay = 20;       // Delay between servo steps in milliseconds
const int servoStepAngle = 2;        // Angle to incrementally move the servo in each step
bool isDoorOpen = false;

void setup() {
  Serial.begin(9600);

  doorServo.attach(servoPin);        // Attach the servo to its pin
  doorServo.write(90);               // Set the initial position of the servo (90 degrees)

  pinMode(irSensorPin, INPUT);
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  delay (1000);
  digitalWrite(led, LOW);
}

void loop() {
  // Read the digital state of the IR sensor
  int irSensorState = digitalRead(irSensorPin);

  // Display the IR sensor state on the serial monitor
  Serial.print("IR Sensor State: ");
  Serial.println(irSensorState == obstacleThreshold ? "Obstacle detected" : "Clear path");

  // Check if an obstacle is detected
  if (isDoorOpen == false && irSensorState == obstacleThreshold) {
    blinkLedDuringLanding();
    delay(DOOR_OPEN_DELAY);
    openDoor();
    digitalWrite(led, HIGH);
    delay(DOOR_CLOSE_DELAY);
    gradualCloseDoor();
    blinkLedPostLanding();
    isDoorOpen = true;
  } 

  if (isDoorOpen == true && irSensorState == clearPathThreshold) {
    isDoorOpen = false;
  }

  // Wait for a moment before taking the next reading
  delay(100);
}

void openDoor() {
  Serial.println("Opening the door");
  for (int angle = 90; angle >= 0; angle -= servoStepAngle) {
    doorServo.write(angle);
    digitalWrite(led, HIGH);
    delay (servoStepDelay);
    digitalWrite(led, LOW);
    delay(servoStepDelay);
  }
}

void blinkLedDuringLanding () {
  for (int i = 0; i <= 100; i++) {
    digitalWrite(led, HIGH);
    delay(TRACING_LED_DELAY);
    digitalWrite(led, LOW);
    delay(TRACING_LED_DELAY);
  }
}

void blinkLedPostLanding () {
  starttime = millis();
  endtime = starttime;
  while((endtime - starttime) <= MOON_STAY_TIME) {
    digitalWrite(led, HIGH);
    delay(400);
    digitalWrite(led, LOW);
    delay(400);
    endtime = millis();
  }
}

void gradualCloseDoor() {
  Serial.println("Gradually closing the door");
  for (int angle = 0; angle <= 90; angle += servoStepAngle) {
    doorServo.write(angle);
    digitalWrite(led, HIGH);
    delay (servoStepDelay);
    digitalWrite(led, LOW);
    delay(servoStepDelay);
  }
}

void delayWhileObstaclePresent() {
  // Wait until the obstacle is no longer detected
  while (digitalRead(irSensorPin) == obstacleThreshold) {
    delay(100);
  }
}
