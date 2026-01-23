/* Sweep
Useless Box Pro
Modern Making Course Project by Justin
By Aslan, Anouk, and Liza
*/

/////////////////////////////// SETUP //////////////////////////////////////

#include <Servo.h>

Servo fingerServo; // controls finger
Servo lidServo; // controls lid

// Pins
const int ledDebugPin = 13; // onboard LED (Arduino Uno), represents system awakening
const int togglePin = 2;
const int fingerServoPin = 5;
const int lidServoPin = 6;

// Servo positions checkpoints
const int fingerServoPosStationary = 0; // pos of fingerServo when stationary
const int fingerServoPosToggle = 60; //  pos of fingerServo when toggle
const int lidServoPosClose = 0; // pos of lidServo close
const int lidServoPosOpen = 20; // pos of lidServo open

// Servo pos variables
int fingerServoPos = fingerServoPosStationary;
int lidServoPos = lidServoPosClose;
int targetDegree = 0; // variable for controlling servo target
int stepDelayMs = 10; // variable for controlling servo speed

// Behavior variables
typedef void (*BehaviorFn)();

struct WeightedBehavior {
  BehaviorFn fn;
  uint8_t weight;
};

// Setup
void setup() {
  pinMode(togglePin, INPUT_PULLUP); // enable internal pull-up
  pinMode(ledDebugPin, OUTPUT); // onboard led shows toggle status
  fingerServo.attach(fingerServoPin); // attaches the servo pin to the Servo object
  fingerServo.write(fingerServoPos); // set the servo initial position
  lidServo.attach(lidServoPin); // attaches the servo pin to the Servo object
  lidServo.write(lidServoPos); // set the servo initial position
  Serial.begin(9600);
  randomSeed(analogRead(A0)); // random seed
}

/////////////////////////////// FUNCTIONS //////////////////////////////////

// Move fingerServo from current pos to targetDegree with a step delay
void fingerServoMove(int targetDegree, int stepDelayMs = 10) {
  //targetDegree = constrain(targetDegree, 0, 180);
  if (fingerServoPos < targetDegree) {
    for (fingerServoPos; fingerServoPos <= targetDegree; fingerServoPos++) {
      fingerServo.write(fingerServoPos);
      delay(stepDelayMs);
    }
  } else {
    for (fingerServoPos; fingerServoPos >= targetDegree; fingerServoPos--) {
      fingerServo.write(fingerServoPos);
      delay(stepDelayMs);
    }
  }
}

// Move lidServo from current pos to targetDegree with a step delay
void lidServoMove(int targetDegree, int stepDelayMs = 10) {
  //targetDegree = constrain(targetDegree, 0, 180);
  if (lidServoPos < targetDegree) {
    for (lidServoPos; lidServoPos <= targetDegree; lidServoPos++) {
      lidServo.write(lidServoPos);
      delay(stepDelayMs);
    }
  } else {
    for (lidServoPos; lidServoPos >= targetDegree; lidServoPos--) {
      lidServo.write(lidServoPos);
      delay(stepDelayMs);
    }
  }
}

// Generate a random number between low and high
int randomBetween(int low, int high) {
  if (low > high) {
    int temp = low;
    low = high;
    high = temp;
  }
  return random(low, high + 1);
}

/////////////////////////////// BEHAVIORS //////////////////////////////////

// open lid fast, move finger fast, return finger fast, close lid fast
void behavior_0() {
  // open lid
  // finger go
  targetDegree = fingerServoPosToggle - fingerServoPosStationary;
  stepDelayMs = 10;
  fingerServoMove(targetDegree, stepDelayMs);
  // finger return 
  targetDegree = fingerServoPosStationary - fingerServoPosToggle;
  stepDelayMs = 10;
  fingerServoMove(targetDegree, stepDelayMs);
  // close lid
}

// open lid fast, move finger slow, return finger slow, close lid fast
void behavior_1() {
  // open lid
  // finger go
  targetDegree = fingerServoPosToggle - fingerServoPosStationary;
  stepDelayMs = 50;
  fingerServoMove(targetDegree, stepDelayMs);
  // finger return 
  targetDegree = fingerServoPosStationary - fingerServoPosToggle;
  stepDelayMs = 50;
  fingerServoMove(targetDegree, stepDelayMs);
  // close lid
}

// open lid fast, move finger slow, return finger fast, close lid fast
void behavior_2() {
  // open lid
  // finger go
  targetDegree = fingerServoPosToggle - fingerServoPosStationary;
  stepDelayMs = 50;
  fingerServoMove(targetDegree, stepDelayMs);
  // finger return 
  targetDegree = fingerServoPosStationary - fingerServoPosToggle;
  stepDelayMs = 10;
  fingerServoMove(targetDegree, stepDelayMs);
  // close lid
}

// behaviors occurance weights
WeightedBehavior behaviors[] = {
  { behavior_0, 1 },
  { behavior_1, 1 },
  { behavior_2, 1 },
};

// compile behaviors
const uint8_t behaviorCount = sizeof(behaviors) / sizeof(behaviors[0]);

BehaviorFn pickBehavior() {
  uint16_t totalWeight = 0;

  for (uint8_t i = 0; i < behaviorCount; i++) {
    totalWeight += behaviors[i].weight;
  }

  uint16_t r = random(totalWeight);

  uint16_t cumulative = 0;
  for (uint8_t i = 0; i < behaviorCount; i++) {
    cumulative += behaviors[i].weight;
    if (r < cumulative) {
      return behaviors[i].fn;
    }
  }

  return behaviors[0].fn; // safety fallback
}

///////////////////////////////// MAIN /////////////////////////////////////

void loop() {
  bool isOn = (digitalRead(togglePin) == LOW); // Check toggle on/off state

  //digitalWrite(ledDebugPin, isOn ? HIGH : LOW);
  if (isOn) {
    digitalWrite(ledDebugPin, HIGH); // DEBUG: toggle on -> led on
    // random behavior
    BehaviorFn toggle = pickBehavior();
    toggle();
  }
  else {
    digitalWrite(ledDebugPin, LOW); // DEBUG: toggle off -> led off
  }

  delay(100);
}