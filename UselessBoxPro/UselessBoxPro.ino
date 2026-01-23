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
const int stepDelayMsFast = 10; // servo step delay resulting in fast speed
const int stepDelayMsSlow = 60; // servo step delay resulting in slow speed

// Behavior context
struct BehaviorContext {
  bool toggleStatus;   // true = ON (switch pressed), false = OFF
  float distanceCm;    // proximity sensor distance in cm
};

typedef void (*BehaviorFn)(const BehaviorContext& ctx);

struct WeightedBehavior {
  BehaviorFn fn;
  uint8_t weight;
};

// Setup
void setup() {
  pinMode(togglePin, INPUT_PULLUP); // enable internal pull-up
  pinMode(ledDebugPin, OUTPUT); // onboard led shows toggle status
  fingerServo.attach(fingerServoPin); // attache the servo pin to the Servo object
  fingerServo.write(fingerServoPos); // set the servo initial position
  lidServo.attach(lidServoPin); // attache the servo pin to the Servo object
  lidServo.write(lidServoPos); // set the servo initial position
  Serial.begin(9600);
  randomSeed(analogRead(A0)); // random seed
}

/////////////////////////////// FUNCTIONS //////////////////////////////////

// Move fingerServo from current pos to target pos with a step delay
void fingerServoMove(int targetPos, int stepDelayMs) {
  if (fingerServoPos < targetPos) {
    for (; fingerServoPos <= targetPos; fingerServoPos++) {
      fingerServo.write(fingerServoPos);
      delay(stepDelayMs);
    }
  } else {
    for (; fingerServoPos >= targetPos; fingerServoPos--) {
      fingerServo.write(fingerServoPos);
      delay(stepDelayMs);
    }
  }
}

// Move lidServo from current pos to target pos with a step delay
void lidServoMove(int targetPos, int stepDelayMs) {
  if (lidServoPos < targetPos) {
    for (; lidServoPos <= targetPos; lidServoPos++) {
      lidServo.write(lidServoPos);
      delay(stepDelayMs);
    }
  } else {
    for (; lidServoPos >= targetPos; lidServoPos--) {
      lidServo.write(lidServoPos);
      delay(stepDelayMs);
    }
  }
}

float readProximity() {
  // TODO: replace with your sensor read (HC-SR04, VL53L0X, Sharp IR, etc.)
  // For now, return a placeholder.
  return 999.0;
}

// Read toggle status
bool readToggleStatus() {
  return (digitalRead(togglePin) == LOW);
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

void behavior_0(const BehaviorContext& ctx) {
  if (ctx.toggleStatus == true) {
    digitalWrite(ledDebugPin, HIGH); // DEBUG

    lidServoMove(lidServoPosOpen, stepDelayMsFast); // open lid fast
    fingerServoMove(fingerServoPosToggle, stepDelayMsFast); // finger go fast
    fingerServoMove(fingerServoPosStationary, stepDelayMsFast); // finger return fast
    lidServoMove(lidServoPosClose, stepDelayMsFast); // close lid fast

    digitalWrite(ledDebugPin, LOW); // DEBUG
  }
}

void behavior_1(const BehaviorContext& ctx) {
  if (ctx.toggleStatus == true) {
    digitalWrite(ledDebugPin, HIGH); // DEBUG

    lidServoMove(lidServoPosOpen, stepDelayMsFast); // open lid fast
    fingerServoMove(fingerServoPosToggle, stepDelayMsSlow); // finger go slow
    fingerServoMove(fingerServoPosStationary, stepDelayMsSlow); // finger return slow
    lidServoMove(lidServoPosClose, stepDelayMsFast); // close lid fast

    digitalWrite(ledDebugPin, LOW); // DEBUG
  }
}

void behavior_2(const BehaviorContext& ctx) {
  if (ctx.toggleStatus == true) {
    digitalWrite(ledDebugPin, HIGH); // DEBUG
    int lidServoStepDelayMs = randomBetween(stepDelayMsSlow, stepDelayMsFast);
    int fingerServoStepDelayMs = randomBetween(stepDelayMsSlow, stepDelayMsFast);

    lidServoMove(lidServoPosOpen, lidServoStepDelayMs); // open lid random speed
    fingerServoMove(fingerServoPosToggle, fingerServoStepDelayMs); // finger go random speed
    fingerServoMove(fingerServoPosStationary, fingerServoStepDelayMs); // finger return random speed
    lidServoMove(lidServoPosClose, lidServoStepDelayMs); // close lid random speed

    digitalWrite(ledDebugPin, LOW); // DEBUG
  }
}

// behaviors occurance weights
WeightedBehavior behaviors[] = {
  { behavior_0, 1 },
  { behavior_1, 1 },
  { behavior_2, 5 },
};

// compile behaviors
const uint8_t behaviorCount = sizeof(behaviors) / sizeof(behaviors[0]);

BehaviorFn pickBehavior() {
  uint16_t totalWeight = 0;
  for (uint8_t i = 0; i < behaviorCount; i++) totalWeight += behaviors[i].weight;

  uint16_t r = random(totalWeight);

  uint16_t cumulative = 0;
  for (uint8_t i = 0; i < behaviorCount; i++) {
    cumulative += behaviors[i].weight;
    if (r < cumulative) return behaviors[i].fn;
  }
  return behaviors[0].fn;
}

///////////////////////////////// MAIN /////////////////////////////////////

void loop() {
  BehaviorContext ctx;
  ctx.toggleStatus = readToggleStatus(); // Check toggle on/off state
  ctx.distanceCm = readProximity(); // Check proximity sensor min value
  BehaviorFn behavior = pickBehavior();
  behavior(ctx);
  delay(50);
}