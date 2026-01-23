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
const uint8_t ledDebugPin = 13; // onboard LED (Arduino Uno), represents system awakening
const uint8_t togglePin = 2;
const uint8_t fingerServoPin = 5;
const uint8_t lidServoPin = 6;
const uint8_t XSHUT_PINS[3] = {9, 10, 11}; // proximity sensor pins

// Servo positions checkpoints
const uint8_t fingerServoPosStationary = 0; // pos of fingerServo when stationary
const uint8_t fingerServoPosToggle = 60; //  pos of fingerServo when toggle
const uint8_t lidServoPosClose = 0; // pos of lidServo close
const uint8_t lidServoPosOpen = 20; // pos of lidServo open
const uint8_t I2C_ADDRESSES[3]  = {0x30, 0x31, 0x32}; // new unique addresses

// Servo pos variables
int fingerServoPos = fingerServoPosStationary;
int lidServoPos = lidServoPosClose;
const int stepDelayMsFast = 10; // servo step delay resulting in fast speed
const int stepDelayMsSlow = 60; // servo step delay resulting in slow speed

// Behavior context
struct BehaviorContext {
  bool toggleStatus;   // true = ON (switch pressed), false = OFF
  float distance;    // proximity sensor distance in cm
};

typedef void (*BehaviorFn)(const BehaviorContext& ctx);

struct WeightedBehavior {
  BehaviorFn fn;
  uint8_t weight;
};

void printContext(const BehaviorContext& ctx) {
  Serial.print("toggle status: ");
  Serial.print(ctx.toggleStatus ? "ON" : "OFF");
  Serial.print(" | distance (cm): ");
  Serial.println(ctx.distance, 1);  // 1 decimal place
}

/////////////////////////////// FUNCTIONS //////////////////////////////////

// Move fingerServo from current pos to target pos with a step delay
void fingerServoMove(int targetPos, int stepDelayMs) {
  fingerServo.attach(fingerServoPin);
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
  fingerServo.detach();
}

// Move lidServo from current pos to target pos with a step delay
void lidServoMove(int targetPos, int stepDelayMs) {
  lidServo.attach(lidServoPin);
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
  lidServo.detach();
}

// Read proximity sensors
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

// Setup
void setup() {
  pinMode(togglePin, INPUT_PULLUP); // enable internal pull-up
  pinMode(ledDebugPin, OUTPUT); // onboard led shows toggle status
  fingerServoMove(fingerServoPos, stepDelayMsFast); // set the servo initial position
  lidServoMove(lidServoPos, stepDelayMsFast); // set the servo initial position
  Serial.begin(9600);
  randomSeed(analogRead(A0)); // random seed
}

/////////////////////////////// BEHAVIORS //////////////////////////////////

void behavior_0(const BehaviorContext& ctx) {
  Serial.println(__func__); // DEBUG
  lidServoMove(lidServoPosOpen, stepDelayMsFast); // open lid fast
  fingerServoMove(fingerServoPosToggle, stepDelayMsFast); // finger go fast
  fingerServoMove(fingerServoPosStationary, stepDelayMsFast); // finger return fast
  lidServoMove(lidServoPosClose, stepDelayMsFast); // close lid fast
}

void behavior_1(const BehaviorContext& ctx) {
  Serial.println(__func__); // DEBUG
  lidServoMove(lidServoPosOpen, stepDelayMsFast); // open lid fast
  fingerServoMove(fingerServoPosToggle, stepDelayMsSlow); // finger go slow
  fingerServoMove(fingerServoPosStationary, stepDelayMsSlow); // finger return slow
  lidServoMove(lidServoPosClose, stepDelayMsFast); // close lid fast
}

void behavior_2(const BehaviorContext& ctx) {
  Serial.println(__func__); // DEBUG
  int lidServoStepDelayMs = randomBetween(stepDelayMsSlow, stepDelayMsFast);
  int fingerServoStepDelayMs = randomBetween(stepDelayMsSlow, stepDelayMsFast);
  lidServoMove(lidServoPosOpen, lidServoStepDelayMs); // open lid random speed
  fingerServoMove(fingerServoPosToggle, fingerServoStepDelayMs); // finger go random speed
  fingerServoMove(fingerServoPosStationary, fingerServoStepDelayMs); // finger return random speed
  lidServoMove(lidServoPosClose, lidServoStepDelayMs); // close lid random speed
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
  ctx.distance = readProximity(); // Check proximity sensor min value
  //printContext(ctx);   // DEBUG OUTPUT
  if (ctx.toggleStatus || ctx.distance < 20) {
    digitalWrite(ledDebugPin, HIGH); // DEBUG
    BehaviorFn behavior = pickBehavior();
    behavior(ctx);
    digitalWrite(ledDebugPin, LOW); // DEBUG
  }
  delay(50);
}