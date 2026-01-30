/* Sweep
Useless Box Pro
Modern Making Course Project by Justin
By Aslan, Anouk, and Liza
*/

///////////////////////////////////// IMPORTS /////////////////////////////////////

#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>  // Install "VL53L0X" (by Pololu) library first

////////////////////////////////////// PINS ///////////////////////////////////////

const uint8_t ledDebugPin = 13; // Onboard LED (Arduino Uno), represents system awakening
const uint8_t togglePin = 2; // Toggle key pin
const uint8_t fingerServoPin = 5; // Servo motor controlling finger pin
const uint8_t lidServoPin = 6; // Servo motor controlling lid pin
const uint8_t XSHUT_PINS[3] = {9, 10, 11}; // Proximity sensor pins

//////////////////////////////// COMPONENTS SETUP /////////////////////////////////

// Finger servo setup
Servo fingerServo; // controls finger
const uint8_t fingerServoPosStationary = 0; // Finger servo pos when stationary
const uint8_t fingerServoPosToggle = 60; //  Finger servo pos when toggle
int fingerServoPos = fingerServoPosStationary; // Finger servo pos variable
const int fingerServoStepDelayFast = 10; // Finger servo step delay resulting in fast speed
const int fingerServoStepDelaySlow = 60; // Finger servo step delay resulting in slow speed

// Lid servo setup
Servo lidServo; // controls lid
const uint8_t lidServoPosClose = 0; // Lid servo pos when lid is closed
const uint8_t lidServoPosOpen = 20; // Lid servo pos when lid is open
int lidServoPos = lidServoPosClose; // Lid servo pos variable
const int lidServoStepDelayFast = 10; // Lid servo step delay resulting in fast speed
const int lidServoStepDelaySlow = 60; // Lid servo step delay resulting in slow speed

// Proximity sensors setup
const uint16_t HAND_NEAR = 100;  // Hand is "near" if distance < HAND_NEAR
const uint8_t I2C_ADDRESSES[3]  = {0x30, 0x31, 0x32}; // New unique I2C addresses
VL53L0X tof[3]; // Proximity sensors (3x VL53L0X)

////////////////////////////////////// TOOLS //////////////////////////////////////

// Behavior context
struct BehaviorContext {
  bool toggleStatus;   // true = ON (switch pressed), false = OFF
  float distance;    // proximity sensor distance in mm
};

typedef void (*BehaviorFn)(const BehaviorContext& ctx);

struct WeightedBehavior {
  BehaviorFn fn;
  uint8_t weight;
};

// Print behavior context in serial (DEBUG)
void printContext(const BehaviorContext& ctx) {
  Serial.print("toggle status: ");
  Serial.print(ctx.toggleStatus ? "ON" : "OFF");
  Serial.print(" | distance (mm): ");
  Serial.println(ctx.distance, 0);  // 0 decimal place
}

// Move fingerServo from current pos to target pos with a step delay
void fingerServoMove(int targetPos, int stepDelay) {
  fingerServo.attach(fingerServoPin);
  if (fingerServoPos < targetPos) {
    for (; fingerServoPos <= targetPos; fingerServoPos++) {
      fingerServo.write(fingerServoPos);
      delay(stepDelay);
    }
  } else {
    for (; fingerServoPos >= targetPos; fingerServoPos--) {
      fingerServo.write(fingerServoPos);
      delay(stepDelay);
    }
  }
  fingerServo.detach();
}

// Move lidServo from current pos to target pos with a step delay
void lidServoMove(int targetPos, int stepDelay) {
  lidServo.attach(lidServoPin);
  if (lidServoPos < targetPos) {
    for (; lidServoPos <= targetPos; lidServoPos++) {
      lidServo.write(lidServoPos);
      delay(stepDelay);
    }
  } else {
    for (; lidServoPos >= targetPos; lidServoPos--) {
      lidServo.write(lidServoPos);
      delay(stepDelay);
    }
  }
  lidServo.detach();
}

// Read proximity sensors (mm) (minimum distance across 3 sensors)
float readProximity()
/*
{
  // If none valid, returns a large number.
  uint16_t best_mm = 65535;
  bool anyValid = false;
  for (uint8_t i = 0; i < 3; i++) {
    uint16_t d = tof[i].readRangeContinuousMillimeters();
    // Pololu library sets timeoutOccurred() if it failed to get a reading
    if (tof[i].timeoutOccurred()) {
      continue;
    }
    // Filter obvious junk / out-of-range values
    if (d == 0 || d > 2000) {
      continue;
    }
    anyValid = true;
    if (d < best_mm) best_mm = d;
  }
  if (!anyValid) return 999.0; // If none valid, returns a large number: "no hand detected"
  return best_mm; // mm
}
*/
{
  return 999;
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

////////////////////////////////////// SETUP //////////////////////////////////////

bool initServoMotors () {
  pinMode(togglePin, INPUT_PULLUP); // enable internal pull-up
  pinMode(ledDebugPin, OUTPUT); // onboard led shows toggle status
  fingerServoMove(fingerServoPos, fingerServoStepDelayFast); // set the servo initial position
  lidServoMove(lidServoPos, lidServoStepDelayFast); // set the servo initial position
  Serial.println(F("Servo motors ready."));
}

bool initToFSensors() {
  Wire.begin(); // Start I2C
  Wire.setClock(400000); // Optional: faster I2C; safe on short wires
  // XSHUT pins as outputs
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(XSHUT_PINS[i], OUTPUT);
  }
  // Shut down all sensors (so only one responds at 0x29)
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(XSHUT_PINS[i], LOW);
  }
  delay(10);
  // Bring them up one by one and re-address
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(XSHUT_PINS[i], HIGH);
    delay(10);
    tof[i].setTimeout(50); // Prevents hanging on bad sensor/wiring
    if (!tof[i].init()) {
      Serial.print(F("ToF init failed at index "));
      Serial.println(i);
      return false;
    }
    tof[i].setAddress(I2C_ADDRESSES[i]);
    // Continuous mode improves read speed + stability
    tof[i].startContinuous(0);
    delay(10);
  }
  Serial.println(F("3x VL53L0X ready."));
  return true;
}

// Setup
void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(A0)); // Random seed
  initServoMotors();
  /*
  if (!initToFSensors()) {
    Serial.println(F("VL53L0X init failed. Check wiring/XSHUT/VIN/GND/SDA/SCL."));
    while (1) {
      digitalWrite(ledDebugPin, !digitalRead(ledDebugPin));
      delay(200);
    }
  }
  */
}

//////////////////////////////////// BEHAVIORS ////////////////////////////////////

void behavior_0(const BehaviorContext& ctx) {
  if (ctx.toggleStatus) {
    Serial.println(__func__); // DEBUG
    digitalWrite(ledDebugPin, HIGH); // DEBUG: System awakening sign

    int lidServoStepDelay = randomBetween(fingerServoStepDelaySlow, fingerServoStepDelayFast);
    int fingerServoStepDelay = randomBetween(lidServoStepDelaySlow, lidServoStepDelayFast);
    int randomDelay = randomBetween(10, 1000);
    delay(randomDelay);
    lidServoMove(lidServoPosOpen, lidServoStepDelay); // Open lid random speed
    fingerServoMove(fingerServoPosToggle, fingerServoStepDelay); // Finger go random speed
    delay(20);
    fingerServoMove(fingerServoPosStationary, fingerServoStepDelay); // Finger return random speed
    lidServoMove(lidServoPosClose, lidServoStepDelay); // Close lid random speed

    digitalWrite(ledDebugPin, LOW); // DEBUG: System sleeping sign
  }
}

void behavior_1(const BehaviorContext& ctx) {
  if (ctx.toggleStatus) {
    behavior_0(ctx);
    return;
  }
  if (ctx.distance <= HAND_NEAR) {
    Serial.println(__func__); // DEBUG
    digitalWrite(ledDebugPin, HIGH); // DEBUG: System awakening sign
    
    lidServoMove(lidServoPosOpen, fingerServoStepDelayFast); // Open lid fast
    while (true) {
      // Dynamic context
      bool toggleOn = readToggleStatus();
      uint16_t d = (uint16_t)readProximity();   // mm
      bool handNear = (d < HAND_NEAR);
      if (!handNear && !toggleOn) {
        lidServoMove(lidServoPosClose, lidServoStepDelayFast);
        digitalWrite(ledDebugPin, LOW); // DEBUG: System sleeping sign
        return;
      }
      if (toggleOn) {
        // Wait until hand goes away
        while (true) {
          d = (uint16_t)readProximity();
          if (d >= HAND_NEAR) break;
          delay(20);
        }
        // Now hand is away: toggle OFF with finger, return stationary, close lid
        fingerServoMove(fingerServoPosToggle, fingerServoStepDelayFast);
        delay(20);
        fingerServoMove(fingerServoPosStationary, fingerServoStepDelayFast);
        lidServoMove(lidServoPosClose, lidServoStepDelayFast);
        digitalWrite(ledDebugPin, LOW); // DEBUG: System sleeping sign
        return;
      }
      delay(20); // IMPORTANT: prevent CPU/I2C hammering
    }
  }
}

// behaviors occurance weights
WeightedBehavior behaviors[] = {
  { behavior_0, 1 }, // Normal behavior, random action speed and delay
  { behavior_1, 1 }, // Open lid in hand proximity
};

//////////////////////////////// COMPILE BEHAVIORS /////////////////////////////////

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

////////////////////////////////////// MAIN //////////////////////////////////////

void loop() {
  BehaviorContext ctx;
  ctx.toggleStatus = readToggleStatus(); // Check toggle on/off state
  ctx.distance = readProximity(); // Check proximity sensor min value
  //printContext(ctx);   // DEBUG OUTPUT
  BehaviorFn behavior = pickBehavior();
  behavior(ctx);
  delay(50);
}