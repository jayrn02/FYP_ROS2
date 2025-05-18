// Version 3 :) // ... rest of comments ...

#include <AccelStepper.h>
#include "config.h"
#include "pinout_uno.h"
#include "endstop.h"
// #include "byj_gripper.h" // Comment out if header included later
#include "logger.h"

// --- AccelStepper Instances ---
AccelStepper stepper_base(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper stepper_shoulder(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper stepper_elbow(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);

// Define Enable Pin
#define ENABLE_PIN 8

// --- BYJ Gripper ---
// Pins A0-A3 (14-17)
#define BYJ_PIN_0 14
#define BYJ_PIN_1 15
#define BYJ_PIN_2 16
#define BYJ_PIN_3 17
#include "byj_gripper.h" // Include header AFTER defining pins

// --- CORRECTION 1: Modify Gripper Instantiation ---
// The modified constructor only takes pins, not BYJ_GRIP_STEPS
BYJ_Gripper gripper(BYJ_PIN_0, BYJ_PIN_1, BYJ_PIN_2, BYJ_PIN_3);
// --- End Correction 1 ---

long gripper_current_steps = 0;
long gripper_target_steps = 0;

// --- Endstops ---

// --- CORRECTION 2: Check Z Endstop 'swap_pin' parameter ---
// Your comment indicates 'false for UNO', but code has 'true'.
// Assuming the comment is correct for UNO pin 11 behavior:
Endstop endstopX(X_MIN_PIN, X_DIR_PIN, X_STEP_PIN, X_ENABLE_PIN, X_MIN_INPUT, X_HOME_STEPS, HOME_DWELL, false);
Endstop endstopY(Y_MIN_PIN, Y_DIR_PIN, Y_STEP_PIN, Y_ENABLE_PIN, Y_MIN_INPUT, Y_HOME_STEPS, HOME_DWELL, false);
Endstop endstopZ(Z_MIN_PIN, Z_DIR_PIN, Z_STEP_PIN, Z_ENABLE_PIN, Z_MIN_INPUT, Z_HOME_STEPS, HOME_DWELL, false); // Changed last argument to false
// --- End Correction 2 ---


// --- Homing Constants ---
const long base_home_steps = Z_HOME_STEPS;
const long shoulder_home_steps = Y_HOME_STEPS;
const long elbow_home_steps = X_HOME_STEPS;

// --- Serial Communication ---
String inputString = "";
bool stringComplete = false;
unsigned long lastStatusTime = 0;
const long statusInterval = 100;

// --- Setup ---
void setup() {
    Serial.begin(BAUD);
    inputString.reserve(200);

    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH);

    // --- Slow Speed Settings --- Looks Correct! ---
    long very_slow_speed = 10 * MICROSTEPS; // e.g., 10 * 16 = 160 steps/sec (VERY slow)
    long very_slow_accel = 5 * MICROSTEPS;  // e.g., 5 * 16 = 80 steps/sec^2 (VERY gentle)

    // Configure AccelStepper - BASE - Correct
    stepper_base.setEnablePin(ENABLE_PIN);
    stepper_base.setPinsInverted(INVERSE_Z_STEPPER, false, true);
    stepper_base.setMaxSpeed(very_slow_speed);
    stepper_base.setAcceleration(very_slow_accel);
    stepper_base.disableOutputs();

    // Configure AccelStepper - SHOULDER - Correct
    stepper_shoulder.setEnablePin(ENABLE_PIN);
    stepper_shoulder.setPinsInverted(INVERSE_Y_STEPPER, false, true);
    stepper_shoulder.setMaxSpeed(very_slow_speed);
    stepper_shoulder.setAcceleration(very_slow_accel);
    stepper_shoulder.disableOutputs();

    // Configure AccelStepper - ELBOW - Correct
    stepper_elbow.setEnablePin(ENABLE_PIN);
    stepper_elbow.setPinsInverted(INVERSE_X_STEPPER, false, true);
    stepper_elbow.setMaxSpeed(very_slow_speed);
    stepper_elbow.setAcceleration(very_slow_accel);
    stepper_elbow.disableOutputs();

    // Gripper setup - Correct
    pinMode(BYJ_PIN_0, OUTPUT);
    pinMode(BYJ_PIN_1, OUTPUT);
    pinMode(BYJ_PIN_2, OUTPUT);
    pinMode(BYJ_PIN_3, OUTPUT);

    // Assume starts open (1200 steps corresponds to the 'open' state)
    gripper_current_steps = 1200; // This means ROS should command 0.017m to stay here
    gripper_target_steps = gripper_current_steps;

    Logger::logINFO("Arduino Ready. Waiting for ROS commands.");
}

// --- Loop ---
void loop() {
    // Check for incoming serial data
    serialEvent();

    // Process complete command
    if (stringComplete) {
        processCommand(inputString);
        inputString = "";
        stringComplete = false;
    }

    // Run arm steppers (non-blocking) - Correct
    stepper_base.run();
    stepper_shoulder.run();
    stepper_elbow.run();

    // Update Gripper state (blocking based on difference) - Correct
    if (gripper_current_steps != gripper_target_steps) {
        long diff = gripper_target_steps - gripper_current_steps;
        if (diff > 0) {
            gripper.cmdOn(diff);
        } else {
            gripper.cmdOff(-diff);
        }
        gripper_current_steps = gripper_target_steps;
    }

    // Send status periodically - Correct
    if (millis() - lastStatusTime >= statusInterval) {
       sendStatus();
       lastStatusTime = millis();
    }

    // --- CORRECTION 3: Remove duplicate status sending block ---
    // The block below was duplicated at the end. Remove it.
    // if (millis() - lastStatusTime >= statusInterval) {
    //    sendStatus();
    //    lastStatusTime = millis();
    // }
    // --- End Correction 3 ---
} // End of loop()


// --- Serial Handling --- Correct
void serialEvent() {
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        inputString += inChar;
        if (inChar == '\n') {
            stringComplete = true;
        }
    }
}

// --- Command Processing --- Correct
void processCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    char commandType = cmd.charAt(0);
    String args = cmd.substring(1);
    long vals[4];

    switch (commandType) {
        case 'P':
            if (sscanf(args.c_str(), "%ld %ld %ld %ld", &vals[0], &vals[1], &vals[2], &vals[3]) == 4) {
                stepper_base.moveTo(vals[0]);
                stepper_shoulder.moveTo(vals[1]);
                stepper_elbow.moveTo(vals[2]);
                // Use BYJ_GRIP_STEPS from config.h for constrain max
                gripper_target_steps = constrain(vals[3], 0, BYJ_GRIP_STEPS);
            } else {
                Logger::logERROR("Invalid P args: " + args);
            }
            break;
        case 'H':
            Logger::logINFO("Homing received...");
            executeHomingUNO();
            Serial.println("OK");
            break;
        case 'E':
            if (args.startsWith("1")) {
                enableSteppers(true);
                Serial.println("OK");
            } else if (args.startsWith("0")) {
                enableSteppers(false);
                Serial.println("OK");
            } else {
                 Logger::logERROR("Invalid E arg: " + args);
            }
            break;
        case 'S':
             sendStatus();
             break;
        default:
            Logger::logERROR("Unknown command: " + cmd);
    }
}

// --- Send Status --- Correct
void sendStatus() {
    String statusMsg = "S ";
    statusMsg += String(stepper_base.currentPosition());
    statusMsg += " ";
    statusMsg += String(stepper_shoulder.currentPosition());
    statusMsg += " ";
    statusMsg += String(stepper_elbow.currentPosition());
    statusMsg += " ";
    statusMsg += String(gripper_current_steps);
    Serial.println(statusMsg);
}

// --- Stepper Enable --- Correct
void enableSteppers(bool enable) {
    if (enable) {
        digitalWrite(ENABLE_PIN, LOW);
        stepper_base.enableOutputs();
        stepper_shoulder.enableOutputs();
        stepper_elbow.enableOutputs();
    } else {
        stepper_base.disableOutputs();
        stepper_shoulder.disableOutputs();
        stepper_elbow.disableOutputs();
        digitalWrite(ENABLE_PIN, HIGH);
    }
}

// --- Homing Sequence (Adapted for AccelStepper) --- Looks Correct
void executeHomingUNO() {
    Logger::logINFO("Starting Homing Sequence (UNO)...");
    enableSteppers(true);

    long initialMaxSpeed = stepper_elbow.maxSpeed();
    long initialAccel = stepper_elbow.acceleration();
    // Use HOME_DWELL from config.h indirectly? AccelStepper doesn't use dwell.
    // Let's define homing speed directly. Make it slow too.
    long homeSpeed = 20 * MICROSTEPS; // Slow homing speed (steps/sec)
    long homeAccel = 400 * MICROSTEPS; // Slow homing accel (steps/sec^2)

    stepper_shoulder.setMaxSpeed(homeSpeed);
    stepper_shoulder.setAcceleration(homeAccel);
    stepper_elbow.setMaxSpeed(homeSpeed);
    stepper_elbow.setAcceleration(homeAccel);
    stepper_base.setMaxSpeed(homeSpeed);
    stepper_base.setAcceleration(homeAccel);

    Logger::logDEBUG("Homing Shoulder (Y) and Elbow (X)...");
    // Move towards negative direction by default. setPinsInverted will handle reversal.
    stepper_shoulder.moveTo(-1000000);
    stepper_elbow.moveTo(-1000000);

    while (!endstopY.state() || !endstopX.state()) {
        if (!endstopY.state()) stepper_shoulder.run();
        if (!endstopX.state()) stepper_elbow.run();
        yield();
    }

    stepper_shoulder.stop();
    stepper_elbow.stop();
    stepper_shoulder.runToPosition();
    stepper_elbow.runToPosition();

    stepper_shoulder.setCurrentPosition(0);
    stepper_elbow.setCurrentPosition(0);
    Logger::logDEBUG("Shoulder/Elbow switches hit.");

    Logger::logDEBUG("Moving to offsets Y: " + String(shoulder_home_steps) + " X: " + String(elbow_home_steps));
    // Move away from endstop (positive direction). setPinsInverted will handle reversal if needed.
    stepper_shoulder.moveTo(shoulder_home_steps);
    stepper_elbow.moveTo(elbow_home_steps);
    while(stepper_shoulder.distanceToGo() != 0 || stepper_elbow.distanceToGo() != 0) {
         stepper_shoulder.run();
         stepper_elbow.run();
         yield();
    }
    stepper_shoulder.runToPosition();
    stepper_elbow.runToPosition();

    // Set position relative to the homed offset
    stepper_shoulder.setCurrentPosition(shoulder_home_steps);
    stepper_elbow.setCurrentPosition(elbow_home_steps);
    Logger::logDEBUG("Shoulder/Elbow homing offset complete.");


    Logger::logDEBUG("Homing Base (Z)...");
    // Move towards negative direction by default. setPinsInverted will handle reversal.
    stepper_base.moveTo(-1000000);
    while (!endstopZ.state()) {
        stepper_base.run();
        yield();
    }
    stepper_base.stop();
    stepper_base.runToPosition();
    stepper_base.setCurrentPosition(0);
    Logger::logDEBUG("Base switch hit.");

    Logger::logDEBUG("Moving to offset Z: " + String(base_home_steps));
    // Move away from endstop (positive direction). setPinsInverted will handle reversal if needed.
    stepper_base.moveTo(base_home_steps);
    while(stepper_base.distanceToGo() != 0) {
        stepper_base.run();
        yield();
    }
    stepper_base.runToPosition();

    // Set position relative to the homed offset
    stepper_base.setCurrentPosition(base_home_steps);
    Logger::logDEBUG("Base homing offset complete.");

    // Restore original (very slow) speed/accel
    stepper_shoulder.setMaxSpeed(initialMaxSpeed);
    stepper_shoulder.setAcceleration(initialAccel);
    stepper_elbow.setMaxSpeed(initialMaxSpeed);
    stepper_elbow.setAcceleration(initialAccel);
    stepper_base.setMaxSpeed(initialMaxSpeed);
    stepper_base.setAcceleration(initialAccel);

    Logger::logINFO("Homing Complete.");
}