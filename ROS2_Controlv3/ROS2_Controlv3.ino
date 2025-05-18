// Version 3 :) 
// ... rest of comments ...

#include <AccelStepper.h>
#include <string.h>
#include "config.h"
#include "pinout_uno.h"
#include "endstop.h"
#include "logger.h"

// --- AccelStepper Instances ---
AccelStepper stepper_base(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);
AccelStepper stepper_shoulder(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper stepper_elbow(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);

// Define Enable Pin
#define ENABLE_PIN 8

// --- BYJ Gripper ---
#define BYJ_PIN_0 14
#define BYJ_PIN_1 15
#define BYJ_PIN_2 16
#define BYJ_PIN_3 17
#include "byj_gripper.h"

// Gripper Instantiation
BYJ_Gripper gripper(BYJ_PIN_0, BYJ_PIN_1, BYJ_PIN_2, BYJ_PIN_3);

long gripper_current_steps = 0;
long gripper_target_steps = 0;

// --- Endstops ---
Endstop endstopX(X_MIN_PIN, X_DIR_PIN, X_STEP_PIN, ENABLE_PIN, X_MIN_INPUT, X_HOME_STEPS, HOME_DWELL, false);
Endstop endstopY(Y_MIN_PIN, Y_DIR_PIN, Y_STEP_PIN, ENABLE_PIN, Y_MIN_INPUT, Y_HOME_STEPS, HOME_DWELL, false);
Endstop endstopZ(Z_MIN_PIN, Z_DIR_PIN, Z_STEP_PIN, ENABLE_PIN, Z_MIN_INPUT, Z_HOME_STEPS, HOME_DWELL, false); // Changed last argument to false based on user comment

// --- Homing Constants ---
const long base_home_steps = Z_HOME_STEPS;
const long shoulder_home_steps = Y_HOME_STEPS;
const long elbow_home_steps = X_HOME_STEPS;

// --- Serial Communication ---
String inputString = "";
bool stringComplete = false;
unsigned long lastStatusTime = 0;
const long statusInterval = 100; // ms interval for sending status

// Function Prototypes
void serialEvent();
void processCommand(String cmd);
void sendStatus();
void enableSteppers(bool enable);
void executeHomingUNO();



// --- Setup ---
void setup() {
    Serial.begin(BAUD);
    inputString.reserve(200);

    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH);

    // Use slow speed/accel for setup/homing initially
    long initial_speed = 20 * MICROSTEPS;
    long initial_accel = 500 * MICROSTEPS;

    // Configure AccelStepper - BASE
    stepper_base.setEnablePin(ENABLE_PIN);
    stepper_base.setPinsInverted(INVERSE_Z_STEPPER, false, true); // dir, step, enable
    stepper_base.setMaxSpeed(initial_speed);
    stepper_base.setAcceleration(initial_accel);
    stepper_base.disableOutputs(); // Ensure disabled initially

    // Configure AccelStepper - SHOULDER
    stepper_shoulder.setEnablePin(ENABLE_PIN);
    stepper_shoulder.setPinsInverted(INVERSE_Y_STEPPER, false, true); // dir, step, enable
    stepper_shoulder.setMaxSpeed(initial_speed);
    stepper_shoulder.setAcceleration(initial_accel);
    stepper_shoulder.disableOutputs();

    // Configure AccelStepper - ELBOW
    stepper_elbow.setEnablePin(ENABLE_PIN);
    stepper_elbow.setPinsInverted(INVERSE_X_STEPPER, false, true); // dir, step, enable
    stepper_elbow.setMaxSpeed(initial_speed);
    stepper_elbow.setAcceleration(initial_accel);
    stepper_elbow.disableOutputs();

    // Gripper setup
    pinMode(BYJ_PIN_0, OUTPUT);
    pinMode(BYJ_PIN_1, OUTPUT);
    pinMode(BYJ_PIN_2, OUTPUT);
    pinMode(BYJ_PIN_3, OUTPUT);

    gripper_current_steps = 1200;
    gripper_target_steps = gripper_current_steps;

    Logger::logINFO("Starting Homing...");

    delay(1000);
    executeHomingUNO();
    stepper_base.setMaxSpeed(400 * MICROSTEPS);
    stepper_base.setAcceleration(200 * MICROSTEPS);
    stepper_shoulder.setMaxSpeed(400 * MICROSTEPS);
    stepper_shoulder.setAcceleration(200 * MICROSTEPS);
    stepper_elbow.setMaxSpeed(400 * MICROSTEPS);
    stepper_elbow.setAcceleration(200 * MICROSTEPS);

    Logger::logINFO("Arduino Ready. Waiting for ROS commands.");
} 

void loop() {
    serialEvent();

    if (stringComplete) {
        processCommand(inputString);
        inputString = "";
        stringComplete = false;
    }

    // Run arm steppers (non-blocking)
    stepper_base.run();
    stepper_shoulder.run();
    stepper_elbow.run();

    // if (gripper_current_steps != gripper_target_steps) {
    //     long diff = gripper_target_steps - gripper_current_steps;
    //     if (diff > 0) {
    //         gripper.cmdOn(diff); // Blocking call
    //     } else {
    //         gripper.cmdOff(-diff); // Blocking call
    //     }
    //     gripper_current_steps = gripper_target_steps;
    // }

    if (millis() - lastStatusTime >= statusInterval) {
        sendStatus();
        lastStatusTime = millis();
    }
}


// Functions Here

void serialEvent() {
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        inputString += inChar;
        if (inChar == '\n') {
            stringComplete = true;
        }
    }
}

void processCommand(String cmd) {
    cmd.trim(); 
    if (cmd.length() == 0) return; 

    char commandType = cmd.charAt(0); 
    String args = ""; 
    if (cmd.length() > 1 && cmd.charAt(1) == ':') { 
        args = cmd.substring(2); 
    } else if (cmd.length() > 1) { 
        if (commandType != 'S' && commandType != 'H') { 
            return; 
        }
    }
// The commands types
    switch (commandType) {
        case 'S':
            sendStatus(); 
            break;

        case 'M': // Move command
            { 
                long vals[4] = {0, 0, 0, 0}; 
                int argIndex = 0; 
                int lastComma = -1; 
                for (int i = 0; i < args.length(); i++) { 
                    if (args.charAt(i) == ',') { 
                        if (argIndex < 4) { 
                            vals[argIndex++] = args.substring(lastComma + 1, i).toInt(); 
                        }
                        lastComma = i; 
                    }
                }
                // Get the last value
                if (argIndex < 4) { 
                    vals[argIndex] = args.substring(lastComma + 1).toInt(); 
                }

                if (argIndex != 3) { // Check if exactly 4 values were processed (indices 0, 1, 2, 3)
                     break; 
                }
                // Set target positions for steppers
                stepper_base.moveTo(vals[0]); 
                stepper_shoulder.moveTo(vals[1]); 
                stepper_elbow.moveTo(vals[2]); 
                // Gripper movement logic
                gripper_target_steps = constrain(vals[3], 0, BYJ_GRIP_STEPS); 
            }
            break;

        case 'H': // Homing command
            executeHomingUNO(); 

            break;

        case 'E': // Enable/Disable Steppers: E:1 (enable) or E:0 (disable)
            { 
                int enable_state = args.toInt(); 
                if (args != "0" && args != "1") {
                    break; 
                }
                enableSteppers(enable_state == 1); 
            }
            break;
        default:
            break;
    }
}

void sendStatus() {
    String statusMsg = "S "; // Format matches C++ read() expectation
    statusMsg += String(stepper_base.currentPosition());
    statusMsg += " ";
    statusMsg += String(stepper_shoulder.currentPosition());
    statusMsg += " ";
    statusMsg += String(stepper_elbow.currentPosition());
    statusMsg += " ";
    statusMsg += String(gripper_current_steps); // Report current gripper steps
    Serial.println(statusMsg); // Use println to ensure newline termination
}

// --- Stepper Enable --- 
void enableSteppers(bool enable) {
    if (enable) {
        digitalWrite(ENABLE_PIN, LOW); // LOW enables most drivers
        stepper_base.enableOutputs();
        stepper_shoulder.enableOutputs();
        stepper_elbow.enableOutputs();
        Logger::logDEBUG("Steppers Enabled");
    } else {
        stepper_base.disableOutputs();
        stepper_shoulder.disableOutputs();
        stepper_elbow.disableOutputs();
        digitalWrite(ENABLE_PIN, HIGH); // HIGH disables most drivers
        Logger::logDEBUG("Steppers Disabled");
    }
}

// --- Homing Sequence modified to use Accell instead from the original ---
void executeHomingUNO() {
    Logger::logINFO("Starting Homing Sequence (UNO)...");
    enableSteppers(true);

    long initialMaxSpeedB = stepper_base.maxSpeed(); // Store initial speeds/accels
    long initialAccelB = stepper_base.acceleration();
    long initialMaxSpeedS = stepper_shoulder.maxSpeed();
    long initialAccelS = stepper_shoulder.acceleration();
    long initialMaxSpeedE = stepper_elbow.maxSpeed();
    long initialAccelE = stepper_elbow.acceleration();

    // Separate speed just in case for testing
    long homeSpeed = 60 * MICROSTEPS; // Slow homing speed (steps/sec)
    long homeAccel = 500 * MICROSTEPS; // Slow homing accel (steps/sec^2)

    stepper_shoulder.setMaxSpeed(homeSpeed);
    stepper_shoulder.setAcceleration(homeAccel);
    stepper_elbow.setMaxSpeed(homeSpeed);
    stepper_elbow.setAcceleration(homeAccel);
    stepper_base.setMaxSpeed(homeSpeed);
    stepper_base.setAcceleration(homeAccel);

    Logger::logDEBUG("Homing Shoulder (Y) and Elbow (X)...");
    stepper_shoulder.moveTo(1000000);
    stepper_elbow.moveTo(1000000);

    while (!endstopY.state() || !endstopX.state()) {
        if (!endstopY.state()) stepper_shoulder.run();
        if (!endstopX.state()) stepper_elbow.run();
        yield();
    }

    // Reverse direction after hitting the endstop
    if (endstopY.state()) {
        stepper_shoulder.setCurrentPosition(0);
        stepper_shoulder.moveTo(-shoulder_home_steps); // Move to offset in the opposite direction
        while (stepper_shoulder.distanceToGo() != 0) {
            stepper_shoulder.run();
            yield();
        }
        stepper_shoulder.setCurrentPosition(shoulder_home_steps);
    }

    if (endstopX.state()) {
        stepper_elbow.setCurrentPosition(0);
        stepper_elbow.moveTo(-elbow_home_steps); // Move to offset in the opposite direction
        while (stepper_elbow.distanceToGo() != 0) {
            stepper_elbow.run();
            yield();
        }
        stepper_elbow.setCurrentPosition(elbow_home_steps);
    }

    Logger::logDEBUG("Shoulder/Elbow homing offset complete.");

    Logger::logDEBUG("Homing Base (Z)...");
    stepper_base.moveTo(-1000000);
    while (!endstopZ.state()) {
        stepper_base.run();
        yield();
    }
    stepper_base.stop();
    stepper_base.runToPosition();
    stepper_base.setCurrentPosition(0);

    Logger::logDEBUG("Moving to offset Z: " + String(base_home_steps));
    stepper_base.moveTo(base_home_steps);
    while (stepper_base.distanceToGo() != 0) {
        stepper_base.run();
        yield();
    }
    stepper_base.runToPosition();

    stepper_base.setCurrentPosition(base_home_steps);
    Logger::logDEBUG("Base homing offset complete.");

    // Restore original (or previously set operational) speed/accel
    stepper_shoulder.setMaxSpeed(initialMaxSpeedS);
    stepper_shoulder.setAcceleration(initialAccelS);
    stepper_elbow.setMaxSpeed(initialMaxSpeedE);
    stepper_elbow.setAcceleration(initialAccelE);
    stepper_base.setMaxSpeed(initialMaxSpeedB);
    stepper_base.setAcceleration(initialAccelB);

    Logger::logINFO("Homing Complete.");
}