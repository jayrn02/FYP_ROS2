#include "byj_gripper.h"
#include <Arduino.h>

// Constructor implementation updated
BYJ_Gripper::BYJ_Gripper(int pin0, int pin1, int pin2, int pin3) {
  // Store pin numbers
  byj_pin_0 = pin0;
  byj_pin_1 = pin1;
  byj_pin_2 = pin2;
  byj_pin_3 = pin3;

  // Initialize state
  step_cycle = 0;
  direction = false; // Default direction (can be arbitrary)

  // Set pin modes
  pinMode(byj_pin_0, OUTPUT);
  pinMode(byj_pin_1, OUTPUT);
  pinMode(byj_pin_2, OUTPUT);
  pinMode(byj_pin_3, OUTPUT);

  // Ensure motor is initially off/outputs low
  digitalWrite(byj_pin_0, LOW);
  digitalWrite(byj_pin_1, LOW);
  digitalWrite(byj_pin_2, LOW);
  digitalWrite(byj_pin_3, LOW);
}

// Move 'steps_to_move' in the "ON" direction (e.g., closing gripper)
void BYJ_Gripper::cmdOn(long steps_to_move) {
  direction = true; // Set direction for step cycle update
  for (long i = 0; i < steps_to_move; i++) {
    moveSteps();
    delay(1); // Small delay between steps - adjust if needed for your motor
  }
}

// Move 'steps_to_move' in the "OFF" direction (e.g., opening gripper)
void BYJ_Gripper::cmdOff(long steps_to_move) {
  direction = false; // Set direction for step cycle update
  for (long i = 0; i < steps_to_move; i++) {
    moveSteps();
    delay(1); // Small delay between steps - adjust if needed for your motor
  }
}

// --- PRIVATE METHODS (Unchanged from original) ---

// Updates the step_cycle based on the current direction
void BYJ_Gripper::setDirection() {
  if (direction == true) {
    step_cycle++;
  }
  if (direction == false) {
    step_cycle--;
  }

  // Wrap around the 8-step sequence
  if (step_cycle > 7) {
    step_cycle = 0;
  }
  if (step_cycle < 0) {
    step_cycle = 7;
  }
}

// Executes one step of the 8-phase sequence based on step_cycle
void BYJ_Gripper::moveSteps() {
  switch (step_cycle) {
  case 0:
    digitalWrite(byj_pin_0, LOW);
    digitalWrite(byj_pin_1, LOW);
    digitalWrite(byj_pin_2, LOW);
    digitalWrite(byj_pin_3, HIGH);
    break;
  case 1:
    digitalWrite(byj_pin_0, LOW);
    digitalWrite(byj_pin_1, LOW);
    digitalWrite(byj_pin_2, HIGH);
    digitalWrite(byj_pin_3, HIGH);
    break;
  case 2:
    digitalWrite(byj_pin_0, LOW);
    digitalWrite(byj_pin_1, LOW);
    digitalWrite(byj_pin_2, HIGH);
    digitalWrite(byj_pin_3, LOW);
    break;
  case 3:
    digitalWrite(byj_pin_0, LOW);
    digitalWrite(byj_pin_1, HIGH);
    digitalWrite(byj_pin_2, HIGH);
    digitalWrite(byj_pin_3, LOW);
    break;
  case 4:
    digitalWrite(byj_pin_0, LOW);
    digitalWrite(byj_pin_1, HIGH);
    digitalWrite(byj_pin_2, LOW);
    digitalWrite(byj_pin_3, LOW);
    break;
  case 5:
    digitalWrite(byj_pin_0, HIGH);
    digitalWrite(byj_pin_1, HIGH);
    digitalWrite(byj_pin_2, LOW);
    digitalWrite(byj_pin_3, LOW);
    break;
  case 6:
    digitalWrite(byj_pin_0, HIGH);
    digitalWrite(byj_pin_1, LOW);
    digitalWrite(byj_pin_2, LOW);
    digitalWrite(byj_pin_3, LOW);
    break;
  case 7:
    digitalWrite(byj_pin_0, HIGH);
    digitalWrite(byj_pin_1, LOW);
    digitalWrite(byj_pin_2, LOW);
    digitalWrite(byj_pin_3, HIGH);
    break;
  default: // Should not happen, but good practice
    digitalWrite(byj_pin_0, LOW);
    digitalWrite(byj_pin_1, LOW);
    digitalWrite(byj_pin_2, LOW);
    digitalWrite(byj_pin_3, LOW);
    break;
  }
  // Update step_cycle for the next call to moveSteps
  setDirection();
}