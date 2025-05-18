#ifndef BYJ_GRIPPER_H_
#define BYJ_GRIPPER_H_

class BYJ_Gripper {
public:
  BYJ_Gripper(int pin0, int pin1, int pin2, int pin3);

  void cmdOn(long steps_to_move);  // Move 'steps_to_move' in the "ON" direction
  void cmdOff(long steps_to_move); // Move 'steps_to_move' in the "OFF" direction

private:
  bool direction; // True for "ON", False for "OFF"
  void moveSteps(); // Performs one step cycle
  void setDirection(); // Updates step_cycle based on direction

  int byj_pin_0;
  int byj_pin_1;
  int byj_pin_2;
  int byj_pin_3;

  int step_cycle;
};

#endif // BYJ_GRIPPER_H_