/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 5
  #define LEFT_MOTOR_BACKWARD  6
  #define RIGHT_MOTOR_FORWARD  9
  #define LEFT_MOTOR_FORWARD   10
  #define RIGHT_MOTOR_ENABLE 12
  #define LEFT_MOTOR_ENABLE 13
#elif defined L298N_MOTOR_DRIVER
  //right wheel
  #define enA 4
  #define in1 8
  #define in2 9
  //left wheel
  #define enB 5
  #define in3 10
  #define in4 11
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
