/***************************************************************
   Motor driver function definitions - by James Nugen
   Modified for BTS7960 motor driver
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 5
  #define LEFT_MOTOR_BACKWARD  6
  #define RIGHT_MOTOR_FORWARD  9
  #define LEFT_MOTOR_FORWARD   10
  #define RIGHT_MOTOR_ENABLE 12
  #define LEFT_MOTOR_ENABLE 13
#elif defined BTS7960_MOTOR_DRIVER
  // BTS7960 Motor Driver Pin Definitions
  // Left Motor
  #define LEFT_MOTOR_FORWARD   5   // L_PWM pin
  #define LEFT_MOTOR_BACKWARD  6   // R_PWM pin
  
  // Right Motor  
  #define RIGHT_MOTOR_FORWARD  9   // L_PWM pin
  #define RIGHT_MOTOR_BACKWARD 10  // R_PWM pin
  
  // Enable pinleri donanımsal olarak bağlı olduğu için tanımlamıyoruz
  // L_EN ve R_EN pinleri VCC'ye bağlı
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
