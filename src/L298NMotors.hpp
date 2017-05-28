#ifndef L298N_MOTORS_HPP_FILE
#define L298N_MOTORS_HPP_FILE

#include <Arduino.h>

class L298NMotors{
public:
  L298NMotors(int L1pin, int L2pin, int R1pin, int R2pin);
  L298NMotors(int L1pin, int L2pin, int R1pin, int R2pin, int LPWM, int RPWM);
  ~L298NMotors();
  void forward();
  void leftForward();
  void rightForward();
  void backward();
  void leftBackward();
  void rightBackward();
  void stop();
  void leftStop();
  void rightStop();

  void setSpeed(unsigned percent);
  void setLeftSpeed(unsigned percent);
  void setRightSpeed(unsigned percent);
  int percentToPinValue(unsigned percent);
protected:
  int m_L1pin;
  int m_L2pin;
  int m_R1pin;
  int m_R2pin;
  int m_LPWM;
  int m_RPWM;
};

#endif
