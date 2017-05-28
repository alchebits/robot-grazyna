#include "L298NMotors.hpp"

L298NMotors::L298NMotors(int L1pin, int L2pin, int R1pin, int R2pin) :
L298NMotors(L1pin, L2pin, R1pin, R2pin, -1, -1)
{
}

L298NMotors::L298NMotors(int L1pin, int L2pin, int R1pin, int R2pin, int LPWM, int RPWM) :
m_L1pin(L1pin),
m_L2pin(L2pin),
m_R1pin(R1pin),
m_R2pin(R2pin),
m_LPWM(LPWM),
m_RPWM(RPWM)
{
  pinMode(m_L1pin, OUTPUT);
  pinMode(m_L2pin, OUTPUT);
  pinMode(m_R1pin, OUTPUT);
  pinMode(m_R2pin, OUTPUT);

  if(m_LPWM >=0)
    pinMode(m_LPWM, OUTPUT);
  if(m_RPWM >= 0)
    pinMode(m_RPWM, OUTPUT);
}

L298NMotors::~L298NMotors()
{
}

void L298NMotors::forward()
{
  leftForward();
  rightForward();
}

void L298NMotors::leftForward()
{
  digitalWrite(m_L1pin, LOW);
  digitalWrite(m_L2pin, HIGH);
}

void L298NMotors::rightForward()
{
  digitalWrite(m_R1pin, LOW);
  digitalWrite(m_R2pin, HIGH);
}

void L298NMotors::backward()
{
  leftBackward();
  rightBackward();
}

void L298NMotors::leftBackward()
{
  digitalWrite(m_L2pin, LOW);
  digitalWrite(m_L1pin, HIGH);
}

void L298NMotors::rightBackward()
{
  digitalWrite(m_R2pin, LOW);
  digitalWrite(m_R1pin, HIGH);
}

void L298NMotors::stop()
{
  leftStop();
  rightStop();
}

void L298NMotors::leftStop()
{
  digitalWrite(m_L1pin, LOW);
  digitalWrite(m_L2pin, LOW);
}

void L298NMotors::rightStop()
{
  digitalWrite(m_R1pin, LOW);
  digitalWrite(m_R2pin, LOW);
}

void L298NMotors::setLeftSpeed(unsigned percent)
{
  if(m_LPWM >=0)
    analogWrite(m_LPWM, percentToPinValue(percent));
}

void L298NMotors::setRightSpeed(unsigned percent)
{
  if(m_RPWM >= 0)
    analogWrite(m_RPWM, percentToPinValue(percent));
}

void L298NMotors::setSpeed(unsigned percent)
{
  setLeftSpeed(percent);
  setRightSpeed(percent);
}

int L298NMotors::percentToPinValue(unsigned percent)
{
  unsigned speedVal = 0;
  if(percent >= 100)
    speedVal = 255;
  else
    speedVal = min(255, (2.55f*percent));

  return speedVal;
}
