#include "PID.hpp"
#include <math.h>
#include "utils.hpp"
#include <Arduino.h>

PID::PID() :
PID(0,0,0,-1000,1000)
{
}

PID::PID(float kp, float ki, float kd, float integral_min, float integral_max) :
Kp(kp), Ki(ki), Kd(kd),
pTerm(0), iTerm(0), dTerm(0),
error(0), last_error(0),
m_integralMin(integral_min), m_integralMax(integral_max)
{
}

PID::~PID()
{
}

float PID::step(float targetPosition, float currentPosition)
{
 error = targetPosition - currentPosition;
 pTerm = Kp * error;
 iTerm += Ki * error;
 iTerm = constrain(iTerm, m_integralMin, m_integralMax);
 dTerm = Kd * (error - last_error);
 last_error = error;
 return (pTerm + iTerm + dTerm);
}

void PID::setKp(float kp)
{
  Kp = kp;
}

void PID::setKi(float ki)
{
  Ki = ki;
}

void PID::setKd(float kd)
{
  Kd = kd;
}

float PID::getPTerm() const
{
  return pTerm;
}

float PID::getITerm() const
{
  return iTerm;
}

float PID::getDTerm() const
{
  return dTerm;
}
