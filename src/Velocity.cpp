#include "Velocity.hpp"

Velocity::Velocity() :
m_V(0.0f),
m_prevV(0.0f),
m_prevAcc(0.0f)
{
}

Velocity::~Velocity()
{
}

float Velocity::step(float acc, unsigned looptimeMS)
{
  float dT = looptimeMS/1.000f; // dT in seconds (units should be the same, acc is in m/s^2)
  m_V = m_prevV + ((acc + m_prevAcc) / 2.0f) * dT;
  m_prevAcc = acc;
  m_prevV = m_V;
  return m_V;
}

float Velocity::getVelocity() const
{
  return m_V;
}

void Velocity::setVelocity(float m_velocity)
{

}
