#ifndef VELOCITY_HEADER_FILE_HPP
#define VELOCITY_HEADER_FILE_HPP

class Velocity
{
public:
  Velocity();
  ~Velocity();
  // acc in m/s^2, 1g == 9.8 m/s^2
  // looptimeMS in ms
  // return current velocity
  float step(float acc, unsigned looptimeMS);
  float getVelocity() const;
  void setVelocity(float m_velocity);
protected:
  float m_V; // m/s
  float m_prevV; // previous velocity
  float m_prevAcc; // previous acceleration
};

#endif
