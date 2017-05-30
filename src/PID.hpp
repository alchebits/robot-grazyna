#ifndef PID_GRAZYNA_HEADER_FILE
#define PID_GRAZYNA_HEADER_FILE

class PID
{
public:
  PID();
  PID(float kp, float ki, float kd, float integral_min, float integral_max);
  ~PID();

  float step(float targetPosition, float currentPosition);

  void setKp(float kp);
  void setKi(float ki);
  void setKd(float kd);
  float getPTerm() const;
  float getITerm() const;
  float getDTerm() const;
protected:
  float Kp, Ki, Kd; // K = return value scaled
  float pTerm, iTerm, dTerm;
  float error, integrated_error, last_error;
  float m_integralMin, m_integralMax;
};

#endif
