#ifndef KALMAN_BALANCE_HEADER_FILE
#define KALMAN_BALANCE_HEADER_FILE

class Kalman
{
public:
  Kalman();
  ~Kalman();
  float calculate(float newAngle, float newRate, int looptime);
protected:
  float Q_angle;
  float Q_gyro;
  float R_angle;

  float x_angle;
  float x_bias;
  float P_00, P_01, P_10, P_11;
  float dt, y, S;
  float K_0, K_1;
};

#endif
