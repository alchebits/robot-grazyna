#ifndef KALMAN_HEADER_FILE_HPP
#define KALMAN_HEADER_FILE_HPP

class Kalman {
  private:
    /* Kalman filter variables */
    double q; //process noise covariance
    double r; //measurement noise covariance
    double x; //value
    double p; //estimation error covariance
    double k; //kalman gain

  public:
    /* The variables are x for the filtered value, q for the process noise,
       r for the sensor noise, p for the estimated error and k for the Kalman Gain.
       The state of the filter is defined by the values of these variables.

       The initial values for p is not very important since it is adjusted
       during the process. It must be just high enough to narrow down.
       The initial value for the readout is also not very important, since
       it is updated during the process.
       But tweaking the values for the process noise and sensor noise
       is essential to get clear readouts.

       For large noise reduction, you can try to start from: (see http://interactive-matter.eu/blog/2009/12/18/filtering-sensor-data-with-a-kalman-filter/ )
       q = 0.125
       r = 32
       p = 1023 //"large enough to narrow down"
       e.g.
       myVar = Kalman(0.125,32,1023,0);
    */

    Kalman(double process_noise, double sensor_noise, double estimated_error, double intial_value);
    double getFilteredValue(double measurement) ;
    void setParameters(double process_noise, double sensor_noise, double estimated_error) ;
    void setParameters(double process_noise, double sensor_noise);
    double getProcessNoise() ;
    double getSensorNoise() ;
    double getEstimatedError() ;
};

#endif // KALMAN_HEADER_FILE_HPP
