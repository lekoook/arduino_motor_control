#include <Motor.h>
#include <Encoder.h>

#define DEF_MIN_SPEED 3000 // In degrees/second.
#define DEF_GAIN 1.0

class SpeedControl
{
  public:
    SpeedControl(Motor *motor, Encoder *encoderOne, Encoder *encoderTwo);
    void correctM1Pwm(void);
    void correctM2Pwm(void);
    void setGains(
      double pGainM1, 
      double iGainM1, 
      double dGainM1, 
      double pGainM2, 
      double iGainM2, 
      double dGainM2
      );
    void setM1Speed(long speed);
    void setM2Speed(long speed);
    void setMinSpeed(long speed);

  private:
    Motor *motor;
    Encoder *encoderOne;
    Encoder *encoderTwo;

    int pwmM1;
    int pwmM2;

    long setPointM1; // Desired speed in degrees/second.
    long setPointM2; // Desired speed in degrees/second.
    long prevM1Speed; // Previous speed set.
    long prevM2Speed; // Previous speed set.
    long minSpeed; // Minimum speed.

    double pGainM1;
    double iGainM1;
    double dGainM1;
    double iTermM1; // Keep track of integral term.
    double pGainM2;
    double iGainM2;
    double dGainM2;
    double iTermM2; // Keep track of integral term.
};