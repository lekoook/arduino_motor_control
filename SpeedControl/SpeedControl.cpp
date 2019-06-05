#include <SpeedControl.h>

/**
 * This class allows PID speed (degrees/second) control of motors.
 * 
 * Each instance requires a Motor and two Encoder objects (one for each motor) to work. This system reads speed from 
 * the Encoder object and calculates the error between the set point (desired speed, set using setSpeed()) and process 
 * variable (current measured speed).
 * 
 * This error is then fed into the PID equation and it outputs the PWM adjustment value. This PWM value is then used 
 * to set the motor's PWM using Motor object.
 * 
 * By calling correctPwm() every deltaTime (in Encoder class), a new process variable is fed back into this system 
 * where a new PWM adjustment value is calculated and used to set motor PWM. This loop process continues until the set 
 * point is equal to process variable (desire speed is met).
 * 
 * NOTE: Depending on the Arduino you are using (Uno, Due, Zero, etc.), int can be stored as 2-bytes or 4-bytes values. 
 * In certain cases, you may wish to replace long with int to save memory. Correspondingly, be wary with the fact that
 * int are stored as 2-bytes on some boards (eg. Uno). You may then want to consider overflow cases. 
 * 
 */

/**
 * @brief Construct a new Speed Control:: Speed Control object.
 * 
 * @param motor Motor object used.
 * @param encoder Encoder object used.
 */
SpeedControl::SpeedControl(Motor *motor, Encoder *encoderOne, Encoder *encoderTwo) 
            : motor(motor), 
            encoderOne(encoderOne), 
            encoderTwo(encoderTwo) 
  {
  pwmM1 = 0;
  pwmM2 = 0;
  setPointM1 = 0;
  setPointM2 = 0;
  prevM1Speed = 0;
  prevM2Speed = 0;
  minSpeed = DEF_MIN_SPEED;
  pGainM1 = DEF_GAIN;
  iGainM1 = DEF_GAIN;
  dGainM1 = DEF_GAIN;
  iTermM1 = 0;
  pGainM2 = DEF_GAIN;
  iGainM2 = DEF_GAIN;
  dGainM2 = DEF_GAIN;
  iTermM2 = 0;
};

/**
 * @brief Corrects the PWM of the first motor to maintain the speed of set point.
 * @details This function must be called every deltaTime (in Encoder class).
 * 
 */
void SpeedControl::correctM1Pwm(void)
{
  // Get the scalar speed.
  long speed = encoderOne->getSpeed();
  if (speed < 0)
  {
    speed *= -1;
  }

  // Find the error.
  long error = setPointM1 - speed;

  // Calculate P, I, D values.
  double pTerm = pGainM1 * (double)error;
  iTermM1 += iGainM1 * (double)error;
  double dTerm = dGainM1 * (speed - prevM1Speed); // Uses derivative on measurement. See: http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/

  // Calculate the pwm value to set.
  int correction = pTerm + iTermM1 - dTerm;
  pwmM1 += correction;
  if (pwmM1 > 255)
  {
    pwmM1 = 255;
  }
  if (pwmM1 < 0)
  {
    pwmM1 = 0;
  }
   
  // Set pwm.
  motor->setM1Pwm(pwmM1);
  prevM1Speed = speed;
}

/**
 * @brief Corrects the PWM of the second motor to maintain the speed of set point.
 * @details This function must be called every deltaTime (in Encoder class).
 * 
 */
void SpeedControl::correctM2Pwm(void)
{
  // Get the scalar speed.
  long speed = encoderTwo->getSpeed();
  if (speed < 0)
  {
    speed *= -1;
  }

  // Find the error.
  long error = setPointM2 - speed;

  // Calculate P, I, D values.
  double pTerm = pGainM2 * (double)error;
  iTermM2 += iGainM2 * (double)error;
  double dTerm = dGainM2 * (speed - prevM2Speed); // Uses derivative on measurement. See: http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/

  // Calculate the pwm value to set.
  int correction = pTerm + iTermM2 - dTerm;
  pwmM2 += correction;
  if (pwmM2 > 255)
  {
    pwmM2 = 255;
  }
  if (pwmM2 < 0)
  {
    pwmM2 = 0;
  }
   
  // Set pwm.
  motor->setM2Pwm(pwmM2);
  prevM2Speed = speed;
}

/**
 * @brief Sets the P, I, D gains of both motors.
 * 
 * @param pGainM1 Proportional gain of first motor.
 * @param iGainM1 Integral gain of first motor.
 * @param dGainM1 Derivative gain first motor.
 * @param pGainM2 Proportional gain of second motor.
 * @param iGainM2 Integral gain of second motor.
 * @param dGainM2 Derivative gain second motor.
 */
void SpeedControl::setGains(
      double pGainM1, 
      double iGainM1, 
      double dGainM1, 
      double pGainM2, 
      double iGainM2, 
      double dGainM2
      )
{
  this->pGainM1 = pGainM1;
  this->iGainM1 = iGainM1;
  this->dGainM1 = dGainM1;
  this->pGainM2 = pGainM2;
  this->iGainM2 = iGainM2;
  this->dGainM2 = dGainM2;
}

/**
 * @brief Sets the set point speed (in degrees/second) of first motor.
 * 
 * @param speed Value to set the set point with.
 */
void SpeedControl::setM1Speed(long speed)
{
  if (speed < 0)
  {
    motor->setM1Dir(DIR_BWD);
    speed *= -1;
  }
  else if (speed > 0)
  {
    motor->setM1Dir(DIR_FWD);
  }
  else
  {
    motor->stopM1();
    motor->setM1Pwm(0);
  }

  if (speed > 0 && speed < minSpeed){
    speed = minSpeed;
  }

  setPointM1 = speed;
}

/**
 * @brief Sets the set point speed (in degrees/second) of second motor.
 * 
 * @param speed Value to set the set point with.
 */
void SpeedControl::setM2Speed(long speed)
{
  if (speed < 0)
  {
    motor->setM2Dir(DIR_BWD);
    speed *= -1;
  }
  else if (speed > 0)
  {
    motor->setM2Dir(DIR_FWD);
  }
  else
  {
    motor->stopM2();
    motor->setM2Pwm(0);
  }

  if (speed > 0 && speed < minSpeed){
    speed = minSpeed;
  }

  setPointM2 = speed;
}

/**
 * @brief Sets the minimum speed to run.
 * 
 * @param speed Value to set the minimum speed with.
 */
void SpeedControl::setMinSpeed(long speed)
{
  if (speed < 0)
  {
    speed = 0;
  }
  if (speed > 255)
  {
    speed = 255;
  }

  minSpeed = speed;
}