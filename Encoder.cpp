#include <Encoder.h>
#include <Arduino.h>

/**
 * @brief Construct a new Encoder:: Encoder object.
 * 
 * @param pinA The pin number to read first encoder output.
 * @param pinB The pin number to read second encoder output.
 * @param deltaTime The time interval (in microseconds) to measure/calculate output shaft speed.
 * @param ticksPerRev The number of ticks that will occur within one revolution of output shaft.
 */
Encoder::Encoder(int pinA, int pinB, long deltaTime, int ticksPerRev) : pinA(pinA), pinB(pinB), deltaTime(deltaTime)
{
  degPerTick = 360.0 / ticksPerRev;
}

/**
 * @brief Updates ticksCount when an encoder interrupt event occurs.
 * @details This function must be called when there is a PIN CHANGE in the interrupt.
 * 
 */
void Encoder::updateCount(void)
{
  if (digitalRead(pinA) == HIGH)
  {
    if (digitalRead(pinB) == HIGH)
    {
      ticksCount++;
    }
    else
    {
      ticksCount--;
    }
  }
  else
  {
    if (digitalRead(pinB) == HIGH)
    {
      ticksCount--;
    }
    else
    {
      ticksCount++;
    }
  }
}

/**
 * @brief Calculates and returns the speed of the motor output shaft in degrees per second (degPerSec).
 * @details This function should be called every deltaTime for correct speed.
 * 
 * @return int The speed in degrees per second.
 */
int Encoder::getSpeed(void)
{
  oldTicksCount = newTicksCount;
  newTicksCount = ticksCount;

  // Calculate the ticks passed since last call of this function.
  int countDiff = newTicksCount - oldTicksCount;

  // Add to total count of ticks.
  totalTicksCount += countDiff;

  long degPerSec;
  // Check if ticksCount has overflowed.
  // Calculate new speed if no overflow.
  if (countDiff < 100000 && countDiff > -100000)
  {
    double intervals = 1000000.0 / deltaTime;
    double ticksPerSec = (double)countDiff * intervals;
    degPerSec = ticksPerSec * degPerTick;
    prevSpeed = degPerSec;
  }
  // Use the previous speed if overflow occurs.
  else
  {
    degPerSec = prevSpeed;
  }
  
  return degPerSec;
}

/**
 * @brief Calculates and returns the distance traveled in degrees per second (degPerSec) since the last call of this function.
 * @details This function must be called regularly such that totalTicksCount would not overflow.
 * 
 * @return int The distance in degrees per second.
 */
int Encoder::getDistance(void)
{
  int distance = totalTicksCount * degPerTick;

  totalTicksCount = 0;

  return distance;
}