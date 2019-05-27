#include <Encoder.h>
#include <Arduino.h>

Encoder::Encoder(int pinA, int pinB) : pinA(pinA), pinB(pinB) {}

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

int Encoder::getSpeed(void)
{
  oldTicksCount = newTicksCount;
  newTicksCount = ticksCount;

  int countDiff = newTicksCount - oldTicksCount;

  totalTicksCount += countDiff;
  
}

// TODO: Remove
int Encoder::getCount(void)
{
  return ticksCount;
}