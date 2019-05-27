#define DEF_PIN_A 2
#define DEF_PIN_B 3

class Encoder
{
  public:
    Encoder(int pinA = DEF_PIN_A, int pinB = DEF_PIN_B);
    void updateCount(void);
    int getSpeed(void);

    int getCount(void); // TODO: remove

  private:
    int pinA; // Pin number for first output
    int pinB; // Pin number for second output

    long ticksCount = 0;
    long oldTicksCount = 0;
    long newTicksCount = 0;
    long totalTicksCount = 0;
};