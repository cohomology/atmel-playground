/* by Kilian Kilger, 2019; released in the public domain */

/* Simple example of a motor, LED and a button. The motor is 
 * driven by PWM and increased in speed if the button is pressed.
 * The LED blinks faster the faster the motor is driving. 
 *
 * In this example, the LED is on PB3, the button on PB4 and the 
 * motor on PB0. We use some poor man's debouncing for the button.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

template<typename T>
const T& min(const T& a, const T& b)
{ 
  return a > b ? b : a;
}

class MotorPwm
{
public:

  static volatile MotorPwm& getInstance() 
  {
    static volatile MotorPwm pwm;
    return pwm;
  }

  void setup() volatile
  {
    cli();
    pinSetup();
    pwmSetup();
    sei();
  } 

  void timerInterrupt() volatile
  {
    uint32_t counter = 0;
    increaseCounter(counter);
    if ((counter % 1024) == 0)
      handleButton();
    lightLED(counter);
  }

private:
  void pwmSetup() volatile
  {
    TIMSK0 |= 1 << TOIE0;                                   // enable overflow interrupt 
    TCCR0B |= 1 << CS00;                                    // prescaler = clock; e.g. 9.6Mhz
    TCCR0A |= (1 << WGM01) | ( 1 << WGM00) | (1 << COM0A1); // FAST PWM, clear 0C0A on compare match, disable OC0B
    OCR0A  = 0;                                             // turn motor off
  } 

  void pinSetup() volatile
  {
    PORTB = 0;                                              // clear output pins
    DDRB  = ( 1 << ledPin ) | ( 1 << motorPin );            // LED and MOTOR are outputs; everything else is input
  } 

  void handleButton() volatile
  {
    const bool buttonState = (PINB & ( 1 << buttonPin )) != 0;
    if (buttonState != m_prevButtonState)
    {
      if (buttonState)
      {
        m_motorState = (m_motorState + 1) % 17;
        OCR0A = min(16 * m_motorState, 255); 
      }
      m_prevButtonState = buttonState;
    }          
  }

  void switchLedOff() volatile
  {
    m_ledOn = false;
    PORTB &= 0b11111111 ^ ( 1 << ledPin ); 
  }       

  void switchLedOn() volatile
  {
    m_ledOn = true;
    PORTB |= 1 << ledPin; 
  }       

  void toggleLed() volatile
  {
    m_ledOn ? switchLedOff() : switchLedOn();
  } 

  void lightLED(uint32_t& counter) volatile
  {
    if (m_motorState == 0)
      switchLedOff();
    else if ((counter % (32768 / m_motorState)) == 0) 
      toggleLed();
  }         

  void increaseCounter(uint32_t& counter) volatile
  {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      counter = m_counter; 
      if (m_counter == 19200000)
        m_counter = 0; 
      else
        ++m_counter;
    }
  }

private:
  const uint8_t ledPin = PB3;
  const uint8_t buttonPin = PB4;
  const uint8_t motorPin = PB0; 

  uint32_t m_counter = 0;
  bool     m_ledOn = false;
  bool     m_prevButtonState = 0;
  uint8_t  m_motorState = 0; 
};

ISR(TIM0_OVF_vect) 
{
  MotorPwm::getInstance().timerInterrupt(); 
}

int main()
{
  MotorPwm::getInstance().setup();
  while(true);
  return 0;
}
