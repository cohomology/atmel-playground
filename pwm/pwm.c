#define F_CPU = 9600000
#include <avr/io.h>

const uint8_t ledPin = PB3;
const uint8_t buttonPin = PB4;
const uint8_t motorPin = PB0;

void pwmSetup() 
{
  TCCR0B |= 1 << CS00;                                    // prescaler = clock; e.g. 9.6Mhz
  TCCR0A |= (1 << WGM01) | ( 1 << WGM00) | (1 << COM0A1); // FAST PWM, clear 0C0A on compare match, disable OC0B
  OCR0A = 127;                                            // 50% duty cycle
}

void pinSetup()
{
  PORTB = 0;
  DDRB |= ( 1 << ledPin ) | ( 1 << motorPin); // LED and MOTOR are outputs
}

void setup()
{
  cli();          // disable interrupts
  pinSetup();
  pwmSetup();
  sei();          // enable interrupts
}

void loop() 
{
}

int main()
{
  setup();
  while(true)
    loop();
  return 0;
}

