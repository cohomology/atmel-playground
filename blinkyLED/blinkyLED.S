#include <avr/io.h>
.text
.global main
.global TIM0_OVF_vect

TIM0_OVF_vect:
  in r20, _SFR_IO_ADDR(PORTB)    ; toggle LED
  ldi r21, 8
  eor r20, r21 
  out _SFR_IO_ADDR(PORTB), r20
  reti 

main:
  cli                            ; disable interupts

  ldi r16, 0b00001000            ; set PB3 as output pin
  ldi r16, 8
  out _SFR_IO_ADDR(DDRB), r16

  ldi r17, 0                     ; set all PINS to LOW voltage
  out _SFR_IO_ADDR(PORTB), r17

  in r18, _SFR_IO_ADDR(TCCR0B)   ; set prescaler to 1/1024
  ori r18, 5                     ; TCCR0B |= _BV(CS02) | _BV(CS00) 
  out _SFR_IO_ADDR(TCCR0B), r18

  in r19, _SFR_IO_ADDR(TIMSK0)   ; enable overflow vector
  ori r19, 2                     ; TIMSK0 |= _BV(TOIE0)
  out _SFR_IO_ADDR(TIMSK0), r19

  sei                   ; enable interupts

loop: rjmp loop

.end
