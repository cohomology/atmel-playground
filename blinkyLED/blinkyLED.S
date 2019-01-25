// by Kilian Kilger, 2019. Releaesed in the public domain.

#include <avr/io.h>

/* GLOBAL DEFINES */

/* Macros to convert between the memory addresses
   and the acutual output port */
.equ _PORTB_, _SFR_IO_ADDR(PORTB) 
.equ _DDRB_, _SFR_IO_ADDR(DDRB)  
.equ _TIMSK0_, _SFR_IO_ADDR(TIMSK0)
.equ _TCCR0B_, _SFR_IO_ADDR(TCCR0B)

.equ LED_PIN, PB3                                          ; LED connected to PIN3 of ATtiny13

/* DATA SECTION */
.data
counter: .byte 0

/* CODE SECTION */
.text

.global TIM0_OVF_vect 
.type TIM0_OVF_vect, @function
TIM0_OVF_vect:
  lds r22, counter                                         ; check if counter == 5
  cpi r22, 5
  brne no_blink                                            ; if no => counter++ and quit
  ldi r22, 0
  sts counter, r22                                         ; if yes, set counter to 0 and toggle
  in r20, _PORTB_                                          ; toggle LED
  ldi r21, (1 << LED_PIN)                                  ; i.e. PORTB ^= (1 << LED_PIN)
  eor r20, r21 
  out _PORTB_, r20
  reti
no_blink:
  inc r22
  sts counter, r22  
  reti 

.section .text.startup
.global main
.type main, @function 
main:
  cli                                                      ; disable interupts

  ldi r16, (1 << LED_PIN)                                  ; set PB3 as output pin, i.e. DDRB = 1 << LED_PIN
  out _DDRB_, r16

  ldi r17, 0                                               ; set all PINS to LOW voltage, i.e. _PORTB_ = 0
  out _PORTB_, r17
  sts counter, r17                                         ; initialize counter to 0

  in r18, _TCCR0B_                                         ; set prescaler to 1/1024
  ori r18, ( (1 << CS02) | ( 1 << CS00) )                  ; i.e.: TCCR0B |= _BV(CS02) | _BV(CS00) 
  out _TCCR0B_, r18

  in r19, _TIMSK0_                                         ; enable overflow vector
  ori r19, (1 << TOIE0)                                    ; TIMSK0 |= _BV(TOIE0)
  out _TIMSK0_, r19
  
  sei                                                      ; enable interupts

loop: rjmp loop

.end
