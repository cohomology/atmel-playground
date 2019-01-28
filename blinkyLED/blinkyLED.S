// by Kilian Kilger, 2019. Releaesed in the public domain.

#include <avr/io.h>

/* GLOBAL DEFINES */

/* Macros to convert between the memory addresses
   and the acutual output port */
.equ _PINB_  , _SFR_IO_ADDR(PINB)
.equ _PORTB_ , _SFR_IO_ADDR(PORTB) 
.equ _DDRB_  , _SFR_IO_ADDR(DDRB)  
.equ _TIMSK0_, _SFR_IO_ADDR(TIMSK0)
.equ _TCCR0A_, _SFR_IO_ADDR(TCCR0A) 
.equ _TCCR0B_, _SFR_IO_ADDR(TCCR0B)
.equ _OCR0A_ , _SFR_IO_ADDR(OCR0A)

.equ LED_PIN   , PB3                                          ; LED connected to PB3 of ATtiny13
.equ BUTTON_PIN, PB4                                          ; Button is connected to PB4
.equ MOTOR_PIN , PB0                                          ; Motor is connected to PB0

/* DATA SECTION */
.section bss
counter:           .byte 1
prev_button_state: .byte 1
button_state:      .byte 1

/* CODE SECTION */
.text

.global TIM0_COMPA_vect 
.type TIM0_COMPA_vect, @function
TIM0_COMPA_vect:
  in   r23, _PINB_
  andi r23, ( 1 << BUTTON_PIN )
  lds  r24, prev_button_state
  sts prev_button_state, r23
  cp   r23, r24                                             ; if ( TCCR0B & ( 1 << BUTTON_PIN ) ) != prev_button_state
  brne button_changed                                       ;   goto button_changed
check_blink:
  lds  r27, button_state       
  cpi  r27, 1
  brne end
blink:
  lds  r22, counter                                         ; check if counter == 5
  cpi  r22, 5
  brne no_blink                                             ; if no => counter++ and quit
  ldi  r22, 0
  sts  counter, r22                                         ; if yes, set counter to 0 and toggle
  in   r20, _PORTB_                                         ; toggle LED
  ldi  r21, (1 << LED_PIN)                                  ; i.e. PORTB ^= (1 << LED_PIN)
  eor  r20, r21 
  out  _PORTB_, r20
  reti
no_blink:
  inc  r22
  sts  counter, r22  
end:
  reti
button_changed:
  cpi  r23, ( 1 << BUTTON_PIN )
  brne check_blink
  lds  r25, button_state 
  ldi  r26, 1
  eor  r25, r26
  sts  button_state, r25
  ldi  r27, 0
  sts  counter, r27 
  brne check_blink  

.section .text.startup
.global main
.type main, @function 
main:
  cli                                                      ; disable interupts

  clr r0                                                   ; r0 always stays zero
  out _PORTB_, r0                                          ; initialize PORTB, i.e. all ports pulled low
  out _DDRB_, r0                                           ; initialize DDRB, i.e. all ports defined as inputs

  sbi _DDRB_, LED_PIN                                      ; set LED pin as output

  in r16, _TCCR0B_                                         ; set prescaler to 1/1024
  ori r16, ( (1 << CS02) | ( 1 << CS00) )                  ; i.e.: TCCR0B |= _BV(CS02) | _BV(CS00) 
  out _TCCR0B_, r16

  in r17, _TCCR0A_                                         ; set timer to CTC mode
  ori r17, ( 1 << WGM01 )
  out _TCCR0A_, r17

  ldi r18, 64                                              ; set counter limit in CTC mode to 64
  out _OCR0A_, r18                                         ; this makes 1/1200000*1024*64 = 54ms 

  in r19, _TIMSK0_                                         ; enable timer on compare match interrupt
  ori r19, (1 << OCIE0A)                                   ; i.e. TIMSK0 |= _BV(OCIE0A)
  out _TIMSK0_, r19 

  sei                                                      ; enable interupts

loop: rjmp loop

.global __do_copy_data                                     ; the following lines are needed to correctly initialize
.global __do_clear_bss                                     ; data and eventual bss sections
.end
