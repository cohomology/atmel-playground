blinkyLED: blinkyLED.elf blinkyLED.hex
	avr-size --format=avr --mcu=attiny13 $<

blinkyLED.hex: blinkyLED.elf
	avr-objcopy -j .text -j .data -O ihex $< $@

blinkyLED.elf: blinkyLED.o
	avr-gcc -g -o $@ -DF_CPU=1200000 -mmcu=attiny13 $< 

blinkyLED.o: blinkyLED.S
	avr-gcc -g -O3 -DF_CPU=1200000 -mmcu=attiny13 $< -c -o $@

.phony: clean
clean:
	rm -f *.elf *.hex *.o a.out

.phony: upload
upload: blinkyLED.hex
	avrdude -p attiny13 -b 19200 -c stk500v1 -P /dev/ttyACM0 -U lfuse:w:0x2A:m -U hfuse:w:0xF9:m -U lock:w:0xFF:m -U flash:w:$<

.phony: show-fuses
show-fuses: 
	avrdude -pattiny13 -b 19200 -c stk500v1 -P /dev/ttyACM0 -nv

.phony: check-interrupts
check-interrupts: blinkyLED.elf
	avr-objdump -h -S -z $< > $(<F).lss 

