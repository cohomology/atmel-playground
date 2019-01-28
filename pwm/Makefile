pwm: pwm.elf pwm.hex
	avr-size --format=avr --mcu=attiny13 $<

pwm.hex: pwm.elf
	avr-objcopy -j .text -j .data -O ihex $< $@

pwm.elf: pwm.o
	avr-gcc -g -o $@ -DF_CPU=9600000 -mmcu=attiny13 $< 

pwm.o: pwm.c
	avr-g++ -std=c++14 -Os -g -DF_CPU=9600000 -mmcu=attiny13 $< -c -o $@

.phony: clean
clean:
	rm -f *.elf *.hex *.o a.out

.phony: upload
upload: pwm.hex
	avrdude -p attiny13 -b 19200 -c stk500v1 -P /dev/ttyACM0 -U lfuse:w:0x3A:m -U hfuse:w:0xF9:m -U lock:w:0xFF:m -U flash:w:$<

.phony: show-fuses
show-fuses: 
	avrdude -pattiny13 -b 19200 -c stk500v1 -P /dev/ttyACM0 -nv

.phony: check-interrupts
check-interrupts: pwm.elf
	avr-objdump -h -S -z $< > $(<F).lss 

