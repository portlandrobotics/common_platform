OBJS=main.o
HEX=main.hex
ELF=main.elf

# set these base on your install locations
BINDIR=/cygdrive/c/Users/jfmiller/src/attiny/avr8-gnu-toolchain-win32_x86/bin/
PACKDIR=C:/Users/jfmiller/src/attiny/packs

GCC_CPU=attiny1604
PYUPDI_CPU=tiny1604

GCC_CPU_FLAGS=-mmcu=$(GCC_CPU) -B $(PACKDIR)/gcc/dev/$(GCC_CPU)/
CFLAGS=-Wall -Werror $(GCC_CPU_FLAGS) -O3 -fshort-enums -I $(PACKDIR)/include/
LDFLAGS=$(GCC_CPU_FLAGS)
CC=$(BINDIR)/avr-gcc
LD=$(BINDIR)/avr-gcc

all:    $(HEX)

$(ELF): $(OBJS)
	$(LD) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)

$(HEX): $(ELF)
	$(BINDIR)/avr-objcopy -O ihex -R .eeprom $< $@
	$(BINDIR)/avr-objdump -Pmem-usage $(ELF)

upload: $(HEX)
	scp $(HEX) pi@10.1.10.196:
	ssh pi@10.1.10.196 /home/pi/bin/pyupdi -d $(PYUPDI_CPU) -c /dev/ttyUSB0 -f $(HEX)

flash:  $(HEX)
	pyupdi -d $(PYUPDI_CPU) -c /dev/tty.usbserial-FTF5HUAV -f $(HEX)

read-fuses:
	pyupdi -d $(PYUPDI_CPU) -c /dev/tty.usbserial-FTF5HUAV -fr

clean:
	rm -f $(OBJS) $(ELF) $(HEX)
