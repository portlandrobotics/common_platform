

CXXFLAGS=-Wall -Werror -O3

LD=g++

all: powerdown powerswitch

powerdown: powerdown.o
	$(LD) $(LDFLAGS) -o $@ $<

powerswitch: powerswitch.o
	$(LD) $(LDFLAGS) -o $@ $<

clean:
	rm -f *.o powerswitch powerdown


install: powerdown powerswitch
	cp powerdown /usr/lib/systemd/system-shutdown/powerdown
	cp powerswitch /usr/local/sbin/powerswitch
	cp powerswitch.service /etc/systemd/system/powerswitch.service
