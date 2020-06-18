CC = gcc
CFLAGS = -Wall -Wextra -pedantic -Werror -g -Og
CFLAGS += -I/usr/include/libusb-1.0
LDFLAGS = -lusb-1.0

all: test_laser jack_bridge

test_laser: test_laser.c
	$(CC) -o $@ $< $(CFLAGS) $(LDFLAGS) -pthread -lm

jack_bridge: jack_bridge.c
	$(CC) -o $@ $< $(CFLAGS) $(LDFLAGS) -ljack 

clean:
	rm -f test_laser
	rm -f jack_bridge