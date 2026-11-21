CC ?= gcc
CFLAGS ?= -std=c11 -O2 -Wall -Wextra -Werror -pedantic

.PHONY: all test clean
all: test

test: test_telemetry_bits
	./test_telemetry_bits

test_telemetry_bits: test_telemetry_bits.c ../app_program/telemetry_bits.c ../app_program/telemetry_bits.h
	$(CC) $(CFLAGS) -o $@ test_telemetry_bits.c ../app_program/telemetry_bits.c

clean:
	rm -f test_telemetry_bits
