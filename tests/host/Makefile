# Host unit tests for firmware/app.c (no hardware or Silicon Labs SDK needed)
CC      ?= cc
CFLAGS  ?= -std=c11 -O1 -g -Wall -Wextra -Wno-unused-function
CFLAGS  += -DVELUX_HOST_TEST -Istubs -I. -I../../firmware

test_firmware: test_firmware.c fake_platform.c fake_platform.h stubs/velux_host_stubs.h ../../firmware/app.c
	$(CC) $(CFLAGS) -o $@ test_firmware.c fake_platform.c

.PHONY: test clean
test: test_firmware
	./test_firmware

clean:
	rm -rf test_firmware test_firmware.dSYM
