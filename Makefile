CROSS_COMPILE = /home/xxha/cross_compile/gcc-4.1.2-glibc-2.5-nptl-3/arm-none-linux-gnueabi/bin/arm-none-linux-gnueabi-
CFLAGS = -Wall -O2

CC=gcc

all:jamplayer

jamplayer:*.c
	$(CROSS_COMPILE)$(CC) $^ -o $@

clean:
	rm -f jamplayer

