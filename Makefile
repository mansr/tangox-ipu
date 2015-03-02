-include config.mk

CROSS_COMPILE ?= mipsel-none-linux-gnu-

CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
SYSROOT = $(ROOT:%=--sysroot=%)
CFLAGS = -O2 -march=mips32r2 -std=c99 -Wall

HOST = ipu-load
IPU = iboot helloworld

all: $(HOST) $(IPU)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.S
	$(CC) $(ASFLAGS) -c -o $@ $<

ipu-load: CC += $(SYSROOT)
ipu-load: LDLIBS += -lelf
ipu-load: ipu-load.o
	$(CC) $(LDFLAGS) -o $@ $< $(LDLIBS)

$(IPU): CFLAGS += -fno-inline -fno-stack-protector -mno-abicalls
$(IPU): ASFLAGS += -march=mips32r2 -mno-abicalls

iboot-obj = iboot-head.o iboot.o
iboot: $(iboot-obj) iboot.x
	$(LD) $(LDFLAGS) -n -T iboot.x -o $@ $(iboot-obj)

helloworld: helloworld.o
	$(LD) $(LDFLAGS) -n -Ttext=0x90000000 -e _ftext -o $@ $<
