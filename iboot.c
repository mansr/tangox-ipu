/*
  Copyright (c) 2015 Mans Rullgard <mans@mansr.com>
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdarg.h>
#include "iboot.h"

extern void dcache_wb(void);
extern void icache_inv(void);

#define XTAL_COUNT	0xa0010048

#define UART_BASE	0xa006c200
#define UART_THR	0x04
#define UART_IER	0x08
#define UART_FCR	0x10
#define UART_LCR	0x14
#define UART_LSR	0x1c
#define UART_CLKDIV	0x28
#define UART_CLKSEL	0x2c
#define UART_GPIO_MODE	0x38

#define always_inline inline __attribute__((always_inline))

static always_inline unsigned readl(unsigned addr)
{
	return *(volatile unsigned *)addr;
}

static always_inline void writel(unsigned val, unsigned addr)
{
	*(volatile unsigned *)addr = val;
	__asm__ volatile ("sync");
}

static void uart_init(void)
{
	unsigned base = UART_BASE;

	writel(1, base + UART_CLKSEL);
	writel(4, base + UART_CLKDIV);
	writel(0, base + UART_IER);
	writel(0, base + UART_FCR);
	writel(3, base + UART_LCR);
	writel(0x1100, base + UART_GPIO_MODE);
}

static void uart_putchar(char c)
{
	unsigned base = UART_BASE;
	int bits = 0x60;
	int status;

	do {
		status = readl(base + UART_LSR);
	} while ((status & bits) != bits);

	writel(c, base + UART_THR);
}

static char buf[16];

static char *tohex(unsigned v)
{
	static char hextab[] = "0123456789abcdef";
	char *p = buf;
	int i;

	for (i = 0; i < 8; i++) {
		*p++ = hextab[v >> 28];
		v <<= 4;
	}

	return buf;
}

static char *toudec(unsigned v)
{
	char *p = &buf[15];

	do {
		*--p = '0' + v % 10;
		v /= 10;
	} while (v);

	return p;
}

static char *tosdec(int v)
{
	char *p = &buf[15];
	int s = v >> 31;

	v = (v ^ s) - s;

	do {
		*--p = '0' + v % 10;
		v /= 10;
	} while (v);

	if (s)
		*--p = '-';

	return p;
}

static void uart_printf(const char *s, ...)
{
	va_list args;

	va_start(args, s);

	while (*s) {
		if (*s == '%') {
			switch (s[1]) {
			case 'd':
			case 'i':
				uart_printf(tosdec(va_arg(args, int)));
				s += 2;
				continue;

			case 'u':
				uart_printf(toudec(va_arg(args, unsigned)));
				s += 2;
				continue;

			case 'x':
				uart_printf(tohex(va_arg(args, unsigned)));
				s += 2;
				continue;
			}
		}

		if (*s == '\n')
			uart_putchar('\r');

		uart_putchar(*s++);
	}

	va_end(args);
}

extern struct image image;

static void copy_segment(struct segment *s)
{
	unsigned *dst = (unsigned *)s->vma;
	unsigned *src = (unsigned *)s->lma;
	unsigned fsz = (s->fsz + 3) / 4;
	unsigned nz = (s->msz + 3) / 4 - fsz;

	uart_printf("Copying segment: dst=%x src=%x fsz=%x msz=%x\n",
		    s->vma, s->lma, s->fsz, s->msz);

	while (fsz--)
		*dst++ = *src++;

	while (nz--)
		*dst++ = 0;
}

typedef void (*entry_fn)(unsigned, unsigned, unsigned, unsigned)
	__attribute__((noreturn));

void iboot_main(void)
{
	unsigned start, stop;
	int i;

	uart_init();
	uart_printf("This is iboot\n");

	start = readl(XTAL_COUNT);

	for (i = 0; i < image.num_segments; i++)
		copy_segment(&image.segments[i]);

	dcache_wb();
	icache_inv();

	stop = readl(XTAL_COUNT);

	uart_printf("Image loaded in %u us\n", (stop - start) / 27);
	uart_printf("Jumping to entry point %x\n", image.entry);

	((entry_fn)image.entry)(0, 0, 0, 0);
}
