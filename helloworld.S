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

#include <regdef.h>

	.set		mips32r2
	.set		noreorder

.macro	print_cp0	name, reg, sel=0
	addiu		a0, s0, \name - _data + 8
	bal		hex2s
	mfc0		a1, \reg, \sel
	bal		puts
	addiu		a0, s0, \name - _data
.endm

start:
	bal		1f
	nop
1:
	addiu		s0, ra, _data - .
	addiu		s1, s0, hex_tab - _data
	li		s2, 0xa006c200

	bal		puts
	move		a0, s1

	print_cp0	status, $12
	print_cp0	intctl, $12, 1
	print_cp0	cause, $13
	print_cp0	prid, $15
	print_cp0	ebase, $15, 1
	print_cp0	config, $16
	print_cp0	config1, $16, 1
	print_cp0	config2, $16, 2
	print_cp0	config3, $16, 3

        li              t0, 0xa00fe000
        li              t1, -1
        sw              t1, 0x0c(t0)
        sw              t1, 0x14(t0)
        li              t1, 1
        sw              t1, 0x08(t0)

        mfc0            t0, $12
        ins             t0, zero, 8, 8
        ori             t0, t0, 1 << 10
        mtc0            t0, $12
        ei

2:	wait
	b		2b
	nop

hex2s:
	li		t2, 8
1:
	srl		t0, a1, 28
	add		t0, t0, s1
	lb		t0, 0(t0)
	sb		t0, 0(a0)
	addiu		t2, t2, -1
	sll		a1, a1, 4
	bnez		t2, 1b
	addiu		a0, a0, 1
	jr		ra
	sb		zero, 0(a0)

puts:
1:	lb		t0, 0(a0)
	li		t2, 0xd
	movn		t2, t0, t0
	addiu		a0, a0, 1
2:	lw		t1, 0x1c(s2)
	andi		t1, t1, 0x60
	xori		t1, t1, 0x60
	bnez		t1, 2b
	nop
	bgtz		t0, 1b
	sw		t2, 4(s2)
	li		t2, 0xa
	beqz		t0, 2b
	li		t0, -1
	jr		ra
	nop

	.p2align	5
_data:
status:	.ascii		"status"
	.p2align	5, 0x20
intctl:	.ascii		"intctl"
	.p2align	5, 0x20
cause:	.ascii		"cause"
	.p2align	5, 0x20
prid:	.ascii		"prid"
	.p2align	5, 0x20
ebase:	.ascii		"ebase"
	.p2align	5, 0x20
config:	.ascii		"config"
	.p2align	5, 0x20
config1:.ascii		"config1"
	.p2align	5, 0x20
config2:.ascii		"config2"
	.p2align	5, 0x20
config3:.ascii		"config3"
	.p2align	5, 0x20
hex_tab:
	.asciz		"0123456789abcdef"
