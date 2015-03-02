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

#define c0_status	$12
#define c0_config	$16
#define c0_config1	$16, 1
#define c0_taglo	$28

#define index_inv_i	0
#define index_wb_inv_d	1
#define index_st_tag_d	9

	.set		mips32r2
	.set		noreorder

	.section	.init, "ax", @progbits

.macro	bentry		size
	.rept		\size / 8
	b		entry
	li		t4, 2
	.endr
.endm

.macro	do_cache	op, cf1, p
	ext		a0, \cf1, \p, 3
	ext		a1, \cf1, \p + 3, 3
	ext		a2, \cf1, \p + 6, 3
	addiu		a0, a0, 1
	sllv		t0, t4, a1
	sllv		t1, t5, a2
	mul		t1, a0, t1
	lui		t2, 0x8000
1:
	cache		\op, 0(t2)
	addiu		t1, t1, -1
	bnez		t1, 1b
	add		t2, t2, t0
.endm

	/* TLB refill vector */
tlbex:
	bentry		0x60

	.org		0x80
entry:
	/* disable cache */
	mfc0		s0, c0_config
	ins		s0, t4, 0, 3
	mtc0		s0, c0_config

	/* clear exception level, disable interrupts */
	mfc0		t0, c0_status
	ins		t0, zero, 0, 5
	mtc0		t0, c0_status

	/* invalidate caches */
	bal		icache_inv
	mtc0		zero, c0_taglo
	do_cache	index_st_tag_d, v0, 7

	/* enable cache */
	ori		s0, s0, 3
	mtc0		s0, c0_config

	/* set up stack pointer and jump to C code */
	lui		sp, %hi(.stack + 4096)
	j		iboot_main
	ori		sp, sp, %lo(.stack + 4096)

	.global		dcache_wb
	.type		dcache_wb, STT_FUNC
dcache_wb:
	mfc0		v0, c0_config1
	li		t4, 2
	li		t5, 64
	do_cache	index_wb_inv_d, v0, 7
	jr		ra
	sync

	.global		icache_inv
	.type		icache_inv, STT_FUNC
icache_inv:
	mfc0		v0, c0_config1
	li		t4, 2
	li		t5, 64
	do_cache	index_inv_i, v0, 16
	jr		ra
	sync

	/* general exception vector */
	.org		0x180
genex:
	bentry		0x380

	.comm		.stack, 4096
