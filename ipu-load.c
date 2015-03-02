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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/cachectl.h>
#include <libelf.h>
#include "iboot.h"

#define IBOOT_PHYS	0xc8000000
#define IMAGE_PHYS	0xcc000000

#define IPU_BASE	0x000f0000
#define IPU_INTC	0xe000
#define IPU_REMAP	0xf000

#define REMAP_SIZE	0x04000000
#define REMAP_MASK	(REMAP_SIZE - 1)
#define REMAP_IDX(a)	(1 + (((a) >> 26) & 7))

#define align(v, a) (((v) + (a) - 1) & ~((a) - 1))
#define max(a, b) ((a) > (b) ? (a) : (b))

static long pagesize;
static long pagemask;

static unsigned remap[8];

static inline void writel(uint32_t val, void *addr)
{
	*(volatile uint32_t *)addr = val;
	__asm__ volatile ("sync");
}

static void update_remap(unsigned va, unsigned pa)
{
	unsigned ri = REMAP_IDX(va);

	if (ri < 8)
		remap[ri] = pa & ~REMAP_MASK;
	else
		fprintf(stderr, "invalid segment address %08x\n", va);
}

static inline unsigned combine(unsigned base, unsigned addr)
{
	return (base & ~REMAP_MASK) | (addr & REMAP_MASK);
}

static inline unsigned virt2phys(unsigned va)
{
	return combine(remap[REMAP_IDX(va)], va);
}

static int load_segment(const char *elf, Elf32_Phdr *ph,
			int memfd, unsigned lma, int dobss)
{
	unsigned map_base, map_size, offset;
	char *mem, *dst, *bss;

	map_base = lma & ~pagemask;
	offset = lma & pagemask;
	map_size = align(ph->p_memsz + offset, pagesize);

	mem = mmap(NULL, map_size, PROT_READ | PROT_WRITE, MAP_SHARED,
		   memfd, map_base);
	if (mem == MAP_FAILED) {
		perror("/dev/mem");
		return 1;
	}

	dst = mem + offset;
	bss = dst + ph->p_filesz;

	memcpy(dst, elf + ph->p_offset, ph->p_filesz);
	memset(bss, 0, -ph->p_filesz & 4);

	if (dobss)
		memset(bss, 0, ph->p_memsz - ph->p_filesz);

	_flush_cache(dst, ph->p_memsz, DCACHE);

	munmap(mem, map_size);

	return 0;
}

static unsigned load_elf(int memfd, const char *name, unsigned pa_base,
			 struct image *img, unsigned vl)
{
	unsigned top = 0;
	unsigned pa, va, pl;
	struct stat st;
	unsigned msize;
	char *elfm;
	Elf *elf;
	Elf32_Phdr *phdr;
	size_t num_phdr;
	int nload = 0;
	int efd;
	int err;
	int i;

	efd = open(name, O_RDONLY);
	if (efd < 0) {
		perror(name);
		return 0;
	}

	err = fstat(efd, &st);
	if (err) {
		perror("fstat");
		return 0;
	}

	msize = align(st.st_size, pagesize);

	elfm = mmap(NULL, msize, PROT_READ, MAP_SHARED, efd, 0);
	if (elfm == MAP_FAILED) {
		perror("mmap");
		goto e_close;
	}

	elf = elf_memory(elfm, st.st_size);
	if (!elf)
		goto e_elf;

	if (elf_getphdrnum(elf, &num_phdr) < 0)
		goto e_elf;

	if (img) {
		Elf32_Ehdr *ehdr = elf32_getehdr(elf);
		img->entry = ehdr->e_entry;
	}

	phdr = elf32_getphdr(elf);
	if (!phdr)
		goto e_elf;

	for (i = 0; i < num_phdr; i++) {
		if (phdr[i].p_type != PT_LOAD)
			continue;

		if (phdr[i].p_vaddr < REMAP_SIZE)
			continue;

		va = phdr[i].p_vaddr;
		pa = combine(pa_base, va);
		pl = vl ? virt2phys(vl) : pa;

		load_segment(elfm, &phdr[i], memfd, pl, !img);
		update_remap(va, pa);

		if (img) {
			struct segment *s = &img->segments[nload++];
			s->vma = va;
			s->lma = vl;
			s->fsz = phdr[i].p_filesz;
			s->msz = phdr[i].p_memsz;
			vl += phdr[i].p_filesz;
		}

		top = max(top, va + phdr[i].p_memsz);
	}

	if (img)
		img->num_segments = nload;

e_elf:
	err = elf_errno();
	if (err)
		fprintf(stderr, "%s: %s\n", name, elf_errmsg(err));
	elf_end(elf);
	munmap(elfm, msize);
e_close:
	close(efd);

	return top;
}

int main(int argc, char **argv)
{
	struct image *img;
	char *img_base;
	int mfd, mfd_nc;
	unsigned tva;
	char *ipu;
	int i;

	if (argc < 3)
		return 1;

	mfd = open("/dev/mem", O_RDWR);
	if (mfd == -1) {
		perror("/dev/mem");
		return 1;
	}

	mfd_nc = open("/dev/mem", O_RDWR | O_SYNC);
	if (mfd_nc == -1) {
		perror("/dev/mem");
		return 1;
	}

	pagesize = sysconf(_SC_PAGESIZE);
	pagemask = pagesize - 1;

	if (elf_version(EV_CURRENT) == EV_NONE) {
		fprintf(stderr, "%s\n", elf_errmsg(0));
		return 1;

	}

	for (i = 0; i < 8; i++)
		remap[i] = -1;

	tva = load_elf(mfd, argv[1], IBOOT_PHYS, NULL, 0);
	if (!tva) {
		fprintf(stderr, "failed to load iboot\n");
		return 1;
	}

	tva = align(tva, 0x10);

	img_base = mmap(NULL, 2 * pagesize, PROT_READ | PROT_WRITE, MAP_SHARED,
			mfd_nc, virt2phys(tva) & ~pagemask);
	if (img_base == MAP_FAILED) {
		perror("mmap");
		return 1;
	}

	img = (struct image *)(img_base + (tva & pagemask));

	tva = load_elf(mfd, argv[2], IMAGE_PHYS, img, tva + 0x100);
	if (!tva) {
		fprintf(stderr, "failed to load image\n");
		return 1;
	}

	munmap(img_base, 2 * pagesize);

	ipu = mmap(NULL, 0x10000, PROT_READ | PROT_WRITE, MAP_SHARED,
		   mfd_nc, IPU_BASE);
	if (ipu == MAP_FAILED) {
		perror("mmap");
		return 1;
	}

	/* disable external interrupts */
	writel(-1, ipu + IPU_INTC + 0x00c);
	writel(-1, ipu + IPU_INTC + 0x10c);
	writel(-1, ipu + IPU_INTC + 0x30c);

	for (i = 0; i < 8; i++)
		writel(remap[i], ipu + IPU_REMAP + 4 * i);

	/* enable and trigger software interrupt */
	writel(1, ipu + IPU_INTC + 0x08);
	writel(1, ipu + IPU_INTC + 0x10);

	munmap(ipu, 0x10000);

	close(mfd);
	close(mfd_nc);

	return 0;
}
