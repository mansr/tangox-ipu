ENTRY(__text)

PHDRS {
	text PT_LOAD FLAGS(7);
}

SECTIONS {
	.text 0x8fb85000 : {
		__text = .;
		*(.init)
		*(.text)
	} :text
	.rodata : {
		*(.rodata*)
	}
	.data : {
		*(.data*)
	}
	.bss : {
		*(.bss*)
	}
	image = ALIGN(0x10);
	/DISCARD/ : {
		*(.reginfo)
	}
}
