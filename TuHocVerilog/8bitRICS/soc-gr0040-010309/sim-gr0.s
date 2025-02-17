; sim.s -- Simple simulator startup code
; Copyright (C) 2000, Gray Research LLC.  All rights reserved.
; Usage subject to XSOC License Agreement.  See the LICENSE file.

global _main

reti:
	jal r0,0(r0)

intr:
	lea sp,-2(sp)
	sw r0,0(sp)
	xor r0,r0

    sb r0,0x8007 ; re-enable timer int

	lw r0,0(sp)
	lea sp,2(sp)
	br reti

align 32
reset:
	xor r0,r0
	lea sp,0x3FE
	call _main
	j reset
