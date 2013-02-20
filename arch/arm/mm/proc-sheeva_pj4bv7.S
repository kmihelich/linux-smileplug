/*
 *  linux/arch/arm/mm/proc-sheeva_pj4bv7.S
 *
 *  Copyright (C) 2001 Deep Blue Solutions Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  This is the "shell" of the ARMv7 processor support.
 */
#include <linux/init.h>
#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/asm-offsets.h>
#include <asm/hwcap.h>
#include <asm/pgtable-hwdef.h>
#include <asm/pgtable.h>
#if defined(CONFIG_ARCH_ARMADA370)
#include <mach/armada370.h>
#include <mach/kw_macro.h>
#elif defined(CONFIG_ARCH_ARMADA_XP)
#include <mach/armadaxp.h>
#endif

#include "proc-macros.S"
#include "sheeva_pj4b-macros.S"

#define TTB_S		(1 << 1)
#define TTB_RGN_NC	(0 << 3)
#define TTB_RGN_OC_WBWA	(1 << 3)
#define TTB_RGN_OC_WT	(2 << 3)
#define TTB_RGN_OC_WB	(3 << 3)
#define TTB_NOS		(1 << 5)
#define TTB_IRGN_NC	((0 << 0) | (0 << 6))
#define TTB_IRGN_WBWA	((0 << 0) | (1 << 6))
#define TTB_IRGN_WT	((1 << 0) | (0 << 6))
#define TTB_IRGN_WB	((1 << 0) | (1 << 6))

#ifndef CONFIG_SMP
/* PTWs cacheable, inner WB not shareable, outer WB not shareable */
#define TTB_FLAGS	TTB_IRGN_WB|TTB_RGN_OC_WB
#define PMD_FLAGS	PMD_SECT_WB
#else
/* PTWs cacheable, inner WBWA shareable, outer WBWA not shareable */
#define TTB_FLAGS	TTB_IRGN_WBWA|TTB_S|TTB_NOS|TTB_RGN_OC_WBWA
#define PMD_FLAGS	PMD_SECT_WBWA|PMD_SECT_S
#endif

ENTRY(cpu_pj4bv7_proc_init)
	mov	pc, lr
ENDPROC(cpu_pj4bv7_proc_init)

ENTRY(cpu_pj4bv7_proc_fin)
	stmfd	sp!, {lr}
	cpsid	if				@ disable interrupts
	bl	v7_flush_kern_cache_all
#ifdef CONFIG_CACHE_AURORA_L2
	mcr	p15, 0, r0, c7, c10, 4		@ Data Synchronization Barrier
	ldr	r0, =0xffff			@ L2C clean all 16 ways
	ldr	r1, =AXP_L2_CLEAN_WAY_REG
	str	r0, [r1]
	mrc	p15, 0, r0, c0, c0, 5		@ Read CPU core number
	and	r0, r0, #0xF
	mov	r1, #0x1
	lsl	r0, r1, r0
1:	ldr	r1, =AXP_L2_MNTNC_STAT_REG	@ Read maintanence status to check done per CPU
	ldr	r1, [r1]
	tst	r0, r1
	bne	1b
	mcr	p15, 0, r0, c7, c10, 4		@ Data Synchronization Barrier
#endif
	mrc	p15, 0, r0, c1, c0, 0		@ ctrl register
	bic	r0, r0, #0x1000			@ ...i............
	bic	r0, r0, #0x0006			@ .............ca.
	mcr	p15, 0, r0, c1, c0, 0		@ disable caches
	ldmfd	sp!, {pc}
ENDPROC(cpu_pj4bv7_proc_fin)

/*
 *	cpu_pj4bv7_reset(loc)
 *
 *	Perform a soft reset of the system.  Put the CPU into the
 *	same state as it would be if it had been reset, and branch
 *	to what would be the reset vector.
 *
 *	- loc   - location to jump to for soft reset
 */
	.align	5
ENTRY(cpu_pj4bv7_reset)
	mov	pc, r0
ENDPROC(cpu_pj4bv7_reset)

/*
 *	cpu_pj4bv7_do_idle()
 *
 *	Idle the processor (eg, wait for interrupt).
 *
 *	IRQs are already disabled.
 */
ENTRY(cpu_pj4bv7_do_idle)
#ifndef CONFIG_SHEEVA_ERRATA_ARM_CPU_BTS61
#ifdef CONFIG_ARCH_ARMADA370
        ldr     r0, support_wait_for_interrupt_address /* this variable set in core.c*/
        ldr     r0, [r0]
        cmp     r0, #1    /* check if the device doesn't support wait for interrupt*/
        bne     1f        /* if yes, then go out*/
#endif
	dsb					@ WFI may enter a low-power mode
	wfi
#ifdef CONFIG_SHEEVA_ERRATA_ARM_CPU_4742
	mcr	p15, 0, r0, c7, c10, 4		@barrier
#endif
#endif
1:
	mov	pc, lr
ENDPROC(cpu_pj4bv7_do_idle)

ENTRY(cpu_pj4bv7_dcache_clean_area)
#ifndef TLB_CAN_READ_FROM_L1_CACHE
	dcache_line_size r2, r3
1:	mcr	p15, 0, r0, c7, c10, 1		@ clean D entry
	add	r0, r0, r2
	subs	r1, r1, r2
	bhi	1b
	dsb
#endif
	mov	pc, lr
ENDPROC(cpu_pj4bv7_dcache_clean_area)

/*
 *	cpu_pj4bv7_switch_mm(pgd_phys, tsk)
 *
 *	Set the translation table base pointer to be pgd_phys
 *
 *	- pgd_phys - physical address of new TTB
 *
 *	It is assumed that:
 *	- we are not using split page tables
 */
ENTRY(cpu_pj4bv7_switch_mm)
#ifdef CONFIG_MMU
	mov	r2, #0
	ldr	r1, [r1, #MM_CONTEXT_ID]	@ get mm->context.id
#ifdef CONFIG_AURORA_L2_PT_WALK 
	orr	r0, r0, #TTB_FLAGS
#else
	bic     r0, r0, #0x18                   @ DONOT Cache the page table in L2
#endif
#ifdef CONFIG_ARM_ERRATA_430973
	mcr	p15, 0, r2, c7, c5, 6		@ flush BTAC/BTB
#endif
	mcr	p15, 0, r2, c13, c0, 1		@ set reserved context ID
	isb
1:	mcr	p15, 0, r0, c2, c0, 0		@ set TTB 0
	isb
	mcr	p15, 0, r1, c13, c0, 1		@ set context ID
	isb
#endif
	mov	pc, lr
ENDPROC(cpu_pj4bv7_switch_mm)

/*
 *	cpu_pj4bv7_set_pte_ext(ptep, pte)
 *
 *	Set a level 2 translation table entry.
 *
 *	- ptep  - pointer to level 2 translation table entry
 *		  (hardware version is stored at -1024 bytes)
 *	- pte   - PTE value to store
 *	- ext	- value for extended PTE bits
 */
ENTRY(cpu_pj4bv7_set_pte_ext)
#ifdef CONFIG_MMU
 ARM(	str	r1, [r0], #-2048	)	@ linux version
 THUMB(	str	r1, [r0]		)	@ linux version
 THUMB(	sub	r0, r0, #2048		)

#ifdef CONFIG_MV_SUPPORT_64KB_PAGE_SIZE
	mov     r3, #0x7C
	and     r3, r3, r0
	mov     r3, r3, lsl #4
	bic     r0, r0, #0x3FC
	bic     r0, r0, #0x400
	orr     r0, r0, r3
#endif
	bic	r3, r1, #0x000003f0
#ifdef CONFIG_MV_SUPPORT_64KB_PAGE_SIZE
	bic	r3, r3, #0x00000F000
#endif
	bic	r3, r3, #PTE_TYPE_MASK
	orr	r3, r3, r2
#ifdef CONFIG_MV_SUPPORT_64KB_PAGE_SIZE
	orr	r3, r3, #PTE_EXT_AP0 | 1
#else
	orr	r3, r3, #PTE_EXT_AP0 | 2
#endif
	tst	r1, #1 << 4
	orrne	r3, r3, #PTE_EXT_TEX(1)

	tst	r1, #L_PTE_WRITE
	tstne	r1, #L_PTE_DIRTY
	orreq	r3, r3, #PTE_EXT_APX

	tst	r1, #L_PTE_USER
	orrne	r3, r3, #PTE_EXT_AP1
	tstne	r3, #PTE_EXT_APX
	bicne	r3, r3, #PTE_EXT_APX | PTE_EXT_AP0

	tst	r1, #L_PTE_EXEC
	orreq	r3, r3, #PTE_EXT_XN

	tst	r1, #L_PTE_YOUNG
	tstne	r1, #L_PTE_PRESENT
	moveq	r3, #0

	str	r3, [r0]
#ifdef CONFIG_MV_SUPPORT_64KB_PAGE_SIZE
	@ Need to duplicate the entry 16 times because of overlapping in PTE index bits.
	str     r3, [r0, #4]
	str     r3, [r0, #8]
	str     r3, [r0, #12]
	str     r3, [r0, #16]
	str     r3, [r0, #20]
	str     r3, [r0, #24]
	str     r3, [r0, #28]
	mcr	p15, 0, r0, c7, c10, 1		@ flush_pte
	add	r0, r0, #32
	str     r3, [r0]
	str     r3, [r0, #4]
	str     r3, [r0, #8]
	str     r3, [r0, #12]
	str     r3, [r0, #16]
	str     r3, [r0, #20]
	str     r3, [r0, #24]
	str     r3, [r0, #28]
	mcr	p15, 0, r0, c7, c10, 1		@ flush_pte
#else
	mcr	p15, 0, r0, c7, c10, 1		@ flush_pte
#endif /* CONFIG_MV_SUPPORT_64KB_PAGE_SIZE */
#endif
	mov	pc, lr
ENDPROC(cpu_pj4bv7_set_pte_ext)

cpu_pj4bv7_name:
	.ascii	"Marvell PJ4Bv7 Processor"
	.align

	__INIT

/*
 *	__pj4bv7_setup
 *
 *	Initialise TLB, Caches, and MMU state ready to switch the MMU
 *	on.  Return in r0 the new CP15 C1 control register setting.
 *
 *	We automatically detect if we have a Harvard cache, and use the
 *	Harvard cache control instructions insead of the unified cache
 *	control instructions.
 *
 *	This should be able to cover all ARMv7 cores.
 *
 *	It is assumed that:
 *	- cache type register is implemented
 */
__pj4bv7_setup:
	sheeva_pj4b_config

#if 0
@ CURRENTLY NOT SUPPORTED 
defined(CONFIG_SMP)
	mrc	p15, 0, r0, c1, c0, 1		@ Enable SMP/nAMP mode
	orr	r0, r0, #0x20
	mcr	p15, 0, r0, c1, c0, 1
#if 1
	mrc 	p15, 1, r0, c15, c2, 0
	orr	r0, r0, #0x2			@ SMP enable 
	mcr 	p15, 1, r0, c15, c2, 0
#endif
#endif

#ifdef CONFIG_SMP
	mrc	p15, 0, r0, c1, c0, 1
	tst	r0, #(1 << 6)			@ SMP/nAMP mode enabled?
	orreq	r0, r0, #(1 << 6) | (1 << 0)	@ Enable SMP/nAMP mode and
	mcreq	p15, 0, r0, c1, c0, 1		@ TLB ops broadcasting
#endif
#ifdef CONFIG_CPU_SHEEVA_PJ4B_PMC_ACCESS_IN_USERMODE
	@ Enable performance counters user access
	mrc     p15, 0, r0, c9, c14, 0
	orr     r0, r0, #0x1
	mcr     p15, 0, r0, c9, c14, 0
#endif /* CONFIG_CPU_SHEEVA_PJ4B_PMC_ACCESS_IN_USERMODE */
	adr	r12, __pj4bv7_setup_stack		@ the local stack
	stmia	r12, {r0-r5, r7, r9, r11, lr}
	bl	v7_flush_dcache_all
	ldmia	r12, {r0-r5, r7, r9, r11, lr}

	mrc	p15, 0, r0, c0, c0, 0		@ read main ID register
	and	r10, r0, #0xff000000		@ ARM?
	teq	r10, #0x41000000
	bne	2f
	and	r5, r0, #0x00f00000		@ variant
	and	r6, r0, #0x0000000f		@ revision
	orr	r0, r6, r5, lsr #20-4		@ combine variant and revision

#ifdef CONFIG_ARM_ERRATA_430973
	teq	r5, #0x00100000			@ only present in r1p*
	mrceq	p15, 0, r10, c1, c0, 1		@ read aux control register
	orreq	r10, r10, #(1 << 6)		@ set IBE to 1
	mcreq	p15, 0, r10, c1, c0, 1		@ write aux control register
#endif
#ifdef CONFIG_ARM_ERRATA_458693
	teq	r0, #0x20			@ only present in r2p0
	mrceq	p15, 0, r10, c1, c0, 1		@ read aux control register
	orreq	r10, r10, #(1 << 5)		@ set L1NEON to 1
	orreq	r10, r10, #(1 << 9)		@ set PLDNOP to 1
	mcreq	p15, 0, r10, c1, c0, 1		@ write aux control register
#endif
#ifdef CONFIG_ARM_ERRATA_460075
	teq	r0, #0x20			@ only present in r2p0
	mrceq	p15, 1, r10, c9, c0, 2		@ read L2 cache aux ctrl register
	tsteq	r10, #1 << 22
	orreq	r10, r10, #(1 << 22)		@ set the Write Allocate disable bit
	mcreq	p15, 1, r10, c9, c0, 2		@ write the L2 cache aux ctrl register
#endif

2:	mov	r10, #0
#ifdef HARVARD_CACHE
	mcr	p15, 0, r10, c7, c5, 0		@ I+BTB cache invalidate
#endif
	dsb
#ifdef CONFIG_MMU
	mcr	p15, 0, r10, c8, c7, 0		@ invalidate I + D TLBs
	mcr	p15, 0, r10, c2, c0, 2		@ TTB control register
#ifdef CONFIG_AURORA_L2_PT_WALK
	orr	r4, r4, #TTB_FLAGS
#else
						@ OC bits in TTB1 register
	bic	r4, r4, #0x18                   @ DONOT Cache the page table in L2
#endif
	mcr	p15, 0, r4, c2, c0, 1		@ load TTB1
	mov	r10, #0x1f			@ domains 0, 1 = manager
	mcr	p15, 0, r10, c3, c0, 0		@ load domain access register
	/*
	 * Memory region attributes with SCTLR.TRE=1
	 *
	 *   n = TEX[0],C,B
	 *   TR = PRRR[2n+1:2n]		- memory type
	 *   IR = NMRR[2n+1:2n]		- inner cacheable property
	 *   OR = NMRR[2n+17:2n+16]	- outer cacheable property
	 *
	 *			n	TR	IR	OR
	 *   UNCACHED		000	00
	 *   BUFFERABLE		001	10	00	00
	 *   WRITETHROUGH	010	10	10	10
	 *   WRITEBACK		011	10	11	11
	 *   reserved		110
	 *   WRITEALLOC		111	10	01	01
	 *   DEV_SHARED		100	01
	 *   DEV_NONSHARED	100	01
	 *   DEV_WC		001	10
	 *   DEV_CACHED		011	10
	 *
	 * Other attributes:
	 *
	 *   DS0 = PRRR[16] = 0		- device shareable property
	 *   DS1 = PRRR[17] = 1		- device shareable property
	 *   NS0 = PRRR[18] = 0		- normal shareable property
	 *   NS1 = PRRR[19] = 1		- normal shareable property
	 *   NOS = PRRR[24+n] = 1	- not outer shareable
	 */
	ldr	r5, =0xff0a81a8			@ PRRR
	ldr	r6, =0x40e040e0			@ NMRR
	mcr	p15, 0, r5, c10, c2, 0		@ write PRRR
	mcr	p15, 0, r6, c10, c2, 1		@ write NMRR
#endif
	adr	r5, v7_crval
	ldmia	r5, {r5, r6}
#ifdef CONFIG_CPU_ENDIAN_BE8
	orr	r6, r6, #1 << 25		@ big-endian page tables
#endif
   	mrc	p15, 0, r0, c1, c0, 0		@ read control register
	bic	r0, r0, r5			@ clear bits them
	orr	r0, r0, r6			@ set them
 THUMB(	orr	r0, r0, #1 << 30	)	@ Thumb exceptions
	mov	pc, lr				@ return to head.S:__ret
ENDPROC(__pj4bv7_setup)

	/*   AT
	 *  TFR   EV X F   I D LR    S
	 * .EEE ..EE PUI. .T.T 4RVI ZWRS BLDP WCAM
	 * rxxx rrxx xxx0 0101 xxxx xxxx x111 xxxx < forced
	 *    1    0 110       0011 1100 .111 1101 < we want
	 */
	.type	v7_crval, #object
v7_crval:
	crval	clear=0x0120c302, mmuset=0x10c03c7d, ucset=0x00c01c7c

__pj4bv7_setup_stack:
	.space	4 * 11				@ 11 registers

	.type	v7_processor_functions, #object
ENTRY(v7_processor_functions)
	.word	v7_early_abort
	.word	v7_pabort
	.word	cpu_pj4bv7_proc_init
	.word	cpu_pj4bv7_proc_fin
	.word	cpu_pj4bv7_reset
	.word	cpu_pj4bv7_do_idle
	.word	cpu_pj4bv7_dcache_clean_area
	.word	cpu_pj4bv7_switch_mm
	.word	cpu_pj4bv7_set_pte_ext
	.size	v7_processor_functions, . - v7_processor_functions

	.type	cpu_arch_name, #object
cpu_arch_name:
	.asciz	"armv7"
	.size	cpu_arch_name, . - cpu_arch_name

	.type	cpu_elf_name, #object
cpu_elf_name:
	.asciz	"v7"
	.size	cpu_elf_name, . - cpu_elf_name
	.align

	.section ".proc.info.init", #alloc, #execinstr

	/*
	 * Match any ARMv7 processor core.
	 */
	.type	__v7_proc_info, #object
__v7_proc_info:
	.long	0x000f0000		@ Required ID value
	.long	0x000f0000		@ Mask for ID
	.long   PMD_TYPE_SECT | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ | \
		PMD_FLAGS
	.long   PMD_TYPE_SECT | \
		PMD_SECT_XN | \
		PMD_SECT_AP_WRITE | \
		PMD_SECT_AP_READ
	b	__pj4bv7_setup
	.long	cpu_arch_name
	.long	cpu_elf_name
	.long	HWCAP_SWP|HWCAP_HALF|HWCAP_THUMB|HWCAP_FAST_MULT|HWCAP_EDSP
	.long	cpu_pj4bv7_name
	.long	v7_processor_functions
	.long	v7wbi_tlb_fns
	.long	v6_user_fns
	.long	v7_cache_fns
	.size	__v7_proc_info, . - __v7_proc_info
