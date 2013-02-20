/*
 * include/asm-arm/arch-mv78xx0/entry-macro.S
 *
 * Low-level IRQ helper macros for Marvell MV78xx0 platforms
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <mach/armadaxp.h>

	.macro  disable_fiq
	.endm

	.macro  arch_ret_to_user, tmp1, tmp2
	.endm

	.macro  get_irqnr_preamble, base, tmp
	ldr	\base, =AXP_IRQ_VIRT_BASE
	.endm

#if defined(CONFIG_CPU_BIG_ENDIAN)
        	.macro	HTOLL sr, tt		@ sr   = A  ,B  ,C  ,D    
        	eor \tt, \sr, \sr, ror #16 ;    @ temp = A^C,B^	  
        	bic \tt, \tt, #0xff0000 ;       @ temp = A^C,0  ,C^A,D^B
        	mov \sr, \sr, ror #8 ;          @ sr   = D  ,A  ,B  ,C
        	eor \sr, \sr, \tt, lsr #8       @ sr   = D  ,C  ,B  ,A
        	.endm
#else		
        	.macro	HTOLL sr, tt	
        	.endm
#endif
	/* TBD - need to be optimized 29*(sel-1) + cls 						*/
	/* r1 - we shouldnt use it here 							*/
	/* in case of SMP we only handle bit 0,1 (doorbell) and 5,6 (timer) from cause Vec 0 	*/
	/* return value is: irqnr and the flag state!!!!!!!!!!!!				*/
	.macro  get_irqnr_and_base, irqnr, irqstat, base, tmp
	@ check low interrupts
	ldr	\irqstat, [\base, #AXP_IRQ_SEL_CAUSE_OFF]			
	HTOLL	\irqstat, \tmp
	ands 	\tmp, \irqstat, #0x80000000		@ did we get irq
	beq	1001f	
	mov 	\tmp, \irqstat

	mov 	\tmp, \tmp, lsr #29 			@ determine the irq group,  
	bics	\tmp, \tmp, #4 	 			@ clear irq_stat bit

#ifdef CONFIG_SMP
	@ if vec sel is 0 and bits 0-7 are set then it is IPI/PMU/Local timer handled seperatly.	
	bne	1000f
	ands	\tmp, \irqstat, #0xff
	beq 	1000f

	@restore tmp - should be optimized!!!!
        mov     \tmp, \irqstat

        mov     \tmp, \tmp, lsr #29                     @ determine the irq group,
        bics    \tmp, \tmp, #4                          @ clear irq_stat bit
	beq	1001f

#endif
1000:
	bic	\irqstat, \irqstat, #(0xE0000000) 	@ leave irq bits, clear the rest				
	mov    	\irqnr, #0x1F
	orrs    \irqnr, \irqnr, \tmp, lsl #5    	@ irqnr = 0x1F, 0x3F, 0x5F, 0x7f, make sure Z is off	
	clz	\irqstat, \irqstat	              	@ find first active interrupt source
	sub	\irqnr, \irqnr, \irqstat
	mov 	r1, #3	
	mul	\tmp, r1, \tmp
	sub 	\irqnr, \irqnr, \tmp
1001:	
	.endm


        /* We assume that irqstat (the raw value of the IRQ acknowledge
         * register) is preserved from the macro above.
         */

	.macro test_for_ipi, irqnr, irqstat, base, tmp
	ands 	\tmp, \irqstat, #0x80000000		@ did we get irq
	beq	1002f
	ands 	\tmp, \irqstat, #0x00000003		@ was it doorbell
	beq	1002f
	mov 	\tmp, #0
	str   	\tmp, [\base, #AXP_IN_DOORBELL_CAUSE]	@ clean irq
1002:
	.endm


	/* As above, this assumes that irqstat and base are preserved.. */

	.macro test_for_ltirq, irqnr, irqstat, base, tmp
	ands 	\tmp, \irqstat, #0x80000000		@ did we get irq
	beq	1003f
	ands 	\tmp, \irqstat, #0x00000060		@ was it timer
1003:
	.endm


	.macro test_for_pmuirq, irqnr, irqstat, base, tmp
	ands 	\tmp, \irqstat, #0x80000000		@ did we get irq
	beq	1005f
	ands 	\tmp, \irqstat, #0x00000008		@ was it mp
1005:
	.endm
	
