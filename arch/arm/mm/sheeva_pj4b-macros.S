	.macro	sheeva_pj4b_config
#ifdef CONFIG_SHEEVA_ERRATA_ARM_CPU_4948
	mrc 	p15, 1, r0, c15, c1, 0
	orr     r0, r0, #1			@ Disable L0 cache.
	mcr 	p15, 1, r0, c15, c1, 0
#endif
#ifdef CONFIG_SHEEVA_ERRATA_ARM_CPU_4786
	mrc 	p15, 1, r0, c15, c1, 0
	orr     r0, r0, #(1 << 15)		@ Disable Dual issue for coprocessor instructions
	mcr 	p15, 1, r0, c15, c1, 0
#endif
	mrc 	p15, 1, r0, c15, c1, 1
	orr     r0, r0, #(1 << 16)		@ Disable data transfer for clean line.
#if defined(CONFIG_ARCH_ARMADA370) && defined(CONFIG_AURORA_IO_CACHE_COHERENCY)
	orr     r0, r0, #(1 << 7)		@ v7 IO coherency support (Single core).
#endif
	mcr 	p15, 1, r0, c15, c1, 1

	mrc 	p15, 1, r0, c15, c1, 2 
	bic     r0, r0, #(1 << 23)		@ Enable fast LDR.
	orr     r0, r0, #(1 << 25)		@ Disable interleave between normal write and snoop data intervene.
	orr     r0, r0, #(1 << 27)		@ Disable Critical Word First feature.
	orr     r0, r0, #(1 << 29)		@ Disable SO/device/EX non cacheable request to get out outstanding.
	orr     r0, r0, #(1 << 30) 		@ MBU change the way when it get second linefill
	mcr 	p15, 1, r0, c15, c1, 2

#ifdef CONFIG_SHEEVA_ERRATA_ARM_CPU_5315
	mrc     p15, 1, r0, c15, c2, 0
	orr     r0, r0, #(1 << 7)              @ Disable spec read from the MBu/LSU
        mcr     p15, 1, r0, c15, c2, 0
#else
	/* Speculative Prefetch is not enabled here - managed at runtime */
#endif
	.endm

