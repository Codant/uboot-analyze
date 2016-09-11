/*******************************************************
*项目名称：分析uboot的启动过程
*作    者：obsession  1162732624@qq.com
*分析时间：20160830
*说    明：欢迎大家一起交流，如有错误请望给小弟指出，谢谢
*******************************************************/
/*
*第一阶段的分析,arch/arm/cpu/armv7/start.S
*/
/*1.异常向量表的设置，调到start_code执行程序*/
/***********************************1.周二磊：第一阶段*************************************************/
.globl _start
_start:	b	start_code
//{
	ldr	pc, _undefined_instruction
	ldr	pc, _software_interrupt
	ldr	pc, _prefetch_abort
	ldr	pc, _data_abort
	ldr	pc, _not_used
	ldr	pc, _irq
	ldr	pc, _fiq

_undefined_instruction:	.word undefined_instruction
_software_interrupt:	.word software_interrupt
_prefetch_abort:		.word prefetch_abort
_data_abort:			.word data_abort
_not_used:				.word not_used
_irq:					.word irq
_fiq:					.word fiq

	.balignl 16,0xdeadbeef  //填充16字节对齐
//}
/*2.设置CPU为SVC模式（管理模式）*/
_start:	b	start_code
//{
start_code:
	mrs	r0, cpsr         //读取状态寄存器的状态到r0中
	bic	r0, r0, #0x1f    //把r0中的弟五位清零
	orr	r0, r0, #0xd3	 //r0 = 0x11010011设置cpu为SVC模式,禁止FIQ和IRQ的中断
	msr	cpsr, r0         //将r0中的数值写回到状态寄存器
/*CPSR : IRQ FIQ T(ARM/Thumb) M4 M3 M2 M1 M0*/	
//}
/*3.协处理器 p15 的 c12 寄存器来重新定位*/	
#if !(defined(CONFIG_OMAP44XX) && defined(CONFIG_SPL_BUILD))
	/* Set V=0 in CP15 SCTRL register - for VBAR to point to vector */
	mrc	p15, 0, r0, c1, c0, 0	@ Read CP15 SCTRL Register
	bic	r0, #CR_V		@ V = 0
	mcr	p15, 0, r0, c1, c0, 0	@ Write CP15 SCTRL Register
	/* Set vector address in CP15 VBAR register */
	ldr	r0, =_start  
	mcr	p15, 0, r0, c12, c0, 0	@Set VBAR
#endif
/*继续执行，*/
#ifndef CONFIG_SKIP_LOWLEVEL_INIT
/*跳转到cpu_init_cp15*/
	bl	cpu_init_cp15
/*4.Bl  cpu_init_cp15（使分支预测无效，数据）
	分支预测：在流水线里，
	会将后面的代码优先加载到处理器中，由于是循环，会使后面加载的代码无效，
	故出现了分支预测技术。(统计跳的次数来选择装载循环的代码还是下面的代码)*/
//{
ENTRY(cpu_init_cp15)
	/*
	 * Invalidate L1 I/D
	 */
	mov	r0, #0			        @ set up for MCR
	mcr	p15, 0, r0, c8, c7, 0	@ invalidate TLBs
	mcr	p15, 0, r0, c7, c5, 0	@ invalidate icache
	mcr	p15, 0, r0, c7, c5, 6	@ invalidate BP array
	mcr     p15, 0, r0, c7, c10, 4	@ DSB//多核CPU对数据处理指令
	mcr     p15, 0, r0, c7, c5, 4	@ ISB//流水线清空指令

	/*
	 * disable MMU stuff and caches
	 */
	mrc	p15, 0, r0, c1, c0, 0
	bic	r0, r0, #0x00002000	@ clear bits 13 (--V-)
	bic	r0, r0, #0x00000007	@ clear bits 2:0 (-CAM)
	orr	r0, r0, #0x00000002	@ set bit 1 (--A-) Align
	orr	r0, r0, #0x00000800	@ set bit 11 (Z---) BTB
	#ifdef CONFIG_SYS_ICACHE_OFF
		bic	r0, r0, #0x00001000	@ clear bit 12 (I) I-cache
	#else
		orr	r0, r0, #0x00001000	@ set bit 12 (I) I-cache
	#endif
		mcr	p15, 0, r0, c1, c0, 0
		mov	pc, lr			@ back to my caller
	ENDPROC(cpu_init_cp15)   //调到cpu_init_cp15这个地方执行程序
//}	
	bl	cpu_init_crit
//{
#ifndef CONFIG_SKIP_LOWLEVEL_INIT	
ENTRY(cpu_init_crit)
	b	lowlevel_init		@ go setup pll,mux,memory//board/snmung/orogen/lowlevel_init.S
/*
*第一阶段的分析,board/snmung/orogen/lowlevel_init.S
*/

/*开发板相关:关看门狗，内存初始化，时钟初始化，串口初始化(board相关，初始化最基本设备）*/
lowlevel_init:
	push	{lr}
//{
	

	/* r5 has always zero */
	mov	r5, #0
	ldr	r7, =EXYNOS4_GPIO_PART1_BASE
	ldr	r6, =EXYNOS4_GPIO_PART2_BASE

	/* check reset status */
	ldr	r0, =(EXYNOS4_POWER_BASE + INFORM1_OFFSET)
	ldr	r1, [r0]

	/* AFTR wakeup reset */
	ldr	r2, =S5P_CHECK_DIDLE
	cmp	r1, r2
	beq	exit_wakeup

	/* LPA wakeup reset */
	ldr	r2, =S5P_CHECK_LPA
	cmp	r1, r2
	beq	exit_wakeup

	/* Sleep wakeup reset */
	ldr	r2, =S5P_CHECK_SLEEP
	cmp	r1, r2
	beq	wakeup_reset
//}
/*************************************************2.朱秀非：系统时钟初始化********************************************/	
/*5.比较当前pc指针域TEXT_BASE的高8位是否一样来判断，当前代码是否在内存中*/	 
//{
	ldr	r0, =0x0ffffff		/* r0 <- Mask Bits*/
	bic	r1, pc, r0			/* pc <- current addr of code */
												/* r1 <- unmasked bits of pc */
	ldr	r2, _TEXT_BASE		/* r2 <- original base addr in ram */
	bic	r2, r2, r0			/* r2 <- unmasked bits of r2*/
	cmp	r1, r2				/* compare r1, r2 */
	beq	1f					/* r0 == r1 then skip sdram init */

//}

/*6.init system clock （系统时钟初始化）*/
	bl system_clock_init
//{
system_clock_init:
	push	{lr}
	ldr	r0, =EXYNOS4_CLOCK_BASE

	/* APLL(1), MPLL(1), CORE(0), HPM(0) */
	ldr	r1, =CLK_SRC_CPU_VAL
	ldr	r2, =CLK_SRC_CPU_OFFSET
	str	r1, [r0, r2]

	/* wait ?us */
	mov	r1, #0x10000
2:	subs	r1, r1, #1
	bne	2b

	ldr	r1, =CLK_SRC_TOP0_VAL
	ldr	r2, =CLK_SRC_TOP0_OFFSET
	str	r1, [r0, r2]

	ldr	r1, =CLK_SRC_TOP1_VAL
	ldr	r2, =CLK_SRC_TOP1_OFFSET
	str	r1, [r0, r2]

	/* DMC */
	ldr	r1, =CLK_SRC_DMC_VAL
	ldr	r2, =CLK_SRC_DMC_OFFSET
	str	r1, [r0, r2]

	/*CLK_SRC_LEFTBUS */
	ldr	r1, =CLK_SRC_LEFTBUS_VAL
	ldr	r2, =CLK_SRC_LEFTBUS_OFFSET
	str	r1, [r0, r2]

	/*CLK_SRC_RIGHTBUS */
	ldr	r1, =CLK_SRC_RIGHTBUS_VAL
	ldr	r2, =CLK_SRC_RIGHTBUS_OFFSET
	str	r1, [r0, r2]

	/* SATA: SCLKMPLL(0), MMC[0:4]: SCLKMPLL(6) */
	ldr	r1, =CLK_SRC_FSYS_VAL
	ldr	r2, =CLK_SRC_FSYS_OFFSET
	str	r1, [r0, r2]

	/* UART[0:4] */
	ldr	r1, =CLK_SRC_PERIL0_VAL
	ldr	r2, =CLK_SRC_PERIL0_OFFSET
	str	r1, [r0, r2]

	/* CAM , FIMC 0-3 */
	ldr	r1, =CLK_SRC_CAM_VAL
	ldr	r2, =CLK_SRC_CAM_OFFSET
	str	r1, [r0, r2]

	/* MFC */
	ldr	r1, =CLK_SRC_MFC_VAL
	ldr	r2, =CLK_SRC_MFC_OFFSET
	str	r1, [r0, r2]

	/* G3D */
	ldr	r1, =CLK_SRC_G3D_VAL
	ldr	r2, =CLK_SRC_G3D_OFFSET
	str	r1, [r0, r2]

	/* LCD0 */
	ldr	r1, =CLK_SRC_LCD0_VAL
	ldr	r2, =CLK_SRC_LCD0_OFFSET
	str	r1, [r0, r2]

	/* wait ?us */
	mov	r1, #0x10000
3:	subs	r1, r1, #1
	bne	3b

	/* CLK_DIV_CPU0 */
	ldr	r1, =CLK_DIV_CPU0_VAL
	ldr	r2, =CLK_DIV_CPU0_OFFSET
	str	r1, [r0, r2]

	/* CLK_DIV_CPU1 */
	ldr	r1, =CLK_DIV_CPU1_VAL
	ldr	r2, =CLK_DIV_CPU1_OFFSET
	str	r1, [r0, r2]

	/* CLK_DIV_DMC0 */
	ldr	r1, =CLK_DIV_DMC0_VAL
	ldr	r2, =CLK_DIV_DMC0_OFFSET
	str	r1, [r0, r2]

	/*CLK_DIV_DMC1 */
	ldr	r1, =CLK_DIV_DMC1_VAL
	ldr	r2, =CLK_DIV_DMC1_OFFSET
	str	r1, [r0, r2]

	/* CLK_DIV_LEFTBUS */
	ldr	r1, =CLK_DIV_LEFTBUS_VAL
	ldr	r2, =CLK_DIV_LEFTBUS_OFFSET
	str	r1, [r0, r2]

	/* CLK_DIV_RIGHTBUS */
	ldr	r1, =CLK_DIV_RIGHTBUS_VAL
	ldr	r2, =CLK_DIV_RIGHTBUS_OFFSET
	str	r1, [r0, r2]

	/* CLK_DIV_TOP */
	ldr	r1, =CLK_DIV_TOP_VAL
	ldr	r2, =CLK_DIV_TOP_OFFSET
	str	r1, [r0, r2]

	/* MMC[0:1] */
	ldr	r1, =CLK_DIV_FSYS1_VAL		/* 800(MPLL) / (15 + 1) */
	ldr	r2, =CLK_DIV_FSYS1_OFFSET
	str	r1, [r0, r2]

	/* MMC[2:3] */
	ldr	r1, =CLK_DIV_FSYS2_VAL		/* 800(MPLL) / (15 + 1) */
	ldr	r2, =CLK_DIV_FSYS2_OFFSET
	str	r1, [r0, r2]

	/* MMC4 */
	ldr	r1, =CLK_DIV_FSYS3_VAL		/* 800(MPLL) / (15 + 1) */
	ldr	r2, =CLK_DIV_FSYS3_OFFSET
	str	r1, [r0, r2]

	/* CLK_DIV_PERIL0: UART Clock Divisors */
	ldr	r1, =CLK_DIV_PERIL0_VAL
	ldr	r2, =CLK_DIV_PERIL0_OFFSET
	str	r1, [r0, r2]

	/* CAM, FIMC 0-3: CAM Clock Divisors */
	ldr	r1, =CLK_DIV_CAM_VAL
	ldr	r2, =CLK_DIV_CAM_OFFSET
	str	r1, [r0, r2]

	/* CLK_DIV_MFC: MFC Clock Divisors */
	ldr	r1, =CLK_DIV_MFC_VAL
	ldr	r2, =CLK_DIV_MFC_OFFSET
	str	r1, [r0, r2]

	/* CLK_DIV_G3D: G3D Clock Divisors */
	ldr	r1, =CLK_DIV_G3D_VAL
	ldr	r2, =CLK_DIV_G3D_OFFSET
	str	r1, [r0, r2]

	/* CLK_DIV_LCD0: LCD0 Clock Divisors */
	ldr	r1, =CLK_DIV_LCD0_VAL
	ldr	r2, =CLK_DIV_LCD0_OFFSET
	str	r1, [r0, r2]

	/* Set PLL locktime */
	ldr	r1, =PLL_LOCKTIME
	ldr	r2, =APLL_LOCK_OFFSET
	str	r1, [r0, r2]

	ldr	r1, =PLL_LOCKTIME
	ldr	r2, =MPLL_LOCK_OFFSET
	str	r1, [r0, r2]

	ldr	r1, =PLL_LOCKTIME
	ldr	r2, =EPLL_LOCK_OFFSET
	str	r1, [r0, r2]

	ldr	r1, =PLL_LOCKTIME
	ldr	r2, =VPLL_LOCK_OFFSET
	str	r1, [r0, r2]

	/* APLL_CON1 */
	ldr	r1, =APLL_CON1_VAL
	ldr	r2, =APLL_CON1_OFFSET
	str	r1, [r0, r2]

	/* APLL_CON0 */
	ldr	r1, =APLL_CON0_VAL
	ldr	r2, =APLL_CON0_OFFSET
	str	r1, [r0, r2]

	/* MPLL_CON1 */
	ldr	r1, =MPLL_CON1_VAL
	ldr	r2, =MPLL_CON1_OFFSET
	str	r1, [r0, r2]

	/* MPLL_CON0 */
	ldr	r1, =MPLL_CON0_VAL
	ldr	r2, =MPLL_CON0_OFFSET
	str	r1, [r0, r2]

	/* EPLL */
	ldr	r1, =EPLL_CON1_VAL
	ldr	r2, =EPLL_CON1_OFFSET
	str	r1, [r0, r2]

	/* EPLL_CON0 */
	ldr	r1, =EPLL_CON0_VAL
	ldr	r2, =EPLL_CON0_OFFSET
	str	r1, [r0, r2]

	/* VPLL_CON1 */
	ldr	r1, =VPLL_CON1_VAL
	ldr	r2, =VPLL_CON1_OFFSET
	str	r1, [r0, r2]

	/* VPLL_CON0 */
	ldr	r1, =VPLL_CON0_VAL
	ldr	r2, =VPLL_CON0_OFFSET
	str	r1, [r0, r2]

	/* wait ?us */
	mov	r1, #0x30000
4:	subs	r1, r1, #1
	bne	4b

	pop	{pc}
 //}
/*************************************3.梁怀文：内存初始化********************************/
/*7.Memory initialize（内存初始化）内存初始代码在board/origen/mem_setup.S */
	bl mem_ctrl_asm_init
//{
	mem_ctrl_asm_init:
	/*
	 * Async bridge configuration at CPU_core:
	 * 1: half_sync
	 * 0: full_sync
	 */
	ldr r0, =ASYNC_CONFIG
	mov r1, #1
	str r1, [r0]

#ifdef SET_MIU
	ldr	r0, =EXYNOS4_MIU_BASE
	/* Interleave: 2Bit, Interleave_bit1: 0x21, Interleave_bit2: 0x7 */
	ldr	r1, =0x20001507
	str	r1, [r0, #APB_SFR_INTERLEAVE_CONF_OFFSET]

	/* Update MIU Configuration */
	ldr	r1, =0x00000001
	str	r1, [r0, #APB_SFR_ARBRITATION_CONF_OFFSET]
#endif
	/* DREX0 */
	ldr	r0, =EXYNOS4_DMC0_BASE

	/*
	 * DLL Parameter Setting:
	 * Termination: Enable R/W
	 * Phase Delay for DQS Cleaning: 180' Shift
	 */
	ldr	r1, =0xe0000086
	str	r1, [r0, #DMC_PHYCONTROL1]

	/*
	 * ZQ Calibration
	 * Termination: Disable
	 * Auto Calibration Start: Enable
	 */
	ldr	r1, =0xE3855703
	str	r1, [r0, #DMC_PHYZQCONTROL]
	/* Wait ?us*/
	mov	r2, #0x100000
1:	subs	r2, r2, #1
	bne	1b

	/*
	 * Update DLL Information:
	 * Force DLL Resyncronization
	 */
	ldr	r1, =0xe000008e
	str	r1, [r0, #DMC_PHYCONTROL1]

	/* Reset Force DLL Resyncronization */
	ldr	r1, =0xe0000086
	str	r1, [r0, #DMC_PHYCONTROL1]

	/* Enable Differential DQS, DLL Off*/
	ldr	r1, =0x71101008
	str	r1, [r0, #DMC_PHYCONTROL0]

	/* Activate PHY DLL: DLL On */
	ldr	r1, =0x7110100A
	str	r1, [r0, #DMC_PHYCONTROL0]

	/* Set DLL Parameters */
	ldr	r1, =0xe0000086
	str	r1, [r0, #DMC_PHYCONTROL1]

	/* DLL Start */
	ldr	r1, =0x7110100B
	str	r1, [r0, #DMC_PHYCONTROL0]

	ldr	r1, =0x00000000
	str	r1, [r0, #DMC_PHYCONTROL2]

	/* Set Clock Ratio of Bus clock to Memory Clock */
	ldr	r1, =0x0FFF301a
	str	r1, [r0, #DMC_CONCONTROL]

	/*
	 * Memor Burst length: 8
	 * Number of chips: 2
	 * Memory Bus width: 32 bit
	 * Memory Type: DDR3
	 * Additional Latancy for PLL: 1 Cycle
	 */
	ldr	r1, =0x00312640
	str	r1, [r0, #DMC_MEMCONTROL]

	/*
	 * Memory Configuration Chip 0
	 * Address Mapping: Interleaved
	 * Number of Column address Bits: 10 bits
	 * Number of Rows Address Bits: 14
	 * Number of Banks: 8
	 */
	ldr	r1, =0x20e01323
	str	r1, [r0, #DMC_MEMCONFIG0]

	/*
	 * Memory Configuration Chip 1
	 * Address Mapping: Interleaved
	 * Number of Column address Bits: 10 bits
	 * Number of Rows Address Bits: 14
	 * Number of Banks: 8
	 */
	ldr	r1, =0x40e01323
	str	r1, [r0, #DMC_MEMCONFIG1]

	/* Config Precharge Policy */
	ldr	r1, =0xff000000
	str	r1, [r0, #DMC_PRECHCONFIG]

	/*
	 * TimingAref, TimingRow, TimingData, TimingPower Setting:
	 * Values as per Memory AC Parameters
	 */
	ldr	r1, =0x000000BB
	str	r1, [r0, #DMC_TIMINGAREF]
	ldr	r1, =0x4046654f
	str	r1, [r0, #DMC_TIMINGROW]
	ldr	r1, =0x46400506
	str	r1, [r0, #DMC_TIMINGDATA]
	ldr	r1, =0x52000A3C
	str	r1, [r0, #DMC_TIMINGPOWER]

	/* Chip0: NOP Command: Assert and Hold CKE to high level */
	ldr	r1, =0x07000000
	str	r1, [r0, #DMC_DIRECTCMD]

	/* Wait ?us*/
	mov	r2, #0x100000
2:	subs	r2, r2, #1
	bne	2b

	/* Chip0: EMRS2, EMRS3, EMRS, MRS Commands Using Direct Command */
	ldr	r1, =0x00020000
	str	r1, [r0, #DMC_DIRECTCMD]
	ldr	r1, =0x00030000
	str	r1, [r0, #DMC_DIRECTCMD]
	ldr	r1, =0x00010002
	str	r1, [r0, #DMC_DIRECTCMD]
	ldr	r1, =0x00000328
	str	r1, [r0, #DMC_DIRECTCMD]

	/* Wait ?us*/
	mov	r2, #0x100000
3:	subs	r2, r2, #1
	bne	3b

	/* Chip 0: ZQINIT */
	ldr	r1, =0x0a000000
	str	r1, [r0, #DMC_DIRECTCMD]

	/* Wait ?us*/
	mov	r2, #0x100000
4:	subs	r2, r2, #1
	bne	4b

	/* Chip1: NOP Command: Assert and Hold CKE to high level */
	ldr	r1, =0x07100000
	str	r1, [r0, #DMC_DIRECTCMD]

	/* Wait ?us*/
	mov	r2, #0x100000
5:	subs	r2, r2, #1
	bne	5b

	/* Chip1: EMRS2, EMRS3, EMRS, MRS Commands Using Direct Command */
	ldr	r1, =0x00120000
	str	r1, [r0, #DMC_DIRECTCMD]
	ldr	r1, =0x00130000
	str	r1, [r0, #DMC_DIRECTCMD]
	ldr	r1, =0x00110002
	str	r1, [r0, #DMC_DIRECTCMD]
	ldr	r1, =0x00100328
	str	r1, [r0, #DMC_DIRECTCMD]

	/* Wait ?us*/
	mov	r2, #0x100000
6:	subs	r2, r2, #1
	bne	6b

	/* Chip1: ZQINIT */
	ldr	r1, =0x0a100000
	str	r1, [r0, #DMC_DIRECTCMD]

	/* Wait ?us*/
	mov	r2, #0x100000
7:	subs	r2, r2, #1
	bne	7b

	ldr	r1, =0xe000008e
	str	r1, [r0, #DMC_PHYCONTROL1]
	ldr	r1, =0xe0000086
	str	r1, [r0, #DMC_PHYCONTROL1]

	/* Wait ?us*/
	mov	r2, #0x100000
8:	subs	r2, r2, #1
	bne	8b

	/* turn on DREX0, DREX1 */
	ldr	r0, =EXYNOS4_DMC0_BASE
	ldr	r1, =0x0FFF303a
	str	r1, [r0, #DMC_CONCONTROL]

	ldr	r0, =EXYNOS4_DMC1_BASE
	ldr	r1, =0x0FFF303a
	str	r1, [r0, #DMC_CONCONTROL]

	mov	pc, lr
	
//}
/******************************************4.刘建峰：串口初始化*********************************/
/*8.串口初始化：for UART */
	bl uart_asm_init
	/*bl tzpc_init*/
//{
	globl uart_asm_init
uart_asm_init:

	/* setup UART0-UART3 GPIOs (part1) */
	mov	r0, r7
	ldr	r1, =EXYNOS4_GPIO_A0_CON_VAL
	str	r1, [r0, #EXYNOS4_GPIO_A0_CON_OFFSET]
	ldr	r1, =EXYNOS4_GPIO_A1_CON_VAL
	str	r1, [r0, #EXYNOS4_GPIO_A1_CON_OFFSET]

	ldr r0, =EXYNOS4_UART_BASE
	add r0, r0, #EXYNOS4_DEFAULT_UART_OFFSET

	ldr	r1, =ULCON_VAL
	str	r1, [r0, #ULCON_OFFSET]
	ldr	r1, =UCON_VAL
	str	r1, [r0, #UCON_OFFSET]
	ldr	r1, =UFCON_VAL
	str	r1, [r0, #UFCON_OFFSET]
	ldr	r1, =UBRDIV_VAL
	str	r1, [r0, #UBRDIV_OFFSET]
	ldr	r1, =UFRACVAL_VAL
	str	r1, [r0, #UFRACVAL_OFFSET]
	mov	pc, lr
	nop
	nop
	nop
//}
	bl tzpc_ini
//{
	tzpc_init:
	ldr	r0, =TZPC0_BASE
	mov	r1, #R0SIZE
	str	r1, [r0]
	mov	r1, #DECPROTXSET
	str	r1, [r0, #TZPC_DECPROT0SET_OFFSET]
	str	r1, [r0, #TZPC_DECPROT1SET_OFFSET]
	str	r1, [r0, #TZPC_DECPROT2SET_OFFSET]
	str	r1, [r0, #TZPC_DECPROT3SET_OFFSET]

	ldr	r0, =TZPC1_BASE
	str	r1, [r0, #TZPC_DECPROT0SET_OFFSET]
	str	r1, [r0, #TZPC_DECPROT1SET_OFFSET]
	str	r1, [r0, #TZPC_DECPROT2SET_OFFSET]
	str	r1, [r0, #TZPC_DECPROT3SET_OFFSET]

	ldr	r0, =TZPC2_BASE
	str	r1, [r0, #TZPC_DECPROT0SET_OFFSET]
	str	r1, [r0, #TZPC_DECPROT1SET_OFFSET]
	str	r1, [r0, #TZPC_DECPROT2SET_OFFSET]
	str	r1, [r0, #TZPC_DECPROT3SET_OFFSET]

	ldr	r0, =TZPC3_BASE
	str	r1, [r0, #TZPC_DECPROT0SET_OFFSET]
	str	r1, [r0, #TZPC_DECPROT1SET_OFFSET]
	str	r1, [r0, #TZPC_DECPROT2SET_OFFSET]
	str	r1, [r0, #TZPC_DECPROT3SET_OFFSET]

	ldr	r0, =TZPC4_BASE
	str	r1, [r0, #TZPC_DECPROT0SET_OFFSET]
	str	r1, [r0, #TZPC_DECPROT1SET_OFFSET]
	str	r1, [r0, #TZPC_DECPROT2SET_OFFSET]
	str	r1, [r0, #TZPC_DECPROT3SET_OFFSET]

	ldr	r0, =TZPC5_BASE
	str	r1, [r0, #TZPC_DECPROT0SET_OFFSET]
	str	r1, [r0, #TZPC_DECPROT1SET_OFFSET]
	str	r1, [r0, #TZPC_DECPROT2SET_OFFSET]
	str	r1, [r0, #TZPC_DECPROT3SET_OFFSET]

	mov	pc, lr	  //返回到压栈的地方  就是初始化串口的跳转处
//}
	pop	{pc}  //返回到压栈的地方 就是回到了lowlevel_init.S跳转处
#endif
ENDPROC(cpu_init_crit)
//}
/*9.返回start.S，跳转到arch/arm/lib/crt0.S*/
#endif	
//}		
   bl _main     //跳转到arch/arm/lib/crt0.S
/*arch/arm/lib/crt0.S*/	
//{
_main:

/*
10. 初始化C运行环境，并调用board_init_f 函数
 */

#if defined(CONFIG_NAND_SPL)
	/* deprecated, use instead CONFIG_SPL_BUILD */
	ldr	sp, =(CONFIG_SYS_INIT_SP_ADDR)
#elif defined(CONFIG_SPL_BUILD) && defined(CONFIG_SPL_STACK)
	ldr	sp, =(CONFIG_SPL_STACK)
#else
	ldr	sp, =(CONFIG_SYS_INIT_SP_ADDR)
#endif
/*
11.保存128B 放GD结构体，存放全局信息，GD的地址存放在r8中
*/
//{
	bic	sp, sp, #7	/* 8-byte alignment for ABI compliance */
	sub	sp, #GD_SIZE	/* allocate one GD above SP */
	bic	sp, sp, #7	/* 8-byte alignment for ABI compliance */
	mov	r8, sp		/* GD is above SP */
	mov	r0, #0
//}
/*
12.跳转到board_init_f
*/
	bl	board_init_f    //调到void board_init_f(ulong bootflag)函数
//{
/*arch/arm/lib/board.c*/
/**************************************5.高逊达和6.王恒board_init_f*************************************************/
void board_init_f(ulong bootflag)
{
/********************************************************************** 
指向板级信息结构
 typedef struct bd_info {  
     int                   bi_baudrate;     // serial console baudrate 
     unsigned long         bi_ip_addr;      // IP Address  
     struct environment_s  *bi_env;         // 板子的环境变量  
     ulong                 bi_arch_number;  // 板子的 id  
     ulong                 bi_boot_params;  // 板子的启动参数 
     struct                                 // RAM 配置   
     {  
         ulong start;  //起始地址
		 		 ulong size;   //终止地址
     } bi_dram[CONFIG_NR_DRAM_BANKS];    //内存的块的数量
 } bd_t;  
********************************************************************/  
	bd_t *bd;  
	init_fnc_t **init_fnc_ptr;//init_fnc_t   是个自定义的函数指针类型，初始化板
/*****************************************************************************	
	全局数据结构 
   typedef struct  global_data {  
       bd_t            *bd;            // 指向板级信息结构
       unsigned long   flags;          // 标记位 
       unsigned long   baudrate;       // 串口波特率 
       unsigned long   have_console;   // serial_init() was called 
       unsigned long   env_addr;       // 环境参数地址 
       unsigned long   env_valid;      // 环境参数 CRC 校验有效标志 
       unsigned long   fb_base;        // fb 起始地址 
   #ifdef CONFIG_VFD  
       unsigned char   vfd_type;       // 显示器类型(VFD代指真空荧光屏) 
   #endif  
   #if 0  
       unsigned long   cpu_clk;        // cpu 频率
       unsigned long   bus_clk;        // bus 频率  
       phys_size_t     ram_size;       // ram 大小 
       unsigned long   reset_status;   // reset status register at boot  
   #endif  
       void            **jt;           // 跳转函数表 
 } gd_t;  
	
***************************************************************************/
	gd_t *id;//gd_t结构体指针  
	ulong addr, addr_sp;//addr将指向用户正常访问的最高地址+1的位置；addr_sp指向堆起始位置 
#ifdef CONFIG_PRAM
	ulong reg;
#endif
	void *new_fdt = NULL;
	size_t fdt_size = 0;

	memset((void *)gd, 0, sizeof(gd_t));//gd的gd_t大小清0

	gd->mon_len = _bss_end_ofs;  //_bss_end_ofs=0x000add6c u-boot code, data & bss段大小总和 
#ifdef CONFIG_OF_EMBED
	/* Get a pointer to the FDT */
	gd->fdt_blob = _binary_dt_dtb_start;
#elif defined CONFIG_OF_SEPARATE
	/* FDT is at end of image */
	gd->fdt_blob = (void *)(_end_ofs + _TEXT_BASE);
#endif
	/* Allow the early environment to override the fdt address */
	gd->fdt_blob = (void *)getenv_ulong("fdtcontroladdr", 16,
						(uintptr_t)gd->fdt_blob);

	for (init_fnc_ptr = init_sequence; *init_fnc_ptr; ++init_fnc_ptr) {
		if ((*init_fnc_ptr)() != 0) {	//如果板的初始化函数序列出错，死循环 
			hang ();
			//{
				void hang(void)
				{
					puts("### ERROR ### Please RESET the board ###\n");
					for (;;);
				}				
			//}
		}
	}

#ifdef CONFIG_OF_CONTROL
	/* For now, put this check after the console is ready */
	if (fdtdec_prepare_fdt()) {
		panic("** CONFIG_OF_CONTROL defined but no FDT - please see "
			"doc/README.fdt-control");
	}
#endif

	debug("monitor len: %08lX\n", gd->mon_len);
	/*
	 * Ram is setup, size stored in gd !!
	 */
	debug("ramsize: %08lX\n", gd->ram_size);
#if defined(CONFIG_SYS_MEM_TOP_HIDE)
	/*
	 * Subtract specified amount of memory to hide so that it won't
	 * get "touched" at all by U-Boot. By fixing up gd->ram_size
	 * the Linux kernel should now get passed the now "corrected"
	 * memory size and won't touch it either. This should work
	 * for arch/ppc and arch/powerpc. Only Linux board ports in
	 * arch/powerpc with bootwrapper support, that recalculate the
	 * memory size from the SDRAM controller setup will have to
	 * get fixed.
	 */
	gd->ram_size -= CONFIG_SYS_MEM_TOP_HIDE;
#endif

	addr = CONFIG_SYS_SDRAM_BASE + gd->ram_size;

#ifdef CONFIG_LOGBUFFER
#ifndef CONFIG_ALT_LB_ADDR
	/* reserve kernel log buffer */
	addr -= (LOGBUFF_RESERVE);
	debug("Reserving %dk for kernel logbuffer at %08lx\n", LOGBUFF_LEN,
		addr);
#endif
#endif

#ifdef CONFIG_PRAM
	/*
	 * reserve protected RAM
	 */
	reg = getenv_ulong("pram", 10, CONFIG_PRAM);
	addr -= (reg << 10);		/* size is in kB */
	debug("Reserving %ldk for protected RAM at %08lx\n", reg, addr);
#endif /* CONFIG_PRAM */

#if !(defined(CONFIG_SYS_ICACHE_OFF) && defined(CONFIG_SYS_DCACHE_OFF))
	/* reserve TLB table */
	gd->tlb_size = 4096 * 4;
	addr -= gd->tlb_size;

	/* round down to next 64 kB limit */
	addr &= ~(0x10000 - 1);

	gd->tlb_addr = addr;
	debug("TLB table from %08lx to %08lx\n", addr, addr + gd->tlb_size);
#endif

	/* round down to next 4 kB limit */
	addr &= ~(4096 - 1);
	debug("Top of RAM usable for U-Boot at: %08lx\n", addr);

#ifdef CONFIG_LCD
#ifdef CONFIG_FB_ADDR
	gd->fb_base = CONFIG_FB_ADDR;
#else
	/* reserve memory for LCD display (always full pages) */
	addr = lcd_setmem(addr);
	gd->fb_base = addr;
#endif /* CONFIG_FB_ADDR */
#endif /* CONFIG_LCD */

	/*
	 * reserve memory for U-Boot code, data & bss
	 * round down to next 4 kB limit
	 */
	addr -= gd->mon_len;
	addr &= ~(4096 - 1);

	debug("Reserving %ldk for U-Boot at: %08lx\n", gd->mon_len >> 10, addr);

#ifndef CONFIG_SPL_BUILD
	/*
	 * reserve memory for malloc() arena
	 */
	addr_sp = addr - TOTAL_MALLOC_LEN;
	debug("Reserving %dk for malloc() at: %08lx\n",
			TOTAL_MALLOC_LEN >> 10, addr_sp);
	/*
	 * (permanently) allocate a Board Info struct
	 * and a permanent copy of the "global" data
	 */
	addr_sp -= sizeof (bd_t);
	bd = (bd_t *) addr_sp;
	gd->bd = bd;
	debug("Reserving %zu Bytes for Board Info at: %08lx\n",
			sizeof (bd_t), addr_sp);

#ifdef CONFIG_MACH_TYPE
	gd->bd->bi_arch_number = CONFIG_MACH_TYPE; /* board id for Linux */
#endif

	addr_sp -= sizeof (gd_t);
	id = (gd_t *) addr_sp;
	debug("Reserving %zu Bytes for Global Data at: %08lx\n",
			sizeof (gd_t), addr_sp);

#if defined(CONFIG_OF_SEPARATE) && defined(CONFIG_OF_CONTROL)
	/*
	 * If the device tree is sitting immediate above our image then we
	 * must relocate it. If it is embedded in the data section, then it
	 * will be relocated with other data.
	 */
	if (gd->fdt_blob) {
		fdt_size = ALIGN(fdt_totalsize(gd->fdt_blob) + 0x1000, 32);

		addr_sp -= fdt_size;
		new_fdt = (void *)addr_sp;
		debug("Reserving %zu Bytes for FDT at: %08lx\n",
		      fdt_size, addr_sp);
	}
#endif

	/* setup stackpointer for exeptions */
	gd->irq_sp = addr_sp;
#ifdef CONFIG_USE_IRQ
	addr_sp -= (CONFIG_STACKSIZE_IRQ+CONFIG_STACKSIZE_FIQ);
	debug("Reserving %zu Bytes for IRQ stack at: %08lx\n",
		CONFIG_STACKSIZE_IRQ+CONFIG_STACKSIZE_FIQ, addr_sp);
#endif
	/* leave 3 words for abort-stack    */
	addr_sp -= 12;

	/* 8-byte alignment for ABI compliance */
	addr_sp &= ~0x07;
#else
	addr_sp += 128;	/* leave 32 words for abort-stack   */
	gd->irq_sp = addr_sp;
#endif

	debug("New Stack Pointer is: %08lx\n", addr_sp);

#ifdef CONFIG_POST
	post_bootmode_init();
	post_run(NULL, POST_ROM | post_bootmode_get(0));
#endif

	gd->bd->bi_baudrate = gd->baudrate;
	/* Ram ist board specific, so move it to board code ... */
	dram_init_banksize();
	display_dram_config();	/* and display it */

	gd->relocaddr = addr;
	gd->start_addr_sp = addr_sp;
	gd->reloc_off = addr - _TEXT_BASE;
	debug("relocation Offset is: %08lx\n", gd->reloc_off);
	if (new_fdt) {
		memcpy(new_fdt, gd->fdt_blob, fdt_size);
		gd->fdt_blob = new_fdt;
	}
	memcpy(id, (void *)gd, sizeof(gd_t));
} 
/***************************************7.王文博：重定向********************************************/
/*继续执行，继续回到 _main*/
//}
/*将 r8 指向新的 gd 地址*/
#if ! defined(CONFIG_SPL_BUILD)


	ldr	sp, [r8, #GD_START_ADDR_SP]	/* r8 = gd->start_addr_sp */
	bic	sp, sp, #7	/* 8-byte alignment for ABI compliance */
	ldr	r8, [r8, #GD_BD]		/* r8 = gd->bd */
	sub	r8, r8, #GD_SIZE		/* new GD is below bd */

	adr	lr, here
	ldr	r0, [r8, #GD_RELOC_OFF]		/* lr = gd->start_addr_sp */
	add	lr, lr, r0
	ldr	r0, [r8, #GD_START_ADDR_SP]	/* r0 = gd->start_addr_sp */
	mov	r1, r8				/* r1 = gd */
	ldr	r2, [r8, #GD_RELOCADDR]		/* r2 = gd->relocaddr */
/*代码重定位arch/arm/cpu/armv7/start.S*/	
	b	relocate_code
	//{
#ifndef CONFIG_SPL_BUILD
/*
 * void relocate_code (addr_sp, gd, addr_moni)
 *
 * This "function" does not return, instead it continues in RAM
 * after relocating the monitor code.
 *
 */

ENTRY(relocate_code)
	mov	r4, r0	/* save addr_sp */
	mov	r5, r1	/* save addr of gd */
	mov	r6, r2	/* save addr of destination */

	adr	r0, _start
	cmp	r0, r6
	moveq	r9, #0		/* no relocation. relocation offset(r9) = 0 */
	beq	relocate_done		/* skip relocation */
	mov	r1, r6			/* r1 <- scratch for copy_loop */
	ldr	r3, _image_copy_end_ofs
	add	r2, r0, r3		/* r2 <- source end address	    */

copy_loop:
	ldmia	r0!, {r9-r10}		/* copy from source address [r0]    */
	stmia	r1!, {r9-r10}		/* copy to   target address [r1]    */
	cmp	r0, r2			/* until source end address [r2]    */
	blo	copy_loop

	/*
	 * fix .rel.dyn relocations
	 */
	ldr	r0, _TEXT_BASE		/* r0 <- Text base */
	sub	r9, r6, r0		/* r9 <- relocation offset */
	ldr	r10, _dynsym_start_ofs	/* r10 <- sym table ofs */
	add	r10, r10, r0		/* r10 <- sym table in FLASH */
	ldr	r2, _rel_dyn_start_ofs	/* r2 <- rel dyn start ofs */
	add	r2, r2, r0		/* r2 <- rel dyn start in FLASH */
	ldr	r3, _rel_dyn_end_ofs	/* r3 <- rel dyn end ofs */
	add	r3, r3, r0		/* r3 <- rel dyn end in FLASH */
fixloop:
	ldr	r0, [r2]		/* r0 <- location to fix up, IN FLASH! */
	add	r0, r0, r9		/* r0 <- location to fix up in RAM */
	ldr	r1, [r2, #4]
	and	r7, r1, #0xff
	cmp	r7, #23			/* relative fixup? */
	beq	fixrel
	cmp	r7, #2			/* absolute fixup? */
	beq	fixabs
	/* ignore unknown type of fixup */
	b	fixnext
fixabs:
	/* absolute fix: set location to (offset) symbol value */
	mov	r1, r1, LSR #4		/* r1 <- symbol index in .dynsym */
	add	r1, r10, r1		/* r1 <- address of symbol in table */
	ldr	r1, [r1, #4]		/* r1 <- symbol value */
	add	r1, r1, r9		/* r1 <- relocated sym addr */
	b	fixnext
fixrel:
	/* relative fix: increase location by offset */
	ldr	r1, [r0]
	add	r1, r1, r9
fixnext:
	str	r1, [r0]
	add	r2, r2, #8		/* each rel.dyn entry is 8 bytes */
	cmp	r2, r3
	blo	fixloop

relocate_done:
	bx	lr

_rel_dyn_start_ofs:
	.word __rel_dyn_start - _start
_rel_dyn_end_ofs:
	.word __rel_dyn_end - _start
_dynsym_start_ofs:
	.word __dynsym_start - _start
ENDPROC(relocate_code)

#endif		
		
	//}
/*重定位到高地址之后，再次回到 _main(arch/arm/lib/crt0.S此时回到的是刚才的重定位的 here 处*/
here:

/* Set up final (full) environment */
/*关 icache，保证数据从SDRAM中更新，更新异常向量表，因为代码被重定位了；清BBS；*/

	bl	c_runtime_cpu_setup	/* we still call old routine here */

	ldr	r0, =__bss_start	/* this is auto-relocated! */
	ldr	r1, =__bss_end__	/* this is auto-relocated! */

	mov	r2, #0x00000000		/* prepare zero to clear BSS */

clbss_l:cmp	r0, r1			/* while not at end of BSS */
	strlo	r2, [r0]		/* clear 32-bit BSS word */
	addlo	r0, r0, #4		/* move to next */
	blo	clbss_l

	bl coloured_LED_init
	bl red_led_on

#if defined(CONFIG_NAND_SPL)

/* 初始化nandflash ,call _nand_boot() */
	ldr     pc, =nand_boot

#else

/* call board_init_r(gd_t *id, ulong dest_addr) */
	mov	r0, r8			/* gd_t */
	ldr	r1, [r8, #GD_RELOCADDR]	/* dest_addr */
/*调用board_init_r主要是对外设的初始化。 call void board_init_r(gd_t *id, ulong dest_addr) */
	ldr	pc, =board_init_r	/* this is auto-relocated! */
//{
/*****************************************9.李小松board_init_r**********************************************/
void board_init_r(gd_t *id, ulong dest_addr)
{
	ulong malloc_start;
#if !defined(CONFIG_SYS_NO_FLASH)
	ulong flash_size;
#endif

	gd->flags |= GD_FLG_RELOC;	/* tell others: relocation done */
	bootstage_mark_name(BOOTSTAGE_ID_START_UBOOT_R, "board_init_r");

	monitor_flash_len = _end_ofs;

	/* Enable caches */
	enable_caches();

	debug("monitor flash len: %08lX\n", monitor_flash_len);
	board_init();	/* Setup chipselects */
	/*
	 * TODO: printing of the clock inforamtion of the board is now
	 * implemented as part of bdinfo command. Currently only support for
	 * davinci SOC's is added. Remove this check once all the board
	 * implement this.
	 */
#ifdef CONFIG_CLOCKS
	set_cpu_clk_info(); /* Setup clock information */
#endif
	serial_initialize();

	debug("Now running in RAM - U-Boot at: %08lx\n", dest_addr);

#ifdef CONFIG_LOGBUFFER
	logbuff_init_ptrs();
#endif

#ifdef CONFIG_POST
	post_output_backlog();
#endif

	/* The Malloc area is immediately below the monitor copy in DRAM */
	malloc_start = dest_addr - TOTAL_MALLOC_LEN;
	mem_malloc_init (malloc_start, TOTAL_MALLOC_LEN);

#ifdef CONFIG_ARCH_EARLY_INIT_R
	arch_early_init_r();
#endif

	power_init_board();

#if !defined(CONFIG_SYS_NO_FLASH)
	puts("Flash: ");

	flash_size = flash_init();
	if (flash_size > 0) {
# ifdef CONFIG_SYS_FLASH_CHECKSUM
		print_size(flash_size, "");
		/*
		 * Compute and print flash CRC if flashchecksum is set to 'y'
		 *
		 * NOTE: Maybe we should add some WATCHDOG_RESET()? XXX
		 */
		if (getenv_yesno("flashchecksum") == 1) {
			printf("  CRC: %08X", crc32(0,
				(const unsigned char *) CONFIG_SYS_FLASH_BASE,
				flash_size));
		}
		putc('\n');
# else	/* !CONFIG_SYS_FLASH_CHECKSUM */
		print_size(flash_size, "\n");
# endif /* CONFIG_SYS_FLASH_CHECKSUM */
	} else {
		puts(failed);
		hang();
	}
#endif

#if defined(CONFIG_CMD_NAND)
	puts("NAND:  ");
	nand_init();		/* go init the NAND */
#endif

#if defined(CONFIG_CMD_ONENAND)
	onenand_init();
#endif

#ifdef CONFIG_GENERIC_MMC
	puts("MMC:   ");
	mmc_initialize(gd->bd);
#endif

#ifdef CONFIG_HAS_DATAFLASH
	AT91F_DataflashInit();
	dataflash_print_info();
#endif

	/* initialize environment */
	if (should_load_env())
		env_relocate();
	else
		set_default_env(NULL);

#if defined(CONFIG_CMD_PCI) || defined(CONFIG_PCI)
	arm_pci_init();
#endif

	stdio_init();	/* get the devices list going. */

	jumptable_init();

#if defined(CONFIG_API)
	/* Initialize API */
	api_init();
#endif

	console_init_r();	/* fully init console as a device */

#ifdef CONFIG_DISPLAY_BOARDINFO_LATE
# ifdef CONFIG_OF_CONTROL
	/* Put this here so it appears on the LCD, now it is ready */
	display_fdt_model(gd->fdt_blob);
# else
	checkboard();
# endif
#endif

#if defined(CONFIG_ARCH_MISC_INIT)
	/* miscellaneous arch dependent initialisations */
	arch_misc_init();
#endif
#if defined(CONFIG_MISC_INIT_R)
	/* miscellaneous platform dependent initialisations */
	misc_init_r();
#endif

	 /* set up exceptions */
	interrupt_init();
	/* enable exceptions */
	enable_interrupts();

	/* Initialize from environment */
	load_addr = getenv_ulong("loadaddr", 16, load_addr);

#ifdef CONFIG_BOARD_LATE_INIT
	board_late_init();
#endif

#ifdef CONFIG_BITBANGMII
	bb_miiphy_init();
#endif
#if defined(CONFIG_CMD_NET)
	puts("Net:   ");
	eth_initialize(gd->bd);
#if defined(CONFIG_RESET_PHY_R)
	debug("Reset Ethernet PHY\n");
	reset_phy();
#endif
#endif

#ifdef CONFIG_POST
	post_run(NULL, POST_RAM | post_bootmode_get(0));
#endif

#if defined(CONFIG_PRAM) || defined(CONFIG_LOGBUFFER)
	/*
	 * Export available size of memory for Linux,
	 * taking into account the protected RAM at top of memory
	 */
	{
		ulong pram = 0;
		uchar memsz[32];

#ifdef CONFIG_PRAM
		pram = getenv_ulong("pram", 10, CONFIG_PRAM);
#endif
#ifdef CONFIG_LOGBUFFER
#ifndef CONFIG_ALT_LB_ADDR
		/* Also take the logbuffer into account (pram is in kB) */
		pram += (LOGBUFF_LEN + LOGBUFF_OVERHEAD) / 1024;
#endif
#endif
		sprintf((char *)memsz, "%ldk", (gd->ram_size / 1024) - pram);
		setenv("mem", (char *)memsz);
	}
#endif

	/* main_loop() can return to retry autoboot, if so just run it again. */
	for (;;) {
/**************************************************何承玉和梁怀文main_loop()解析*****************************************************/
		main_loop();    //调到main_loop
	}

	/* NOTREACHED - no way out of command loop except booting */
}
	
	
//}
#endif

	/* we should not return here. */

#endif
//}


	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	