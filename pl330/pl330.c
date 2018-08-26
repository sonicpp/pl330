/*
 * pl330.c - PL330 driver
 *
 * Copyright (C) 2015 Jan Havran <xhavra13@stud.fit.vutbr.cz>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/amba/bus.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>

#include "pl330.h"

#define DRIVER_NAME	"pl330"
#define DRIVER_VERSION	"0.3"
#define DRIVER_LICENSE	"GPL v2"

/* Default size of description pool */
#define DEFAULT_POOL_SIZE	2
/*
 * Offsets for DMAC registers
 */
#define REG_DS		0x000	/* DMA Status Register */
#define REG_DPC		0x004	/* DMA Program Counter Register */
#define REG_INTEN	0x020	/* Interrupt Enable Register */
#define REG_ES		0x024	/* Event Status Register */
#define REG_INTSTATUS	0x028	/* Interrupt Status Register */
#define REG_INTCLR	0x02C	/* Interrupt Clear Register */
#define REG_FSM		0x030	/* Fault Status DMA Manager Register */
#define REG_FSC		0x034	/* Fault Status DMA Channel Register */
#define REG_FTM		0x038	/* Fault Type DMA Manager Register */

/*
 * Fault Type DMA Channel Registers
 * FTCn = 0x040 + n * 4
 */
#define _FTC		0x040
#define REG_FTC(n)	(_FTC + ((n) << 2))

/*
 * Channel Status Registers
 * CSn = 0x100 + n * 8
 */
#define _CS		0x100
#define REG_CS(n)	(_CS + ((n) << 3))

/*
 * Channel Program Counter Registers
 * CPCn = 0x104 + n * 8
 */
#define _CPC		0x104
#define REG_CPC(n)	(_CPC + ((n) << 3))

/*
 * Source Address Registers
 * SAn = 0x400 + n * 32
 */
#define _SA		0x400
#define REG_SA(n)	(_SA + ((n) << 5))

/*
 * Destination Address Registers
 * DAn = 0x404 + n * 32
 */
#define _DA		0x404
#define REG_DA(n)	(_DA + ((n) << 5))

/*
 * Channel Control Registers
 * CCn = 0x408 + n * 32
 */
#define _CC		0x408
#define REG_CC(n)	(_CC + ((n) << 5))

/*
 * Loop Counter 0 Registers
 * LC0n = 0x40C + n * 32
 */
#define _LC0		0x40C
#define REG_LC0(n)	(_LC0 + ((n) << 5))

/*
 * Loop Counter 1 Registers
 * LC1n = 0x410 + n * 32
 */
#define _LC1		0x410
#define REG_LC1(n)	(_LC1 + ((n) << 5))

#define REG_DGBSTATUS	0xD00	/* Debug Status Register */
#define REG_DBGCMD	0xD04	/* Debug Command Register */
#define REG_DBGINST0	0xD08	/* Debug Instruction-0 Register */
#define REG_DBGINST1	0xD0C	/* Debug Instruction-1 Register */

#define REG_CR0		0xE00	/* Configuration Register 0 */
#define REG_CR1		0xE04	/* Configuration Register 1 */
#define REG_CR2		0xE08	/* Configuration Register 2 */
#define REG_CR3		0xE0C	/* Configuration Register 3 */
#define REG_CR4		0xE10	/* Configuration Register 4 */
#define REG_CRD		0xE14	/* Configuration Register Dn */

/*
 * Peripheral Identification Registers
 * periph_id_n = 0xFE0 + n * 4
 */
#define _periph_id	0xFE0
#define REG_periph_id(n)	(_periph_id + ((n) << 2))

/*
 * PrimeCell Identification Registers
 * pcell_id_n = 0xFF0 + n * 4
 */
#define _pcell_id	0xFF0
#define REG_pcell_id(n)	(_pcell_id + ((n) << 2))

/*
 * Macros to obtain information from Configuration Register 0
 */
#define CR0_GET_NUM_EVENTS(CR0)		(((CR0) >> 17) & 0x1F)
#define CR0_GET_NUM_PERIPH(CR0)		(((CR0) >> 12) & 0x1F)
#define CR0_GET_NUM_CHNLS(CR0)		(((CR0) >> 4)  & 0x07)
#define CR0_GET_MNGR_NS(CR0)		(((CR0) >> 2)  & 0x01)
#define CR0_GET_BOOT_EN(CR0)		(((CR0) >> 1)  & 0x01)
#define CR0_GET_PERIPH_REQ(CR0)		((CR0) & 0x01)

/*
 * Opcodes of DMAC instructions
 */
#define CMD_DMAADDH	0x54	/* Add Halfword */
#define CMD_DMAEND	0x00	/* End */
#define CMD_DMAFLUSHP	0x35	/* Flush and notify Peripheral */
#define CMD_DMAGO	0xA0	/* Go */
#define CMD_DMALD	0x04	/* Load */
#define CMD_DMALDP	0x25	/* Load Peripheral */
#define CMD_DMALP	0x20	/* Loop */
#define CMD_DMALPEND	0x28	/* Loop End */
#define CMD_DMAKILL	0x01	/* Kill */
#define CMD_DMAMOV	0xBC	/* Move */
#define CMD_DMANOP	0x18	/* No operation */
#define CMD_DMARMB	0x12	/* Read Memory Barrier */
#define CMD_DMASEV	0x34	/* Send Event */
#define CMD_DMAST	0x08	/* Store */
#define CMD_DMASTP	0x29	/* Store and notify Peripheral */
#define CMD_DMASTZ	0x0C	/* Store Zero */
#define CMD_DMAWFE	0x36	/* Wait For Event */
#define CMD_DMAWFP	0x30	/* Wait For Peripheral */
#define CMD_DMAWMB	0x13	/* Write Memory Barrier */

/*
 * Size of DMAC instructions
 */
#define SZ_DMAADDH	3
#define SZ_DMAEND	1
#define SZ_DMAFLUSHP	2
#define SZ_DMAGO	6
#define SZ_DMALD	1
#define SZ_DMALDP	2
#define SZ_DMALP	2
#define SZ_DMALPEND	2
#define SZ_DMAKILL	1
#define SZ_DMAMOV	6
#define SZ_DMANOP	1
#define SZ_DMARMB	1
#define SZ_DMASEV	2
#define SZ_DMAST	1
#define SZ_DMASTP	2
#define SZ_DMASTZ	1
#define SZ_DMAWFE	2
#define SZ_DMAWFP	2
#define SZ_DMAWMB	1

/**
 * struct pl330_config - Stores DMAC configuration information.
 * @num_events:		Number of interrupt that the DMAC provides.
 * @num_periph_req:	Number of peripheral request interfaces that the DMAC
 *			provides.
 * @num_chnls:		Number of DMA channels that the DMAC supports.
 *
 * Stores configuration information obtained from DMAC Configuration
 * Registers (CRn).
 */
struct pl330_config {
	unsigned int num_events : 6;
	unsigned int num_periph_req : 6;
	unsigned int num_chnls : 4;
};

/**
 * struct pl330_chan - Information about channel.
 * @dmae_chan:		DMA-Engine channel.
 * @peri:		Peripheral Request Interface support.
 * @chan_id:		Channel ID.
 * @irq:		IRQ assigned to this channel.
 * @desc_prep_pool:	Pool of prepared descriptors.
 * @desc_sub_pool:	Pool of already submitted descriptors.
 * @lock:		Spinlock for operating with this channel.
 * @dmac:		Pointer to top level struct.
 *
 * This struct extends "dma_chan" for some additional information.
 */
struct pl330_chan {
	struct dma_chan dmae_chan;
	bool peri;
	unsigned chan_id;
	int irq;
	struct list_head desc_prep_pool;
	struct list_head desc_sub_pool;
	spinlock_t lock;
	struct pl330_dmac *dmac;
};

/**
 * enum pl330_desc_status - Current state of pl330 descriptor.
 * @STAT_FREE:		Free to use.
 *			Descriptor is in descriptor pool.
 *			This state can be changed by pl330_prep_*() only.
 * @STAT_PREP:		Allocated to some channel by pl330_prep_* func.
 *			Descriptor is in channel's desc_prep_pool.
 *			This state can be changed by pl330_tx_submit() only.
 * @STAT_SUBMIT:	Descriptor is submitted and waiting in the queue.
 *			Descriptor is in channel's desc_sub_pool.
 *			This state chan be changed by pl330_tasklet() only.
 * @STAT_BUSY:		Channel is executing this descriptor.
 *			Descriptor is in channel's desc_sub_pool.
 *			This state chan be changed by pl330_done_chan() only.
 */
enum pl330_desc_status {
	STAT_FREE,
	STAT_PREP,
	STAT_SUBMIT,
	STAT_BUSY,
};

/**
 * struct pl330_desc - Additional information for DMA-Engine descrioptor.
 * @dmae_desc:		DMA-Engine descriptor.
 * @stat:		Current status of this descriptor.
 * @node:		Node for descriptor pools.
 * @custom_prog:	Descriptor has custom written program.
 * @buf_virt:		Virtual address of pl330 program buffer.
 * @buf_bus:		Bus address of pl330 program buffer.
 * @custom_bus:		Bus address of custom written pl330 program.
 *
 * This struct extends "dma_async_tx_descriptor" for some additional
 * information.
 */
struct pl330_desc {
	struct dma_async_tx_descriptor dmae_desc;
	enum pl330_desc_status stat;
	struct list_head node;
	bool custom_prog;
	u8 *buf_virt;
	dma_addr_t buf_bus;
	dma_addr_t custom_bus;
};

/**
 * struct pl330_dmac - Stores global info about PL330 - top level struct.
 * @dmae_dev:		DMA-Engine Device.
 * @config:		PL330 configuration obtained from
 *			Configuration Registeres.
 * @dev:		Owning device.
 * @base:		DMAC base address.
 * @lock:		IRQ safe(!) spinlock for operating with DMAC
 *			(reading/writing from/to pl330 registers, etc).
 * @desc_pool:		Pool of free descriptors.
 * @pool_lock:		Spinlock for descriptor pool.
 * @chan_cnt:		Total number of channels provided by this DMAC.
 * @irq_mngr:		IRQ number of Manager thread.
 * @chans:		PL330 channels.
 *
 * TODO	- create pool for pl330 programs (4KB for single program is too much).
 */
struct pl330_dmac {
	struct dma_device dmae_dev;
	struct pl330_config config;
	struct device *dev;
	void __iomem *base;
	spinlock_t lock;
	struct list_head desc_pool;
	spinlock_t pool_lock;
	unsigned chan_cnt;
	int irq_mngr;
	struct pl330_chan *chans;
};

/**
 * struct pl330_build_args - Arguments for PL330 program builder.
 * @src_addr		Source (bus) address for DMA transfer.
 * @dst_addr		Destination (bus) address for DMA transfer.
 * @chan_id		Channel ID for which will be program built.
 * @src_inc		Defines fixed/incrementing source address.
 * @dst_inc		Defines fixed/incrementing destination address.
 * @count		Total count of bytes to be transferred.
 */
struct pl330_build_args {
	u32 src_addr;
	u32 dst_addr;
	u8 chan_id;
	bool src_inc;
	bool dst_inc;
	size_t count;
};

static void pl330_finish_transfer(struct pl330_chan *pl_chan);
static void pl330_tasklet(struct pl330_chan *pl_chan);
static struct pl330_desc *create_desc(struct pl330_dmac *dmac, gfp_t gfp_flag);

static inline int emit_dmaaddh(u8 buf[], u8 ra, u16 imm)
{
	/*
	 * DMAADDH encoding:
	 * | 23 ... 16 | 15 ...08 | 07 06 05 04 03 02 01 00 |
	 * | imm[15:8] | imm[7:0] | 00 01 00 01 00 01 ra 00 |
	 */
	buf[0] = CMD_DMAADDH | ((ra << 1) & 0x02);
	*((u16 *) &buf[1]) = imm;

	return SZ_DMAADDH;
}

static inline int emit_dmaend(u8 buf[])
{
	/*
	 * DMAEND encoding:
	 * | 7 6 5 4 3 2 1 0 |
	 * | 0 0 0 0 0 0 0 0 |
	 */
	buf[0] = CMD_DMAEND;

	return SZ_DMAEND;
}

static inline int emit_dmaflushp(u8 buf[], u8 periph)
{
	/*
	 * DMAFLUSHP encoding:
	 * | 15 . . . 11 10 9 8 | 7 6 5 4 3 2 1 0 |
	 * | periph[4:0]  0 0 0 | 0 0 1 1 0 1 0 1 |
	 */
	buf[0] = CMD_DMAFLUSHP;
	buf[1] = periph << 3;

	return SZ_DMAFLUSHP;
}

static inline int emit_dmago(u8 buf[], u8 cn, u32 imm, u8 ns)
{
	/*
	 * DMAGO encoding:
	 * | 15 14 13 12 11 10...08 | 07 06 05 04 03 02 01 00 |
	 * |  0  0  0  0  0 cn[2:0] |  1  0  1  0  0  0 ns  0 |
	 *
	 * | 47 ... 16 |
	 * | imm[31:0] |
	 */
	buf[0] = CMD_DMAGO | ((ns << 1) & 0x02);
	buf[1] = cn & 0x07;
	*((u32 *) &buf[2]) = imm;

	return SZ_DMAGO;
}

static inline int emit_dmald(u8 buf[], enum pl330_cond bs)
{
	/*
	 * DMALD[S|B] encoding:
	 * | 07 06 05 04 03 02 01 00 |
	 * |  0  0  0  0  0  1 bs  x |
	 */
	buf[0] = CMD_DMALD;
	if (bs == PL330_BURST)
		buf[0] |= 0x03;
	else if (bs == PL330_SINGLE)
		buf[0] |= 0x01;

	return SZ_DMALD;
}

static inline int emit_dmaldp(u8 buf[], enum pl330_cond bs, u8 periph)
{
	/*
	 * DMALDP<S|B> encoding:
	 * | 15 . . . 11 10 09 08 | 07 06 05 04 03 02 01 00 |
	 * | periph[4:0]  0  0  0 |  0  0  1  0  0  1 bs  1 |
	 */
	buf[0] = CMD_DMALDP | ((bs << 1) & 0x02);
	buf[1] = periph	<< 3;

	return SZ_DMALDP;
}

static inline int emit_dmalp(u8 buf[], u8 iter, u8 lc)
{
	/*
	 * DMALP encoding:
	 * | 15 ... 08 | 07 06 05 04 03 02 01 00 |
	 * | iter[7:0] |  0  0  1  0  0  0 lc  0 |
	 */
	buf[0] = CMD_DMALP;
	if (lc)
		buf[0] |= 0x02;
	buf[1] = iter;

	return SZ_DMALP;
}

static inline int emit_dmalpend(u8 buf[], enum pl330_cond bs, u8 lc,
	enum pl330_lp_type nf, u8 backwards_jump)
{
	/*
	 * DMALPEND[S|B] encoding:
	 * | 15      ...      08 | 07 06 05 04 03 02 01 00 |
	 * | backwards_jump[7:0] |  0  0  1 nf  1 lc bs  x |
	 */
	buf[0] = CMD_DMALPEND;
	if (nf == TYPE_LP) {
		buf[0] |= 0x10;
		buf[0] |= ((lc << 2) & 0x04);
		if (bs == PL330_SINGLE)
			buf[0] |= 0x01;
		else if (bs == PL330_BURST)
			buf[0] |= 0x03;
	}

	buf[1] = backwards_jump;

	return SZ_DMALPEND;
}

static inline int emit_dmakill(u8 buf[])
{
	/*
	 * DMAKILL encoding:
	 * | 7 6 5 4 3 2 1 0 |
	 * | 0 0 0 0 0 0 0 1 |
	 */
	buf[0] = CMD_DMAKILL;

	return SZ_DMAKILL;
}

static inline int emit_dmamov(u8 buf[], u8 rd, u32 imm)
{
	/*
	 * DMAMOV encoding:
	 * | 15 14 13 12 11 10 .. 8 | 7 6 5 4 3 2 1 0 |
	 * |  0  0  0  0  0 rd[2:0] | 1 0 1 1 1 1 0 0 |
	 *
	 * | 47 ... 16 |
	 * | imm[31:0] |
	 */
	buf[0] = CMD_DMAMOV;
	buf[1] = (rd <= 2) ? (rd & 0x07) : 0;
	*((u32 *) &buf[2]) = imm;

	return SZ_DMAMOV;
}

static inline int emit_dmanop(u8 buf[])
{
	/*
	 * DMANOP encoding:
	 * | 7 6 5 4 3 2 1 0 |
	 * | 0 0 0 1 1 0 0 0 |
	 */
	buf[0] = CMD_DMANOP;

	return SZ_DMANOP;
}

static inline int emit_dmarmb(u8 buf[])
{
	/*
	 * DMARMB encoding:
	 * | 7 6 5 4 3 2 1 0 |
	 * | 0 0 0 1 0 0 1 0 |
	 */
	buf[0] = CMD_DMARMB;

	return SZ_DMARMB;
}

static inline int emit_dmasev(u8 buf[], u8 event_num)
{
	/*
	 * DMASEV encoding:
	 * | 15   ...    11 10 9 8 | 7 6 5 4 3 2 1 0 |
	 * | event_num[4:0]  0 0 0 | 0 0 1 1 0 1 0 0 |
	 */
	buf[0] = CMD_DMASEV;
	buf[1] = (event_num << 3) & 0xF8;

	return SZ_DMASEV;
}

static inline int emit_dmast(u8 buf[], enum pl330_cond bs)
{
	/*
	 * DMAST[S|B] encoding:
	 * | 07 06 05 04 03 02 01 00 |
	 * |  0  0  0  0  1  0 bs  x |
	 */
	buf[0] = CMD_DMAST;

	if (bs == PL330_BURST)
		buf[0] |= 0x03;
	else if (bs == PL330_SINGLE)
		buf[0] |= 0x01;

	return SZ_DMAST;
}

static inline int emit_dmastp(u8 buf[], enum pl330_cond bs, u8 periph)
{
	/*
	 * DMASTP<S|B> encoding:
	 * | 15  ...  11 10 09 08 | 07 06 05 04 03 02 01 00 |
	 * | periph[4:0]  0  0  0 |  0  0  1  0  1  0 bs  1 |
	 */
	buf[0] = CMD_DMASTP | ((bs << 1) & 0x02);
	buf[1] = periph << 3;

	return SZ_DMASTP;
}

static inline int emit_dmastz(u8 buf[])
{
	/*
	 * DMASTZ encoding:
	 * | 7 6 5 4 3 2 1 0 |
	 * | 0 0 0 0 1 1 0 0 |
	 */
	buf[0] = CMD_DMASTZ;

	return SZ_DMASTZ;
}

static inline int emit_dmawfe(u8 buf[], u8 event_num, bool i)
{
	/*
	 * DMAWFE encoding:
	 * | 15   ...    11 10 9 8 | 7 6 5 4 3 2 1 0 |
	 * | event_num[4:0]  0 i 0 | 0 0 1 1 0 1 1 0 |
	 */
	buf[0] = CMD_DMAWFE;
	buf[1] = (event_num << 3) | ((i << 1) & 0x02);

	return SZ_DMAWFE;
}

static inline int emit_dmawfp(u8 buf[], enum pl330_cond bsp, u8 peripheral)
{
	/*
	 * DMAWFP<S|B|P> encoding:
	 * | 15    ...    11 10 09 08 | 07 06 05 04 03 02 01 00 |
	 * | peripheral[4:0]  0  0  0 |  0  0  1  1  0  0 bs  p |
	 */
	buf[0] = CMD_DMAWFP;
	if (bsp == PL330_BURST)
		buf[0] |= 0x02;
	else if (bsp == PL330_PERIPH)
		buf[0] |= 0x01;

	buf[1] = peripheral << 3;

	return SZ_DMAWFP;
}

static inline int emit_dmawmb(u8 buf[])
{
	/*
	 * DMAWMB encoding:
	 * | 7 6 5 4 3 2 1 0 |
	 * | 0 0 0 1 0 0 1 1 |
	 */
	buf[0] = CMD_DMAWMB;

	return SZ_DMAWMB;
}

void DMAADDH(struct pl330_prog *prog, enum pl330_addh_dst dst, u16 imm)
{
	prog->buf += emit_dmaaddh(prog->buf, dst, imm);
}
EXPORT_SYMBOL(DMAADDH);

void DMAEND(struct pl330_prog *prog)
{
	prog->buf += emit_dmaend(prog->buf);
}
EXPORT_SYMBOL(DMAEND);

void DMAFLUSHP(struct pl330_prog *prog, u8 periph)
{
	prog->buf += emit_dmaflushp(prog->buf, periph);
}
EXPORT_SYMBOL(DMAFLUSHP);

void DMALD(struct pl330_prog *prog)
{
	prog->buf += emit_dmald(prog->buf, PL330_ALWAYS);
}
EXPORT_SYMBOL(DMALD);

void DMALDS(struct pl330_prog *prog)
{
	prog->buf += emit_dmald(prog->buf, PL330_SINGLE);
}
EXPORT_SYMBOL(DMALDS);

void DMALDB(struct pl330_prog *prog)
{
	prog->buf += emit_dmald(prog->buf, PL330_BURST);
}
EXPORT_SYMBOL(DMALDB);

void DMALDPS(struct pl330_prog *prog, u8 periph)
{
	prog->buf += emit_dmaldp(prog->buf, PL330_SINGLE, periph);
}
EXPORT_SYMBOL(DMALDPS);

void DMALDPB(struct pl330_prog *prog, u8 periph)
{
	prog->buf += emit_dmaldp(prog->buf, PL330_BURST, periph);
}
EXPORT_SYMBOL(DMALDPB);

void DMALP(struct pl330_prog *prog, u8 iter)
{
	int lp = 0;
	int lp_cnt = 0;

	if (prog->lp_top != LP_MAX - 1) {
		/* Count normal loops in stack */
		for (lp_cnt = 0; lp_cnt <= prog->lp_top; lp_cnt++) {
			if (prog->lp_stack[lp_cnt].type == TYPE_LP)
				lp++;
		}

		/* PL330 supports only two levels of loops */
		if (lp != 2) {
			prog->buf += emit_dmalp(prog->buf, iter - 1, lp);

			/* Push it to stack */
			prog->lp_top++;
			prog->lp_stack[prog->lp_top].type = TYPE_LP;
			prog->lp_stack[prog->lp_top].addr = prog->buf;
		}
	}
}
EXPORT_SYMBOL(DMALP);

static void _DMALPEND(struct pl330_prog *prog, enum pl330_cond bs)
{
	enum pl330_lp_type nf = TYPE_FE;
	int lp = 0;
	int lp_cnt;

	/* Check loop stack */
	if (prog->lp_top != -1) {
		/* Normal loop */
		if (prog->lp_stack[prog->lp_top].type == TYPE_LP) {
			/* Count normal loops in stack */
			for (lp_cnt = 0; lp_cnt <= prog->lp_top; lp_cnt++) {
				if (prog->lp_stack[lp_cnt].type == TYPE_LP)
					lp++;
			}
			lp--;
			nf = TYPE_LP;
		}
		prog->buf += emit_dmalpend(prog->buf, bs, lp, nf,
			prog->buf - prog->lp_stack[prog->lp_top].addr);

		/* Pull loop from stack */
		prog->lp_top--;
	}
}

void DMALPEND(struct pl330_prog *prog)
{
	_DMALPEND(prog, PL330_ALWAYS);
}
EXPORT_SYMBOL(DMALPEND);

void DMALPENDS(struct pl330_prog *prog)
{
	_DMALPEND(prog, PL330_SINGLE);
}
EXPORT_SYMBOL(DMALPENDS);

void DMALPENDB(struct pl330_prog *prog)
{
	_DMALPEND(prog, PL330_BURST);
}
EXPORT_SYMBOL(DMALPENDB);

void DMALPFE(struct pl330_prog *prog)
{
	if (prog->lp_top != LP_MAX - 1) {
		prog->lp_top++;
		prog->lp_stack[prog->lp_top].type = TYPE_FE;
		prog->lp_stack[prog->lp_top].addr = prog->buf;
	}

}
EXPORT_SYMBOL(DMALPFE);

void DMAKILL(struct pl330_prog *prog)
{
	prog->buf += emit_dmakill(prog->buf);
}
EXPORT_SYMBOL(DMAKILL);

void DMAMOV(struct pl330_prog *prog, enum pl330_mov_dst dst, u32 imm)
{
	prog->buf += emit_dmamov(prog->buf, dst, imm);
}
EXPORT_SYMBOL(DMAMOV);

void DMANOP(struct pl330_prog *prog)
{
	prog->buf += emit_dmanop(prog->buf);
}
EXPORT_SYMBOL(DMANOP);

void DMARMB(struct pl330_prog *prog)
{
	prog->buf += emit_dmarmb(prog->buf);
}
EXPORT_SYMBOL(DMARMB);

void DMASEV(struct pl330_prog *prog, u8 event_num)
{
	prog->buf += emit_dmasev(prog->buf, event_num);
}
EXPORT_SYMBOL(DMASEV);

void DMAST(struct pl330_prog *prog)
{
	prog->buf += emit_dmast(prog->buf, PL330_ALWAYS);
}
EXPORT_SYMBOL(DMAST);

void DMASTS(struct pl330_prog *prog)
{
	prog->buf += emit_dmast(prog->buf, PL330_SINGLE);
}
EXPORT_SYMBOL(DMASTS);

void DMASTB(struct pl330_prog *prog)
{
	prog->buf += emit_dmast(prog->buf, PL330_BURST);
}
EXPORT_SYMBOL(DMASTB);

void DMASTPS(struct pl330_prog *prog, u8 periph)
{
	prog->buf += emit_dmastp(prog->buf, PL330_SINGLE, periph);
}
EXPORT_SYMBOL(DMASTPS);

void DMASTPB(struct pl330_prog *prog, u8 periph)
{
	prog->buf += emit_dmastp(prog->buf, PL330_BURST, periph);
}
EXPORT_SYMBOL(DMASTPB);

void DMASTZ(struct pl330_prog *prog)
{
	prog->buf += emit_dmastz(prog->buf);
}
EXPORT_SYMBOL(DMASTZ);

void DMAWFE(struct pl330_prog *prog, u8 event_num, bool invalid)
{
	prog->buf += emit_dmawfe(prog->buf, event_num, invalid);
}
EXPORT_SYMBOL(DMAWFE);

void DMAWFPS(struct pl330_prog *prog, u8 periph)
{
	prog->buf += emit_dmawfp(prog->buf, PL330_SINGLE, periph);
}
EXPORT_SYMBOL(DMAWFPS);

void DMAWFPB(struct pl330_prog *prog, u8 periph)
{
	prog->buf += emit_dmawfp(prog->buf, PL330_BURST, periph);
}
EXPORT_SYMBOL(DMAWFPB);

void DMAWFPP(struct pl330_prog *prog, u8 periph)
{
	prog->buf += emit_dmawfp(prog->buf, PL330_PERIPH, periph);
}
EXPORT_SYMBOL(DMAWFPP);

void DMAWMB(struct pl330_prog *prog)
{
	prog->buf += emit_dmawmb(prog->buf);
}
EXPORT_SYMBOL(DMAWMB);

void pl330_init_prog(struct pl330_prog *prog, u8 *buf)
{
	if (prog && buf) {
		prog->lp_top = -1;
		prog->buf = buf;
	}
}
EXPORT_SYMBOL(pl330_init_prog);

/**
 * pl330_build_prog - Build pl330 program.
 * @prog_buf:	Buffer for storage pl330 program.
 * @args:	Transfer description.
 *
 * Build pl330 program based on description from @args. Function support
 * only mem to mem transfer right now.
 *
 * TODO	- Support burst transactions.
 *	- Support second level loop.
 *
 * Returns true on success, false otherwise.
 */
static bool pl330_build_prog(u8 *prog_buf, struct pl330_build_args *args)
{
	struct pl330_prog prog;

	pl330_init_prog(&prog, prog_buf);

	/* If number of bytes to transfer is too big, rather leave */
	if (args->count > 256)
		return false;

	/* Set source and destionation */
	DMAMOV(&prog, PL330_SAR, args->src_addr);
	DMAMOV(&prog, PL330_DAR, args->dst_addr);

	/* Set up transaction control */
	DMAMOV(&prog, PL330_CCR,
		CC_SET_SRC_INC(args->src_inc) | CC_SET_DST_INC(args->dst_inc) |
		CC_SET_SRC_BURST_LEN(0) | CC_SET_SRC_BURST_SIZE(0) |
		CC_SET_DST_BURST_LEN(0) | CC_SET_DST_BURST_SIZE(0));

	/* Should we build loop? */
	if (args->count > 1) {
		DMALP(&prog, args->count);
	}

	/* Load and store */
	DMALD(&prog);
	DMAST(&prog);

	/* End loop */
	if (args->count > 1) {
		DMALPEND(&prog);
	}

	/* Invoke interrupt and exit */
	DMAWMB(&prog);
	DMASEV(&prog, args->chan_id);
	DMAEND(&prog);

	return true;
}

/**
 * chan_to_pl330chan - Converts dma_chan struct to pl330_chan.
 * @chan:	DMA-Engine's channel struct.
 *
 * Returns pl330_chan struct.
 */
struct pl330_chan *chan_to_pl330chan(struct dma_chan *chan)
{
	return container_of(chan, struct pl330_chan, dmae_chan);
}

/**
 * chan_to_pl330chan - Converts dma_async_tx_descriptor struct to pl330_desc.
 * @desc:	DMA-Engine's descriptor struct.
 *
 * Returns pl330_desc struct.
 */
struct pl330_desc *desc_to_pl330desc(struct dma_async_tx_descriptor *desc)
{
	return container_of(desc, struct pl330_desc, dmae_desc);
}

/**
 * pl330_get_chan_by_irq - find channel by IRQ number.
 * @irq:	IRQ number.
 * @dmac:	Top level structure containing channels.
 *
 * Returns found channel, NULL otherwise.
 */
struct pl330_chan *pl330_get_chan_by_irq(int irq, struct pl330_dmac *dmac)
{
	int i;
	struct pl330_chan *chan;

	if (dmac == NULL)
		return NULL;

	for (i = 0; i < dmac->chan_cnt; i++) {
		chan = &dmac->chans[i];

		if (chan->irq == irq)
			return chan;
	}

	/* No channel found */
	return NULL;
}

/**
 * pl330_irq_handler - handler for PL330 interrupts.
 * @irq:	IRQ number.
 * @dmac:	Pointer to top level struct.
 *
 * Returns IRQ_HANDLED in case of successfully handled IRQ, IRQ_NONE otherwise.
 */
static irqreturn_t pl330_irq_handler(int irq, void *dmac)
{
	struct pl330_dmac *pl_dmac = dmac;
	struct pl330_chan *pl_chan;
	bool handled = false;
	unsigned long flags;
	u32 val;
	int i;
	u8 kill_prog[SZ_DMAKILL];

	/* IRQ of manager thread */
	if (pl_dmac->irq_mngr == irq) {
		spin_lock_irqsave(&pl_dmac->lock, flags);

		val = ioread32(pl_dmac->base + REG_FSM);
		/* DMA manager thread is in Faulting state */
		if (val) {
			val = ioread32(pl_dmac->base + REG_FTM);
			dev_err(pl_dmac->dev, "Manager in Faulting state "
				"(error: 0x%x)\n", val);
		}

		val = ioread32(pl_dmac->base + REG_FSC);

		spin_unlock_irqrestore(&pl_dmac->lock, flags);

		/* There may be some Faulting channel threads...
		 * ... let's kill them! */
		for (i = 0; i < pl_dmac->config.num_chnls; i++) {
			/* Channel "i" is in Faulting state */
			if ((val >> i) & 0x1) {
				spin_lock_irqsave(&pl_dmac->lock, flags);

				val = ioread32(pl_dmac->base + REG_FTC(i));
				dev_err(pl_dmac->dev, "Channel %d in Faulting "
				"state (error: 0x%x)\n", i, val);

				val = ioread32(pl_dmac->base + REG_SA(i));
				dev_err(pl_dmac->dev, "Source address %x\n",
					val);

				val = ioread32(pl_dmac->base + REG_DA(i));
				dev_err(pl_dmac->dev, "Dest address %x\n",
					val);

				emit_dmakill(kill_prog);

				/* Write DMAKILL instruction into register
				 * Debug Instruction-0 */
				val = kill_prog[0] << 16 | 0x1 | (i << 8);
				iowrite32(val, pl_dmac->base + REG_DBGINST0);

				/* Wait if DMAC is busy */
				while (ioread32(pl_dmac->base + REG_DGBSTATUS))
					;

				/* Kill it! */
				iowrite32(0, pl_dmac->base + REG_DBGCMD);

				spin_unlock_irqrestore(&pl_dmac->lock, flags);

				pl330_finish_transfer(&pl_dmac->chans[i]);
				pl330_tasklet(&pl_dmac->chans[i]);
			}
		}

		return IRQ_HANDLED;
	}

	/* This should never happen */
	if ((pl_chan = pl330_get_chan_by_irq(irq, pl_dmac)) == NULL) {

		dev_err(pl_dmac->dev, "irq handler: no channel for this"
			"IRQ (%d)!\n", irq);
		return IRQ_NONE;
	}

	/* IRQ of some channel thread */
	spin_lock_irqsave(&pl_dmac->lock, flags);
	val = ioread32(pl_dmac->base + REG_INTSTATUS);

	if ((val >> pl_chan->chan_id) & 0x1) {
		/* Clear interrupt flag for this channel only */
		iowrite32(0x1 << pl_chan->chan_id, pl_dmac->base + REG_INTCLR);
		handled = true;
	}
	else {
		/* DMA channel raised wrong IRQ */
		printk(KERN_ERR "%s: irq handler: wrong IRQ (%d) for this "
			"channel (%d)!\n", DRIVER_NAME, irq, pl_chan->chan_id);
		handled = false;
	}

	spin_unlock_irqrestore(&pl_dmac->lock, flags);

	if (handled) {
		pl330_finish_transfer(pl_chan);
		pl330_tasklet(pl_chan);

		return IRQ_HANDLED;
	}
	else
		return IRQ_NONE;
}

int pl330_alloc_chan_resources(struct dma_chan *chan)
{
	printk(KERN_DEBUG "%s: alloc_chan_resources\n", DRIVER_NAME);

	return 1;
}

/**
 * pl330_free_chan_resources - Free resources for @chan.
 * @chan:	Channel to be freed.
 */
void pl330_free_chan_resources(struct dma_chan *chan)
{
	struct pl330_chan *pl_chan;
	struct pl330_dmac *pl_dmac;
	struct pl330_desc *pl_desc;
	u8 kill_prog[SZ_DMAKILL];
	u32 w_reg;
	unsigned long flags;

	if (chan == NULL)
		return;

	pl_chan = chan_to_pl330chan(chan);
	pl_dmac = pl_chan->dmac;

	spin_lock_irqsave(&pl_dmac->lock, flags);

	/* Disable generating interrupt */
	iowrite32(ioread32(pl_dmac->base + REG_INTEN) &
		~(1 << pl_chan->chan_id), pl_dmac->base + REG_INTEN);

	/* If channel is not stopped, kill it */
	if (ioread32(pl_dmac->base + REG_CS(pl_chan->chan_id)) & 0xF) {
		emit_dmakill(kill_prog);

		/* Wait if DMAC is busy */
		while (ioread32(pl_dmac->base + REG_DGBSTATUS))
			;

		w_reg = (kill_prog[0] << 16) | ((pl_chan->chan_id & 0x7) << 8) | 1;

		iowrite32(w_reg, pl_dmac->base + REG_DBGINST0);
		iowrite32(0, pl_dmac->base + REG_DBGINST1);

		/* EXECUTE! */
		iowrite32(0, pl_dmac->base + REG_DBGCMD);
	}

	spin_unlock_irqrestore(&pl_dmac->lock, flags);

	spin_lock_irqsave(&pl_dmac->pool_lock, flags);

	/* Release descriptors from prepared popol */
	while (!list_empty(&pl_chan->desc_prep_pool)) {
		pl_desc = list_entry(pl_chan->desc_prep_pool.next,
			struct pl330_desc, node);
		list_del_init(&pl_desc->node);

		pl_desc->dmae_desc.callback = NULL;
		pl_desc->dmae_desc.callback_param = NULL;
		pl_desc->stat = STAT_FREE;
		pl_desc->custom_prog = false;

		list_add_tail(&pl_desc->node, &pl_dmac->desc_pool);
	}

	/* Release descriptors from submitted popol */
	while (!list_empty(&pl_chan->desc_sub_pool)) {
		pl_desc = list_entry(pl_chan->desc_sub_pool.next,
			struct pl330_desc, node);
		list_del_init(&pl_desc->node);

		pl_desc->dmae_desc.callback = NULL;
		pl_desc->dmae_desc.callback_param = NULL;
		pl_desc->stat = STAT_FREE;
		pl_desc->custom_prog = false;

		list_add_tail(&pl_desc->node, &pl_dmac->desc_pool);
	}

	spin_unlock_irqrestore(&pl_dmac->pool_lock, flags);

	printk(KERN_DEBUG "%s: free_chan_resources\n", DRIVER_NAME);

	return;
}

/**
 * pl330_prep_dma_memcpy - Prepare memory to memory transfer.
 * @chan:	DMA-Engine channel.
 * @dst:	Destination (bus) address for DMA transfer.
 * @src:	Source (bus) address for DMA transfer.
 * @len:	Total count of bytes to be transferred.
 * @flags:	Flags for this DMA transfer.
 *
 * Builds program for mem to mem transfer. This function is called
 * by another Linux module.
 *
 * Returns tx descriptor on success, NULL otherwise.
 */
struct dma_async_tx_descriptor *pl330_prep_dma_memcpy(
	struct dma_chan *chan, dma_addr_t dst, dma_addr_t src,
	size_t len, unsigned long flags)
{
	struct pl330_build_args args;
	unsigned long lock_flags;
	struct pl330_dmac *pl_dmac;
	struct pl330_chan *pl_chan;
	struct pl330_desc *pl_desc;
	struct dma_async_tx_descriptor *desc = NULL;

	if (!chan)
		return NULL;

	pl_chan = chan_to_pl330chan(chan);
	pl_dmac = pl_chan->dmac;

	spin_lock_irqsave(&pl_dmac->pool_lock, lock_flags);

	/* Is there any free descriptor? */
	if (list_empty(&pl_dmac->desc_pool)) {
		spin_unlock_irqrestore(&pl_dmac->pool_lock, flags);
		/* Create new descriptor and add it into descriptor pool */
		pl_desc = create_desc(pl_dmac, GFP_ATOMIC);
		if (!pl_desc) {
			dev_warn(pl_dmac->dev, "Creating new descriptor "
				"failed\n");
			spin_unlock_irqrestore(&pl_dmac->pool_lock, lock_flags);
			return NULL;
		}
		printk(KERN_DEBUG "New descriptor added\n");
		spin_lock_irqsave(&pl_dmac->pool_lock, lock_flags);
	}

	pl_desc = list_entry(pl_dmac->desc_pool.next, struct pl330_desc, node);
	list_del_init(&pl_desc->node);

	spin_unlock_irqrestore(&pl_dmac->pool_lock, lock_flags);

	/* Set up params for program builder */
	args.src_addr = (u32) src;
	args.dst_addr = (u32) dst;
	args.src_inc = 1;
	args.dst_inc = 1;
	args.chan_id = pl_chan->chan_id;
	args.count = len;

	/* Build program for mem to mem transfer */
	if (!pl330_build_prog(pl_desc->buf_virt, &args)) {
		spin_lock_irqsave(&pl_dmac->pool_lock, lock_flags);
		list_add_tail(&pl_desc->node, &pl_dmac->desc_pool);
		spin_unlock_irqrestore(&pl_dmac->pool_lock, lock_flags);
		return NULL;
	}

	pl_desc->dmae_desc.chan = chan;
	pl_desc->dmae_desc.callback = NULL;
	pl_desc->dmae_desc.callback_param = NULL;
	pl_desc->stat = STAT_PREP;
	pl_desc->custom_prog = false;

	desc = &pl_desc->dmae_desc;

	/* Move descriptor into prepared pool */
	spin_lock_irqsave(&pl_chan->lock, lock_flags);
	list_add_tail(&pl_desc->node, &pl_chan->desc_prep_pool);
	spin_unlock_irqrestore(&pl_chan->lock, lock_flags);

	return desc;
}

struct dma_async_tx_descriptor *pl330_prep_program_addr(
	struct dma_chan *chan, dma_addr_t start_address)
{
	struct pl330_dmac *pl_dmac;
	struct pl330_chan *pl_chan;
	struct pl330_desc *pl_desc;
	struct dma_async_tx_descriptor *desc = NULL;
	unsigned long flags;

	if (!chan)
		return NULL;

	pl_chan = chan_to_pl330chan(chan);
	pl_dmac = pl_chan->dmac;

	spin_lock_irqsave(&pl_dmac->pool_lock, flags);

	/* Is there any free descriptor? */
	if (list_empty(&pl_dmac->desc_pool)) {
		/* Create new descriptor and add it into descriptor pool */
		spin_unlock_irqrestore(&pl_dmac->pool_lock, flags);
		pl_desc = create_desc(pl_dmac, GFP_ATOMIC);
		if (!pl_desc) {
			dev_warn(pl_dmac->dev, "Creating new descriptor "
				"failed\n");
			return NULL;
		}
		printk(KERN_DEBUG "New descriptor added\n");
		spin_lock_irqsave(&pl_dmac->pool_lock, flags);
	}

	pl_desc = list_entry(pl_dmac->desc_pool.next, struct pl330_desc, node);
	list_del_init(&pl_desc->node);
	
	spin_unlock_irqrestore(&pl_dmac->pool_lock, flags);

	pl_desc->dmae_desc.chan = chan;
	pl_desc->dmae_desc.callback = NULL;
	pl_desc->dmae_desc.callback_param = NULL;
	pl_desc->stat = STAT_PREP;
	pl_desc->custom_prog = true;
	pl_desc->custom_bus = start_address;

	desc = &pl_desc->dmae_desc;

	/* Move descriptor into prepared pool */
	spin_lock_irqsave(&pl_chan->lock, flags);
	list_add_tail(&pl_desc->node, &pl_chan->desc_prep_pool);
	spin_unlock_irqrestore(&pl_chan->lock, flags);

	return desc;

}
EXPORT_SYMBOL(pl330_prep_program_addr);

bool pl330_filter_peri(struct dma_chan *chan, void *param)
{
	struct pl330_chan *pl_chan;
	bool *peri;

	if (!chan || !param)
		return false;

	pl_chan = chan_to_pl330chan(chan);
	peri = (bool *) param;

	return pl_chan->peri == *peri;
}
EXPORT_SYMBOL(pl330_filter_peri);

bool pl330_filter_chan_id(struct dma_chan *chan, void *param)
{
	struct pl330_chan *pl_chan;
	int *id;

	if (!chan || !param)
		return false;

	pl_chan = chan_to_pl330chan(chan);
	id = (int *) param;

	return pl_chan->chan_id == *id;
}
EXPORT_SYMBOL(pl330_filter_chan_id);

int pl330_get_channel_id(struct dma_chan *chan)
{
	struct pl330_chan *pl_chan;

	if (!chan)
		return -1;

	pl_chan = chan_to_pl330chan(chan);

	return pl_chan->chan_id;
}
EXPORT_SYMBOL(pl330_get_channel_id);

bool pl330_has_channel_pri(struct dma_chan *chan)
{
	struct pl330_chan *pl_chan;

	if (!chan)
		return false;

	pl_chan = chan_to_pl330chan(chan);

	return pl_chan->peri;
}
EXPORT_SYMBOL(pl330_has_channel_pri);

static enum dma_status pl330_tx_status(struct dma_chan *chan,
	dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	printk(KERN_DEBUG "%s: tx status\n", DRIVER_NAME);

	return 0;	//TODO
}

/**
 * pl330_issue_pending - push pending transaction to PL330.
 * @chan:	DMA-Engine channel.
 *
 * Push all pending transaction from queue of channel "chan"
 * to hardware (PL330).
 */
static void pl330_issue_pending(struct dma_chan *chan)
{
	struct pl330_chan *pl_chan;

	if (chan == NULL)
		return;

	pl_chan = chan_to_pl330chan(chan);

	pl330_tasklet(pl_chan);
}

/**
 * pl330_tx_submit - Submit tx descriptor into waiting queue.
 * @tx:		DMA-Engine tx descriptor.
 *
 * TODO return values.
 */
dma_cookie_t pl330_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct pl330_desc *pl_desc;
	struct pl330_chan *pl_chan;
	unsigned long flags;

	if (tx == NULL || tx->chan == NULL)
		return -1;

	pl_desc = desc_to_pl330desc(tx);
	pl_chan = chan_to_pl330chan(tx->chan);

	spin_lock_irqsave(&pl_chan->lock, flags);

	/* descriptor is in desc_prep_pool -> move it to desc_sub_pool */
	if (pl_desc->stat == STAT_PREP) {
		list_move_tail(&pl_desc->node, &pl_chan->desc_sub_pool);
		pl_desc->stat = STAT_SUBMIT;
	}

	spin_unlock_irqrestore(&pl_chan->lock, flags);

	return 0;
}

/**
 * pl330_finish_transfer - Finish transfer - move active descriptor into
 *			descriptor pool, do callback (if set).
 * @pl_chan:		Channel which finished transfer.
 */
static void pl330_finish_transfer(struct pl330_chan *pl_chan)
{
	struct pl330_dmac *pl_dmac = pl_chan->dmac;
	struct pl330_desc *pl_desc;
	struct dma_async_tx_descriptor *tx_desc;
	unsigned long flags;

	spin_lock_irqsave(&pl_chan->lock, flags);

	/* Rather check desc pool */
	if (list_empty(&pl_chan->desc_sub_pool)) {
		dev_err(pl_dmac->dev, "pl330_finish_transfer(): no descriptor"
			"here\n");
		spin_unlock_irqrestore(&pl_chan->lock, flags);
		return;
	}

	pl_desc = list_entry(pl_chan->desc_sub_pool.next, struct pl330_desc, node);
	list_del_init(&pl_desc->node);

	spin_unlock_irqrestore(&pl_chan->lock, flags);

	tx_desc = &pl_desc->dmae_desc;

	/* Do callback, if any */
	if (tx_desc->callback)
		tx_desc->callback(tx_desc->callback_param);

	pl_desc->stat = STAT_FREE;

	/* Move descriptor into free descriptor pool */
	spin_lock_irqsave(&pl_dmac->lock, flags);
	list_add_tail(&pl_desc->node, &pl_dmac->desc_pool);
	spin_unlock_irqrestore(&pl_dmac->lock, flags);
}

/**
 * pl330_tasklet - Start next transfer in the submit queue.
 * @pl_chan:		Target channel.
 */
static void pl330_tasklet(struct pl330_chan *pl_chan)
{
	struct pl330_dmac *pl_dmac;
	struct pl330_desc *pl_desc;
	u32 w_reg;
	unsigned long flags;
	u8 go_prog[SZ_DMAGO];

	pl_dmac = pl_chan->dmac;

	spin_lock_irqsave(&pl_chan->lock, flags);

	/* Check if there is any submitted desc */
	if (list_empty(&pl_chan->desc_sub_pool)) {
		spin_unlock_irqrestore(&pl_chan->lock, flags);
		return;
	}

	/* Check if descriptor is busy */
	pl_desc = list_entry(pl_chan->desc_sub_pool.next, struct pl330_desc, node);
	if (pl_desc->stat != STAT_SUBMIT) {
		spin_unlock_irqrestore(&pl_chan->lock, flags);
		return;
	}

	/* Preparing transfer...descriptor is busy now */
	pl_desc->stat = STAT_BUSY;
	spin_unlock_irqrestore(&pl_chan->lock, flags);

	spin_lock_irqsave(&pl_dmac->lock, flags);

	/* Wait if channel is not stopped */
	while (ioread32(pl_dmac->base + REG_CS(pl_chan->chan_id)) & 0xF)
		;

	/* Enable generating interrupt */
	iowrite32(ioread32(pl_dmac->base + REG_INTEN) |
		(1 << pl_chan->chan_id), pl_dmac->base + REG_INTEN);

	emit_dmago(go_prog, pl_chan->chan_id, (pl_desc->custom_prog) ?
		(pl_desc->custom_bus) : (pl_desc->buf_bus), 0);

	/* Wait if DMAC is busy */
	while (ioread32(pl_dmac->base + REG_DGBSTATUS))
		;

	/* Write first 2 bytes of DMAGO instruction into register
	 * Debug Instruction-0 */
	w_reg = go_prog[1] << 24 | go_prog[0] << 16;
	iowrite32(w_reg, pl_dmac->base + REG_DBGINST0);

	/* Write rest of DMAGO instruction into register
	 * Debug Instruction-1 */
	w_reg = *((u32 *)&go_prog[2]);
	iowrite32(w_reg, pl_dmac->base + REG_DBGINST1);

	/* EXECUTE! */
	iowrite32(0, pl_dmac->base + REG_DBGCMD);

	spin_unlock_irqrestore(&pl_dmac->lock, flags);
}

/**
 * create_desc - Create new descriptor and add it into descriptor pool.
 * @pl_dmac:		Top level structure for DMAC.
 * @gfp_flag:		Flag for allocator.
 *
 * Returns pointer to the created descriptor, NULL otherwise.
 */
static struct pl330_desc *create_desc(struct pl330_dmac *dmac, gfp_t gfp_flag)
{
	struct pl330_desc *desc;
	unsigned long flags;

	desc = kzalloc(sizeof(struct pl330_desc), gfp_flag);
	if (!desc) {
		dev_err(dmac->dev, "unable to alloc memory (descriptors)\n");
		return NULL;
	}

	/*
	 * Alloc buffer for pl330 program
	 * TODO Create program pool in the future
	 */
	desc->buf_virt = dma_alloc_coherent(dmac->dev, 0x1000,
		&desc->buf_bus, gfp_flag);
	if (!desc->buf_virt) {
		dev_err(dmac->dev, "unable to alloc coherent (pl330 prog)\n");
		kfree(desc);
		return NULL;
	}

	/* Init desc */
	desc->stat = STAT_FREE;
	desc->dmae_desc.tx_submit = pl330_tx_submit;
	INIT_LIST_HEAD(&desc->node);

	/* Add desc into descriptor pool */
	spin_lock_irqsave(&dmac->pool_lock, flags);
	list_add_tail(&desc->node, &dmac->desc_pool);
	spin_unlock_irqrestore(&dmac->pool_lock, flags);

	return desc;
}

/**
 * pl330_read_config - Read DMAC Configuration Registers.
 * @base:	DMAC base address.
 * @config:	Struct where configuration will be stored.
 */
static void pl330_read_config(void __iomem *base, struct pl330_config *config)
{
	u32 val;

	/* Read Configuration Register 0 */
	val = ioread32(base + REG_CR0);

	config->num_events = CR0_GET_NUM_EVENTS(val) + 1;
	if (!CR0_GET_PERIPH_REQ(val))
		config->num_periph_req = 0;
	else
		config->num_periph_req = CR0_GET_NUM_PERIPH(val) + 1;
	config->num_chnls = CR0_GET_NUM_CHNLS(val) + 1;
	
	return;
}

/**
 * pl330_probe - Device probe.
 * @adev:	Pointer to the amba device struct.
 * @aid:	Pointer to the amba id struct.
 *
 * Returns 0 on success, negative error otherwise.
 */
static int pl330_probe(struct amba_device *adev, const struct amba_id *aid)
{
	struct pl330_dmac *p_dmac;
	struct dma_device *p_dma_dev;
	struct pl330_desc *desc;
	struct pl330_chan *chan;
	int i, ret = 0;
	int irq[AMBA_NR_IRQS];

	/* Check args */
	if (!adev) {
		printk(KERN_ERR "%s: probe called with NULL amba_device\n",
			DRIVER_NAME);
		return -ENODEV;
	}
	if (!aid) {
		dev_err(&adev->dev, "probe called with NULL amba_id.\n");
		return -ENODEV;
	}

	/* Alloc mem to store information for DMAC - driver data */
	p_dmac = devm_kzalloc(&adev->dev, sizeof(struct pl330_dmac), GFP_KERNEL);
	if (!p_dmac) {
		dev_err(&adev->dev, "unable to allocate memory (pl330_dmac)\n");
		return -ENOMEM;
	}
	spin_lock_init(&p_dmac->lock);
	spin_lock_init(&p_dmac->pool_lock);
	INIT_LIST_HEAD(&p_dmac->desc_pool);
	p_dmac->dev = &adev->dev;
	amba_set_drvdata(adev, p_dmac);

	/* Remap DMAC address space */
	p_dmac->base = devm_ioremap_resource(&adev->dev, &adev->res);
	if (IS_ERR(p_dmac->base)) {
		dev_err(&adev->dev, "pl330 DMAC ioremap failed.\n");
		return PTR_ERR(p_dmac->base);
	}

	/* Request IRQs */
	for (i = 0; i < AMBA_NR_IRQS; i++) {
		irq[i] = adev->irq[i];
		ret = devm_request_irq(&adev->dev, irq[i], pl330_irq_handler, 0,
			dev_name(&adev->dev), p_dmac);
		if (ret) {
			dev_err(&adev->dev, "IRQ request failed\n");
			return ret;
		}
	}
	p_dmac->irq_mngr = irq[0];	/* Set IRQ for manager thread */

	/* Create descriptor pool */
	for (i = 0; i < DEFAULT_POOL_SIZE; i++) {
		desc = create_desc(p_dmac, GFP_KERNEL);
		if (!desc) {
			ret = -ENOMEM;
			goto err_desc_pool;
		}
	}

	/* Read DMAC settings from Configuration Registers */
	pl330_read_config(p_dmac->base, &p_dmac->config);

	/* Set up device for DMA Engine */
	p_dma_dev = &p_dmac->dmae_dev;
	p_dma_dev->dev = &adev->dev;
	dma_cap_set(DMA_MEMCPY, p_dma_dev->cap_mask);
	p_dma_dev->device_alloc_chan_resources = pl330_alloc_chan_resources;
	p_dma_dev->device_free_chan_resources = pl330_free_chan_resources;
	p_dma_dev->device_prep_dma_memcpy = pl330_prep_dma_memcpy;
	p_dma_dev->device_tx_status = pl330_tx_status;
	p_dma_dev->device_issue_pending = pl330_issue_pending;

	/* Init channels */
	INIT_LIST_HEAD(&p_dma_dev->channels);
	p_dmac->chans = kzalloc(
		p_dmac->config.num_chnls * sizeof(struct pl330_chan),
		GFP_KERNEL);
	if (!p_dmac->chans) {
		dev_err(&adev->dev, "unable to allocate memory (dma_chan)\n");
		ret = -ENOMEM;
		goto err_chans;
	}

	/* Set up channels and add them into device */
	p_dmac->chan_cnt = 0;
	for (i = 0; i < p_dmac->config.num_chnls; i++) {
		chan = &p_dmac->chans[i];

		if (i == AMBA_NR_IRQS - 1)
			break;	/* we run out of IRQs */

		p_dmac->chan_cnt++;
		chan->irq = irq[i + 1];	/* first IRQ is for manager thread */
		chan->peri = (i < p_dmac->config.num_periph_req) ? true : false;
		chan->dmac = p_dmac;
		chan->chan_id = i;
		chan->dmae_chan.device = p_dma_dev;
		INIT_LIST_HEAD(&chan->desc_prep_pool);
		INIT_LIST_HEAD(&chan->desc_sub_pool);
		spin_lock_init(&chan->lock);
		list_add_tail(&chan->dmae_chan.device_node,
			&p_dma_dev->channels);
	}

	/* Register device in DMA Engine */
	ret = dma_async_device_register(p_dma_dev);
	if (ret) {
		dev_err(&adev->dev, "unable to register DMA\n");
		goto err_dma_register;
	}

	dev_info(&adev->dev, "Loaded driver for PL330 DMAC (id %x, mask %x)\n",
		aid->id, aid->mask);
	dev_info(&adev->dev, "Registered %d of %d channels, %d events and %d "
		"peripherals\n", p_dmac->chan_cnt, p_dmac->config.num_chnls,
		p_dmac->config.num_events, p_dmac->config.num_periph_req);

	return 0;

	err_dma_register:
		kfree(p_dmac->chans);
	err_chans:
	err_desc_pool:
		while (!list_empty(&p_dmac->desc_pool)) {
			desc = list_entry(p_dmac->desc_pool.next,
				struct pl330_desc, node);
			list_del(&desc->node);
			dma_free_coherent(p_dmac->dev, 0x1000, desc->buf_virt,
				desc->buf_bus);
			kfree(desc);
		}

	return ret;
}

/**
 * pl330_remove - Remove device
 * @adev:	Pointer to the amba device struct.
 *
 * Returns 0 on success, negative error otherwise.
 */
static int pl330_remove(struct amba_device *adev)
{
	struct pl330_dmac *pl_dmac = amba_get_drvdata(adev);
	struct pl330_desc *pl_desc;

	dma_async_device_unregister(&pl_dmac->dmae_dev);

	while (!list_empty(&pl_dmac->desc_pool)) {
		pl_desc = list_entry(pl_dmac->desc_pool.next,
			struct pl330_desc, node);
		list_del(&pl_desc->node);
		dma_free_coherent(pl_dmac->dev, 0x1000, pl_desc->buf_virt,
			pl_desc->buf_bus);
		kfree(pl_desc);
	}

	kfree(pl_dmac->chans);

	printk(KERN_DEBUG "%s: removing\n", DRIVER_NAME);

	return 0;
}

static struct amba_id pl330_id_table[] = {
	{
		/* ARM	ID = 0x41
		   DMAC	ID = 0x330 */
		.id	= 0x00041330,
		.mask	= 0x000fffff
	},
	{
		/* Nulova maska je zarazka */
		.id	= 0,
		.mask	= 0
	}
};

static struct amba_driver pl330_amba_driver = {
	.probe = pl330_probe,
	.remove = pl330_remove,
	.id_table = pl330_id_table,
	.drv = {
		.owner = THIS_MODULE,
		.name = DRIVER_NAME,
	}
};

module_amba_driver(pl330_amba_driver);

MODULE_AUTHOR("Jan Havran <xhavra13@stud.fit.vutbr.cz>");
MODULE_DESCRIPTION("PL330 driver");
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE(DRIVER_LICENSE);
