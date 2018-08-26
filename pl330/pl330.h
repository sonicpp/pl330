/* pl330.h
 *
 * Copyright (C) 2015 Jan Havran <xhavra13@stud.fit.vutbr.cz>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef	__AMBA_PL330_H__
#define	__AMBA_PL330_H__

#define LP_MAX	3

/*
 * Macros to set the bits in Channel Control Registers
 */
#define CC_SET_ENDIAN_SZ(VAL)		(((VAL) & 0x0F) << 28)
#define CC_SET_DST_CACHE_CTRL(VAL)	(((VAL) & 0X07) << 25)
#define CC_SET_DST_PROT_CTRL(VAL)	(((VAL) & 0X07) << 22)
#define CC_SET_DST_BURST_LEN(VAL)	(((VAL) & 0X0F) << 18)
#define CC_SET_DST_BURST_SIZE(VAL)	(((VAL) & 0X07) << 15)
#define CC_SET_DST_INC(VAL)		(((VAL) & 0X01) << 14)
#define CC_SET_SRC_CACHE_CTRL(VAL)	(((VAL) & 0X07) << 11)
#define CC_SET_SRC_PROT_CTRL(VAL)	(((VAL) & 0X07) << 8)
#define CC_SET_SRC_BURST_LEN(VAL)	(((VAL) & 0X0F) << 4)
#define CC_SET_SRC_BURST_SIZE(VAL)	(((VAL) & 0X07) << 1)
#define CC_SET_SRC_INC(VAL)		((VAL) & 0x01)

/**
 * enum pl330_lp_type - Type of loop.
 * @TYPE_LP:		Classic loop (DMALP).
 * @TYPE_FE:		Loop forever (DMALPFE).
 */
enum pl330_lp_type {
	TYPE_LP,
	TYPE_FE,
};

/**
 * struct pl330_lp - Information about loop.
 * @addr:		Address of loop instruction. Will be used to count
 *			relative location of the first instruction in
 *			the program loop.
 * @type:		Type of loop (DMALP or DMALPFE).
 */
struct pl330_lp {
	u8 *addr;
	enum pl330_lp_type type;
};

/**
 * struct pl330_prog - Stores information for pl330 program.
 * @lp_stack:		Stack for loops.
 * @buf:		Program buffer.
 * @lp_top:		Top of @lp_stack.
 */
struct pl330_prog {
	struct pl330_lp lp_stack[LP_MAX];
	u8 *buf;
	int lp_top;
};

/**
 * enum pl330_addh_dst - Destination register for DMAADDH instruction.
 * @PL330_SA:		Source Address Register as destination.
 * @PL330_DA:		Destination Address Register as destination.
 */
enum pl330_addh_dst {
	PL330_SA,
	PL330_DA,
};

/**
 * enum pl330_mov_dst - Destination register for DMAMOV instruction.
 * @PL330_SAR:		Source Address Register as destination.
 * @PL330_CCR:		Channel Control Register as destination.
 * @PL330_DAR:		Destination Address Register as destination.
 */
enum pl330_mov_dst {
	PL330_SAR,
	PL330_CCR,
	PL330_DAR,
};

/**
 * enum pl330_cond - Specifies condition for conditional instructions.
 * @PL330_SINGLE:	Conditional on single transfer.
 * @PL330_BURST:	Conditional on burst transfer.
 * @PL330_ALWAYS:	Non-conditional instruction.
 * @PL330_PERIPH:	Condition for DMAWFPP
 */
enum pl330_cond {
	PL330_SINGLE,
	PL330_BURST,
	PL330_ALWAYS,
	PL330_PERIPH,
};

/**
 * DMAADDH - Add @imm to @dst register.
 * @prog:	Struct containing program info.
 * @dst:	PL330_SA - selects Source Address Register.
 *		PL330_DA - selects Destination Address Register.
 * @imm:	Value to be added into @dst register.
 */
extern void DMAADDH(struct pl330_prog *prog, enum pl330_addh_dst dst, u16 imm);

/**
 * DMAEND - Signals to the DMAC that the DMA sequence is complete.
 * @prog:	Struct containing program info.
 */
extern void DMAEND(struct pl330_prog *prog);

/**
 * DMAFLUSHP - Flush Peripheral.
 * @prog:	Struct containing program info.
 * @periph:	Peripheral ID.
 */
extern void DMAFLUSHP(struct pl330_prog *prog, u8 periph);

/**
 * DMALD - DMA load.
 * @prog:	Struct containing program info.
 */
extern void DMALD(struct pl330_prog *prog);

/**
 * DMALDS - DMA conditional load on the request_flag=Single.
 * @prog:	Struct containing program info.
 */
extern void DMALDS(struct pl330_prog *prog);

/**
 * DMALDB - DMA conditional load on the request_flag=Burst.
 * @prog:	Struct containing program info.
 */
extern void DMALDB(struct pl330_prog *prog);

/**
 * DMALDPS - DMA conditioal load (on the request_flag=Single) and notify
 *		Peripheral.
 * @prog:	Struct containing program info.
 * @periph:	Peripheral ID.
 */
extern void DMALDPS(struct pl330_prog *prog, u8 periph);

/**
 * DMALDPB - DMA conditioal load (on the request_flag=Burst) and notify
 *		Peripheral.
 * @prog:	Struct containing program info.
 * @periph:	Peripheral ID.
 */
extern void DMALDPB(struct pl330_prog *prog, u8 periph);

/**
 * DMALP - DMA loop @iter times.
 * @prog:	Struct containing program info.
 * @iter:	Number of loops to perform.
 */
extern void DMALP(struct pl330_prog *prog, u8 iter);

/**
 * DMALPEND - DMA loop end.
 * @prog:	Struct containing program info.
 */
extern void DMALPEND(struct pl330_prog *prog);

/**
 * DMALPENDS - DMA conditional loop on the request_flag=Single.
 * @prog:	Struct containing program info.
 */
extern void DMALPENDS(struct pl330_prog *prog);

/**
 * DMALPENDB - DMA conditional loop on the request_flag=Burst.
 * @prog:	Struct containing program info.
 */
extern void DMALPENDB(struct pl330_prog *prog);

/**
 * DMALPFE - DMA loop forever (until signal drlast is set).
 * @prog:	Struct containing program info.
 */
extern void DMALPFE(struct pl330_prog *prog);

/**
 * DMAKILL - Kill thread.
 * @prog:	Struct containing program info.
 */
extern void DMAKILL(struct pl330_prog *prog);

/**
 * DMAMOV - Move @imm value to @dst register.
 * @prog:	Struct containing program info.
 * @dst:	PL330_SAR - selects Source Address Register.
 *		PL330_CCR - selects Channel Control Register.
 *		PL330_DAR - selects Destination Address Register.
 * @imm		Value that is written to the specified destination register.
 */
extern void DMAMOV(struct pl330_prog *prog, enum pl330_mov_dst dst, u32 imm);

/**
 * DMANOP - Do nothing.
 * @prog:	Struct containing program info.
 */
extern void DMANOP(struct pl330_prog *prog);

/**
 * DMARMB - Wait until all active AXI read transactions are complete.
 * @prog:	Struct containing program info.
 */
extern void DMARMB(struct pl330_prog *prog);

/**
 * DMASEV - Send @event_num event.
 * @prog:	Struct containing program info.
 * @event_num:	Event number.
 */
extern void DMASEV(struct pl330_prog *prog, u8 event_num);

/**
 * DMAST - DMA store.
 * @prog:	Struct containing program info.
 */
extern void DMAST(struct pl330_prog *prog);

/**
 * DMASTS - DMA conditional store on the request_flag=Single.
 * @prog:	Struct containing program info.
 */
extern void DMASTS(struct pl330_prog *prog);

/**
 * DMASTB - DMA conditional store on the request_flag=Burst.
 * @prog:	Struct containing program info.
 */
extern void DMASTB(struct pl330_prog *prog);

/**
 * DMASTPS - DMA conditioal store (on the request_flag=Single) and notify
 *		Peripheral.
 * @prog:	Struct containing program info.
 * @periph:	Peripheral ID.
 */
extern void DMASTPS(struct pl330_prog *prog, u8 periph);

/**
 * DMASTPB - DMA conditioal store (on the request_flag=Burst) and notify
 *		Peripheral.
 * @prog:	Struct containing program info.
 * @periph:	Peripheral ID.
 */
extern void DMASTPB(struct pl330_prog *prog, u8 periph);

/**
 * DMAST - DMA store zero value.
 * @prog:	Struct containing program info.
 */
extern void DMASTZ(struct pl330_prog *prog);

/**
 * DMAWFE - Wait for event @event_num.
 * @prog:	Struct containing program info.
 * @event_num:	Event number.
 * @invalid:	Invalidates the instruction cache.
 */
extern void DMAWFE(struct pl330_prog *prog, u8 event_num, bool invalid);

/**
 * DMAWFPS - Wait until until the @periph signals single or burst request.
 *		The DMAC sets the request_flag to single.
 * @prog:	Struct containing program info.
 * @periph:	Peripheral ID.
 */
extern void DMAWFPS(struct pl330_prog *prog, u8 periph);

/**
 * DMAWFPS - Wait until until the @periph signals burst request.
 *		The DMAC sets the request_flag to burst.
 * @prog:	Struct containing program info.
 * @periph:	Peripheral ID.
 */
extern void DMAWFPB(struct pl330_prog *prog, u8 periph);

/**
 * DMAWFPS - Wait until until the @periph signals single or burst request.
 *		The DMAC sets the request_flag to request type (single
 *		or burst).
 * @prog:	Struct containing program info.
 * @periph:	Peripheral ID.
 */
extern void DMAWFPP(struct pl330_prog *prog, u8 periph);

/**
 * DMAWMB - Wait until all active AXI write transactions are complete.
 * @prog:	Struct containing program info.
 */
extern void DMAWMB(struct pl330_prog *prog);

/**
 * pl330_init_prog - init struct @prog.
 * @prog:	Struct containing information for PL330 program.
 * @buf:	Program buffer address.
 */
extern void pl330_init_prog(struct pl330_prog *prog, u8 *buf);

/**
 * pl330_prep_program_addr - Prepare transfer for custom PL330 program.
 * @chan:		Channel which will transfer data.
 * @start_address:	Bus address of custom program.
 *
 * Returns descriptor on success, NULL otherwise.
 */
extern struct dma_async_tx_descriptor *pl330_prep_program_addr(
	struct dma_chan *chan, dma_addr_t start_address);

/**
 * pl330_filter_peri - filter function for dma_request_channel. This function
 *			filter channels by peripheral request interface.
 * @chan:	Channel to be filtered.
 * @param:	Pointer to bool value:
 *		true - filter channels WITH PRI.
 *		false - filter channels WITHOUT PRI.
 * Returns true when @chan's periph is equal to @param, false otherwise.
 */
extern bool pl330_filter_peri(struct dma_chan *chan, void *param);

/**
 * pl330_filter_chan_id - filter function for dma_request_channel. This function
 *			filter channels by peripheral channel ID.
 * @chan:	Channel to be filtered.
 * @param:	Pointer to int value representing channel ID.
 *
 * Returns true when @chan's ID is equal to @param, false otherwise.
 */
extern bool pl330_filter_chan_id(struct dma_chan *chan, void *param);

/**
 * pl330_get_channel_id - get channel ID.
 * @chan:	Required channel.
 *
 * Return @chan's ID on success, -1 otherwise.
 */
extern int pl330_get_channel_id(struct dma_chan *chan);

/**
 * pl330_has_channel_pri - check support of Peripheral request interface.
 * @chan:	Required channel.
 *
 * Returns true in case of supported peripheral interface, false otherwise.
 */
extern bool pl330_has_channel_pri(struct dma_chan *chan);

#endif	/* __AMBA_PL330_H__ */
