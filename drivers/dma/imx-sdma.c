// SPDX-License-Identifier: GPL-2.0+
//
// drivers/dma/imx-sdma.c
//
// This file contains a driver for the Freescale Smart DMA engine
//
// Copyright 2010 Sascha Hauer, Pengutronix <s.hauer@pengutronix.de>
//
// Based on code from Freescale:
//
// Copyright 2004-2009 Freescale Semiconductor, Inc. All Rights Reserved.

#include <linux/init.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/genalloc.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/dmaengine.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/workqueue.h>

#include <asm/irq.h>
#include <linux/dma/imx-dma.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>

#include "dmaengine.h"
#include "virt-dma.h"

/* SDMA registers */
#define SDMA_H_C0PTR		0x000
#define SDMA_H_INTR		0x004
#define SDMA_H_STATSTOP		0x008
#define SDMA_H_START		0x00c
#define SDMA_H_EVTOVR		0x010
#define SDMA_H_DSPOVR		0x014
#define SDMA_H_HOSTOVR		0x018
#define SDMA_H_EVTPEND		0x01c
#define SDMA_H_DSPENBL		0x020
#define SDMA_H_RESET		0x024
#define SDMA_H_EVTERR		0x028
#define SDMA_H_INTRMSK		0x02c
#define SDMA_H_PSW		0x030
#define SDMA_H_EVTERRDBG	0x034
#define SDMA_H_CONFIG		0x038
#define SDMA_ONCE_ENB		0x040
#define SDMA_ONCE_DATA		0x044
#define SDMA_ONCE_INSTR		0x048
#define SDMA_ONCE_STAT		0x04c
#define SDMA_ONCE_CMD		0x050
#define SDMA_EVT_MIRROR		0x054
#define SDMA_ILLINSTADDR	0x058
#define SDMA_CHN0ADDR		0x05c
#define SDMA_ONCE_RTB		0x060
#define SDMA_XTRIG_CONF1	0x070
#define SDMA_XTRIG_CONF2	0x074
#define SDMA_CHNENBL0_IMX35	0x200
#define SDMA_CHNENBL0_IMX31	0x080
#define SDMA_CHNPRI_0		0x100
#define SDMA_DONE0_CONFIG	0x1000

/*
 * Buffer descriptor status values.
 */
#define BD_DONE  0x01
#define BD_WRAP  0x02
#define BD_CONT  0x04
#define BD_INTR  0x08
#define BD_RROR  0x10
#define BD_LAST  0x20
#define BD_EXTD  0x80

/*
 * Data Node descriptor status values.
 */
#define DND_END_OF_FRAME  0x80
#define DND_END_OF_XFER   0x40
#define DND_DONE          0x20
#define DND_UNUSED        0x01

/*
 * IPCV2 descriptor status values.
 */
#define BD_IPCV2_END_OF_FRAME  0x40

#define IPCV2_MAX_NODES        50
/*
 * Error bit set in the CCB status field by the SDMA,
 * in setbd routine, in case of a transfer error
 */
#define DATA_ERROR  0x10000000

/*
 * Buffer descriptor commands.
 */
#define C0_ADDR             0x01
#define C0_LOAD             0x02
#define C0_DUMP             0x03
#define C0_SETCTX           0x07
#define C0_GETCTX           0x03
#define C0_SETDM            0x01
#define C0_SETPM            0x04
#define C0_GETDM            0x02
#define C0_GETPM            0x08
/*
 * Change endianness indicator in the BD command field
 */
#define CHANGE_ENDIANNESS   0x80

/*
 *  p_2_p watermark_level description
 *	Bits		Name			Description
 *	0-7		Lower WML		Lower watermark level
 *	8		PS			1: Pad Swallowing
 *						0: No Pad Swallowing
 *	9		PA			1: Pad Adding
 *						0: No Pad Adding
 *	10		SPDIF			If this bit is set both source
 *						and destination are on SPBA
 *	11		Source Bit(SP)		1: Source on SPBA
 *						0: Source on AIPS
 *	12		Destination Bit(DP)	1: Destination on SPBA
 *						0: Destination on AIPS
 *	13-15		---------		MUST BE 0
 *	16-23		Higher WML		HWML
 *	24-27		N			Total number of samples after
 *						which Pad adding/Swallowing
 *						must be done. It must be odd.
 *	28		Lower WML Event(LWE)	SDMA events reg to check for
 *						LWML event mask
 *						0: LWE in EVENTS register
 *						1: LWE in EVENTS2 register
 *	29		Higher WML Event(HWE)	SDMA events reg to check for
 *						HWML event mask
 *						0: HWE in EVENTS register
 *						1: HWE in EVENTS2 register
 *	30		---------		MUST BE 0
 *	31		CONT			1: Amount of samples to be
 *						transferred is unknown and
 *						script will keep on
 *						transferring samples as long as
 *						both events are detected and
 *						script must be manually stopped
 *						by the application
 *						0: The amount of samples to be
 *						transferred is equal to the
 *						count field of mode word
 */
#define SDMA_WATERMARK_LEVEL_LWML	0xFF
#define SDMA_WATERMARK_LEVEL_PS		BIT(8)
#define SDMA_WATERMARK_LEVEL_PA		BIT(9)
#define SDMA_WATERMARK_LEVEL_SPDIF	BIT(10)
#define SDMA_WATERMARK_LEVEL_SP		BIT(11)
#define SDMA_WATERMARK_LEVEL_DP		BIT(12)
#define SDMA_WATERMARK_LEVEL_SD		BIT(13)
#define SDMA_WATERMARK_LEVEL_DD		BIT(14)
#define SDMA_WATERMARK_LEVEL_HWML	(0xFF << 16)
#define SDMA_WATERMARK_LEVEL_LWE	BIT(28)
#define SDMA_WATERMARK_LEVEL_HWE	BIT(29)
#define SDMA_WATERMARK_LEVEL_CONT	BIT(31)

#define SDMA_DMA_BUSWIDTHS	(BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) | \
				 BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) | \
				 BIT(DMA_SLAVE_BUSWIDTH_3_BYTES) | \
				 BIT(DMA_SLAVE_BUSWIDTH_4_BYTES))

#define SDMA_DMA_DIRECTIONS	(BIT(DMA_DEV_TO_MEM) | \
				 BIT(DMA_MEM_TO_DEV) | \
				 BIT(DMA_DEV_TO_DEV))

#define SDMA_WATERMARK_LEVEL_N_FIFOS	GENMASK(15, 12)
#define SDMA_WATERMARK_LEVEL_OFF_FIFOS  GENMASK(19, 16)
#define SDMA_WATERMARK_LEVEL_WORDS_PER_FIFO   GENMASK(31, 28)
#define SDMA_WATERMARK_LEVEL_SW_DONE	BIT(23)
#define SDMA_WATERMARK_LEVEL_SW_DONE_SEL_OFF 24

#define SDMA_DONE0_CONFIG_DONE_SEL	0x7
#define SDMA_DONE0_CONFIG_DONE_DIS	0x6

/**
 * struct sdma_script_start_addrs - SDMA script start pointers
 *
 * start addresses of the different functions in the physical
 * address space of the SDMA engine.
 */
struct sdma_script_start_addrs {
	s32 ap_2_ap_addr;
	s32 ap_2_bp_addr;
	s32 ap_2_ap_fixed_addr;
	s32 bp_2_ap_addr;
	s32 loopback_on_dsp_side_addr;
	s32 mcu_interrupt_only_addr;
	s32 firi_2_per_addr;
	s32 firi_2_mcu_addr;
	s32 per_2_firi_addr;
	s32 mcu_2_firi_addr;
	s32 uart_2_per_addr;
	s32 uart_2_mcu_addr;
	s32 per_2_app_addr;
	s32 mcu_2_app_addr;
	s32 per_2_per_addr;
	s32 uartsh_2_per_addr;
	s32 uartsh_2_mcu_addr;
	s32 per_2_shp_addr;
	s32 mcu_2_shp_addr;
	s32 ata_2_mcu_addr;
	s32 mcu_2_ata_addr;
	s32 app_2_per_addr;
	s32 app_2_mcu_addr;
	s32 shp_2_per_addr;
	s32 shp_2_mcu_addr;
	s32 mshc_2_mcu_addr;
	s32 mcu_2_mshc_addr;
	s32 spdif_2_mcu_addr;
	s32 mcu_2_spdif_addr;
	s32 asrc_2_mcu_addr;
	s32 ext_mem_2_ipu_addr;
	s32 descrambler_addr;
	s32 dptc_dvfs_addr;
	s32 utra_addr;
	s32 ram_code_start_addr;
	/* End of v1 array */
	s32 mcu_2_ssish_addr;
	s32 ssish_2_mcu_addr;
	s32 hdmi_dma_addr;
	/* End of v2 array */
	s32 zcanfd_2_mcu_addr;
	s32 zqspi_2_mcu_addr;
	s32 mcu_2_ecspi_addr;
	s32 mcu_2_sai_addr;
	s32 sai_2_mcu_addr;
	s32 uart_2_mcu_rom_addr;
	s32 uartsh_2_mcu_rom_addr;
	s32 i2c_2_mcu_addr;
	s32 mcu_2_i2c_addr;
	/* End of v3 array */
	s32 mcu_2_zqspi_addr;
	/* End of v4 array */
};

/*
 * Mode/Count of data node descriptors - IPCv2
 */
struct sdma_mode_count {
#define SDMA_BD_MAX_CNT	0xffff
	u32 count   : 16; /* size of the buffer pointed by this BD */
	u32 status  :  8; /* E,R,I,C,W,D status bits stored here */
	u32 command :  8; /* command mostly used for channel 0 */
};

/*
 * Buffer descriptor
 */
struct sdma_buffer_descriptor {
	struct sdma_mode_count  mode;
	u32 buffer_addr;	/* address of the buffer described */
	u32 ext_buffer_addr;	/* extended buffer address */
} __attribute__ ((packed));

/**
 * struct sdma_channel_control - Channel control Block
 *
 * @current_bd_ptr:	current buffer descriptor processed
 * @base_bd_ptr:	first element of buffer descriptor array
 * @unused:		padding. The SDMA engine expects an array of 128 byte
 *			control blocks
 */
struct sdma_channel_control {
	u32 current_bd_ptr;
	u32 base_bd_ptr;
	u32 unused[2];
} __attribute__ ((packed));

/**
 * struct sdma_state_registers - SDMA context for a channel
 *
 * @pc:		program counter
 * @unused1:	unused
 * @t:		test bit: status of arithmetic & test instruction
 * @rpc:	return program counter
 * @unused0:	unused
 * @sf:		source fault while loading data
 * @spc:	loop start program counter
 * @unused2:	unused
 * @df:		destination fault while storing data
 * @epc:	loop end program counter
 * @lm:		loop mode
 */
struct sdma_state_registers {
	u32 pc     :14;
	u32 unused1: 1;
	u32 t      : 1;
	u32 rpc    :14;
	u32 unused0: 1;
	u32 sf     : 1;
	u32 spc    :14;
	u32 unused2: 1;
	u32 df     : 1;
	u32 epc    :14;
	u32 lm     : 2;
} __attribute__ ((packed));

/**
 * struct sdma_context_data - sdma context specific to a channel
 *
 * @channel_state:	channel state bits
 * @gReg:		general registers
 * @mda:		burst dma destination address register
 * @msa:		burst dma source address register
 * @ms:			burst dma status register
 * @md:			burst dma data register
 * @pda:		peripheral dma destination address register
 * @psa:		peripheral dma source address register
 * @ps:			peripheral dma status register
 * @pd:			peripheral dma data register
 * @ca:			CRC polynomial register
 * @cs:			CRC accumulator register
 * @dda:		dedicated core destination address register
 * @dsa:		dedicated core source address register
 * @ds:			dedicated core status register
 * @dd:			dedicated core data register
 * @scratch0:		1st word of dedicated ram for context switch
 * @scratch1:		2nd word of dedicated ram for context switch
 * @scratch2:		3rd word of dedicated ram for context switch
 * @scratch3:		4th word of dedicated ram for context switch
 * @scratch4:		5th word of dedicated ram for context switch
 * @scratch5:		6th word of dedicated ram for context switch
 * @scratch6:		7th word of dedicated ram for context switch
 * @scratch7:		8th word of dedicated ram for context switch
 */
struct sdma_context_data {
	struct sdma_state_registers  channel_state;
	u32  gReg[8];
	u32  mda;
	u32  msa;
	u32  ms;
	u32  md;
	u32  pda;
	u32  psa;
	u32  ps;
	u32  pd;
	u32  ca;
	u32  cs;
	u32  dda;
	u32  dsa;
	u32  ds;
	u32  dd;
	u32  scratch0;
	u32  scratch1;
	u32  scratch2;
	u32  scratch3;
	u32  scratch4;
	u32  scratch5;
	u32  scratch6;
	u32  scratch7;
} __attribute__ ((packed));


struct sdma_engine;

/**
 * struct sdma_desc - descriptor structor for one transfer
 * @vd:			descriptor for virt dma
 * @num_bd:		number of descriptors currently handling
 * @bd_phys:		physical address of bd
 * @buf_tail:		ID of the buffer that was processed
 * @buf_ptail:		ID of the previous buffer that was processed
 * @period_len:		period length, used in cyclic.
 * @chn_real_count:	the real count updated from bd->mode.count
 * @chn_count:		the transfer count set
 * @sdmac:		sdma_channel pointer
 * @bd:			pointer of allocate bd
 */
struct sdma_desc {
	struct virt_dma_desc	vd;
	unsigned int		num_bd;
	dma_addr_t		bd_phys;
	unsigned int		buf_tail;
	unsigned int		buf_ptail;
	unsigned int		period_len;
	unsigned int		chn_real_count;
	unsigned int		chn_count;
	struct sdma_channel	*sdmac;
	struct sdma_buffer_descriptor *bd;
};

/**
 * struct sdma_channel - housekeeping for a SDMA channel
 *
 * @vc:			virt_dma base structure
 * @desc:		sdma description including vd and other special member
 * @sdma:		pointer to the SDMA engine for this channel
 * @channel:		the channel number, matches dmaengine chan_id + 1
 * @direction:		transfer type. Needed for setting SDMA script
 * @slave_config:	Slave configuration
 * @peripheral_type:	Peripheral type. Needed for setting SDMA script
 * @event_id0:		aka dma request line
 * @event_id1:		for channels that use 2 events
 * @word_size:		peripheral access size
 * @pc_from_device:	script address for those device_2_memory
 * @pc_to_device:	script address for those memory_2_device
 * @device_to_device:	script address for those device_2_device
 * @pc_to_pc:		script address for those memory_2_memory
 * @flags:		loop mode or not
 * @per_address:	peripheral source or destination address in common case
 *                      destination address in p_2_p case
 * @per_address2:	peripheral source address in p_2_p case
 * @event_mask:		event mask used in p_2_p script
 * @watermark_level:	value for gReg[7], some script will extend it from
 *			basic watermark such as p_2_p
 * @shp_addr:		value for gReg[6]
 * @per_addr:		value for gReg[2]
 * @status:		status of dma channel
 * @context_loaded:	ensure context is only loaded once
 * @data:		specific sdma interface structure
 * @bd_pool:		dma_pool for bd
 * @terminate_worker:	used to call back into terminate work function
 * @terminated:		terminated list
 * @is_ram_script:	flag for script in ram
 * @n_fifos_src:	number of source device fifos
 * @n_fifos_dst:	number of destination device fifos
 * @stride_fifos_src:	stride for source device FIFOs
 * @stride_fifos_dst:	stride for destination device FIFOs
 * @words_per_fifo:	copy number of words one time for one FIFO
 * @sw_done:		software done flag
 */
struct sdma_channel {
	struct virt_dma_chan		vc;
	struct sdma_desc		*desc;
	struct sdma_engine		*sdma;
	unsigned int			channel;
	enum dma_transfer_direction	direction;
	struct dma_slave_config		slave_config;
	struct sdma_peripheral_config	*audio_config;
	enum sdma_peripheral_type	peripheral_type;
	unsigned int			event_id0;
	unsigned int			event_id1;
	enum dma_slave_buswidth		word_size;
	unsigned int			pc_from_device, pc_to_device;
	unsigned int			device_to_device;
	unsigned int                    pc_to_pc;
	unsigned long			flags;
	dma_addr_t			per_address, per_address2;
	unsigned long			event_mask[2];
	unsigned long			watermark_level;
	u32				shp_addr, per_addr;
	enum dma_status			status;
	struct imx_dma_data		data;
	struct work_struct		terminate_worker;
	struct list_head                terminated;
	bool				is_ram_script;
	int				prio;
	unsigned int                    n_fifos_src;
	unsigned int                    n_fifos_dst;
	unsigned int                    stride_fifos_src;
	unsigned int                    stride_fifos_dst;
	unsigned int                    words_per_fifo;
	bool                            sw_done;
};

#define IMX_DMA_SG_LOOP		BIT(0)

#define MAX_DMA_CHANNELS 32
#define MXC_SDMA_DEFAULT_PRIORITY 1
#define MXC_SDMA_MIN_PRIORITY 1
#define MXC_SDMA_MAX_PRIORITY 7

/*
 * 0x78(SDMA_XTRIG_CONF2+4)~0x100(SDMA_CHNPRI_O) registers are reserved and
 * can't be accessed. Skip these register touch in suspend/resume. Also below
 * two macros are only used on i.mx6sx.
 */
#define MXC_SDMA_RESERVED_REG (SDMA_CHNPRI_0 - SDMA_XTRIG_CONF2 - 4)
#define MXC_SDMA_SAVED_REG_NUM (((SDMA_CHNENBL0_IMX35 + 4 * 48) - \
				MXC_SDMA_RESERVED_REG) / 4)

#define SDMA_FIRMWARE_MAGIC 0x414d4453

/**
 * struct sdma_firmware_header - Layout of the firmware image
 *
 * @magic:		"SDMA"
 * @version_major:	increased whenever layout of struct
 *			sdma_script_start_addrs changes.
 * @version_minor:	firmware minor version (for binary compatible changes)
 * @script_addrs_start:	offset of struct sdma_script_start_addrs in this image
 * @num_script_addrs:	Number of script addresses in this image
 * @ram_code_start:	offset of SDMA ram image in this firmware image
 * @ram_code_size:	size of SDMA ram image
 * @script_addrs:	Stores the start address of the SDMA scripts
 *			(in SDMA memory space)
 */
struct sdma_firmware_header {
	u32	magic;
	u32	version_major;
	u32	version_minor;
	u32	script_addrs_start;
	u32	num_script_addrs;
	u32	ram_code_start;
	u32	ram_code_size;
};

struct sdma_driver_data {
	int chnenbl0;
	int num_events;
	struct sdma_script_start_addrs	*script_addrs;
	bool check_ratio;
	/*
	 * ecspi ERR009165 fixed should be done in sdma script
	 * and it has been fixed in soc from i.mx6ul.
	 * please get more information from the below link:
	 * https://www.nxp.com/docs/en/errata/IMX6DQCE.pdf
	 */
	bool ecspi_fixed;
	bool has_done0;
	bool pm_runtime;
};

struct sdma_engine {
	struct device			*dev;
	struct sdma_channel		channel[MAX_DMA_CHANNELS];
	struct sdma_channel_control	*channel_control;
	u32				save_regs[MXC_SDMA_SAVED_REG_NUM];
	u32				save_done0_regs[2];
	const char			*fw_name;
	void __iomem			*regs;
	struct sdma_context_data	*context;
	dma_addr_t			context_phys;
	dma_addr_t			ccb_phys;
	bool				is_on;
	struct dma_device		dma_device;
	struct clk			*clk_ipg;
	struct clk			*clk_ahb;
	spinlock_t			channel_0_lock;
	u32				script_number;
	struct sdma_script_start_addrs	*script_addrs;
	const struct sdma_driver_data	*drvdata;
	u32				spba_start_addr;
	u32				spba_end_addr;
	unsigned int			irq;
	dma_addr_t			bd0_phys;
	struct sdma_buffer_descriptor	*bd0;
	/* clock ratio for AHB:SDMA core. 1:1 is 1, 2:1 is 0*/
	bool				clk_ratio;
	bool                            fw_loaded;
	struct gen_pool			*iram_pool;
	u32				fw_fail;
	u8				*fw_data;
	unsigned short			ram_code_start;
};

static int sdma_config_write(struct dma_chan *chan,
		       struct dma_slave_config *dmaengine_cfg,
		       enum dma_transfer_direction direction);

static struct sdma_driver_data sdma_imx31 = {
	.chnenbl0 = SDMA_CHNENBL0_IMX31,
	.num_events = 32,
};

static struct sdma_script_start_addrs sdma_script_imx25 = {
	.ap_2_ap_addr = 729,
	.uart_2_mcu_addr = 904,
	.per_2_app_addr = 1255,
	.mcu_2_app_addr = 834,
	.uartsh_2_mcu_addr = 1120,
	.per_2_shp_addr = 1329,
	.mcu_2_shp_addr = 1048,
	.ata_2_mcu_addr = 1560,
	.mcu_2_ata_addr = 1479,
	.app_2_per_addr = 1189,
	.app_2_mcu_addr = 770,
	.shp_2_per_addr = 1407,
	.shp_2_mcu_addr = 979,
};

static struct sdma_driver_data sdma_imx25 = {
	.chnenbl0 = SDMA_CHNENBL0_IMX35,
	.num_events = 48,
	.script_addrs = &sdma_script_imx25,
};

static struct sdma_driver_data sdma_imx35 = {
	.chnenbl0 = SDMA_CHNENBL0_IMX35,
	.num_events = 48,
};

static struct sdma_script_start_addrs sdma_script_imx51 = {
	.ap_2_ap_addr = 642,
	.uart_2_mcu_addr = 817,
	.mcu_2_app_addr = 747,
	.mcu_2_shp_addr = 961,
	.ata_2_mcu_addr = 1473,
	.mcu_2_ata_addr = 1392,
	.app_2_per_addr = 1033,
	.app_2_mcu_addr = 683,
	.shp_2_per_addr = 1251,
	.shp_2_mcu_addr = 892,
};

static struct sdma_driver_data sdma_imx51 = {
	.chnenbl0 = SDMA_CHNENBL0_IMX35,
	.num_events = 48,
	.script_addrs = &sdma_script_imx51,
};

static struct sdma_script_start_addrs sdma_script_imx53 = {
	.ap_2_ap_addr = 642,
	.app_2_mcu_addr = 683,
	.mcu_2_app_addr = 747,
	.uart_2_mcu_addr = 817,
	.shp_2_mcu_addr = 891,
	.mcu_2_shp_addr = 960,
	.uartsh_2_mcu_addr = 1032,
	.spdif_2_mcu_addr = 1100,
	.mcu_2_spdif_addr = 1134,
	.firi_2_mcu_addr = 1193,
	.mcu_2_firi_addr = 1290,
};

static struct sdma_driver_data sdma_imx53 = {
	.chnenbl0 = SDMA_CHNENBL0_IMX35,
	.num_events = 48,
	.script_addrs = &sdma_script_imx53,
};

static struct sdma_script_start_addrs sdma_script_imx6q = {
	.ap_2_ap_addr = 642,
	.uart_2_mcu_addr = 817,
	.mcu_2_app_addr = 747,
	.per_2_per_addr = 6331,
	.uartsh_2_mcu_addr = 1032,
	.mcu_2_shp_addr = 960,
	.app_2_mcu_addr = 683,
	.shp_2_mcu_addr = 891,
	.spdif_2_mcu_addr = 1100,
	.mcu_2_spdif_addr = 1134,
};

static struct sdma_driver_data sdma_imx6q = {
	.chnenbl0 = SDMA_CHNENBL0_IMX35,
	.num_events = 48,
	.script_addrs = &sdma_script_imx6q,
};

static struct sdma_driver_data sdma_imx6sx = {
	.chnenbl0 = SDMA_CHNENBL0_IMX35,
	.num_events = 48,
	.script_addrs = &sdma_script_imx6q,
};

static struct sdma_driver_data sdma_imx6ul = {
	.chnenbl0 = SDMA_CHNENBL0_IMX35,
	.num_events = 48,
	.script_addrs = &sdma_script_imx6q,
	.ecspi_fixed = true,
};

static struct sdma_script_start_addrs sdma_script_imx7d = {
	.ap_2_ap_addr = 644,
	.uart_2_mcu_addr = 819,
	.mcu_2_app_addr = 749,
	.uartsh_2_mcu_addr = 1034,
	.mcu_2_shp_addr = 962,
	.app_2_mcu_addr = 685,
	.shp_2_mcu_addr = 893,
	.spdif_2_mcu_addr = 1102,
	.mcu_2_spdif_addr = 1136,
};

static struct sdma_driver_data sdma_imx7d = {
	.chnenbl0 = SDMA_CHNENBL0_IMX35,
	.num_events = 48,
	.script_addrs = &sdma_script_imx7d,
};

static struct sdma_driver_data sdma_imx8mq = {
	.chnenbl0 = SDMA_CHNENBL0_IMX35,
	.num_events = 48,
	.script_addrs = &sdma_script_imx7d,
	.check_ratio = 1,
};

static struct sdma_driver_data sdma_imx8mp = {
	.chnenbl0 = SDMA_CHNENBL0_IMX35,
	.num_events = 48,
	.script_addrs = &sdma_script_imx7d,
	.check_ratio = 1,
	.ecspi_fixed = true,
	.has_done0 = true,
	.pm_runtime = true,
};

static const struct of_device_id sdma_dt_ids[] = {
	{ .compatible = "fsl,imx6q-sdma", .data = &sdma_imx6q, },
	{ .compatible = "fsl,imx53-sdma", .data = &sdma_imx53, },
	{ .compatible = "fsl,imx51-sdma", .data = &sdma_imx51, },
	{ .compatible = "fsl,imx35-sdma", .data = &sdma_imx35, },
	{ .compatible = "fsl,imx31-sdma", .data = &sdma_imx31, },
	{ .compatible = "fsl,imx25-sdma", .data = &sdma_imx25, },
	{ .compatible = "fsl,imx7d-sdma", .data = &sdma_imx7d, },
	{ .compatible = "fsl,imx6sx-sdma", .data = &sdma_imx6sx, },
	{ .compatible = "fsl,imx6ul-sdma", .data = &sdma_imx6ul, },
	{ .compatible = "fsl,imx8mq-sdma", .data = &sdma_imx8mq, },
	{ .compatible = "fsl,imx8mp-sdma", .data = &sdma_imx8mp, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sdma_dt_ids);

#define SDMA_H_CONFIG_DSPDMA	BIT(12) /* indicates if the DSPDMA is used */
#define SDMA_H_CONFIG_RTD_PINS	BIT(11) /* indicates if Real-Time Debug pins are enabled */
#define SDMA_H_CONFIG_ACR	BIT(4)  /* indicates if AHB freq /core freq = 2 or 1 */
#define SDMA_H_CONFIG_CSM	(3)       /* indicates which context switch mode is selected*/

static void sdma_pm_clk_enable(struct sdma_engine *sdma, bool direct, bool enable)
{
	if (enable) {
		if (sdma->drvdata->pm_runtime)
			pm_runtime_get_sync(sdma->dev);
		else {
			clk_enable(sdma->clk_ipg);
			clk_enable(sdma->clk_ahb);
		}
	} else {
		if (sdma->drvdata->pm_runtime) {
			if (direct) {
				pm_runtime_put_sync_suspend(sdma->dev);
			} else {
				pm_runtime_mark_last_busy(sdma->dev);
				pm_runtime_put_autosuspend(sdma->dev);
			}
		} else {
			clk_disable(sdma->clk_ipg);
			clk_disable(sdma->clk_ahb);
		}
	}
}

static inline u32 chnenbl_ofs(struct sdma_engine *sdma, unsigned int event)
{
	u32 chnenbl0 = sdma->drvdata->chnenbl0;
	return chnenbl0 + event * 4;
}

static int sdma_config_ownership(struct sdma_channel *sdmac,
		bool event_override, bool mcu_override, bool dsp_override)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;
	unsigned long evt, mcu, dsp;

	if (event_override && mcu_override && dsp_override)
		return -EINVAL;

	evt = readl_relaxed(sdma->regs + SDMA_H_EVTOVR);
	mcu = readl_relaxed(sdma->regs + SDMA_H_HOSTOVR);
	dsp = readl_relaxed(sdma->regs + SDMA_H_DSPOVR);

	if (dsp_override)
		__clear_bit(channel, &dsp);
	else
		__set_bit(channel, &dsp);

	if (event_override)
		__clear_bit(channel, &evt);
	else
		__set_bit(channel, &evt);

	if (mcu_override)
		__clear_bit(channel, &mcu);
	else
		__set_bit(channel, &mcu);

	writel_relaxed(evt, sdma->regs + SDMA_H_EVTOVR);
	writel_relaxed(mcu, sdma->regs + SDMA_H_HOSTOVR);
	writel_relaxed(dsp, sdma->regs + SDMA_H_DSPOVR);

	return 0;
}

static int is_sdma_channel_enabled(struct sdma_engine *sdma, int channel)
{
	return !!(readl(sdma->regs + SDMA_H_STATSTOP) & BIT(channel));
}

static void sdma_enable_channel(struct sdma_engine *sdma, int channel)
{
	writel(BIT(channel), sdma->regs + SDMA_H_START);
}

/*
 * sdma_run_channel0 - run a channel and wait till it's done
 */
static int sdma_run_channel0(struct sdma_engine *sdma)
{
	int ret;
	u32 reg;

	sdma_enable_channel(sdma, 0);
	/* enlarge timeout to 5ms for safe since many highest channel
	 * also may run at the same time like asrc/easrc tough test.
	 */
	ret = readl_relaxed_poll_timeout_atomic(sdma->regs + SDMA_H_STATSTOP,
						reg, !(reg & 1), 1, 5000);
	if (ret)
		dev_err(sdma->dev, "Timeout waiting for CH0 ready\n");

	/* Set bits of CONFIG register with dynamic context switching */
	reg = readl(sdma->regs + SDMA_H_CONFIG);
	if ((reg & SDMA_H_CONFIG_CSM) == 0) {
		reg |= SDMA_H_CONFIG_CSM;
		writel_relaxed(reg, sdma->regs + SDMA_H_CONFIG);
	}

	return ret;
}


#define SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V1	34
#define SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V2	38
#define SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V3	47
#define SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V4	48

static void sdma_add_scripts(struct sdma_engine *sdma,
			     const struct sdma_script_start_addrs *addr)
{
	s32 *addr_arr = (u32 *)addr;
	s32 *saddr_arr = (u32 *)sdma->script_addrs;
	int i;

	/* use the default firmware in ROM if missing external firmware */
	if (!sdma->script_number)
		sdma->script_number = SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V1;

	if (sdma->script_number > sizeof(struct sdma_script_start_addrs)
				  / sizeof(s32)) {
		dev_err(sdma->dev,
			"SDMA script number %d not match with firmware.\n",
			sdma->script_number);
		return;
	}

	for (i = 0; i < sdma->script_number; i++)
		if (addr_arr[i] > 0)
			saddr_arr[i] = addr_arr[i];

	/*
	 * For compatibility with NXP internal legacy kernel before 4.19 which
	 * is based on uart ram script and mainline kernel based on uart rom
	 * script, both uart ram/rom scripts are present in newer sdma
	 * firmware. Use the rom versions if they are present (V3 or newer).
	 */
	if (sdma->script_number >= SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V3) {
		if (addr->uart_2_mcu_rom_addr)
			sdma->script_addrs->uart_2_mcu_addr = addr->uart_2_mcu_rom_addr;
		if (addr->uartsh_2_mcu_rom_addr)
			sdma->script_addrs->uartsh_2_mcu_addr = addr->uartsh_2_mcu_rom_addr;
	}
}

static int sdma_load_script(struct sdma_engine *sdma)
{
	struct sdma_buffer_descriptor *bd0 = sdma->bd0;
	const struct sdma_script_start_addrs *addr;
	struct sdma_firmware_header *header;
	unsigned short *ram_code;
	void *buf_virt;
	dma_addr_t buf_phys;
	int ret;
	unsigned long flags;

	header = (struct sdma_firmware_header *)sdma->fw_data;

	addr = (void *)header + header->script_addrs_start;
	ram_code = (void *)header + header->ram_code_start;
	sdma->ram_code_start = header->ram_code_start;

	buf_virt = dma_alloc_coherent(sdma->dev, header->ram_code_size,
				      &buf_phys, GFP_KERNEL);
	if (!buf_virt)
		return -ENOMEM;

	spin_lock_irqsave(&sdma->channel_0_lock, flags);

	bd0->mode.command = C0_SETPM;
	bd0->mode.status = BD_DONE | BD_WRAP | BD_EXTD;
	bd0->mode.count = header->ram_code_size / 2;
	bd0->buffer_addr = buf_phys;
	bd0->ext_buffer_addr = addr->ram_code_start_addr;

	memcpy(buf_virt, ram_code, header->ram_code_size);

	ret = sdma_run_channel0(sdma);

	spin_unlock_irqrestore(&sdma->channel_0_lock, flags);

	dma_free_coherent(sdma->dev, header->ram_code_size, buf_virt, buf_phys);

	sdma_add_scripts(sdma, addr);

	sdma->fw_loaded = true;

	dev_info_once(sdma->dev, "loaded firmware %d.%d\n",
			header->version_major,
			header->version_minor);
	return ret;
}

static void sdma_event_enable(struct sdma_channel *sdmac, unsigned int event)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;
	unsigned long val;
	u32 chnenbl = chnenbl_ofs(sdma, event);

	val = readl_relaxed(sdma->regs + chnenbl);
	__set_bit(channel, &val);
	writel_relaxed(val, sdma->regs + chnenbl);

	/* Set SDMA_DONEx_CONFIG is sw_done enabled */
	if (sdmac->audio_config && (sdmac->audio_config->sw_done_sel & BIT(31) ||
					sdmac->sw_done)) {
		u32 sw_done_sel = sdmac->audio_config->sw_done_sel & 0xff;
		u32 offset = SDMA_DONE0_CONFIG + sw_done_sel / 4;
		u32 done_sel = SDMA_DONE0_CONFIG_DONE_SEL +
				((sw_done_sel % 4) << 3);
		u32 sw_done_dis = SDMA_DONE0_CONFIG_DONE_DIS +
				((sw_done_sel % 4) << 3);

		val = readl_relaxed(sdma->regs + offset);
		__set_bit(done_sel, &val);
		__clear_bit(sw_done_dis, &val);
		writel_relaxed(val, sdma->regs + offset);
	}

}

static int sdma_set_channel_priority(struct sdma_channel *sdmac,
				     unsigned int priority)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;

	if (priority < MXC_SDMA_MIN_PRIORITY
	    || priority > MXC_SDMA_MAX_PRIORITY) {
		return -EINVAL;
	}

	writel_relaxed(priority, sdma->regs + SDMA_CHNPRI_0 + 4 * channel);

	return 0;
}

static void sdma_event_disable(struct sdma_channel *sdmac, unsigned int event)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;
	u32 chnenbl = chnenbl_ofs(sdma, event);
	unsigned long val;

	val = readl_relaxed(sdma->regs + chnenbl);
	__clear_bit(channel, &val);
	writel_relaxed(val, sdma->regs + chnenbl);
}

static struct sdma_desc *to_sdma_desc(struct dma_async_tx_descriptor *t)
{
	return container_of(t, struct sdma_desc, vd.tx);
}

static void sdma_start_desc(struct sdma_channel *sdmac)
{
	struct virt_dma_desc *vd = vchan_next_desc(&sdmac->vc);
	struct sdma_desc *desc;
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;

	if (!vd) {
		sdmac->desc = NULL;
		return;
	}
	sdmac->desc = desc = to_sdma_desc(&vd->tx);

	list_del(&vd->node);

	sdma->channel_control[channel].base_bd_ptr = desc->bd_phys;
	sdma->channel_control[channel].current_bd_ptr = desc->bd_phys;
	sdma_enable_channel(sdma, sdmac->channel);
}

static void sdma_update_channel_loop(struct sdma_channel *sdmac)
{
	struct sdma_buffer_descriptor *bd;
	int error = 0;
	enum dma_status	old_status = sdmac->status;

	/*
	 * loop mode. Iterate over descriptors, re-setup them and
	 * call callback function.
	 */
	while (sdmac->desc) {
		struct sdma_desc *desc = sdmac->desc;

		bd = &desc->bd[desc->buf_tail];

		if (bd->mode.status & BD_DONE)
			break;

		if (bd->mode.status & BD_RROR) {
			bd->mode.status &= ~BD_RROR;
			sdmac->status = DMA_ERROR;
			error = -EIO;
		}

	       /*
		* We use bd->mode.count to calculate the residue, since contains
		* the number of bytes present in the current buffer descriptor.
		* Note: in IMX_DMATYPE_MULTI_SAI case, bd->mode.count used as
		* remaining bytes instead so that one register could be saved.
		* so chn_real_count = desc->period_len - bd->mode.count.
		*/
		if (sdmac->peripheral_type == IMX_DMATYPE_MULTI_SAI)
			desc->chn_real_count = desc->period_len - bd->mode.count;
		else
			desc->chn_real_count = bd->mode.count;

		bd->mode.status |= BD_DONE;
		bd->mode.count = desc->period_len;
		desc->buf_ptail = desc->buf_tail;
		desc->buf_tail = (desc->buf_tail + 1) % desc->num_bd;

		/*
		 * The callback is called from the interrupt context in order
		 * to reduce latency and to avoid the risk of altering the
		 * SDMA transaction status by the time the client tasklet is
		 * executed.
		 */
		spin_unlock(&sdmac->vc.lock);
		dmaengine_desc_get_callback_invoke(&desc->vd.tx, NULL);
		spin_lock(&sdmac->vc.lock);

		/* Assign buffer ownership to SDMA */
		bd->mode.status |= BD_DONE;

		if (error)
			sdmac->status = old_status;
	}

	/*
	 * SDMA stops cyclic channel when DMA request triggers a channel and no SDMA
	 * owned buffer is available (i.e. BD_DONE was set too late).
	 */
	if (sdmac->desc && !is_sdma_channel_enabled(sdmac->sdma, sdmac->channel)) {
		dev_warn(sdmac->sdma->dev, "restart cyclic channel %d\n", sdmac->channel);
		sdma_enable_channel(sdmac->sdma, sdmac->channel);
	}
}

static void mxc_sdma_handle_channel_normal(struct sdma_channel *data)
{
	struct sdma_channel *sdmac = (struct sdma_channel *) data;
	struct sdma_buffer_descriptor *bd;
	int i, error = 0;

	sdmac->desc->chn_real_count = 0;
	/*
	 * non loop mode. Iterate over all descriptors, collect
	 * errors and call callback function
	 */
	for (i = 0; i < sdmac->desc->num_bd; i++) {
		bd = &sdmac->desc->bd[i];

		if (bd->mode.status & (BD_DONE | BD_RROR))
			error = -EIO;
		sdmac->desc->chn_real_count += bd->mode.count;
	}

	if (error)
		sdmac->status = DMA_ERROR;
	else
		sdmac->status = DMA_COMPLETE;
}

static irqreturn_t sdma_int_handler(int irq, void *dev_id)
{
	struct sdma_engine *sdma = dev_id;
	unsigned long stat;

	sdma_pm_clk_enable(sdma, false, true);
	stat = readl_relaxed(sdma->regs + SDMA_H_INTR);
	writel_relaxed(stat, sdma->regs + SDMA_H_INTR);
	/* channel 0 is special and not handled here, see run_channel0() */
	stat &= ~1;

	while (stat) {
		int channel = fls(stat) - 1;
		struct sdma_channel *sdmac = &sdma->channel[channel];
		struct sdma_desc *desc;

		spin_lock(&sdmac->vc.lock);
		desc = sdmac->desc;
		if (desc) {
			if (sdmac->flags & IMX_DMA_SG_LOOP) {
				if (sdmac->peripheral_type != IMX_DMATYPE_HDMI)
					sdma_update_channel_loop(sdmac);
				else
					vchan_cyclic_callback(&desc->vd);
			} else {
				mxc_sdma_handle_channel_normal(sdmac);
				vchan_cookie_complete(&desc->vd);
				sdma_start_desc(sdmac);
			}
		}

		spin_unlock(&sdmac->vc.lock);
		__clear_bit(channel, &stat);
	}

	sdma_pm_clk_enable(sdma, false, false);

	return IRQ_HANDLED;
}

/*
 * sets the pc of SDMA script according to the peripheral type
 */
static int sdma_get_pc(struct sdma_channel *sdmac,
		enum sdma_peripheral_type peripheral_type)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int per_2_emi = 0, emi_2_per = 0;
	/*
	 * These are needed once we start to support transfers between
	 * two peripherals or memory-to-memory transfers
	 */
	int per_2_per = 0, emi_2_emi = 0;

	sdmac->pc_from_device = 0;
	sdmac->pc_to_device = 0;
	sdmac->device_to_device = 0;
	sdmac->pc_to_pc = 0;
	sdmac->is_ram_script = false;

	switch (peripheral_type) {
	case IMX_DMATYPE_MEMORY:
		emi_2_emi = sdma->script_addrs->ap_2_ap_addr;
		break;
	case IMX_DMATYPE_DSP:
		emi_2_per = sdma->script_addrs->bp_2_ap_addr;
		per_2_emi = sdma->script_addrs->ap_2_bp_addr;
		break;
	case IMX_DMATYPE_FIRI:
		per_2_emi = sdma->script_addrs->firi_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_firi_addr;
		break;
	case IMX_DMATYPE_UART:
		per_2_emi = sdma->script_addrs->uart_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_app_addr;
		break;
	case IMX_DMATYPE_UART_SP:
		per_2_emi = sdma->script_addrs->uartsh_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_shp_addr;
		break;
	case IMX_DMATYPE_ATA:
		per_2_emi = sdma->script_addrs->ata_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_ata_addr;
		break;
	case IMX_DMATYPE_CSPI:
		per_2_emi = sdma->script_addrs->app_2_mcu_addr;

		/* Use rom script mcu_2_app if ERR009165 fixed */
		if (sdmac->sdma->drvdata->ecspi_fixed) {
			emi_2_per = sdma->script_addrs->mcu_2_app_addr;
		} else {
			emi_2_per = sdma->script_addrs->mcu_2_ecspi_addr;
			sdmac->is_ram_script = true;
		}

		break;
	case IMX_DMATYPE_EXT:
	case IMX_DMATYPE_SSI:
	case IMX_DMATYPE_SAI:
		per_2_emi = sdma->script_addrs->app_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_app_addr;
		break;
	case IMX_DMATYPE_SSI_DUAL:
		per_2_emi = sdma->script_addrs->ssish_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_ssish_addr;
		sdmac->is_ram_script = true;
		break;
	case IMX_DMATYPE_SSI_SP:
	case IMX_DMATYPE_MMC:
	case IMX_DMATYPE_SDHC:
	case IMX_DMATYPE_CSPI_SP:
	case IMX_DMATYPE_ESAI:
	case IMX_DMATYPE_MSHC_SP:
		per_2_emi = sdma->script_addrs->shp_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_shp_addr;
		break;
	case IMX_DMATYPE_ASRC:
		per_2_emi = sdma->script_addrs->asrc_2_mcu_addr;
		emi_2_per = sdma->script_addrs->asrc_2_mcu_addr;
		per_2_per = sdma->script_addrs->per_2_per_addr;
		sdmac->is_ram_script = true;
		break;
	case IMX_DMATYPE_ASRC_SP:
		per_2_emi = sdma->script_addrs->shp_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_shp_addr;
		per_2_per = sdma->script_addrs->per_2_per_addr;
		break;
	case IMX_DMATYPE_MSHC:
		per_2_emi = sdma->script_addrs->mshc_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_mshc_addr;
		break;
	case IMX_DMATYPE_CCM:
		per_2_emi = sdma->script_addrs->dptc_dvfs_addr;
		break;
	case IMX_DMATYPE_SPDIF:
		per_2_emi = sdma->script_addrs->spdif_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_spdif_addr;
		break;
	case IMX_DMATYPE_IPU_MEMORY:
		emi_2_per = sdma->script_addrs->ext_mem_2_ipu_addr;
		break;
	case IMX_DMATYPE_MULTI_SAI:
		per_2_emi = sdma->script_addrs->sai_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_sai_addr;
		sdmac->is_ram_script = true;
		break;
	case IMX_DMATYPE_I2C:
		per_2_emi = sdma->script_addrs->i2c_2_mcu_addr;
		emi_2_per = sdma->script_addrs->mcu_2_i2c_addr;
		sdmac->is_ram_script = true;
		break;
	case IMX_DMATYPE_HDMI:
		emi_2_per = sdma->script_addrs->hdmi_dma_addr;
		sdmac->is_ram_script = true;
		break;
	default:
		dev_err(sdma->dev, "Unsupported transfer type %d\n",
			peripheral_type);
		return -EINVAL;
	}

	sdmac->pc_from_device = per_2_emi;
	sdmac->pc_to_device = emi_2_per;
	sdmac->device_to_device = per_2_per;
	sdmac->pc_to_pc = emi_2_emi;

	if (sdma->ram_code_start &&
	   ((sdmac->pc_from_device >= sdma->ram_code_start) ||
	   (sdmac->pc_to_device >= sdma->ram_code_start) ||
	   (sdmac->device_to_device >= sdma->ram_code_start ||
	   (sdmac->pc_to_pc >= sdma->ram_code_start))))
		sdmac->is_ram_script = true;

	return 0;
}

static int sdma_load_context(struct sdma_channel *sdmac)
{
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;
	int load_address;
	struct sdma_context_data *context = sdma->context;
	struct sdma_buffer_descriptor *bd0 = sdma->bd0;
	int ret;
	unsigned long flags;

	if (sdmac->direction == DMA_DEV_TO_MEM)
		load_address = sdmac->pc_from_device;
	else if (sdmac->direction == DMA_DEV_TO_DEV)
		load_address = sdmac->device_to_device;
	else if (sdmac->direction == DMA_MEM_TO_MEM)
		load_address = sdmac->pc_to_pc;
	else
		load_address = sdmac->pc_to_device;

	if (load_address < 0)
		return load_address;

	dev_dbg(sdma->dev, "load_address = %d\n", load_address);
	dev_dbg(sdma->dev, "wml = 0x%08x\n", (u32)sdmac->watermark_level);
	dev_dbg(sdma->dev, "shp_addr = 0x%08x\n", sdmac->shp_addr);
	dev_dbg(sdma->dev, "per_addr = 0x%08x\n", sdmac->per_addr);
	dev_dbg(sdma->dev, "event_mask0 = 0x%08x\n", (u32)sdmac->event_mask[0]);
	dev_dbg(sdma->dev, "event_mask1 = 0x%08x\n", (u32)sdmac->event_mask[1]);

	spin_lock_irqsave(&sdma->channel_0_lock, flags);

	memset(context, 0, sizeof(*context));
	context->channel_state.pc = load_address;

	/* Send by context the event mask,base address for peripheral
	 * and watermark level
	 */
	if (sdmac->peripheral_type == IMX_DMATYPE_HDMI) {
		context->gReg[4] = sdmac->per_addr;
		context->gReg[6] = sdmac->shp_addr;
	} else {
		context->gReg[0] = sdmac->event_mask[1];
		context->gReg[1] = sdmac->event_mask[0];
		context->gReg[2] = sdmac->per_addr;
		context->gReg[6] = sdmac->shp_addr;
		context->gReg[7] = sdmac->watermark_level;
	}

	bd0->mode.command = C0_SETDM;
	bd0->mode.status = BD_DONE | BD_WRAP | BD_EXTD;
	bd0->mode.count = sizeof(*context) / 4;
	bd0->buffer_addr = sdma->context_phys;
	bd0->ext_buffer_addr = 2048 + (sizeof(*context) / 4) * channel;
	ret = sdma_run_channel0(sdma);

	spin_unlock_irqrestore(&sdma->channel_0_lock, flags);

	return ret;
}

static int sdma_save_restore_context(struct sdma_engine *sdma, bool save)
{
	struct sdma_context_data *context = sdma->context;
	struct sdma_buffer_descriptor *bd0 = sdma->bd0;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&sdma->channel_0_lock, flags);

	if (save)
		bd0->mode.command = C0_GETDM;
	else
		bd0->mode.command = C0_SETDM;

	bd0->mode.status = BD_DONE | BD_WRAP | BD_EXTD;
	bd0->mode.count = MAX_DMA_CHANNELS * sizeof(*context) / 4;
	bd0->buffer_addr = sdma->context_phys;
	bd0->ext_buffer_addr = 2048;
	ret = sdma_run_channel0(sdma);

	spin_unlock_irqrestore(&sdma->channel_0_lock, flags);

	return ret;
}

static struct sdma_channel *to_sdma_chan(struct dma_chan *chan)
{
	return container_of(chan, struct sdma_channel, vc.chan);
}

static int sdma_disable_channel(struct dma_chan *chan)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;

	writel_relaxed(BIT(channel), sdma->regs + SDMA_H_STATSTOP);
	sdmac->status = DMA_ERROR;

	return 0;
}
static void sdma_channel_terminate_work(struct work_struct *work)
{
	struct sdma_channel *sdmac = container_of(work, struct sdma_channel,
						  terminate_worker);
	/*
	 * According to NXP R&D team a delay of one BD SDMA cost time
	 * (maximum is 1ms) should be added after disable of the channel
	 * bit, to ensure SDMA core has really been stopped after SDMA
	 * clients call .device_terminate_all.
	 */
	usleep_range(1000, 2000);

	vchan_dma_desc_free_list(&sdmac->vc, &sdmac->terminated);
}

static int sdma_terminate_all(struct dma_chan *chan)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	unsigned long flags;

	sdma_pm_clk_enable(sdmac->sdma, false, true);
	spin_lock_irqsave(&sdmac->vc.lock, flags);

	sdma_disable_channel(chan);

	if (sdmac->desc) {
		vchan_terminate_vdesc(&sdmac->desc->vd);
		/*
		 * move out current descriptor into terminated list so that
		 * it could be free in sdma_channel_terminate_work alone
		 * later without potential involving next descriptor raised
		 * up before the last descriptor terminated.
		 */
		vchan_get_all_descriptors(&sdmac->vc, &sdmac->terminated);
		sdmac->desc = NULL;
		schedule_work(&sdmac->terminate_worker);
	}

	spin_unlock_irqrestore(&sdmac->vc.lock, flags);

	sdma_pm_clk_enable(sdmac->sdma, false, false);

	return 0;
}

static void sdma_channel_synchronize(struct dma_chan *chan)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);

	vchan_synchronize(&sdmac->vc);

	flush_work(&sdmac->terminate_worker);
}

static void sdma_set_watermarklevel_for_p2p(struct sdma_channel *sdmac)
{
	struct sdma_engine *sdma = sdmac->sdma;

	int lwml = sdmac->watermark_level & SDMA_WATERMARK_LEVEL_LWML;
	int hwml = (sdmac->watermark_level & SDMA_WATERMARK_LEVEL_HWML) >> 16;

	set_bit(sdmac->event_id0 % 32, &sdmac->event_mask[1]);
	set_bit(sdmac->event_id1 % 32, &sdmac->event_mask[0]);

	if (sdmac->event_id0 > 31)
		sdmac->watermark_level |= SDMA_WATERMARK_LEVEL_LWE;

	if (sdmac->event_id1 > 31)
		sdmac->watermark_level |= SDMA_WATERMARK_LEVEL_HWE;

	/*
	 * If LWML(src_maxburst) > HWML(dst_maxburst), we need
	 * swap LWML and HWML of INFO(A.3.2.5.1), also need swap
	 * r0(event_mask[1]) and r1(event_mask[0]).
	 */
	if (lwml > hwml) {
		sdmac->watermark_level &= ~(SDMA_WATERMARK_LEVEL_LWML |
						SDMA_WATERMARK_LEVEL_HWML);
		sdmac->watermark_level |= hwml;
		sdmac->watermark_level |= lwml << 16;
		swap(sdmac->event_mask[0], sdmac->event_mask[1]);
	}

	if (sdmac->per_address2 >= sdma->spba_start_addr &&
			sdmac->per_address2 <= sdma->spba_end_addr)
		sdmac->watermark_level |= SDMA_WATERMARK_LEVEL_SP;

	if (sdmac->per_address >= sdma->spba_start_addr &&
			sdmac->per_address <= sdma->spba_end_addr)
		sdmac->watermark_level |= SDMA_WATERMARK_LEVEL_DP;

	sdmac->watermark_level |= SDMA_WATERMARK_LEVEL_CONT;

	if (sdmac->n_fifos_src > 1)
		sdmac->watermark_level |= SDMA_WATERMARK_LEVEL_SD;
	if (sdmac->n_fifos_dst > 1)
		sdmac->watermark_level |= SDMA_WATERMARK_LEVEL_DD;
}

static void sdma_set_watermarklevel_for_sais(struct sdma_channel *sdmac)
{
	unsigned int n_fifos;
	unsigned int stride_fifos;
	unsigned int words_per_fifo;

	if (sdmac->audio_config && (sdmac->audio_config->sw_done_sel & BIT(31) ||
					sdmac->sw_done))
		sdmac->watermark_level |= SDMA_WATERMARK_LEVEL_SW_DONE |
				(sdmac->audio_config->sw_done_sel & 0xff) <<
				SDMA_WATERMARK_LEVEL_SW_DONE_SEL_OFF;

	if (sdmac->direction == DMA_DEV_TO_MEM) {
		n_fifos = sdmac->n_fifos_src;
		stride_fifos = sdmac->stride_fifos_src;
	} else {
		n_fifos = sdmac->n_fifos_dst;
		stride_fifos = sdmac->stride_fifos_dst;
	}

	words_per_fifo = sdmac->words_per_fifo;

	sdmac->watermark_level |=
			FIELD_PREP(SDMA_WATERMARK_LEVEL_N_FIFOS, n_fifos);

	sdmac->watermark_level |=
			FIELD_PREP(SDMA_WATERMARK_LEVEL_OFF_FIFOS, stride_fifos);

	if (words_per_fifo)
		sdmac->watermark_level |=
			FIELD_PREP(SDMA_WATERMARK_LEVEL_WORDS_PER_FIFO, (words_per_fifo - 1));
}

static int sdma_config_channel(struct dma_chan *chan)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	int ret;

	sdma_disable_channel(chan);

	sdmac->event_mask[0] = 0;
	sdmac->event_mask[1] = 0;
	sdmac->shp_addr = 0;
	sdmac->per_addr = 0;

	switch (sdmac->peripheral_type) {
	case IMX_DMATYPE_DSP:
		sdma_config_ownership(sdmac, false, true, true);
		break;
	case IMX_DMATYPE_MEMORY:
		sdma_config_ownership(sdmac, false, true, false);
		break;
	default:
		sdma_config_ownership(sdmac, true, true, false);
		break;
	}

	ret = sdma_get_pc(sdmac, sdmac->peripheral_type);
	if (ret)
		return ret;
	sdma_set_channel_priority(sdmac, sdmac->prio);

	sdma_event_enable(sdmac, sdmac->event_id0);
	if (sdmac->event_id1)
		sdma_event_enable(sdmac, sdmac->event_id1);

	if ((sdmac->peripheral_type != IMX_DMATYPE_MEMORY) &&
			(sdmac->peripheral_type != IMX_DMATYPE_DSP)) {
		/* Handle multiple event channels differently */
		if (sdmac->event_id1) {
			if (sdmac->peripheral_type == IMX_DMATYPE_ASRC_SP ||
			    sdmac->peripheral_type == IMX_DMATYPE_ASRC)
				sdma_set_watermarklevel_for_p2p(sdmac);
		} else {
			if (sdmac->peripheral_type == IMX_DMATYPE_MULTI_SAI)
				sdma_set_watermarklevel_for_sais(sdmac);

			__set_bit(sdmac->event_id0, sdmac->event_mask);
		}

		/* Address */
		sdmac->shp_addr = sdmac->per_address;
		sdmac->per_addr = sdmac->per_address2;
	} else {
		sdmac->watermark_level = 0; /* FIXME: M3_BASE_ADDRESS */
	}

	return 0;
}

static int sdma_request_channel0(struct sdma_engine *sdma)
{
	int ret = -EBUSY;

	if (sdma->iram_pool)
		sdma->bd0 = gen_pool_dma_alloc(sdma->iram_pool,
					sizeof(struct sdma_buffer_descriptor),
					&sdma->bd0_phys);
	else
		sdma->bd0 = dma_alloc_coherent(sdma->dev,
					sizeof(struct sdma_buffer_descriptor),
					&sdma->bd0_phys, GFP_NOWAIT);
	if (!sdma->bd0) {
		ret = -ENOMEM;
		goto out;
	}

	sdma->channel_control[0].base_bd_ptr = sdma->bd0_phys;
	sdma->channel_control[0].current_bd_ptr = sdma->bd0_phys;

	sdma_set_channel_priority(&sdma->channel[0], MXC_SDMA_DEFAULT_PRIORITY);
	return 0;
out:

	return ret;
}


static int sdma_alloc_bd(struct sdma_desc *desc)
{
	u32 bd_size = desc->num_bd * sizeof(struct sdma_buffer_descriptor);
	struct sdma_engine *sdma = desc->sdmac->sdma;
	int ret = 0;

	if (sdma->iram_pool)
		desc->bd = gen_pool_dma_alloc(sdma->iram_pool, bd_size,
					      &desc->bd_phys);
	else
		desc->bd = dma_alloc_coherent(sdma->dev, bd_size,
					      &desc->bd_phys, GFP_NOWAIT);
	if (!desc->bd) {
		ret = -ENOMEM;
		goto out;
	}
out:
	return ret;
}

static void sdma_free_bd(struct sdma_desc *desc)
{
	u32 bd_size = desc->num_bd * sizeof(struct sdma_buffer_descriptor);
	struct sdma_engine *sdma = desc->sdmac->sdma;

	if (sdma->iram_pool)
		gen_pool_free(sdma->iram_pool, (unsigned long)desc->bd,
			      bd_size);
	else
		dma_free_coherent(desc->sdmac->sdma->dev, bd_size, desc->bd,
				  desc->bd_phys);
}

static void sdma_desc_free(struct virt_dma_desc *vd)
{
	struct sdma_desc *desc = container_of(vd, struct sdma_desc, vd);

	sdma_free_bd(desc);
	kfree(desc);
}

static int sdma_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sdma_engine *sdma = platform_get_drvdata(pdev);

	if (!sdma->is_on)
		return 0;

	sdma->fw_loaded = false;
	sdma->is_on = false;

	clk_disable(sdma->clk_ipg);
	clk_disable(sdma->clk_ahb);

	/* free channel0 bd */
	if (sdma->iram_pool)
		gen_pool_free(sdma->iram_pool, (unsigned long)sdma->bd0,
			      sizeof(struct sdma_buffer_descriptor));
	else
		dma_free_coherent(sdma->dev,
				  sizeof(struct sdma_buffer_descriptor),
				  sdma->bd0, sdma->bd0_phys);

	return 0;
}

static int sdma_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sdma_engine *sdma = platform_get_drvdata(pdev);
	int i, ret = 0;

	ret = clk_enable(sdma->clk_ipg);
	if (ret)
		return ret;
	ret = clk_enable(sdma->clk_ahb);
	if (ret)
		goto disable_clk_ipg;

	/* Do nothing at HW level if audiomix which shared with audio driver
	 * not off indeed.
	 */
	if (readl_relaxed(sdma->regs + SDMA_H_C0PTR)) {
		if (sdma->iram_pool)
			sdma->bd0 = gen_pool_dma_alloc(sdma->iram_pool,
					sizeof(struct sdma_buffer_descriptor),
					&sdma->bd0_phys);
		else
			sdma->bd0 = dma_alloc_coherent(sdma->dev,
					sizeof(struct sdma_buffer_descriptor),
					&sdma->bd0_phys, GFP_NOWAIT);
		if (!sdma->bd0)
			ret = -ENOMEM;

		sdma->channel_control[0].base_bd_ptr = sdma->bd0_phys;
		sdma->channel_control[0].current_bd_ptr = sdma->bd0_phys;

		sdma->is_on = true;
		sdma->fw_loaded = true;

		return ret;
	}

	/* Be sure SDMA has not started yet */
	writel_relaxed(0, sdma->regs + SDMA_H_C0PTR);

	/* disable all channels */
	for (i = 0; i < sdma->drvdata->num_events; i++)
		writel_relaxed(0, sdma->regs + chnenbl_ofs(sdma, i));

	/* All channels have priority 0 */
	for (i = 0; i < MAX_DMA_CHANNELS; i++)
		writel_relaxed(0, sdma->regs + SDMA_CHNPRI_0 + i * 4);

	ret = sdma_request_channel0(sdma);
	if (ret)
		return ret;

	sdma_config_ownership(&sdma->channel[0], false, true, false);

	/* Set Command Channel (Channel Zero) */
	writel_relaxed(0x4050, sdma->regs + SDMA_CHN0ADDR);

	/* Set bits of CONFIG register but with static context switching */
	if (sdma->clk_ratio)
		writel_relaxed(SDMA_H_CONFIG_ACR, sdma->regs + SDMA_H_CONFIG);
	else
		writel_relaxed(0, sdma->regs + SDMA_H_CONFIG);

	writel_relaxed(sdma->ccb_phys, sdma->regs + SDMA_H_C0PTR);

	/* Initializes channel's priorities */
	sdma_set_channel_priority(&sdma->channel[0], 7);

	if (!sdma->fw_data)
		dev_dbg(sdma->dev, "firmware not ready.\n");
	else if (sdma_load_script(sdma))
		dev_warn(sdma->dev, "failed to load script.\n");

	sdma->is_on = true;

	return 0;

disable_clk_ipg:
	clk_disable(sdma->clk_ipg);
	dev_err(sdma->dev, "initialisation failed with %d\n", ret);

	return ret;
}

static int sdma_alloc_chan_resources(struct dma_chan *chan)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	struct imx_dma_data *data = chan->private;
	struct imx_dma_data mem_data;
	int prio, ret;

	/*
	 * MEMCPY may never setup chan->private by filter function such as
	 * dmatest, thus create 'struct imx_dma_data mem_data' for this case.
	 * Please note in any other slave case, you have to setup chan->private
	 * with 'struct imx_dma_data' in your own filter function if you want to
	 * request dma channel by dma_request_channel() rather than
	 * dma_request_slave_channel(). Othwise, 'MEMCPY in case?' will appear
	 * to warn you to correct your filter function.
	 */
	if (!data) {
		dev_dbg(sdmac->sdma->dev, "MEMCPY in case?\n");
		mem_data.priority = 2;
		mem_data.peripheral_type = IMX_DMATYPE_MEMORY;
		mem_data.dma_request = 0;
		mem_data.dma_request2 = 0;
		data = &mem_data;

		ret = sdma_get_pc(sdmac, IMX_DMATYPE_MEMORY);
		if (ret)
			return ret;
	}

	switch (data->priority) {
	case DMA_PRIO_HIGH:
		prio = 3;
		break;
	case DMA_PRIO_MEDIUM:
		prio = 2;
		break;
	case DMA_PRIO_LOW:
	default:
		prio = 1;
		break;
	}

	sdmac->peripheral_type = data->peripheral_type;
	sdmac->event_id0 = data->dma_request;
	sdmac->event_id1 = data->dma_request2;
	sdmac->prio = prio;

	return 0;
}

static void sdma_free_chan_resources(struct dma_chan *chan)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);

	/*
	 * Per fw_data is null which means firmware not loaded and sdma
	 * not initialized, directly return. This happens in below case:
	 *
	 * -- driver request dma chan in probe phase, after that driver
	 *    fall into -EPROBE_DEFER and free channel again without
	 *    anything else about dma, so just return directly, otherwise
	 *    kernel could hang since dma hardware not ready if drvdata->
	 *    pm_runtime is false.
	 *
	 */
	if (unlikely(!sdmac->sdma->fw_data))
		return;

	sdma_pm_clk_enable(sdmac->sdma, false, true);
	sdma_terminate_all(chan);

	sdma_channel_synchronize(chan);

	sdma_event_disable(sdmac, sdmac->event_id0);
	if (sdmac->event_id1)
		sdma_event_disable(sdmac, sdmac->event_id1);

	sdmac->event_id0 = 0;
	sdmac->event_id1 = 0;

	sdma_set_channel_priority(sdmac, 0);

	kfree(sdmac->audio_config);
	sdmac->audio_config = NULL;
	sdma_pm_clk_enable(sdmac->sdma, false, false);
}

static struct sdma_desc *sdma_transfer_init(struct sdma_channel *sdmac,
				enum dma_transfer_direction direction, u32 bds)
{
	struct sdma_desc *desc;

	desc = kzalloc((sizeof(*desc)), GFP_NOWAIT);
	if (!desc)
		goto err_out;

	sdmac->status = DMA_IN_PROGRESS;
	sdmac->direction = direction;
	sdmac->flags = 0;

	desc->chn_count = 0;
	desc->chn_real_count = 0;
	desc->buf_tail = 0;
	desc->buf_ptail = 0;
	desc->sdmac = sdmac;
	desc->num_bd = bds;

	if (bds && sdma_alloc_bd(desc))
		goto err_desc_out;

	/* No slave_config called in MEMCPY case, so do here */
	if (direction == DMA_MEM_TO_MEM) {
		sdma_config_ownership(sdmac, false, true, false);
		sdma_set_channel_priority(sdmac, sdmac->prio);
	}

	if (sdma_load_context(sdmac))
		goto err_bd_out;

	return desc;

err_bd_out:
	sdma_free_bd(desc);
err_desc_out:
	kfree(desc);
err_out:
	return NULL;
}

static struct dma_async_tx_descriptor *sdma_prep_memcpy(
		struct dma_chan *chan, dma_addr_t dma_dst,
		dma_addr_t dma_src, size_t len, unsigned long flags)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	struct sdma_engine *sdma = sdmac->sdma;
	int channel = sdmac->channel;
	size_t count;
	int i = 0, param;
	struct sdma_buffer_descriptor *bd;
	struct sdma_desc *desc;

	if (!chan || !len)
		return NULL;

	dev_dbg(sdma->dev, "memcpy: %pad->%pad, len=%zu, channel=%d.\n",
		&dma_src, &dma_dst, len, channel);

	sdma_pm_clk_enable(sdmac->sdma, false, true);
	desc = sdma_transfer_init(sdmac, DMA_MEM_TO_MEM,
					len / SDMA_BD_MAX_CNT + 1);
	if (!desc) {
		sdma_pm_clk_enable(sdmac->sdma, true, false);
		return NULL;
	}

	do {
		count = min_t(size_t, len, SDMA_BD_MAX_CNT);
		bd = &desc->bd[i];
		bd->buffer_addr = dma_src;
		bd->ext_buffer_addr = dma_dst;
		bd->mode.count = count;
		desc->chn_count += count;
		bd->mode.command = 0;

		dma_src += count;
		dma_dst += count;
		len -= count;
		i++;

		param = BD_DONE | BD_EXTD | BD_CONT;
		/* last bd */
		if (!len) {
			param |= BD_INTR;
			param |= BD_LAST;
			param &= ~BD_CONT;
		}

		dev_dbg(sdma->dev, "entry %d: count: %zd dma: 0x%x %s%s\n",
				i, count, bd->buffer_addr,
				param & BD_WRAP ? "wrap" : "",
				param & BD_INTR ? " intr" : "");

		bd->mode.status = param;
	} while (len);

	sdma_pm_clk_enable(sdmac->sdma, false, false);

	return vchan_tx_prep(&sdmac->vc, &desc->vd, flags);
}

static struct dma_async_tx_descriptor *sdma_prep_slave_sg(
		struct dma_chan *chan, struct scatterlist *sgl,
		unsigned int sg_len, enum dma_transfer_direction direction,
		unsigned long flags, void *context)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	struct sdma_engine *sdma = sdmac->sdma;
	int i, count;
	int channel = sdmac->channel;
	struct scatterlist *sg;
	struct sdma_desc *desc;
	int ret;

	sdma_pm_clk_enable(sdmac->sdma, false, true);
	ret = sdma_config_write(chan, &sdmac->slave_config, direction);
	if (ret)
		goto err_out;

	desc = sdma_transfer_init(sdmac, direction, sg_len);
	if (!desc)
		goto err_out;

	dev_dbg(sdma->dev, "setting up %d entries for channel %d.\n",
			sg_len, channel);

	for_each_sg(sgl, sg, sg_len, i) {
		struct sdma_buffer_descriptor *bd = &desc->bd[i];
		int param;

		bd->buffer_addr = sg->dma_address;

		count = sg_dma_len(sg);

		if (count > SDMA_BD_MAX_CNT) {
			dev_err(sdma->dev, "SDMA channel %d: maximum bytes for sg entry exceeded: %d > %d\n",
					channel, count, SDMA_BD_MAX_CNT);
			goto err_bd_out;
		}

		bd->mode.count = count;
		desc->chn_count += count;

		if (sdmac->word_size > DMA_SLAVE_BUSWIDTH_4_BYTES)
			goto err_bd_out;

		switch (sdmac->word_size) {
		case DMA_SLAVE_BUSWIDTH_4_BYTES:
			bd->mode.command = 0;
			if (count & 3 || sg->dma_address & 3)
				goto err_bd_out;
			break;
		case DMA_SLAVE_BUSWIDTH_3_BYTES:
			bd->mode.command = 3;
			break;
		case DMA_SLAVE_BUSWIDTH_2_BYTES:
			bd->mode.command = 2;
			if (count & 1 || sg->dma_address & 1)
				goto err_bd_out;
			break;
		case DMA_SLAVE_BUSWIDTH_1_BYTE:
			bd->mode.command = 1;
			break;
		default:
			goto err_bd_out;
		}

		param = BD_DONE | BD_EXTD | BD_CONT;

		if (i + 1 == sg_len) {
			param |= BD_INTR;
			param |= BD_LAST;
			param &= ~BD_CONT;
		}

		dev_dbg(sdma->dev, "entry %d: count: %d dma: %#llx %s%s\n",
				i, count, (u64)sg->dma_address,
				param & BD_WRAP ? "wrap" : "",
				param & BD_INTR ? " intr" : "");

		bd->mode.status = param;
	}

	sdma_pm_clk_enable(sdmac->sdma, false, false);

	return vchan_tx_prep(&sdmac->vc, &desc->vd, flags);
err_bd_out:
	sdma_free_bd(desc);
	kfree(desc);
err_out:
	sdmac->status = DMA_ERROR;
	sdma_pm_clk_enable(sdmac->sdma, true, false);

	return NULL;
}

static struct dma_async_tx_descriptor *sdma_prep_dma_cyclic(
		struct dma_chan *chan, dma_addr_t dma_addr, size_t buf_len,
		size_t period_len, enum dma_transfer_direction direction,
		unsigned long flags)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	struct sdma_engine *sdma = sdmac->sdma;
	int num_periods = 0;
	int channel = sdmac->channel;
	int i = 0, buf = 0;
	struct sdma_desc *desc;
	int ret;

	dev_dbg(sdma->dev, "%s channel: %d\n", __func__, channel);

	sdma_pm_clk_enable(sdmac->sdma, false, true);

	if (sdmac->peripheral_type != IMX_DMATYPE_HDMI)
		num_periods = buf_len / period_len;

	ret = sdma_config_write(chan, &sdmac->slave_config, direction);
	if (ret)
		goto err_out;

	desc = sdma_transfer_init(sdmac, direction, num_periods);
	if (!desc)
		goto err_out;

	desc->period_len = period_len;

	sdmac->flags |= IMX_DMA_SG_LOOP;

	if (period_len > SDMA_BD_MAX_CNT) {
		dev_err(sdma->dev, "SDMA channel %d: maximum period size exceeded: %zu > %d\n",
				channel, period_len, SDMA_BD_MAX_CNT);
		goto err_bd_out;
	}

	if (sdmac->peripheral_type == IMX_DMATYPE_HDMI)
		return vchan_tx_prep(&sdmac->vc, &desc->vd, flags);

	while (buf < buf_len) {
		struct sdma_buffer_descriptor *bd = &desc->bd[i];
		int param;

		bd->buffer_addr = dma_addr;

		bd->mode.count = period_len;

		if (sdmac->word_size > DMA_SLAVE_BUSWIDTH_4_BYTES)
			goto err_bd_out;
		if (sdmac->word_size == DMA_SLAVE_BUSWIDTH_4_BYTES)
			bd->mode.command = 0;
		else
			bd->mode.command = sdmac->word_size;

		param = BD_DONE | BD_EXTD | BD_CONT | BD_INTR;
		if (i + 1 == num_periods)
			param |= BD_WRAP;

		dev_dbg(sdma->dev, "entry %d: count: %zu dma: %#llx %s%s\n",
				i, period_len, (u64)dma_addr,
				param & BD_WRAP ? "wrap" : "",
				param & BD_INTR ? " intr" : "");

		bd->mode.status = param;

		dma_addr += period_len;
		buf += period_len;

		i++;
	}

	sdma_pm_clk_enable(sdmac->sdma, false, false);

	return vchan_tx_prep(&sdmac->vc, &desc->vd, flags);
err_bd_out:
	sdma_free_bd(desc);
	kfree(desc);
err_out:
	sdmac->status = DMA_ERROR;
	sdma_pm_clk_enable(sdmac->sdma, true, false);

	return NULL;
}

static int sdma_config_write(struct dma_chan *chan,
		       struct dma_slave_config *dmaengine_cfg,
		       enum dma_transfer_direction direction)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);

	sdmac->watermark_level = 0;
	sdma_get_pc(sdmac, sdmac->peripheral_type);

	if (!sdmac->sdma->is_on || (!sdmac->sdma->fw_loaded && sdmac->is_ram_script)) {
		dev_warn_once(sdmac->sdma->dev, "sdma or sdma firmware not ready!\n");
		return -EPERM;
	}

	if (direction == DMA_DEV_TO_MEM) {
		sdmac->per_address = dmaengine_cfg->src_addr;
		sdmac->watermark_level = dmaengine_cfg->src_maxburst *
			dmaengine_cfg->src_addr_width;
		sdmac->word_size = dmaengine_cfg->src_addr_width;
	} else if (direction == DMA_DEV_TO_DEV) {
		sdmac->per_address2 = dmaengine_cfg->src_addr;
		sdmac->per_address = dmaengine_cfg->dst_addr;
		sdmac->watermark_level = dmaengine_cfg->src_maxburst &
			SDMA_WATERMARK_LEVEL_LWML;
		sdmac->watermark_level |= (dmaengine_cfg->dst_maxburst << 16) &
			SDMA_WATERMARK_LEVEL_HWML;
		sdmac->word_size = dmaengine_cfg->dst_addr_width;
	} else if (sdmac->peripheral_type == IMX_DMATYPE_HDMI) {
		sdmac->per_address = dmaengine_cfg->dst_addr;
		sdmac->per_address2 = dmaengine_cfg->src_addr;
		sdmac->watermark_level = 0;
	} else {
		sdmac->per_address = dmaengine_cfg->dst_addr;
		sdmac->watermark_level = dmaengine_cfg->dst_maxburst *
			dmaengine_cfg->dst_addr_width;
		sdmac->word_size = dmaengine_cfg->dst_addr_width;
	}
	sdmac->direction = direction;
	return sdma_config_channel(chan);
}

static int sdma_config(struct dma_chan *chan,
		       struct dma_slave_config *dmaengine_cfg)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	struct sdma_engine *sdma = sdmac->sdma;
	void *tmp;

	memcpy(&sdmac->slave_config, dmaengine_cfg, sizeof(*dmaengine_cfg));

	if (dmaengine_cfg->peripheral_config) {
		struct sdma_peripheral_config *sdmacfg = dmaengine_cfg->peripheral_config;
		if (dmaengine_cfg->peripheral_size != sizeof(struct sdma_peripheral_config)) {
			dev_err(sdma->dev, "Invalid peripheral size %zu, expected %zu\n",
				dmaengine_cfg->peripheral_size,
				sizeof(struct sdma_peripheral_config));
			return -EINVAL;
		}

		tmp = krealloc(sdmac->audio_config, dmaengine_cfg->peripheral_size, GFP_NOWAIT);
		if (!tmp)
			return -ENOMEM;

		sdmac->audio_config = (struct sdma_peripheral_config *)tmp;
		memcpy(tmp, sdmacfg, dmaengine_cfg->peripheral_size);

		sdmac->n_fifos_src = sdmacfg->n_fifos_src;
		sdmac->n_fifos_dst = sdmacfg->n_fifos_dst;
		sdmac->stride_fifos_src = sdmacfg->stride_fifos_src;
		sdmac->stride_fifos_dst = sdmacfg->stride_fifos_dst;
		sdmac->words_per_fifo = sdmacfg->words_per_fifo;
		sdmac->sw_done = sdmacfg->sw_done;
	}

	/* Set ENBLn earlier to make sure dma request triggered after that */
	if (sdmac->event_id0 >= sdmac->sdma->drvdata->num_events)
		return -EINVAL;


	if (sdmac->event_id1 &&
	    sdmac->event_id1 >= sdmac->sdma->drvdata->num_events)
		return -EINVAL;

	return 0;
}

static enum dma_status sdma_tx_status(struct dma_chan *chan,
				      dma_cookie_t cookie,
				      struct dma_tx_state *txstate)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	struct sdma_desc *desc = NULL;
	u32 residue;
	struct virt_dma_desc *vd;
	enum dma_status ret;
	unsigned long flags;

	ret = dma_cookie_status(chan, cookie, txstate);
	if (ret == DMA_COMPLETE || !txstate)
		return ret;

	spin_lock_irqsave(&sdmac->vc.lock, flags);

	vd = vchan_find_desc(&sdmac->vc, cookie);
	if (vd)
		desc = to_sdma_desc(&vd->tx);
	else if (sdmac->desc && sdmac->desc->vd.tx.cookie == cookie)
		desc = sdmac->desc;

	if (desc) {
		if (sdmac->flags & IMX_DMA_SG_LOOP)
			residue = (desc->num_bd - desc->buf_ptail) *
				desc->period_len - desc->chn_real_count;
		else
			residue = desc->chn_count - desc->chn_real_count;
	} else {
		residue = 0;
	}

	spin_unlock_irqrestore(&sdmac->vc.lock, flags);

	dma_set_tx_state(txstate, chan->completed_cookie, chan->cookie,
			 residue);

	return sdmac->status;
}

static void sdma_issue_pending(struct dma_chan *chan)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	unsigned long flags;

	sdma_pm_clk_enable(sdmac->sdma, false, true);
	spin_lock_irqsave(&sdmac->vc.lock, flags);
	if (vchan_issue_pending(&sdmac->vc) && !sdmac->desc)
		sdma_start_desc(sdmac);
	spin_unlock_irqrestore(&sdmac->vc.lock, flags);

	sdma_pm_clk_enable(sdmac->sdma, false, false);
}

static void sdma_load_firmware(const struct firmware *fw, void *context)
{
	struct sdma_engine *sdma = context;
	const struct sdma_firmware_header *header;

	if (!fw) {
		/* Load firmware once more time if timeout */
		if (sdma->fw_fail)
			dev_info(sdma->dev, "external firmware not found, using ROM firmware\n");
		else {
			/*add a bit delay to wait for firmware priv released */
			msleep(20);
			request_firmware_nowait(THIS_MODULE,
					FW_ACTION_HOTPLUG, sdma->fw_name,
					sdma->dev, GFP_KERNEL, sdma,
					sdma_load_firmware);
			sdma->fw_fail++;
		}

		return;
	}

	if (fw->size < sizeof(*header) || sdma->fw_loaded)
		goto err_firmware;

	header = (struct sdma_firmware_header *)fw->data;

	if (header->magic != SDMA_FIRMWARE_MAGIC)
		goto err_firmware;
	if (header->ram_code_start + header->ram_code_size > fw->size)
		goto err_firmware;
	switch (header->version_major) {
	case 1:
		sdma->script_number = SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V1;
		break;
	case 2:
		sdma->script_number = SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V2;
		break;
	case 3:
		sdma->script_number = SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V3;
		break;
	case 4:
		sdma->script_number = SDMA_SCRIPT_ADDRS_ARRAY_SIZE_V4;
		break;
	default:
		dev_err(sdma->dev, "unknown firmware version\n");
		goto err_firmware;
	}

	dev_info(sdma->dev, "firmware found.\n");

	if (!sdma->fw_data) {
		sdma->fw_data = kmalloc(fw->size, GFP_KERNEL);
		if (!sdma->fw_data)
			goto err_firmware;

		memcpy(sdma->fw_data, fw->data, fw->size);

		if (!sdma->drvdata->pm_runtime)
			sdma_runtime_resume(sdma->dev);
	}

err_firmware:
	release_firmware(fw);
}

#define EVENT_REMAP_CELLS 3

static int sdma_event_remap(struct sdma_engine *sdma)
{
	struct device_node *np = sdma->dev->of_node;
	struct device_node *gpr_np = of_parse_phandle(np, "gpr", 0);
	struct property *event_remap;
	struct regmap *gpr;
	char propname[] = "fsl,sdma-event-remap";
	u32 reg, val, shift, num_map, i;
	int ret = 0;

	if (IS_ERR(np) || !gpr_np)
		goto out;

	event_remap = of_find_property(np, propname, NULL);
	num_map = event_remap ? (event_remap->length / sizeof(u32)) : 0;
	if (!num_map) {
		dev_dbg(sdma->dev, "no event needs to be remapped\n");
		goto out;
	} else if (num_map % EVENT_REMAP_CELLS) {
		dev_err(sdma->dev, "the property %s must modulo %d\n",
				propname, EVENT_REMAP_CELLS);
		ret = -EINVAL;
		goto out;
	}

	gpr = syscon_node_to_regmap(gpr_np);
	if (IS_ERR(gpr)) {
		dev_err(sdma->dev, "failed to get gpr regmap\n");
		ret = PTR_ERR(gpr);
		goto out;
	}

	for (i = 0; i < num_map; i += EVENT_REMAP_CELLS) {
		ret = of_property_read_u32_index(np, propname, i, &reg);
		if (ret) {
			dev_err(sdma->dev, "failed to read property %s index %d\n",
					propname, i);
			goto out;
		}

		ret = of_property_read_u32_index(np, propname, i + 1, &shift);
		if (ret) {
			dev_err(sdma->dev, "failed to read property %s index %d\n",
					propname, i + 1);
			goto out;
		}

		ret = of_property_read_u32_index(np, propname, i + 2, &val);
		if (ret) {
			dev_err(sdma->dev, "failed to read property %s index %d\n",
					propname, i + 2);
			goto out;
		}

		regmap_update_bits(gpr, reg, BIT(shift), val << shift);
	}

out:
	if (gpr_np)
		of_node_put(gpr_np);

	return ret;
}

static int sdma_get_firmware(struct sdma_engine *sdma,
		const char *fw_name)
{
	int ret;

	ret = request_firmware_nowait(THIS_MODULE,
			FW_ACTION_HOTPLUG, fw_name, sdma->dev,
			GFP_KERNEL, sdma, sdma_load_firmware);

	return ret;
}

static int sdma_init_sw(struct sdma_engine *sdma)
{
	int ret, ccbsize;

	if (sdma->drvdata->check_ratio &&
	    (clk_get_rate(sdma->clk_ahb) == clk_get_rate(sdma->clk_ipg)))
		sdma->clk_ratio = 1;

	ccbsize = MAX_DMA_CHANNELS * (sizeof(struct sdma_channel_control)
		+ sizeof(struct sdma_context_data));

	if (sdma->iram_pool)
		sdma->channel_control = gen_pool_dma_alloc(sdma->iram_pool,
							   ccbsize, &sdma->ccb_phys);
	else
		sdma->channel_control = dma_alloc_coherent(sdma->dev, ccbsize,
						&sdma->ccb_phys, GFP_KERNEL);
	if (!sdma->channel_control) {
		ret = -ENOMEM;
		return ret;
	}

	sdma->context = (void *)sdma->channel_control +
		MAX_DMA_CHANNELS * sizeof(struct sdma_channel_control);
	sdma->context_phys = sdma->ccb_phys +
		MAX_DMA_CHANNELS * sizeof(struct sdma_channel_control);

	return 0;
}

static bool sdma_filter_fn(struct dma_chan *chan, void *fn_param)
{
	struct sdma_channel *sdmac = to_sdma_chan(chan);
	struct imx_dma_data *data = fn_param;

	if (!imx_dma_is_general_purpose(chan))
		return false;

	sdmac->data = *data;
	chan->private = &sdmac->data;

	return true;
}

static struct dma_chan *sdma_xlate(struct of_phandle_args *dma_spec,
				   struct of_dma *ofdma)
{
	struct sdma_engine *sdma = ofdma->of_dma_data;
	dma_cap_mask_t mask = sdma->dma_device.cap_mask;
	struct imx_dma_data data;

	if (dma_spec->args_count != 3)
		return NULL;

	memset(&data, 0, sizeof(data));

	data.dma_request = dma_spec->args[0];
	data.peripheral_type = dma_spec->args[1];
	data.priority = dma_spec->args[2] & 0xff;
	/*
	 * init dma_request2 to zero, which is not used by the dts.
	 * For P2P, dma_request2 is init from dma_request_channel(),
	 * chan->private will point to the imx_dma_data, and in
	 * device_alloc_chan_resources(), imx_dma_data.dma_request2 will
	 * be set to sdmac->event_id1.
	 */
	data.dma_request2 = 0;

	return __dma_request_channel(&mask, sdma_filter_fn, &data,
				     ofdma->of_node);
}

static int sdma_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *spba_bus;
	const char *fw_name;
	int ret;
	int irq;
	struct resource *iores;
	struct resource spba_res;
	int i;
	struct sdma_engine *sdma;
	s32 *saddr_arr;

	ret = dma_coerce_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	sdma = devm_kzalloc(&pdev->dev, sizeof(*sdma), GFP_KERNEL);
	if (!sdma)
		return -ENOMEM;

	spin_lock_init(&sdma->channel_0_lock);

	sdma->dev = &pdev->dev;
	sdma->drvdata = of_device_get_match_data(sdma->dev);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	iores = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	sdma->regs = devm_ioremap_resource(&pdev->dev, iores);
	if (IS_ERR(sdma->regs))
		return PTR_ERR(sdma->regs);

	sdma->clk_ipg = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(sdma->clk_ipg))
		return PTR_ERR(sdma->clk_ipg);

	sdma->clk_ahb = devm_clk_get(&pdev->dev, "ahb");
	if (IS_ERR(sdma->clk_ahb))
		return PTR_ERR(sdma->clk_ahb);

	ret = clk_prepare(sdma->clk_ipg);
	if (ret)
		return ret;

	ret = clk_prepare(sdma->clk_ahb);
	if (ret)
		goto err_clk;

	ret = devm_request_irq(&pdev->dev, irq, sdma_int_handler, 0,
				dev_name(&pdev->dev), sdma);
	if (ret)
		goto err_irq;

	sdma->irq = irq;

	sdma->script_addrs = kzalloc(sizeof(*sdma->script_addrs), GFP_KERNEL);
	if (!sdma->script_addrs) {
		ret = -ENOMEM;
		goto err_irq;
	}

	/* initially no scripts available */
	saddr_arr = (s32 *)sdma->script_addrs;
	for (i = 0; i < sizeof(*sdma->script_addrs) / sizeof(s32); i++)
		saddr_arr[i] = -EINVAL;

	dma_cap_set(DMA_SLAVE, sdma->dma_device.cap_mask);
	dma_cap_set(DMA_CYCLIC, sdma->dma_device.cap_mask);
	dma_cap_set(DMA_MEMCPY, sdma->dma_device.cap_mask);

	INIT_LIST_HEAD(&sdma->dma_device.channels);
	/* Initialize channel parameters */
	for (i = 0; i < MAX_DMA_CHANNELS; i++) {
		struct sdma_channel *sdmac = &sdma->channel[i];

		sdmac->sdma = sdma;

		sdmac->channel = i;
		sdmac->vc.desc_free = sdma_desc_free;
		INIT_LIST_HEAD(&sdmac->terminated);
		INIT_WORK(&sdmac->terminate_worker,
				sdma_channel_terminate_work);
		/*
		 * Add the channel to the DMAC list. Do not add channel 0 though
		 * because we need it internally in the SDMA driver. This also means
		 * that channel 0 in dmaengine counting matches sdma channel 1.
		 */
		if (i)
			vchan_init(&sdmac->vc, &sdma->dma_device);
	}

	if (np) {
		sdma->iram_pool = of_gen_pool_get(np, "iram", 0);
		if (sdma->iram_pool)
			dev_info(&pdev->dev, "alloc bd from iram.\n");
	}

	ret = sdma_init_sw(sdma);
	if (ret)
		goto err_init;

	ret = sdma_event_remap(sdma);
	if (ret)
		goto err_init;

	if (sdma->drvdata->script_addrs)
		sdma_add_scripts(sdma, sdma->drvdata->script_addrs);

	sdma->dma_device.dev = &pdev->dev;

	sdma->dma_device.device_alloc_chan_resources = sdma_alloc_chan_resources;
	sdma->dma_device.device_free_chan_resources = sdma_free_chan_resources;
	sdma->dma_device.device_tx_status = sdma_tx_status;
	sdma->dma_device.device_prep_slave_sg = sdma_prep_slave_sg;
	sdma->dma_device.device_prep_dma_cyclic = sdma_prep_dma_cyclic;
	sdma->dma_device.device_config = sdma_config;
	sdma->dma_device.device_terminate_all = sdma_terminate_all;
	sdma->dma_device.device_synchronize = sdma_channel_synchronize;
	sdma->dma_device.src_addr_widths = SDMA_DMA_BUSWIDTHS;
	sdma->dma_device.dst_addr_widths = SDMA_DMA_BUSWIDTHS;
	sdma->dma_device.directions = SDMA_DMA_DIRECTIONS;
	sdma->dma_device.residue_granularity = DMA_RESIDUE_GRANULARITY_SEGMENT;
	sdma->dma_device.device_prep_dma_memcpy = sdma_prep_memcpy;
	sdma->dma_device.device_issue_pending = sdma_issue_pending;
	sdma->dma_device.copy_align = 2;
	dma_set_max_seg_size(sdma->dma_device.dev, SDMA_BD_MAX_CNT);

	platform_set_drvdata(pdev, sdma);

	ret = dma_async_device_register(&sdma->dma_device);
	if (ret) {
		dev_err(&pdev->dev, "unable to register\n");
		goto err_init;
	}

	if (np) {
		ret = of_dma_controller_register(np, sdma_xlate, sdma);
		if (ret) {
			dev_err(&pdev->dev, "failed to register controller\n");
			goto err_register;
		}

		spba_bus = of_find_compatible_node(NULL, NULL, "fsl,spba-bus");
		ret = of_address_to_resource(spba_bus, 0, &spba_res);
		if (!ret) {
			sdma->spba_start_addr = spba_res.start;
			sdma->spba_end_addr = spba_res.end;
		}
		of_node_put(spba_bus);
	}

	/*
	 * Because that device tree does not encode ROM script address,
	 * the RAM script in firmware is mandatory for device tree
	 * probe, otherwise it fails.
	 */
	ret = of_property_read_string(np, "fsl,sdma-ram-script-name",
				      &fw_name);
	if (ret)
		dev_warn(&pdev->dev, "failed to get firmware name\n");
	else
		sdma->fw_name = fw_name;

	ret = sdma_get_firmware(sdma, sdma->fw_name);
	if (ret)
		dev_warn(sdma->dev, "failed to get firmware.\n");

	/* enable autosuspend for pm_runtime */
	if (sdma->drvdata->pm_runtime) {
		/* For the largest period size 64KB, the dma transfer time
		 * is about 8 sec.
		 */
		pm_runtime_set_autosuspend_delay(&pdev->dev, 8000);
		pm_runtime_use_autosuspend(&pdev->dev);
		pm_runtime_mark_last_busy(&pdev->dev);
		pm_runtime_set_suspended(&pdev->dev);
		pm_runtime_enable(&pdev->dev);
	}

	return 0;

err_register:
	dma_async_device_unregister(&sdma->dma_device);
err_init:
	kfree(sdma->script_addrs);
err_irq:
	clk_unprepare(sdma->clk_ahb);
err_clk:
	clk_unprepare(sdma->clk_ipg);
	return ret;
}

static int sdma_remove(struct platform_device *pdev)
{
	struct sdma_engine *sdma = platform_get_drvdata(pdev);
	int i;

	devm_free_irq(&pdev->dev, sdma->irq, sdma);
	dma_async_device_unregister(&sdma->dma_device);
	kfree(sdma->script_addrs);
	kfree(sdma->fw_data);
	clk_unprepare(sdma->clk_ahb);
	clk_unprepare(sdma->clk_ipg);
	/* Kill the tasklet */
	for (i = 0; i < MAX_DMA_CHANNELS; i++) {
		struct sdma_channel *sdmac = &sdma->channel[i];

		tasklet_kill(&sdmac->vc.task);
		sdma_free_chan_resources(&sdmac->vc.chan);
	}

	if (sdma->drvdata->pm_runtime) {
		pm_runtime_disable(&pdev->dev);
		pm_runtime_dont_use_autosuspend(&pdev->dev);
	} else {
		sdma_runtime_suspend(&pdev->dev);
	}

	platform_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int sdma_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sdma_engine *sdma = platform_get_drvdata(pdev);
	int i, ret = 0;

	/* Do nothing if not i.MX6SX/6UL or i.MX7D, i.MX8MP */
	if (sdma->drvdata != &sdma_imx6sx && sdma->drvdata != &sdma_imx7d
	    && sdma->drvdata != &sdma_imx6ul && sdma->drvdata != &sdma_imx8mp)
		return 0;

	if (!sdma->is_on)
		return 0;

	ret = sdma_save_restore_context(sdma, true);
	if (ret) {
		dev_err(sdma->dev, "save context error!\n");
		return ret;
	}

	/* save regs */
	for (i = 0; i < MXC_SDMA_SAVED_REG_NUM; i++) {
		/*
		 * 0x78(SDMA_XTRIG_CONF2+4)~0x100(SDMA_CHNPRI_O) registers are
		 * reserved and can't be touched. Skip these regs.
		 */
		if (i > SDMA_XTRIG_CONF2 / 4)
			sdma->save_regs[i] = readl_relaxed(sdma->regs +
							   MXC_SDMA_RESERVED_REG
							   + 4 * i);
		else
			sdma->save_regs[i] = readl_relaxed(sdma->regs + 4 * i);
	}

	if (sdma->drvdata->has_done0) {
		for (i = 0; i < 2; i++)
			sdma->save_done0_regs[i] =
			readl_relaxed(sdma->regs + SDMA_DONE0_CONFIG + 4 * i);
	}

	return 0;
}

static int sdma_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sdma_engine *sdma = platform_get_drvdata(pdev);
	int i, ret;

	/* Do nothing if not i.MX6SX/6UL or i.MX7D, i.MX8MP */
	if (sdma->drvdata != &sdma_imx6sx && sdma->drvdata != &sdma_imx7d
	    && sdma->drvdata != &sdma_imx6ul && sdma->drvdata != &sdma_imx8mp)
		return 0;

	if (!sdma->is_on)
		return 0;

	/* Do nothing if mega/fast mix not turned off */
	if (readl_relaxed(sdma->regs + SDMA_H_C0PTR))
		return 0;

	/* Firmware was lost, mark as "not ready" */
	sdma->fw_loaded = false;

	/* restore regs and load firmware */
	for (i = 0; i < MXC_SDMA_SAVED_REG_NUM; i++) {
		/*
		 * 0x78(SDMA_XTRIG_CONF2+4)~0x100(SDMA_CHNPRI_O) registers are
		 * reserved and can't be touched. Skip these regs.
		 */
		if (i > SDMA_XTRIG_CONF2 / 4)
			writel_relaxed(sdma->save_regs[i], sdma->regs +
				       MXC_SDMA_RESERVED_REG + 4 * i);
		/* set static context switch  mode before channel0 running */
		else if (i == SDMA_H_CONFIG / 4)
			writel_relaxed(sdma->save_regs[i] & ~SDMA_H_CONFIG_CSM,
					sdma->regs + SDMA_H_CONFIG);
		else
			writel_relaxed(sdma->save_regs[i], sdma->regs + 4 * i);
	}

	/* restore SDMA_DONEx_CONFIG */
	if (sdma->drvdata->has_done0) {
		for (i = 0; i < 2; i++)
			writel_relaxed(sdma->save_done0_regs[i],
				sdma->regs + SDMA_DONE0_CONFIG + 4 * i);
	}

	/* prepare priority for channel0 to start */
	sdma_set_channel_priority(&sdma->channel[0], MXC_SDMA_MAX_PRIORITY);

	if (sdma_load_script(sdma))
		dev_warn(sdma->dev, "failed to load firmware.\n");

	ret = sdma_save_restore_context(sdma, false);
	if (ret)
		dev_err(sdma->dev, "restore context error!\n");

	return ret;
}
#endif

static const struct dev_pm_ops sdma_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(sdma_suspend, sdma_resume)
	SET_RUNTIME_PM_OPS(sdma_runtime_suspend, sdma_runtime_resume, NULL)
};

static struct platform_driver sdma_driver = {
	.driver		= {
		.name	= "imx-sdma",
		.of_match_table = sdma_dt_ids,
		.pm = &sdma_pm_ops,
	},
	.remove		= sdma_remove,
	.probe		= sdma_probe,
};

module_platform_driver(sdma_driver);

MODULE_AUTHOR("Sascha Hauer, Pengutronix <s.hauer@pengutronix.de>");
MODULE_DESCRIPTION("i.MX SDMA driver");
#if IS_ENABLED(CONFIG_SOC_IMX6Q)
MODULE_FIRMWARE("imx/sdma/sdma-imx6q.bin");
#endif
#if IS_ENABLED(CONFIG_SOC_IMX7D) || IS_ENABLED(CONFIG_SOC_IMX8M)
MODULE_FIRMWARE("imx/sdma/sdma-imx7d.bin");
#endif
MODULE_LICENSE("GPL");
