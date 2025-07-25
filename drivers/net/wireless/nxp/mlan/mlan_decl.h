/** @file mlan_decl.h
 *
 *  @brief This file declares the generic data structures and APIs.
 *
 *
 *  Copyright 2008-2022, 2024-2025 NXP
 *
 *  This software file (the File) is distributed by NXP
 *  under the terms of the GNU General Public License Version 2, June 1991
 *  (the License).  You may use, redistribute and/or modify the File in
 *  accordance with the terms and conditions of the License, a copy of which
 *  is available by writing to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 *  worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 *  THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 *  ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 *  this warranty disclaimer.
 *
 */

#ifndef _MLAN_DECL_H_
#define _MLAN_DECL_H_

/** Re-define generic data types for MLAN/MOAL */
/** Signed char (1-byte) */
typedef signed char t_s8, *t_ps8;
/** Unsigned char (1-byte) */
typedef unsigned char t_u8, *t_pu8;
/** Signed short (2-bytes) */
typedef short t_s16, *t_ps16;
/** Unsigned short (2-bytes) */
typedef unsigned short t_u16, *t_pu16;
/** Signed long (4-bytes) */
typedef int t_s32, *t_ps32;
/** Unsigned long (4-bytes) */
typedef unsigned int t_u32, *t_pu32;
/** Signed long long 8-bytes) */
typedef long long t_s64, *t_ps64;
/** Unsigned long long 8-bytes) */
typedef unsigned long long t_u64, *t_pu64;
/** Void pointer (4-bytes) */
typedef void t_void, *t_pvoid;
/** Size type */
typedef t_u32 t_size;
/** Boolean type */
typedef t_u8 t_bool;

#ifdef MLAN_64BIT
/** Pointer type (64-bit) */
typedef t_u64 t_ptr;
/** Signed value (64-bit) */
typedef t_s64 t_sval;
#else
/** Pointer type (32-bit) */
typedef t_u32 t_ptr;
/** Signed value (32-bit) */
typedef t_s32 t_sval;
#endif

/** Constants below */

#ifdef __GNUC__
/** Structure packing begins */
#define MLAN_PACK_START
/** Structure packeing end */
#define MLAN_PACK_END __attribute__((packed))
#else /* !__GNUC__ */
#ifdef PRAGMA_PACK
/** Structure packing begins */
#define MLAN_PACK_START
/** Structure packeing end */
#define MLAN_PACK_END
#else /* !PRAGMA_PACK */
/** Structure packing begins */
#define MLAN_PACK_START __packed
/** Structure packing end */
#define MLAN_PACK_END
#endif /* PRAGMA_PACK */
#endif /* __GNUC__ */

#ifndef INLINE
#ifdef __GNUC__
/** inline directive */
#define INLINE inline
#else
/** inline directive */
#define INLINE __inline
#endif
#endif

/** MLAN TRUE */
#define MTRUE (1)
/** MLAN FALSE */
#define MFALSE (0)

#define CHANNEL_SPEC_SNIFFER_MODE 1

#ifndef MACSTR
/** MAC address security format */
#define MACSTR "%02x:XX:XX:XX:%02x:%02x"
#endif

#ifndef MAC2STR
/** MAC address security print arguments */
#define MAC2STR(a) (a)[0], (a)[4], (a)[5]
#endif

#ifndef FULL_MACSTR
#define FULL_MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#endif
#ifndef FULL_MAC2STR
#define FULL_MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#endif

/** Macros for Data Alignment : size */
#define ALIGN_SZ(p, a) (((p) + ((a)-1)) & ~((a)-1))

/** Macros for Data Alignment : address */
#define ALIGN_ADDR(p, a)                                                       \
	((((t_ptr)(p)) + (((t_ptr)(a)) - 1)) & ~(((t_ptr)(a)) - 1))

/** Return the byte offset of a field in the given structure */
#define MLAN_FIELD_OFFSET(type, field) ((t_u32)(t_ptr) & (((type *)0)->field))
/** Return aligned offset */
#define OFFSET_ALIGN_ADDR(p, a) (t_u32)(ALIGN_ADDR(p, a) - (t_ptr)p)

/** Maximum BSS numbers */
#define MLAN_MAX_BSS_NUM (16)

/** NET IP alignment */
#define MLAN_NET_IP_ALIGN 2

/** US country code */
#define COUNTRY_CODE_US 0x10

/** DMA alignment */
/* SDIO3.0 Inrevium Adapter require 32 bit DMA alignment */
#define DMA_ALIGNMENT 32

/** max size of TxPD */
#define MAX_TXPD_SIZE 32

/** Minimum data header length */
#define MLAN_MIN_DATA_HEADER_LEN (DMA_ALIGNMENT + MAX_TXPD_SIZE)

/** rx data header length */
#define MLAN_RX_HEADER_LEN MLAN_MIN_DATA_HEADER_LEN

/** This is current limit on Maximum Tx AMPDU allowed */
#define MLAN_MAX_TX_BASTREAM_SUPPORTED 16
#define MLAN_MAX_TX_BASTREAM_DEFAULT 2
/** This is current limit on Maximum Rx AMPDU allowed */
#define MLAN_MAX_RX_BASTREAM_SUPPORTED 16

#ifdef STA_SUPPORT
/** Default Win size attached during ADDBA request */
#define MLAN_STA_AMPDU_DEF_TXWINSIZE 64
/** Default Win size attached during ADDBA response */
#define MLAN_STA_AMPDU_DEF_RXWINSIZE 64
/** RX winsize for COEX */
#define MLAN_STA_COEX_AMPDU_DEF_RXWINSIZE 16
#endif /* STA_SUPPORT */
#ifdef UAP_SUPPORT
/** Default Win size attached during ADDBA request */
#define MLAN_UAP_AMPDU_DEF_TXWINSIZE 64
/** Default Win size attached during ADDBA response */
#define MLAN_UAP_AMPDU_DEF_RXWINSIZE 64
/** RX winsize for COEX */
#define MLAN_UAP_COEX_AMPDU_DEF_RXWINSIZE 16
#endif /* UAP_SUPPORT */

#ifdef WIFI_DIRECT_SUPPORT
/** WFD use the same window size for tx/rx */
#define MLAN_WFD_AMPDU_DEF_TXRXWINSIZE 64
/** RX winsize for COEX */
#define MLAN_WFD_COEX_AMPDU_DEF_RXWINSIZE 16
#endif

/** NAN use the same window size for tx/rx */
#define MLAN_NAN_AMPDU_DEF_TXRXWINSIZE 16
/** RX winsize for COEX */
#define MLAN_NAN_COEX_AMPDU_DEF_RXWINSIZE 16

/** Block ack timeout value */
#define MLAN_DEFAULT_BLOCK_ACK_TIMEOUT 0xffff
/** Maximum Tx Win size configured for ADDBA request [10 bits] */
#define MLAN_AMPDU_MAX_TXWINSIZE 0x3ff
/** Maximum Rx Win size configured for ADDBA request [10 bits] */
#define MLAN_AMPDU_MAX_RXWINSIZE 0x3ff

/** Rate index for HR/DSSS 0 */
#define MLAN_RATE_INDEX_HRDSSS0 0
/** Rate index for HR/DSSS 3 */
#define MLAN_RATE_INDEX_HRDSSS3 3
/** Rate index for OFDM 0 */
#define MLAN_RATE_INDEX_OFDM0 4
/** Rate index for OFDM 7 */
#define MLAN_RATE_INDEX_OFDM7 11
/** Rate index for MCS 0 */
#define MLAN_RATE_INDEX_MCS0 0
/** Rate index for MCS 2 */
#define MLAN_RATE_INDEX_MCS2 2
/** Rate index for MCS 4 */
#define MLAN_RATE_INDEX_MCS4 4
/** Rate index for MCS 7 */
#define MLAN_RATE_INDEX_MCS7 7
/** Rate index for MCS 8 */
#define MLAN_RATE_INDEX_MCS8 8
/** Rate index for MCS 9 */
#define MLAN_RATE_INDEX_MCS9 9
/** Rate index for MCS11 */
#define MLAN_RATE_INDEX_MCS11 11
/** Rate index for MCS15 */
#define MLAN_RATE_INDEX_MCS15 15
/** Rate index for MCS 32 */
#define MLAN_RATE_INDEX_MCS32 32
/** Rate index for MCS 127 */
#define MLAN_RATE_INDEX_MCS127 127
#define MLAN_RATE_NSS1 1
#define MLAN_RATE_NSS2 2

/** Rate bitmap for OFDM 0 */
#define MLAN_RATE_BITMAP_OFDM0 16
/** Rate bitmap for OFDM 7 */
#define MLAN_RATE_BITMAP_OFDM7 23
/** Rate bitmap for MCS 0 */
#define MLAN_RATE_BITMAP_MCS0 32
/** Rate bitmap for MCS 127 */
#define MLAN_RATE_BITMAP_MCS127 159
#define MLAN_RATE_BITMAP_NSS1_MCS0 160
#define MLAN_RATE_BITMAP_NSS1_MCS9 169
#define MLAN_RATE_BITMAP_NSS2_MCS0 176
#define MLAN_RATE_BITMAP_NSS2_MCS9 185

/** MU beamformer */
#define DEFALUT_11AC_CAP_BEAMFORMING_RESET_MASK (MBIT(19))

/** Size of rx data buffer 3839+256 */
#define MLAN_RX_DATA_BUF_SIZE 4096

/** Size of command buffer */
/** because cal_data_size 2.4 k */
#define MRVDRV_SIZE_OF_CMD_BUFFER (3 * 1024)
/** Size of rx command buffer */
#define MLAN_RX_CMD_BUF_SIZE MRVDRV_SIZE_OF_CMD_BUFFER
/** Upload size */
#define WLAN_UPLD_SIZE MRVDRV_SIZE_OF_CMD_BUFFER

#if defined(PCIE)
#define MLAN_SSU_MAX_PKT_SIZE (283 * 4)
#define MLAN_SSU_HEADER_SIZE 256
/**
 * Size of DMA buffer to collect 10ms SSU data:
 * 2500 spectral packets, plus header
 */
#define MLAN_SSU_BUF_SIZE_1MS (MLAN_SSU_MAX_PKT_SIZE * 250)
#define MLAN_SSU_BUF_SIZE (MLAN_SSU_HEADER_SIZE + MLAN_SSU_BUF_SIZE_1MS * 10)
#define MLAN_SSU_BUF_SIZE_HOST (MLAN_SSU_BUF_SIZE)
#endif

/** driver initial the fw reset */
#define FW_RELOAD_SDIO_INBAND_RESET 1
/** out band reset trigger reset, no interface re-emulation */
#define FW_RELOAD_NO_EMULATION 2
/** out band reset with interface re-emulation */
#define FW_RELOAD_WITH_EMULATION 3
/** pcie card reset */
#define FW_RELOAD_PCIE_RESET 4
/** sdio hw reset */
#define FW_RELOAD_SDIO_HW_RESET 5
/** pcie inband reset */
#define FW_RELOAD_PCIE_INBAND_RESET 6

/** auto fw reload enable */
#define AUTO_FW_RELOAD_ENABLE MBIT(0)
/** auto fw reload enable pcie inband reset */
#define AUTO_FW_RELOAD_PCIE_INBAND_RESET MBIT(1)

#ifdef PCIE
/* Interrupt type */
enum {
	RX_DATA,
	RX_EVENT,
	TX_COMPLETE,
	RX_CMD_RESP,
	RX_CMD_DNLD,
	RX_DATA_DELAY
};
#endif
#ifdef USB
#define MLAN_USB_BLOCK_SIZE (512)
#define MLAN_USB_AGGR_MODE_NUM (0)
#define MLAN_USB_AGGR_MODE_LEN (1)
#define MLAN_USB_AGGR_MODE_LEN_V2 (2)
#define MLAN_USB_TX_AGGR_MAX_LEN (16000)
#define MLAN_USB_TX_AGGR_MAX_NUM 10
#define MLAN_USB_TX_AGGR_V2_ALIGN 4
#define MLAN_USB_TX_AGGR_HEADER 4
#define MLAN_USB_MAX_PKT_SIZE (MLAN_USB_BLOCK_SIZE * 4)

#define MLAN_USB_RX_ALIGN_SIZE MLAN_USB_BLOCK_SIZE
#define MLAN_USB_RX_MAX_AGGR_NUM (8)
#define MLAN_USB_RX_DEAGGR_TIMEOUT_USEC (200)

#define MLAN_USB_TX_AGGR_ALIGN (MLAN_USB_BLOCK_SIZE * 4)
#define MLAN_USB_TX_MAX_AGGR_NUM (8)
#define MLAN_USB_TX_MAX_AGGR_SIZE                                              \
	(MLAN_USB_BLOCK_SIZE * 4 * MLAN_USB_TX_MAX_AGGR_NUM)
#define MLAN_USB_TX_MIN_AGGR_TIMEOUT (1)
#define MLAN_USB_TX_MAX_AGGR_TIMEOUT (4)
#define MLAN_USB_TX_AGGR_TIMEOUT_MSEC MLAN_USB_TX_MIN_AGGR_TIMEOUT
#define MLAN_USB_TX_AGGR_TIMEOUT_DYN (0xFFFF)
#endif /*USB*/

/** MLAN MAC Address Length */
#define MLAN_MAC_ADDR_LENGTH (6)
/** MLAN 802.11 MAC Address */
typedef t_u8 mlan_802_11_mac_addr[MLAN_MAC_ADDR_LENGTH];

/** MLAN Maximum SSID Length */
#define MLAN_MAX_SSID_LENGTH (32)

/** RTS/FRAG related defines */
/** Minimum RTS value */
#define MLAN_RTS_MIN_VALUE (0)
/** Maximum RTS value */
#define MLAN_RTS_MAX_VALUE (2347)
/** Minimum FRAG value */
#define MLAN_FRAG_MIN_VALUE (256)
/** Maximum FRAG value */
#define MLAN_FRAG_MAX_VALUE (2346)

/** Minimum tx retry count */
#define MLAN_TX_RETRY_MIN (0)
/** Maximum tx retry count */
#define MLAN_TX_RETRY_MAX (14)

/** max Wmm AC queues */
#define MAX_AC_QUEUES 4

#ifdef SDIO
/** define SDIO block size for data Tx/Rx */
/* We support up to 480-byte block size due to FW buffer limitation. */
#define MLAN_SDIO_BLOCK_SIZE 256

/** define allocated buffer size */
#define ALLOC_BUF_SIZE MLAN_RX_DATA_BUF_SIZE
/** SDIO MP aggr pkt limit */
#define SDIO_MP_AGGR_DEF_PKT_LIMIT (16)
/** SDIO MP aggr pkt limit 8 */
#define SDIO_MP_AGGR_DEF_PKT_LIMIT_8 (8)
/** max SDIO MP aggr pkt limit */
#define SDIO_MP_AGGR_DEF_PKT_LIMIT_MAX (16)

/** SDIO IO Port mask */
#define MLAN_SDIO_IO_PORT_MASK 0xfffff
/** SDIO Block/Byte mode mask */
#define MLAN_SDIO_BYTE_MODE_MASK 0x80000000
#endif /* SDIO */

/** SD Interface */
#define INTF_SD MBIT(0)
#define IS_SD(ct) (ct & (INTF_SD << 8))
/** PCIE Interface */
#define INTF_PCIE MBIT(1)
#define IS_PCIE(ct) (ct & (INTF_PCIE << 8))
/** USB Interface */
#define INTF_USB MBIT(2)
#define IS_USB(ct) (ct & (INTF_USB << 8))

/** 8887 card type */
#define CARD_TYPE_8887 0x01
/** 8897 card type */
#define CARD_TYPE_8897 0x02
/** 8977 card type */
#define CARD_TYPE_8977 0x03
/** 8997 card type */
#define CARD_TYPE_8997 0x04
/** 8987 card type */
#define CARD_TYPE_8987 0x05
/** 9098 card type */
#define CARD_TYPE_9098 0x06
/** 9097 card type */
#define CARD_TYPE_9097 0x07
/** 8978 card type */
#define CARD_TYPE_8978 0x08
/** 9177 card type */
#define CARD_TYPE_9177 0x09
/** 8801 card type */
#define CARD_TYPE_8801 0x0a
/** OWL card type */
#define CARD_TYPE_IW624 0x0b
/** Black bird card type */
#define CARD_TYPE_AW693 0x0c
/** IW610 card type */
#define CARD_TYPE_IW610 0x0d

/** 9098 A0 reverion num */
#define CHIP_9098_REV_A0 1
#define CHIP_9098_REV_A1 2
/** 9097 CHIP REV */
#define CHIP_9097_REV_B0 1
/** Blackbird reverion num */
#define CHIP_AW693_REV_A0 1
#define CHIP_AW693_REV_A1 2

#define INTF_MASK 0xff
#define CARD_TYPE_MASK 0xff

#ifdef SDIO
/** SD8887 card type */
#define CARD_TYPE_SD8887 (CARD_TYPE_8887 | (INTF_SD << 8))
/** SD8897 card type */
#define CARD_TYPE_SD8897 (CARD_TYPE_8897 | (INTF_SD << 8))
/** SD8977 card type */
#define CARD_TYPE_SD8977 (CARD_TYPE_8977 | (INTF_SD << 8))
/** SD8978 card type */
#define CARD_TYPE_SD8978 (CARD_TYPE_8978 | (INTF_SD << 8))
/** SD8997 card type */
#define CARD_TYPE_SD8997 (CARD_TYPE_8997 | (INTF_SD << 8))
/** SD8987 card type */
#define CARD_TYPE_SD8987 (CARD_TYPE_8987 | (INTF_SD << 8))
/** SD9097 card type */
#define CARD_TYPE_SD9097 (CARD_TYPE_9097 | (INTF_SD << 8))
/** SD9098 card type */
#define CARD_TYPE_SD9098 (CARD_TYPE_9098 | (INTF_SD << 8))
/** SD9177 card type */
#define CARD_TYPE_SD9177 (CARD_TYPE_9177 | (INTF_SD << 8))
/** SD8801 card type */
#define CARD_TYPE_SD8801 (CARD_TYPE_8801 | (INTF_SD << 8))
/** SD_IW624 card type */
#define CARD_TYPE_SDIW624 (CARD_TYPE_IW624 | (INTF_SD << 8))
/** SD_IW624 card type */
#define CARD_TYPE_SDAW693 (CARD_TYPE_AW693 | (INTF_SD << 8))
/** SD_IW610 card type */
#define CARD_TYPE_SDIW610 (CARD_TYPE_IW610 | (INTF_SD << 8))

#define IS_SD8887(ct) (CARD_TYPE_SD8887 == (ct))
#define IS_SD8897(ct) (CARD_TYPE_SD8897 == (ct))
#define IS_SD8977(ct) (CARD_TYPE_SD8977 == (ct))
#define IS_SD8978(ct) (CARD_TYPE_SD8978 == (ct))
#define IS_SD8997(ct) (CARD_TYPE_SD8997 == (ct))
#define IS_SD8987(ct) (CARD_TYPE_SD8987 == (ct))
#define IS_SD9097(ct) (CARD_TYPE_SD9097 == (ct))
#define IS_SD9098(ct) (CARD_TYPE_SD9098 == (ct))
#define IS_SD9177(ct) (CARD_TYPE_SD9177 == (ct))
#define IS_SD8801(ct) (CARD_TYPE_SD8801 == (ct))
#define IS_SDIW624(ct) (CARD_TYPE_SDIW624 == (ct))
#define IS_SDAW693(ct) (CARD_TYPE_SDAW693 == (ct))
#define IS_SDIW610(ct) (CARD_TYPE_SDIW610 == (ct))

/** SD8887 Card */
#define CARD_SD8887 "SD8887"
/** SD8897 Card */
#define CARD_SD8897 "SD8897"
/** SD8977 Card */
#define CARD_SD8977 "SD8977"
/** SD8978 Card */
#define CARD_SD8978 "SDIW416"
/** SD8997 Card */
#define CARD_SD8997 "SD8997"
/** SD8987 Card */
#define CARD_SD8987 "SD8987"
/** SD9097 Card */
#define CARD_SD9097 "SDIW620"
/** SD9098 Card */
#define CARD_SD9098 "SD9098"
/** SD9177 Card */
#define CARD_SD9177 "SDIW612"
/** SD8801 Card */
#define CARD_SD8801 "SD8801"
/** SDIW624 Card */
#define CARD_SDIW624 "SDIW624"
/** SDAW693 Card */
#define CARD_SDAW693 "SDAW693"
/** SDIW610 Card */
#define CARD_SDIW610 "SDIW610"
/** SDIW623 Card */
#define CARD_SDIW623 "SDIW623"
#endif

#ifdef PCIE
/** PCIE8897 card type */
#define CARD_TYPE_PCIE8897 (CARD_TYPE_8897 | (INTF_PCIE << 8))
/** PCIE8997 card type */
#define CARD_TYPE_PCIE8997 (CARD_TYPE_8997 | (INTF_PCIE << 8))
/** PCIE9097 card type */
#define CARD_TYPE_PCIE9097 (CARD_TYPE_9097 | (INTF_PCIE << 8))
/** PCIE9098 card type */
#define CARD_TYPE_PCIE9098 (CARD_TYPE_9098 | (INTF_PCIE << 8))
/** PCIEIW624 card type */
#define CARD_TYPE_PCIEIW624 (CARD_TYPE_IW624 | (INTF_PCIE << 8))
/** PCIEAW693 card type */
#define CARD_TYPE_PCIEAW693 (CARD_TYPE_AW693 | (INTF_PCIE << 8))

#define IS_PCIE8897(ct) (CARD_TYPE_PCIE8897 == (ct))
#define IS_PCIE8997(ct) (CARD_TYPE_PCIE8997 == (ct))
#define IS_PCIE9097(ct) (CARD_TYPE_PCIE9097 == (ct))
#define IS_PCIE9098(ct) (CARD_TYPE_PCIE9098 == (ct))
#define IS_PCIEIW624(ct) (CARD_TYPE_PCIEIW624 == (ct))
#define IS_PCIEAW693(ct) (CARD_TYPE_PCIEAW693 == (ct))

/** PCIE8897 Card */
#define CARD_PCIE8897 "PCIE8897"
/** PCIE8997 Card */
#define CARD_PCIE8997 "PCIE8997"
/** PCIE9097 Card */
#define CARD_PCIE9097 "PCIEIW620"
/** PCIE9000S Card */
#define CARD_PCIE9000S "PCIE9000S"
/** PCIE9098 Card */
#define CARD_PCIE9098 "PCIE9098"
/** PCIEAW690 Card */
#define CARD_PCIEAW690 "PCIEAW690"
/** PCIEIW624 Card */
#define CARD_PCIEIW624 "PCIEIW624"
/** PCIEAW693 Card */
#define CARD_PCIEAW693 "PCIEAW693"
/** PCIEIW629 Card */
#define CARD_PCIEIW629 "PCIEIW629"
/** PCIEIW623 Card */
#define CARD_PCIEIW623 "PCIEIW623"
#endif

#ifdef USB
/** USB8801 card type */
#define CARD_TYPE_USB8801 (CARD_TYPE_8801 | (INTF_USB << 8))
/** USB8897 card type */
#define CARD_TYPE_USB8897 (CARD_TYPE_8897 | (INTF_USB << 8))
/** USB8997 card type */
#define CARD_TYPE_USB8997 (CARD_TYPE_8997 | (INTF_USB << 8))
/** USB8978 card type */
#define CARD_TYPE_USB8978 (CARD_TYPE_8978 | (INTF_USB << 8))
/** USB9098 card type */
#define CARD_TYPE_USB9098 (CARD_TYPE_9098 | (INTF_USB << 8))
/** USB9097 card type */
#define CARD_TYPE_USB9097 (CARD_TYPE_9097 | (INTF_USB << 8))
/** USBIW624 card type */
#define CARD_TYPE_USBIW624 (CARD_TYPE_IW624 | (INTF_USB << 8))
/** USBIW610 card type */
#define CARD_TYPE_USBIW610 (CARD_TYPE_IW610 | (INTF_USB << 8))

#define IS_USB8801(ct) (CARD_TYPE_USB8801 == (ct))
#define IS_USB8897(ct) (CARD_TYPE_USB8897 == (ct))
#define IS_USB8997(ct) (CARD_TYPE_USB8997 == (ct))
#define IS_USB8978(ct) (CARD_TYPE_USB8978 == (ct))
#define IS_USB9098(ct) (CARD_TYPE_USB9098 == (ct))
#define IS_USB9097(ct) (CARD_TYPE_USB9097 == (ct))
#define IS_USBIW624(ct) (CARD_TYPE_USBIW624 == (ct))
#define IS_USBIW610(ct) (CARD_TYPE_USBIW610 == (ct))

/** USB8801 Card */
#define CARD_USB8801 "USB8801"
/** USB8897 Card */
#define CARD_USB8897 "USB8897"
/** USB8997 Card */
#define CARD_USB8997 "USB8997"
/** USB8978 Card */
#define CARD_USB8978 "USBIW416"
/** USB9098 Card */
#define CARD_USB9098 "USB9098"
/** USB9097 Card */
#define CARD_USB9097 "USBIW620"
/** USBIW624 Card */
#define CARD_USBIW624 "USBIW624"
/** USBIW610 Card */
#define CARD_USBIW610 "USBIW610"
#endif

#define IS_CARD8801(ct) (CARD_TYPE_8801 == ((ct)&0xf))
#define IS_CARD8887(ct) (CARD_TYPE_8887 == ((ct)&0xf))
#define IS_CARD8897(ct) (CARD_TYPE_8897 == ((ct)&0xf))
#define IS_CARD8977(ct) (CARD_TYPE_8977 == ((ct)&0xf))
#define IS_CARD8978(ct) (CARD_TYPE_8978 == ((ct)&0xf))
#define IS_CARD8997(ct) (CARD_TYPE_8997 == ((ct)&0xf))
#define IS_CARD8987(ct) (CARD_TYPE_8987 == ((ct)&0xf))
#define IS_CARD9098(ct) (CARD_TYPE_9098 == ((ct)&0xf))
#define IS_CARD9097(ct) (CARD_TYPE_9097 == ((ct)&0xf))
#define IS_CARD9177(ct) (CARD_TYPE_9177 == ((ct)&0xf))
#define IS_CARDIW624(ct) (CARD_TYPE_IW624 == ((ct)&0xf))
#define IS_CARDAW693(ct) (CARD_TYPE_AW693 == ((ct)&0xf))
#define IS_CARDIW610(ct) (CARD_TYPE_IW610 == ((ct)&0xf))

typedef struct _card_type_entry {
	t_u16 card_type;
	t_u16 func_id;
	char *name;
} card_type_entry;

#if defined(SDIO) || defined(PCIE)
/** Max retry number of IO write */
#define MAX_WRITE_IOMEM_RETRY 2
#endif /* SDIO || PCIE */

#ifdef PCIE
typedef enum {
	PCIE_INT_MODE_LEGACY = 0,
	PCIE_INT_MODE_MSI,
	PCIE_INT_MODE_MSIX,
	PCIE_INT_MODE_MAX,
} PCIE_INT_MODE;
#endif /* PCIE */

/** IN parameter */
#define IN
/** OUT parameter */
#define OUT

/** BIT value */
#define MBIT(x) (((t_u32)1) << (x))

/** Buffer flag for requeued packet */
#define MLAN_BUF_FLAG_REQUEUED_PKT MBIT(0)
/** Buffer flag for transmit buf from moal */
#define MLAN_BUF_FLAG_MOAL_TX_BUF MBIT(1)
/** Buffer flag for malloc mlan_buffer */
#define MLAN_BUF_FLAG_MALLOC_BUF MBIT(2)

/** Buffer flag for bridge packet */
#define MLAN_BUF_FLAG_BRIDGE_BUF MBIT(3)

#ifdef USB
/** Buffer flag for deaggregated rx packet */
#define MLAN_BUF_FLAG_RX_DEAGGR MBIT(5)

/** Buffer flag for sleep confirm resp packet */
#define MLAN_BUF_FLAG_SLEEPCFM_RESP MBIT(6)

/** Buffer flag for USB TX AGGR */
#define MLAN_BUF_FLAG_USB_TX_AGGR MBIT(7)
#endif

/** Buffer flag for TDLS */
#define MLAN_BUF_FLAG_TDLS MBIT(8)

/** Buffer flag for TCP_ACK */
#define MLAN_BUF_FLAG_TCP_ACK MBIT(9)

/** Buffer flag for TX_STATUS */
#define MLAN_BUF_FLAG_TX_STATUS MBIT(10)

/** Buffer flag for NET_MONITOR */
#define MLAN_BUF_FLAG_NET_MONITOR MBIT(11)

/** Buffer flag for NULL data packet */
#define MLAN_BUF_FLAG_NULL_PKT MBIT(12)
/** Buffer flag for Diag pkt */
#define MLAN_BUF_FLAG_DIAG_BUF MBIT(13)

#define MLAN_BUF_FLAG_TX_CTRL MBIT(14)

#define MLAN_BUF_FLAG_EASYMESH MBIT(16)

#define MLAN_BUF_FLAG_MC_AGGR_PKT MBIT(17)

#define MLAN_BUF_FLAG_TCP_PKT MBIT(18)

#define MLAN_BUF_FLAG_LLDE_PKT_FILTER MBIT(19)

#ifdef DEBUG_LEVEL1
/** Debug level bit definition */
#define MMSG MBIT(0)
#define MFATAL MBIT(1)
#define MERROR MBIT(2)
#define MDATA MBIT(3)
#define MCMND MBIT(4)
#define MEVENT MBIT(5)
#define MINTR MBIT(6)
#define MIOCTL MBIT(7)
#define MREG_D MBIT(9)
#define MREG MBIT(10)

#define MMPA_D MBIT(15)
#define MDAT_D MBIT(16)
#define MCMD_D MBIT(17)
#define MEVT_D MBIT(18)
#define MFW_D MBIT(19)
#define MIF_D MBIT(20)
#ifdef FWDUMP_VIA_PRINT
#define MFWDP_D MBIT(21)
#endif /*FWDUMP_VIA_PRINT*/
#define MSCH_D MBIT(22)
#define MENTRY MBIT(28)
#define MWARN MBIT(29)
#define MINFO MBIT(30)
#define MHEX_DUMP MBIT(31)
#endif /* DEBUG_LEVEL1 */

/** Memory allocation type: DMA */
#define MLAN_MEM_DMA MBIT(0)
/** Memory allocation flag: ATOMIC */
#define MLAN_MEM_FLAG_ATOMIC MBIT(1)
/** Memory allocation flag: DIRTY - do not memset to 0 buffer */
#define MLAN_MEM_FLAG_DIRTY MBIT(2)

/** Default memory allocation flag */
#define MLAN_MEM_DEF 0

/** mlan_status */
typedef enum _mlan_status {
	MLAN_STATUS_FAILURE = 0xffffffff,
	MLAN_STATUS_SUCCESS = 0,
	MLAN_STATUS_PENDING,
	MLAN_STATUS_RESOURCE,
#ifdef USB
	/* Status pending and no resource */
	MLAN_STATUS_PRESOURCE,
#endif
	MLAN_STATUS_COMPLETE,
	MLAN_STATUS_FILE_ERR,
} mlan_status;

/** mlan_error_code */
typedef enum _mlan_error_code {
	/** No error */
	MLAN_ERROR_NO_ERROR = 0,
	/** Firmware/device errors below (MSB=0) */
	MLAN_ERROR_FW_NOT_READY = 0x00000001,
	MLAN_ERROR_FW_BUSY = 0x00000002,
	MLAN_ERROR_FW_CMDRESP = 0x00000003,
	MLAN_ERROR_DATA_TX_FAIL = 0x00000004,
	MLAN_ERROR_DATA_RX_FAIL = 0x00000005,
	/** Driver errors below (MSB=1) */
	MLAN_ERROR_PKT_SIZE_INVALID = 0x80000001,
	MLAN_ERROR_PKT_TIMEOUT = 0x80000002,
	MLAN_ERROR_PKT_INVALID = 0x80000003,
	MLAN_ERROR_CMD_INVALID = 0x80000004,
	MLAN_ERROR_CMD_TIMEOUT = 0x80000005,
	MLAN_ERROR_CMD_DNLD_FAIL = 0x80000006,
	MLAN_ERROR_CMD_CANCEL = 0x80000007,
	MLAN_ERROR_CMD_RESP_FAIL = 0x80000008,
	MLAN_ERROR_CMD_ASSOC_FAIL = 0x80000009,
	MLAN_ERROR_CMD_SCAN_FAIL = 0x8000000A,
	MLAN_ERROR_IOCTL_INVALID = 0x8000000B,
	MLAN_ERROR_IOCTL_FAIL = 0x8000000C,
	MLAN_ERROR_EVENT_UNKNOWN = 0x8000000D,
	MLAN_ERROR_INVALID_PARAMETER = 0x8000000E,
	MLAN_ERROR_NO_MEM = 0x8000000F,
	/** More to add */
} mlan_error_code;

/** mlan_buf_type */
typedef enum _mlan_buf_type {
	MLAN_BUF_TYPE_CMD = 1,
	MLAN_BUF_TYPE_DATA,
	MLAN_BUF_TYPE_EVENT,
	MLAN_BUF_TYPE_RAW_DATA,
#ifdef SDIO
	MLAN_BUF_TYPE_SPA_DATA,
#endif
} mlan_buf_type;

/** assoc_logger_data */
typedef struct _assoc_logger {
	/** vendor specific */
	t_u8 oui[3];
	/** bssid */
	t_u8 bssid[MLAN_MAC_ADDR_LENGTH];
	/** ssid length */
	t_u8 ssid_len;
	/** ssid */
	t_u8 ssid[MLAN_MAX_SSID_LENGTH];
	/** rssi */
	t_s32 rssi;
	/** channel */
	t_u32 channel;
} assoc_logger_data;

#define SCAN_STATE_SCAN_START MBIT(0)
#define SCAN_STATE_EXT_SCAN MBIT(1)
#define SCAN_STATE_EXT_SCAN_ENH MBIT(2)
#define SCAN_STATE_EXT_SCAN_CANCEL MBIT(3)
#define SCAN_STATE_EXT_SCAN_CMDRESP MBIT(4)
#define SCAN_STATE_EXT_SCAN_ENH_CMDRESP MBIT(5)
#define SCAN_STATE_EXT_SCAN_CANCEL_CMDRESP MBIT(6)
#define SCAN_STATE_EXT_SCAN_RESULT MBIT(7)
#define SCAN_STATE_LAST_EXT_SCAN_RESULT MBIT(8)
#define SCAN_STATE_EXT_SCAN_STATUS MBIT(9)
#define SCAN_STATE_SCAN_COMPLETE MBIT(10)

#ifdef USB
/** mlan_usb_ep */
typedef enum _mlan_usb_ep {
	MLAN_USB_EP_CTRL = 0,
	MLAN_USB_EP_CMD_EVENT = 1,
	MLAN_USB_EP_DATA = 2,
	MLAN_USB_EP_DATA_CH2 = 3,
	MLAN_USB_EP_CMD_EVENT_IF2 = 4,
	MLAN_USB_EP_DATA_IF2 = 5,
	MLAN_USB_EP_DATA_CH2_IF2 = 6,
} mlan_usb_ep;

/** Timeout in milliseconds for usb_bulk_msg function */
#define MLAN_USB_BULK_MSG_TIMEOUT 100
#endif /* USB */

/** MLAN BSS type */
typedef enum _mlan_bss_type {
	MLAN_BSS_TYPE_STA = 0,
	MLAN_BSS_TYPE_UAP = 1,
#ifdef WIFI_DIRECT_SUPPORT
	MLAN_BSS_TYPE_WIFIDIRECT = 2,
#endif
	MLAN_BSS_TYPE_NAN = 4,
	MLAN_BSS_TYPE_DFS = 8,
	MLAN_BSS_TYPE_ANY = 0xff,
} mlan_bss_type;

/** MLAN BSS role */
typedef enum _mlan_bss_role {
	MLAN_BSS_ROLE_STA = 0,
	MLAN_BSS_ROLE_UAP = 1,
	MLAN_BSS_ROLE_ANY = 0xff,
} mlan_bss_role;

/** BSS role mask */
#define BSS_ROLE_MASK (MBIT(0) | MBIT(1))

/** Get BSS role */
#define GET_BSS_ROLE(priv) ((priv)->bss_role & BSS_ROLE_MASK)

/** mlan_data_frame_type */
typedef enum _mlan_data_frame_type {
	MLAN_DATA_FRAME_TYPE_ETH_II = 0,
	MLAN_DATA_FRAME_TYPE_802_11,
} mlan_data_frame_type;

/** mlan_event_id */
typedef enum _mlan_event_id {
	/* Event generated by firmware (MSB=0) */
	MLAN_EVENT_ID_FW_UNKNOWN = 0x00000001,
	MLAN_EVENT_ID_FW_DISCONNECTED = 0x00000004,
	MLAN_EVENT_ID_FW_MIC_ERR_UNI = 0x00000005,
	MLAN_EVENT_ID_FW_MIC_ERR_MUL = 0x00000006,
	MLAN_EVENT_ID_FW_BCN_RSSI_LOW = 0x00000007,
	MLAN_EVENT_ID_FW_BCN_RSSI_HIGH = 0x00000008,
	MLAN_EVENT_ID_FW_BCN_SNR_LOW = 0x00000009,
	MLAN_EVENT_ID_FW_BCN_SNR_HIGH = 0x0000000A,
	MLAN_EVENT_ID_FW_MAX_FAIL = 0x0000000B,
	MLAN_EVENT_ID_FW_DATA_RSSI_LOW = 0x0000000C,
	MLAN_EVENT_ID_FW_DATA_RSSI_HIGH = 0x0000000D,
	MLAN_EVENT_ID_FW_DATA_SNR_LOW = 0x0000000E,
	MLAN_EVENT_ID_FW_DATA_SNR_HIGH = 0x0000000F,
	MLAN_EVENT_ID_FW_LINK_QUALITY = 0x00000010,
	MLAN_EVENT_ID_FW_PORT_RELEASE = 0x00000011,
	MLAN_EVENT_ID_FW_PRE_BCN_LOST = 0x00000012,
	MLAN_EVENT_ID_FW_DEBUG_INFO = 0x00000013,
	MLAN_EVENT_ID_FW_WMM_CONFIG_CHANGE = 0x0000001A,
	MLAN_EVENT_ID_FW_HS_WAKEUP = 0x0000001B,
	MLAN_EVENT_ID_FW_BG_SCAN = 0x0000001D,
	MLAN_EVENT_ID_FW_BG_SCAN_STOPPED = 0x0000001E,
	MLAN_EVENT_ID_FW_WEP_ICV_ERR = 0x00000020,
	MLAN_EVENT_ID_FW_STOP_TX = 0x00000021,
	MLAN_EVENT_ID_FW_START_TX = 0x00000022,
	MLAN_EVENT_ID_FW_CHANNEL_SWITCH_ANN = 0x00000023,
	MLAN_EVENT_ID_FW_RADAR_DETECTED = 0x00000024,
	MLAN_EVENT_ID_FW_CHANNEL_REPORT_RDY = 0x00000025,
	MLAN_EVENT_ID_FW_BW_CHANGED = 0x00000026,
	MLAN_EVENT_ID_FW_REMAIN_ON_CHAN_EXPIRED = 0x0000002B,

#ifdef UAP_SUPPORT
	MLAN_EVENT_ID_UAP_FW_BSS_START = 0x0000002C,
	MLAN_EVENT_ID_UAP_FW_BSS_ACTIVE = 0x0000002D,
	MLAN_EVENT_ID_UAP_FW_BSS_IDLE = 0x0000002E,
	MLAN_EVENT_ID_UAP_FW_MIC_COUNTERMEASURES = 0x0000002F,
	MLAN_EVENT_ID_UAP_FW_STA_CONNECT = 0x00000030,
	MLAN_EVENT_ID_UAP_FW_STA_DISCONNECT = 0x00000031,
#endif

	MLAN_EVENT_ID_FW_DUMP_INFO = 0x00000033,

	MLAN_EVENT_ID_FW_TX_STATUS = 0x00000034,
	MLAN_EVENT_ID_FW_CHAN_SWITCH_COMPLETE = 0x00000036,
#if defined(PCIE)
	MLAN_EVENT_ID_SSU_DUMP_FILE = 0x00000039,
#endif /* SSU_SUPPORT */
	MLAN_EVENT_ID_CSI = 0x00000040,
	MLAN_EVENT_ID_FW_TX_BULK_STATUS = 0x00000043,
	/* Event generated by MLAN driver (MSB=1) */
	MLAN_EVENT_ID_DRV_CONNECTED = 0x80000001,
	MLAN_EVENT_ID_DRV_DEFER_HANDLING = 0x80000002,
	MLAN_EVENT_ID_DRV_HS_ACTIVATED = 0x80000003,
	MLAN_EVENT_ID_DRV_HS_DEACTIVATED = 0x80000004,
	MLAN_EVENT_ID_DRV_MGMT_FRAME = 0x80000005,
	MLAN_EVENT_ID_DRV_OBSS_SCAN_PARAM = 0x80000006,
	MLAN_EVENT_ID_DRV_PASSTHRU = 0x80000007,
	MLAN_EVENT_ID_DRV_SCAN_REPORT = 0x80000009,
	MLAN_EVENT_ID_DRV_MEAS_REPORT = 0x8000000A,
	MLAN_EVENT_ID_DRV_ASSOC_FAILURE_REPORT = 0x8000000B,
	MLAN_EVENT_ID_DRV_REPORT_STRING = 0x8000000F,
	MLAN_EVENT_ID_DRV_DBG_DUMP = 0x80000012,
	MLAN_EVENT_ID_DRV_BGSCAN_RESULT = 0x80000013,
	MLAN_EVENT_ID_DRV_FLUSH_RX_WORK = 0x80000015,
	MLAN_EVENT_ID_DRV_DEFER_RX_WORK = 0x80000016,
	MLAN_EVENT_ID_DRV_TDLS_TEARDOWN_REQ = 0x80000017,
	MLAN_EVENT_ID_DRV_FT_RESPONSE = 0x80000018,
	MLAN_EVENT_ID_DRV_FLUSH_MAIN_WORK = 0x80000019,
#ifdef UAP_SUPPORT
	MLAN_EVENT_ID_DRV_UAP_CHAN_INFO = 0x80000020,
#endif
	MLAN_EVENT_ID_FW_ROAM_OFFLOAD_RESULT = 0x80000023,
	MLAN_EVENT_ID_NAN_STARTED = 0x80000024,
	MLAN_EVENT_ID_DRV_RTT_RESULT = 0x80000025,
	MLAN_EVENT_ID_DRV_ASSOC_FAILURE_LOGGER = 0x80000026,
	MLAN_EVENT_ID_DRV_ASSOC_SUCC_LOGGER = 0x80000027,
	MLAN_EVENT_ID_DRV_DISCONNECT_LOGGER = 0x80000028,
	MLAN_EVENT_ID_DRV_WIFI_STATUS = 0x80000029,
	MLAN_EVENT_ID_STORE_HOST_CMD_RESP = 0x80000030,
	MLAN_EVENT_ID_DRV_ADDBA_TIMEOUT = 0x80000031,
#ifdef PCIE
	MLAN_EVENT_ID_DRV_DEFER_RX_DATA = 0x80000032,
	MLAN_EVENT_ID_DRV_DEFER_CMDRESP = 0x80000034,
	MLAN_EVENT_ID_DRV_DEFER_TX_COMPLTE = 0x80000035,
	MLAN_EVENT_ID_DRV_DELAY_TX_COMPLETE = 0x80000036,
#endif
	MLAN_EVENT_ID_DRV_RGPWR_KEY_MISMATCH = 0x80000037,
#if defined(STA_SUPPORT)
	MLAN_EVENT_ID_FW_CHAN_SWITCH_REGINFO = 0x80000038,
#endif
	MLAN_EVENT_ID_DRV_ASSOC_FAILURE = 0x80000039,
} mlan_event_id;

/** Data Structures */
/** mlan_image data structure */
typedef struct _mlan_fw_image {
	/** Firmware image buffer pointer */
	const t_u8 *pfw_buf;
	/** Firmware image length */
	t_u32 fw_len;
	/** Firmware reload flag */
	t_u8 fw_reload;
} mlan_fw_image, *pmlan_fw_image;

/** MrvlIEtypesHeader_t */
typedef MLAN_PACK_START struct _MrvlIEtypesHeader {
	/** Header type */
	t_u16 type;
	/** Header length */
	t_u16 len;
} MLAN_PACK_END MrvlIEtypesHeader_t;

/** MrvlExtIEtypesHeader_t */
typedef MLAN_PACK_START struct _MrvlExtIEtypesHeader {
	/** Header type */
	t_u16 type;
	/** Header length */
	t_u16 len;
	/** ext id */
	t_u8 ext_id;
} MLAN_PACK_END MrvlExtIEtypesHeader_t;

/** MrvlIEtypes_Data_t */
typedef MLAN_PACK_START struct _MrvlExtIEtypes_Data_t {
	/** Header */
	MrvlExtIEtypesHeader_t header;
	/** Data */
	t_u8 data[];
} MLAN_PACK_END MrvlExtIEtypes_Data_t;

/** MrvlIEtypes_Data_t */
typedef MLAN_PACK_START struct _MrvlIEtypes_Data_t {
	/** Header */
	MrvlIEtypesHeader_t header;
	/** Data */
	t_u8 data[];
} MLAN_PACK_END MrvlIEtypes_Data_t;

#define OID_TYPE_CAL 0x2
#define OID_TYPE_DPD 0xa
#define NONE_TYPE_DPD 0xb
#define UNKNOW_DPD_LENGTH 0xffffffff
#define NONE_DPD_LENGTH 0xfafafafa

/** Custom data structure */
typedef struct _mlan_init_param {
	/** DPD data buffer pointer */
	t_u8 *pdpd_data_buf;
	/** DPD data length */
	t_u32 dpd_data_len;
	/** region txpowerlimit cfg data buffer pointer */
	t_u8 *ptxpwr_data_buf;
	/** region txpowerlimit cfg data length */
	t_u32 txpwr_data_len;
	/** Cal data buffer pointer */
	t_u8 *pcal_data_buf;
	/** Cal data length */
	t_u32 cal_data_len;
	/** Other custom data */
} mlan_init_param, *pmlan_init_param;

/** channel type */
enum mlan_channel_type {
	CHAN_NO_HT,
	CHAN_HT20,
	CHAN_HT40MINUS,
	CHAN_HT40PLUS,
	CHAN_VHT80
};

/** channel band */
enum {
	BAND_2GHZ = 0,
	BAND_5GHZ = 1,
	BAND_6GHZ = 2,
	BAND_4GHZ = 3,
};

/** channel offset */
enum {
	SEC_CHAN_NONE = 0,
	SEC_CHAN_ABOVE = 1,
	SEC_CHAN_5MHZ = 2,
	SEC_CHAN_BELOW = 3
};

/** channel bandwidth */
enum {
	CHAN_BW_20MHZ = 0,
	CHAN_BW_10MHZ,
	CHAN_BW_40MHZ,
	CHAN_BW_80MHZ,
};

/** scan mode */
enum {
	SCAN_MODE_MANUAL = 0,
	SCAN_MODE_ACS,
	SCAN_MODE_USER,
};

/** DFS state */
typedef enum _dfs_state_t {
	/** Channel can be used, CAC (Channel Availability Check) must be done
	   before using it */
	DFS_USABLE = 0,
	/** Channel is not available, radar was detected */
	DFS_UNAVAILABLE = 1,
	/** Channel is Available, CAC is done and is free of radar */
	DFS_AVAILABLE = 2,
} dfs_state_t;

/** max cac time 10 minutes */
#define MAX_CAC_DWELL_TIME 600000
/** default cac time 60 seconds */
#define DEF_CAC_DWELL_TIME 60000
/** start freq for 5G */
#define START_FREQ_11A_BAND 5000

typedef enum _dfs_w53_cfg_t {
	/** DFS W53 Default Fw Value */
	DFS_W53_DEFAULT_FW = 0,
	/** DFS W53 New W53 Rules/Standard */
	DFS_W53_NEW = 1,
	/** DFS W53 Old W53 Rules/Standard */
	DFS_W53_OLD = 2
} dfs_w53_cfg_t;

typedef enum _dfs_moe_t {
	/** driver default DFS behavior */
	DFS_MODE_DEFAULT = 0,
	/* disable DFS master when uap and station operate in same DFS channel
	 */
	DFS_MODE_ENH = 1,
} dfs_mode_t;

/** Band_Config_t */
typedef MLAN_PACK_START struct _Band_Config_t {
#ifdef BIG_ENDIAN_SUPPORT
	/** Channel Selection Mode - (00)=manual, (01)=ACS,  (02)=user*/
	t_u8 scanMode : 2;
	/** Secondary Channel Offset - (00)=None, (01)=Above, (11)=Below */
	t_u8 chan2Offset : 2;
	/** Channel Width - (00)=20MHz, (10)=40MHz, (11)=80MHz */
	t_u8 chanWidth : 2;
	/** Band Info - (00)=2.4GHz, (01)=5GHz, (10)=6GHz */
	t_u8 chanBand : 2;
#else
	/** Band Info - (00)=2.4GHz, (01)=5GHz, (10)=6GHz */
	t_u8 chanBand : 2;
	/** Channel Width - (00)=20MHz, (10)=40MHz, (11)=80MHz */
	t_u8 chanWidth : 2;
	/** Secondary Channel Offset - (00)=None, (01)=Above, (11)=Below */
	t_u8 chan2Offset : 2;
	/** Channel Selection Mode - (00)=manual, (01)=ACS, (02)=Adoption mode*/
	t_u8 scanMode : 2;
#endif
} MLAN_PACK_END Band_Config_t;

/** channel_band_t */
typedef MLAN_PACK_START struct _chan_band_info {
	/** Band Configuration */
	Band_Config_t bandcfg;
	/** channel */
	t_u8 channel;
	/** 11n flag */
	t_u8 is_11n_enabled;
	/** center channel */
	t_u8 center_chan;
	/** dfs channel flag */
	t_u8 is_dfs_chan;
} MLAN_PACK_END chan_band_info;

/** remain_on_channel_info  */
typedef MLAN_PACK_START struct _remain_on_channel_info {
	/* Is previous AP lost when waiting doing Auth RoC */
	t_bool delay_link_lost;
} MLAN_PACK_END remain_on_channel_info;

#if defined(STA_SUPPORT)
/** MrvlIEtypes_chan_band_reginfo_t */
typedef MLAN_PACK_START struct _MrvlIEtypes_chan_band_reginfo_t {
	/** Header */
	MrvlIEtypesHeader_t header;
	/** Band Configuration */
	Band_Config_t bandcfg;
	/** channel */
	t_u8 channel;
	/** Reg Info */
	t_u8 regInfo;
} MLAN_PACK_END MrvlIEtypes_chan_band_reginfo_t;

/** Structure to store 6E Channel/Band/Regulatory Info */
typedef MLAN_PACK_START struct _chan_band_reginfo_t {
	/** Band Configuration */
	Band_Config_t bandcfg;
	/** channel */
	t_u8 channel;
	/** Reg Info */
	t_u8 regInfo;
} MLAN_PACK_END chan_band_reginfo_t;
#endif

/** Channel usability flags */
#define NXP_CHANNEL_NO_OFDM MBIT(9)
#define NXP_CHANNEL_NO_CCK MBIT(8)
#define NXP_CHANNEL_DISABLED MBIT(7)
/* BIT 5/6 resevered for FW */
#define NXP_CHANNEL_NOHT160 MBIT(4)
#define NXP_CHANNEL_NOHT80 MBIT(3)
#define NXP_CHANNEL_NOHT40 MBIT(2)
#define NXP_CHANNEL_DFS MBIT(1)
#define NXP_CHANNEL_PASSIVE MBIT(0)

/** CFP dynamic (non-const) elements */
typedef struct _cfp_dyn_t {
	/** extra flags to specify channel usability
	 *  bit 9 : if set, channel is non-OFDM
	 *  bit 8 : if set, channel is non-CCK
	 *  bit 7 : if set, channel is disabled
	 *  bit  5/6 resevered for FW
	 *  bit 4 : if set, 160MHz on channel is disabled
	 *  bit 3 : if set, 80MHz on channel is disabled
	 *  bit 2 : if set, 40MHz on channel is disabled
	 *  bit 1 : if set, channel is DFS channel
	 *  bit 0 : if set, channel is passive
	 */
	t_u16 flags;
	/** TRUE: Channel is blacklisted (do not use) */
	t_bool blacklist;
	/** DFS state of the channel
	 * 0:DFS_USABLE  1:DFS_AVAILABLE  2:DFS_UNAVAILABLE */
	dfs_state_t dfs_state;
} cfp_dyn_t;

/** Chan-Freq-TxPower mapping table*/
typedef struct _chan_freq_power_t {
	/** Channel Number */
	t_u16 channel;
	/** Frequency of this Channel */
	t_u32 freq;
	/** Max allowed Tx power level */
	t_u16 max_tx_power;
	/** TRUE:radar detect required for BAND A or passive scan for BAND B/G;
	 * FALSE:radar detect not required for BAND A or active scan for BAND
	 * B/G*/
	t_bool passive_scan_or_radar_detect;
	/** Elements associated to cfp that change at run-time */
	cfp_dyn_t dynamic;
} chan_freq_power_t;

/** mlan_event data structure */
typedef struct _mlan_event {
	/** BSS index number for multiple BSS support */
	t_u32 bss_index;
	/** Event ID */
	mlan_event_id event_id;
	/** Event length */
	t_u32 event_len;
	/** Event buffer */
	t_u8 event_buf[];
} mlan_event, *pmlan_event;

/** mlan_cmdresp_event data structure */
typedef struct _mlan_cmdresp_event {
	/** BSS index number for multiple BSS support */
	t_u32 bss_index;
	/** Event ID */
	mlan_event_id event_id;
	/** Event length */
	t_u32 event_len;
	/** resp buffer pointer */
	t_u8 *resp;
} mlan_cmdresp_event, *pmlan_cmdresp_event;

/** csi event data structure */
typedef MLAN_PACK_START struct _csi_record_ds {
	/** Length in DWORDS, including header */
	t_u16 Len;
	/** CSI signature. 0xABCD fixed */
	t_u16 CSI_Sign;
	/** User defined HeaderID  */
	t_u32 CSI_HeaderID;
	/** Packet info field */
	t_u16 PKT_info;
	/** Frame control field for the received packet*/
	t_u16 FCF;
	/** Timestamp when packet received */
	t_u64 TSF;
	/** Received Packet Destination MAC Address */
	t_u8 Dst_MAC[6];
	/** Received Packet Source MAC Address */
	t_u8 Src_MAC[6];
	/** RSSI for antenna A */
	t_u8 Rx_RSSI_A;
	/** RSSI for antenna B */
	t_u8 Rx_RSSI_B;
	/** Noise floor for antenna A */
	t_u8 Rx_NF_A;
	/** Noise floor for antenna A */
	t_u8 Rx_NF_B;
	/** Rx signal strength above noise floor */
	t_u8 Rx_SINR;
	/** Channel */
	t_u8 channel;
	/** user defined Chip ID */
	t_u16 chip_id;
	/** Reserved */
	t_u32 rsvd;
	/** CSI data length in DWORDs */
	t_u32 CSI_Data_Length;
	/** Start of CSI data */
	t_u8 CSI_Data[0];
	/** At the end of CSI raw data, user defined TailID of 4 bytes*/
} MLAN_PACK_END csi_record_ds, *pcsi_record_ds;

/** mlan_ioctl_req data structure */
typedef struct _mlan_ioctl_req {
	/** Pointer to previous mlan_ioctl_req */
	struct _mlan_ioctl_req *pprev;
	/** Pointer to next mlan_ioctl_req */
	struct _mlan_ioctl_req *pnext;
	/** Status code from firmware/driver */
	t_u32 status_code;
	/** BSS index number for multiple BSS support */
	t_u32 bss_index;
	/** Request id */
	t_u32 req_id;
	/** Action: set or get */
	t_u16 action;
	/** Pointer to buffer */
	t_u8 *pbuf;
	/** Length of buffer */
	t_u32 buf_len;
	/** Length of the data read/written in buffer */
	t_u32 data_read_written;
	/** Length of buffer needed */
	t_u32 buf_len_needed;
	/** Reserved for MOAL module */
	t_ptr reserved_1;
} mlan_ioctl_req, *pmlan_ioctl_req;

typedef MLAN_PACK_START struct _mix_rate_info {
	/**  bit0: LGI: gi=0, SGI: gi= 1 */
	/**  bit1-2: 20M: bw=0, 40M: bw=1, 80M: bw=2, 160M: bw=3  */
	/**  bit3-4: LG: format=0, HT: format=1, VHT: format=2 */
	/**  bit5: LDPC: 0-not support,  1-support */
	/**  bit6-7:reserved */
	t_u8 rate_info;
	/** MCS index */
	t_u8 mcs_index;
	/** bitrate, in 500Kbps */
	t_u16 bitrate;
	/** NSS */
	t_u8 nss_index;
	/** DCM */
	t_u8 dcm;
} MLAN_PACK_END mix_rate_info, *pmix_rate_info;

typedef MLAN_PACK_START struct _radiotap_timestamp {
	/* device timestamp */
	t_u64 device_timestamp;
	/* accuracy */
	t_u16 accuracy;
	/* unit:
	 * 0 milliseconds,
	 * 1 microseconds,
	 * 2 nanoseconds,
	 * 3-15 reserved */
	// coverity[misra_c_2012_rule_6_1_violation:SUPPRESS]
	t_u8 unit : 4;
	/* position:
	 * 0 first bit (or symbol containing it) of MPDU - matches TSFT field
	 * 1 signal acquisition at start of PLCP
	 * 2 end of PPDU
	 * 3 end of MPDU (after FCS)
	 * 4-14 reserved
	 * 15 unknown or vendor/OOB defined
	 */
	// coverity[misra_c_2012_rule_6_1_violation:SUPPRESS]
	t_u8 position : 4;
	/* flags
	 * 0x01 32-bit counter (high 32 bits are unused)
	 * 0x02 accuracy known
	 * 0xFC reserved
	 */
	t_u8 flags;
} MLAN_PACK_END radiotap_timestamp, *pradiotap_timestamp;

typedef MLAN_PACK_START struct _rxpd_extra_info {
	/** flags */
	t_u8 flags;
	/** channel.flags */
	t_u16 channel_flags;
	/** mcs.known */
	t_u8 mcs_known;
	/** mcs.flags */
	t_u8 mcs_flags;
	/** vht/he sig1 */
	t_u32 vht_he_sig1;
	/** vht/he sig2 */
	t_u32 vht_he_sig2;
	/** HE user idx */
	t_u32 user_idx;
	/** timestamp */
	radiotap_timestamp timestamp;
	/** PLCP CRC Failed */
	t_u8 plcp_crc_failed;
	t_u8 rssi_dbm_a;
	t_u8 rssi_dbm_b;
} MLAN_PACK_END rxpd_extra_info, *prxpd_extra_info;

typedef MLAN_PACK_START struct _radiotap_info {
	/** Rate Info */
	mix_rate_info rate_info;
	/** SNR */
	t_s8 snr;
	/** Noise Floor */
	t_s8 nf;
	/** band config */
	t_u8 band_config;
	/** chan number */
	t_u8 chan_num;
	t_u8 antenna;
	/** extra radiotap */
	t_u8 radiotap_extra;
	/** extra rxpd info from FW */
	rxpd_extra_info extra_info;
} MLAN_PACK_END radiotap_info, *pradiotap_info;

/** txpower structure */
typedef MLAN_PACK_START struct {
#ifdef BIG_ENDIAN_SUPPORT
	/** Host tx power ctrl:
	     0x0: use fw setting for TX power
	     0x1: value specified in bit[6] and bit[5:0] are valid */
	t_u8 hostctl : 1;
	/** Sign of the power specified in bit[5:0] */
	t_u8 sign : 1;
	/** Power to be used for transmission(in dBm) */
	t_u8 abs_val : 6;
#else
	/** Power to be used for transmission(in dBm) */
	t_u8 abs_val : 6;
	/** Sign of the power specified in bit[5:0] */
	t_u8 sign : 1;
	/** Host tx power ctrl:
	     0x0: use fw setting for TX power
	     0x1: value specified in bit[6] and bit[5:0] are valid */
	t_u8 hostctl : 1;
#endif
} MLAN_PACK_END tx_power_t;
/* pkt_txctrl */
typedef MLAN_PACK_START struct _pkt_txctrl {
	/**Data rate in unit of 0.5Mbps */
	t_u16 data_rate;
	/*Channel number to transmit the frame */
	t_u8 channel;
	/** Bandwidth to transmit the frame*/
	t_u8 bw;
	/** Power to be used for transmission*/
	union {
		tx_power_t tp;
		t_u8 val;
	} tx_power;
	/** Retry time of tx transmission*/
	t_u8 retry_limit;
} MLAN_PACK_END pkt_txctrl, *ppkt_txctrl;

/** pkt_rxinfo */
typedef MLAN_PACK_START struct _pkt_rxinfo {
	/** Data rate of received paccket*/
	t_u16 data_rate;
	/** Channel on which packet was received*/
	t_u8 channel;
	/** Rx antenna*/
	t_u8 antenna;
	/** Rx Rssi*/
	t_u8 rssi;
} MLAN_PACK_END pkt_rxinfo, *ppkt_rxinfo;

#define MC_FLAG_RETRY MBIT(0)
#define MC_FLAG_START_CYCLE MBIT(1)
#define MC_FLAG_END_CYCLE MBIT(2)
#define MC_FLAG_START_AMPDU MBIT(3)
#define MC_FLAG_END_AMPDU MBIT(4)
/* mc pkt txcontrol */
typedef MLAN_PACK_START struct _mc_txcontrol {
	/** Data rate in mcs index, 0-7 */
	t_u8 mcs_index;
	/** band width 0-20Mhz, 1-40Mhz */
	t_u8 bandwidth;
	/** seq_num */
	t_u16 seq_num;
	/** packet expiry time */
	t_u32 pkt_expiry;
	/** mc_pkt_flags */
	t_u8 mc_pkt_flags;
} MLAN_PACK_END mc_txcontrol, *pmc_txcontrol;
typedef MLAN_PACK_START struct MAPP_Stats_Mcast_drv {
	t_u32 cycle_recv_under_2300usec;
	t_u32 cycle_recv_in_time;
	t_u32 cycle_recv_over_2900usec;
	t_u32 cycle_recv_over_3500usec;
	t_u32 cycle_recv_over_5000usec;
	t_u32 cycle_recv_over_10000usec;
	t_u32 cycle_recv_over_15000usec;
	t_u32 spent_time_under_1000usec;
	t_u32 spent_time_over_1000usec;
	t_u32 spent_time_over_2000usec;
	t_u32 spent_time_over_3000usec;
} MLAN_PACK_END Stats_mcast_drv_t, *pStats_mcast_drv_t;
/** mlan_buffer data structure */
typedef struct _mlan_buffer {
	/** Pointer to previous mlan_buffer */
	struct _mlan_buffer *pprev;
	/** Pointer to next mlan_buffer */
	struct _mlan_buffer *pnext;
	/** Status code from firmware/driver */
	t_u32 status_code;
	/** Flags for this buffer */
	t_u32 flags;
	/** BSS index number for multiple BSS support */
	t_u32 bss_index;
	/** Buffer descriptor, e.g. skb in Linux */
	t_void *pdesc;
	/** Pointer to buffer */
	t_u8 *pbuf;
#ifdef PCIE
	/** Physical address of the pbuf pointer */
	t_u64 buf_pa;
	t_u32 total_pcie_buf_len;
#endif
	/** Offset to data */
	t_u32 data_offset;
	/** Data length */
	t_u32 data_len;
	/** Buffer type: data, cmd, event etc. */
	mlan_buf_type buf_type;

	/** Fields below are valid for data packet only */
	/** QoS priority */
	t_u32 priority;
	/** Time stamp when packet is received (seconds) */
	t_u32 in_ts_sec;
	/** Time stamp when packet is received (micro seconds) */
	t_u32 in_ts_usec;
	/** Time stamp when packet is processed (seconds) */
	t_u32 out_ts_sec;
	/** Time stamp when packet is processed (micro seconds) */
	t_u32 out_ts_usec;
	/** tx_seq_num */
	t_u32 tx_seq_num;
	/** Time stamp when packet is deque from rx_q(seconds) */
	t_u32 extra_ts_sec;
	/** Time stamp when packet is dequed from rx_q(micro seconds) */
	t_u32 extra_ts_usec;
	/** When TX ra mac address,  When Rx Ta mac address*/
	t_u8 mac[MLAN_MAC_ADDR_LENGTH];
	/** Fields below are valid for MLAN module only */
	/** Pointer to parent mlan_buffer */
	struct _mlan_buffer *pparent;
	/** Use count for this buffer */
	t_u32 use_count;
	union {
		mc_txcontrol mc_tx_info;
		pkt_txctrl tx_info;
		pkt_rxinfo rx_info;
	} u;
} mlan_buffer, *pmlan_buffer, **ppmlan_buffer;

/** mlan_hw_info data structure */
typedef struct _mlan_hw_info {
	t_u32 fw_cap;
	t_u32 fw_cap_ext;
} mlan_hw_info, *pmlan_hw_info;

/** mlan_bss_attr data structure */
typedef struct _mlan_bss_attr {
	/** BSS type */
	t_u32 bss_type;
	/** Data frame type: Ethernet II, 802.11, etc. */
	t_u32 frame_type;
	/** The BSS is active (non-0) or not (0). */
	t_u32 active;
	/** BSS Priority */
	t_u32 bss_priority;
	/** BSS number */
	t_u32 bss_num;
	/** The BSS is virtual */
	t_u32 bss_virtual;
} mlan_bss_attr, *pmlan_bss_attr;

/** bss tbl data structure */
typedef struct _mlan_bss_tbl {
	/** BSS Attributes */
	mlan_bss_attr bss_attr[MLAN_MAX_BSS_NUM];
} mlan_bss_tbl, *pmlan_bss_tbl;

#ifdef PRAGMA_PACK
#pragma pack(push, 1)
#endif

/** Type enumeration for the command result */
typedef MLAN_PACK_START enum _mlan_cmd_result_e {
	MLAN_CMD_RESULT_SUCCESS = 0,
	MLAN_CMD_RESULT_FAILURE = 1,
	MLAN_CMD_RESULT_TIMEOUT = 2,
	MLAN_CMD_RESULT_INVALID_DATA = 3
} MLAN_PACK_END mlan_cmd_result_e;

/** Type enumeration of WMM AC_QUEUES */
typedef MLAN_PACK_START enum _mlan_wmm_ac_e {
	WMM_AC_BK,
	WMM_AC_BE,
	WMM_AC_VI,
	WMM_AC_VO
} MLAN_PACK_END mlan_wmm_ac_e;

/** Type enumeration for the action field in the Queue Config command */
typedef MLAN_PACK_START enum _mlan_wmm_queue_config_action_e {
	MLAN_WMM_QUEUE_CONFIG_ACTION_GET = 0,
	MLAN_WMM_QUEUE_CONFIG_ACTION_SET = 1,
	MLAN_WMM_QUEUE_CONFIG_ACTION_DEFAULT = 2,
	MLAN_WMM_QUEUE_CONFIG_ACTION_MAX
} MLAN_PACK_END mlan_wmm_queue_config_action_e;

/** Type enumeration for the action field in the queue stats command */
typedef MLAN_PACK_START enum _mlan_wmm_queue_stats_action_e {
	MLAN_WMM_STATS_ACTION_START = 0,
	MLAN_WMM_STATS_ACTION_STOP = 1,
	MLAN_WMM_STATS_ACTION_GET_CLR = 2,
	MLAN_WMM_STATS_ACTION_SET_CFG = 3, /* Not currently used */
	MLAN_WMM_STATS_ACTION_GET_CFG = 4, /* Not currently used */
	MLAN_WMM_STATS_ACTION_MAX
} MLAN_PACK_END mlan_wmm_queue_stats_action_e;

/**
 *  @brief IOCTL structure for a Traffic stream status.
 *
 */
typedef MLAN_PACK_START struct {
	/** TSID: Range: 0->7 */
	t_u8 tid;
	/** TSID specified is valid */
	t_u8 valid;
	/** AC TSID is active on */
	t_u8 access_category;
	/** UP specified for the TSID */
	t_u8 user_priority;
	/** Power save mode for TSID: 0 (legacy), 1 (UAPSD) */
	t_u8 psb;
	/** Upstream(0), Downlink(1), Bidirectional(3) */
	t_u8 flow_dir;
	/** Medium time granted for the TSID */
	t_u16 medium_time;
} MLAN_PACK_END wlan_ioctl_wmm_ts_status_t,
	/** Type definition of mlan_ds_wmm_ts_status for
	   MLAN_OID_WMM_CFG_TS_STATUS */
	mlan_ds_wmm_ts_status, *pmlan_ds_wmm_ts_status;

/** Max Ie length */
#define MAX_IE_SIZE 256

/** custom IE */
typedef MLAN_PACK_START struct _custom_ie {
	/** IE Index */
	t_u16 ie_index;
	/** Mgmt Subtype Mask */
	t_u16 mgmt_subtype_mask;
	/** IE Length */
	t_u16 ie_length;
	/** IE buffer */
	t_u8 ie_buffer[MAX_IE_SIZE];
} MLAN_PACK_END custom_ie;

/** Max IE index to FW */
#define MAX_MGMT_IE_INDEX_TO_FW 4
/** Max IE index per BSS */
#define MAX_MGMT_IE_INDEX 26

/** custom IE info */
typedef MLAN_PACK_START struct _custom_ie_info {
	/** size of buffer */
	t_u16 buf_size;
	/** no of buffers of buf_size */
	t_u16 buf_count;
} MLAN_PACK_END custom_ie_info;

/** TLV buffer : Max Mgmt IE */
typedef MLAN_PACK_START struct _tlvbuf_max_mgmt_ie {
	/** Type */
	t_u16 type;
	/** Length */
	t_u16 len;
	/** No of tuples */
	t_u16 count;
	/** custom IE info tuples */
	custom_ie_info info[MAX_MGMT_IE_INDEX];
} MLAN_PACK_END tlvbuf_max_mgmt_ie;

/** TLV buffer : custom IE */
typedef MLAN_PACK_START struct _tlvbuf_custom_ie {
	/** Type */
	t_u16 type;
	/** Length */
	t_u16 len;
	/** IE data */
	custom_ie ie_data_list[MAX_MGMT_IE_INDEX_TO_FW];
	/** Max mgmt IE TLV */
	tlvbuf_max_mgmt_ie max_mgmt_ie;
} MLAN_PACK_END mlan_ds_misc_custom_ie;

#define FW_STUCK_CODE_VERSION_MISMATCH (0x90)

/** Max TDLS config data length */
#define MAX_TDLS_DATA_LEN 1024

/** Action commands for TDLS enable/disable */
#define WLAN_TDLS_CONFIG 0x00
/** Action commands for TDLS configuration :Set */
#define WLAN_TDLS_SET_INFO 0x01
/** Action commands for TDLS configuration :Discovery Request */
#define WLAN_TDLS_DISCOVERY_REQ 0x02
/** Action commands for TDLS configuration :Setup Request */
#define WLAN_TDLS_SETUP_REQ 0x03
/** Action commands for TDLS configuration :Tear down Request */
#define WLAN_TDLS_TEAR_DOWN_REQ 0x04
/** Action ID for TDLS power mode */
#define WLAN_TDLS_POWER_MODE 0x05
/**Action ID for init TDLS Channel Switch*/
#define WLAN_TDLS_INIT_CHAN_SWITCH 0x06
/** Action ID for stop TDLS Channel Switch */
#define WLAN_TDLS_STOP_CHAN_SWITCH 0x07
/** Action ID for configure CS related parameters */
#define WLAN_TDLS_CS_PARAMS 0x08
/** Action ID for Disable CS */
#define WLAN_TDLS_CS_DISABLE 0x09
/** Action ID for TDLS link status */
#define WLAN_TDLS_LINK_STATUS 0x0A
/** Action ID for Host TDLS config uapsd and CS */
#define WLAN_HOST_TDLS_CONFIG 0x0D
/** Action ID for TDLS CS immediate return */
#define WLAN_TDLS_DEBUG_CS_RET_IM 0xFFF7
/** Action ID for TDLS Stop RX */
#define WLAN_TDLS_DEBUG_STOP_RX 0xFFF8
/** Action ID for TDLS Allow weak security for links establish */
#define WLAN_TDLS_DEBUG_ALLOW_WEAK_SECURITY 0xFFF9
/** Action ID for TDLS Ignore key lifetime expiry */
#define WLAN_TDLS_DEBUG_IGNORE_KEY_EXPIRY 0xFFFA
/** Action ID for TDLS Higher/Lower mac Test */
#define WLAN_TDLS_DEBUG_HIGHER_LOWER_MAC 0xFFFB
/** Action ID for TDLS Prohibited Test */
#define WLAN_TDLS_DEBUG_SETUP_PROHIBITED 0xFFFC
/** Action ID for TDLS Existing link Test */
#define WLAN_TDLS_DEBUG_SETUP_SAME_LINK 0xFFFD
/** Action ID for TDLS Fail Setup Confirm */
#define WLAN_TDLS_DEBUG_FAIL_SETUP_CONFIRM 0xFFFE
/** Action commands for TDLS debug: Wrong BSS Request */
#define WLAN_TDLS_DEBUG_WRONG_BSS 0xFFFF

/** tdls each link rate information */
typedef MLAN_PACK_START struct _tdls_link_rate_info {
	/** Tx Data Rate */
	t_u8 tx_data_rate;
	/** Tx Rate HT info*/
	t_u8 tx_rate_htinfo;
} MLAN_PACK_END tdls_link_rate_info;

/** tdls each link status */
typedef MLAN_PACK_START struct _tdls_each_link_status {
	/** peer mac Address */
	t_u8 peer_mac[MLAN_MAC_ADDR_LENGTH];
	/** Link Flags */
	t_u8 link_flags;
	/** Traffic Status */
	t_u8 traffic_status;
	/** Tx Failure Count */
	t_u8 tx_fail_count;
	/** Channel Number */
	t_u32 active_channel;
	/** Last Data RSSI in dBm */
	t_s16 data_rssi_last;
	/** Last Data NF in dBm */
	t_s16 data_nf_last;
	/** AVG DATA RSSI in dBm */
	t_s16 data_rssi_avg;
	/** AVG DATA NF in dBm */
	t_s16 data_nf_avg;
	union {
		/** tdls rate info */
		tdls_link_rate_info rate_info;
		/** tdls link final rate*/
		t_u16 final_data_rate;
	} u;
	/** Security Method */
	t_u8 security_method;
	/** Key Lifetime in milliseconds */
	t_u32 key_lifetime;
	/** Key Length */
	t_u8 key_length;
	/** actual key */
	t_u8 key[];
} MLAN_PACK_END tdls_each_link_status;

/** TDLS configuration data */
typedef MLAN_PACK_START struct _tdls_all_config {
	union {
		/** TDLS state enable disable */
		MLAN_PACK_START struct _tdls_config {
			/** enable or disable */
			t_u16 enable;
		} MLAN_PACK_END tdls_config;
		/** Host tdls config */
		MLAN_PACK_START struct _host_tdls_cfg {
			/** support uapsd */
			t_u8 uapsd_support;
			/** channel_switch */
			t_u8 cs_support;
			/** TLV  length */
			t_u16 tlv_len;
			/** tdls info */
			t_u8 tlv_buffer[];
		} MLAN_PACK_END host_tdls_cfg;
		/** TDLS set info */
		MLAN_PACK_START struct _tdls_set_data {
			/** (tlv + capInfo) length */
			t_u16 tlv_length;
			/** Cap Info */
			t_u16 cap_info;
			/** TLV buffer */
			t_u8 tlv_buffer[];
		} MLAN_PACK_END tdls_set;

		/** TDLS discovery and others having mac argument */
		MLAN_PACK_START struct _tdls_discovery_data {
			/** peer mac Address */
			t_u8 peer_mac_addr[MLAN_MAC_ADDR_LENGTH];
		} MLAN_PACK_END tdls_discovery, tdls_stop_chan_switch,
			tdls_link_status_req;

		/** TDLS discovery Response */
		MLAN_PACK_START struct _tdls_discovery_resp {
			/** payload length */
			t_u16 payload_len;
			/** peer mac Address */
			t_u8 peer_mac_addr[MLAN_MAC_ADDR_LENGTH];
			/** RSSI */
			t_s8 rssi;
			/** Cap Info */
			t_u16 cap_info;
			/** TLV buffer */
			t_u8 tlv_buffer[];
		} MLAN_PACK_END tdls_discovery_resp;

		/** TDLS setup request */
		MLAN_PACK_START struct _tdls_setup_data {
			/** peer mac Address */
			t_u8 peer_mac_addr[MLAN_MAC_ADDR_LENGTH];
			/** timeout value in milliseconds */
			t_u32 setup_timeout;
			/** key lifetime in milliseconds */
			t_u32 key_lifetime;
		} MLAN_PACK_END tdls_setup;

		/** TDLS tear down info */
		MLAN_PACK_START struct _tdls_tear_down_data {
			/** peer mac Address */
			t_u8 peer_mac_addr[MLAN_MAC_ADDR_LENGTH];
			/** reason code */
			t_u16 reason_code;
		} MLAN_PACK_END tdls_tear_down, tdls_cmd_resp;

		/** TDLS power mode info */
		MLAN_PACK_START struct _tdls_power_mode_data {
			/** peer mac Address */
			t_u8 peer_mac_addr[MLAN_MAC_ADDR_LENGTH];
			/** Power Mode */
			t_u16 power_mode;
		} MLAN_PACK_END tdls_power_mode;

		/** TDLS channel switch info */
		MLAN_PACK_START struct _tdls_chan_switch {
			/** peer mac Address */
			t_u8 peer_mac_addr[MLAN_MAC_ADDR_LENGTH];
			/** Channel Switch primary channel no */
			t_u8 primary_channel;
			/** Channel Switch secondary channel offset */
			t_u8 secondary_channel_offset;
			/** Channel Switch Band */
			t_u8 band;
			/** Channel Switch time in milliseconds */
			t_u16 switch_time;
			/** Channel Switch timeout in milliseconds */
			t_u16 switch_timeout;
			/** Channel Regulatory class*/
			t_u8 regulatory_class;
			/** peridicity flag*/
			t_u8 periodicity;
		} MLAN_PACK_END tdls_chan_switch;

		/** TDLS channel switch paramters */
		MLAN_PACK_START struct _tdls_cs_params {
			/** unit time, multiples of 10ms */
			t_u8 unit_time;
			/** threshold for other link */
			t_u8 threshold_otherlink;
			/** threshold for direct link */
			t_u8 threshold_directlink;
		} MLAN_PACK_END tdls_cs_params;

		/** tdls disable channel switch */
		MLAN_PACK_START struct _tdls_disable_cs {
			/** Data*/
			t_u16 data;
		} MLAN_PACK_END tdls_disable_cs;
		/** TDLS debug data */
		MLAN_PACK_START struct _tdls_debug_data {
			/** debug data */
			t_u16 debug_data;
		} MLAN_PACK_END tdls_debug_data;

		/** TDLS link status Response */
		MLAN_PACK_START struct _tdls_link_status_resp {
			/** payload length */
			t_u16 payload_len;
			/** number of links */
			t_u8 active_links;
			/** structure for link status */
			tdls_each_link_status link_stats[];
		} MLAN_PACK_END tdls_link_status_resp;

	} u;
} MLAN_PACK_END tdls_all_config;

/** TDLS configuration buffer */
typedef MLAN_PACK_START struct _buf_tdls_config {
	/** TDLS Action */
	t_u16 tdls_action;
	/** TDLS data */
	t_u8 tdls_data[MAX_TDLS_DATA_LEN];
} MLAN_PACK_END mlan_ds_misc_tdls_config;

/** Event structure for tear down */
typedef struct _tdls_tear_down_event {
	/** Peer mac address */
	t_u8 peer_mac_addr[MLAN_MAC_ADDR_LENGTH];
	/** Reason code */
	t_u16 reason_code;
} tdls_tear_down_event;

/** Event structure for addba timeout */
typedef struct _addba_timeout_event {
	/** Peer mac address */
	t_u8 peer_mac_addr[MLAN_MAC_ADDR_LENGTH];
	/** Tid */
	t_u8 tid;
} addba_timeout_event;

/** channel width */
typedef enum wifi_channel_width {
	WIFI_CHAN_WIDTH_20 = 0,
	WIFI_CHAN_WIDTH_40 = 1,
	WIFI_CHAN_WIDTH_80 = 2,
	WIFI_CHAN_WIDTH_160 = 3,
	WIFI_CHAN_WIDTH_80P80 = 4,
	WIFI_CHAN_WIDTH_5 = 5,
	WIFI_CHAN_WIDTH_10 = 6,
	WIFI_CHAN_WIDTH_INVALID = -1
} wifi_channel_width_t;

/** channel information */
typedef struct {
	/** channel width (20, 40, 80, 80+80, 160) */
	wifi_channel_width_t width;
	/** primary 20 MHz channel */
	int center_freq;
	/** center frequency (MHz) first segment */
	int center_freq0;
	/** center frequency (MHz) second segment */
	int center_freq1;
} wifi_channel_info;

/** wifi rate */
typedef struct {
	/** 0: OFDM, 1:CCK, 2:HT 3:VHT 4..7 reserved */
	t_u32 preamble : 3;
	/** 0:1x1, 1:2x2, 3:3x3, 4:4x4 */
	t_u32 nss : 2;
	/** 0:20MHz, 1:40Mhz, 2:80Mhz, 3:160Mhz */
	t_u32 bw : 3;
	/** OFDM/CCK rate code would be as per ieee std in the units of 0.5mbps
	 */
	/** HT/VHT it would be mcs index */
	t_u32 rateMcsIdx : 8;
	/** reserved */
	t_u32 reserved : 16;
	/** units of 100 Kbps */
	t_u32 bitrate;
} wifi_rate;

/** wifi Preamble type */
typedef enum {
	WIFI_PREAMBLE_LEGACY = 0x1,
	WIFI_PREAMBLE_HT = 0x2,
	WIFI_PREAMBLE_VHT = 0x4
} wifi_preamble;

/** timeval */
typedef struct {
	/** Time (seconds) */
	t_u32 time_sec;
	/** Time (micro seconds) */
	t_u32 time_usec;
} wifi_timeval;

#define timeval_to_msec(timeval)                                               \
	(t_u64)((t_u64)(timeval.time_sec) * 1000 +                             \
		(t_u64)(timeval.time_usec) / 1000)
#define timeval_to_usec(timeval)                                               \
	(t_u64)((t_u64)(timeval.time_sec) * 1000 * 1000 +                      \
		(t_u64)(timeval.time_usec))
#define is_zero_timeval(timeval)                                               \
	((timeval.time_sec == 0) && (timeval.time_usec == 0))

#define MAX_NUM_RATE 32
#define MAX_RADIO 2
#define MAX_NUM_CHAN 1
#define VHT_NUM_SUPPORT_MCS 10
#define MCS_NUM_SUPP 16

#define BUF_MAXLEN 4096
/** connection state */
typedef enum {
	MLAN_DISCONNECTED = 0,
	MLAN_AUTHENTICATING = 1,
	MLAN_ASSOCIATING = 2,
	MLAN_ASSOCIATED = 3,
	/** if done by firmware/driver */
	MLAN_EAPOL_STARTED = 4,
	/** if done by firmware/driver */
	MLAN_EAPOL_COMPLETED = 5,
} mlan_connection_state;
/** roam state */
typedef enum {
	MLAN_ROAMING_IDLE = 0,
	MLAN_ROAMING_ACTIVE = 1,
} mlan_roam_state;
/** interface mode */
typedef enum {
	MLAN_INTERFACE_STA = 0,
	MLAN_INTERFACE_SOFTAP = 1,
	MLAN_INTERFACE_P2P_CLIENT = 3,
	MLAN_INTERFACE_P2P_GO = 4,
	MLAN_INTERFACE_NAN = 5,
	MLAN_INTERFACE_MESH = 6,
} mlan_interface_mode;

/** set for QOS association */
#define MLAN_CAPABILITY_QOS 0x00000001
/** set for protected association (802.11 beacon frame control protected bit
 * set) */
#define MLAN_CAPABILITY_PROTECTED 0x00000002
/** set if 802.11 Extended Capabilities element interworking bit is set */
#define MLAN_CAPABILITY_INTERWORKING 0x00000004
/** set for HS20 association */
#define MLAN_CAPABILITY_HS20 0x00000008
/** set is 802.11 Extended Capabilities element UTF-8 SSID bit is set */
#define MLAN_CAPABILITY_SSID_UTF8 0x00000010
/** set is 802.11 Country Element is present */
#define MLAN_CAPABILITY_COUNTRY 0x00000020

/** link layer status */
typedef struct {
	/** interface mode */
	mlan_interface_mode mode;
	/** interface mac address (self) */
	t_u8 mac_addr[6];
	/** connection state (valid for STA, CLI only) */
	mlan_connection_state state;
	/** roaming state */
	mlan_roam_state roaming;
	/** WIFI_CAPABILITY_XXX (self) */
	t_u32 capabilities;
	/** null terminated SSID */
	t_u8 ssid[33];
	/** bssid */
	t_u8 bssid[6];
	/** country string advertised by AP */
	t_u8 ap_country_str[3];
	/** country string for this association */
	t_u8 country_str[3];
} mlan_interface_link_layer_info, *mlan_interface_handle;

/** channel statistics */
typedef struct {
	/** channel */
	wifi_channel_info channel;
	/** msecs the radio is awake (32 bits number accruing over time) */
	t_u32 on_time;
	/** msecs the CCA register is busy (32 bits number accruing over time)
	 */
	t_u32 cca_busy_time;
} wifi_channel_stat;

/** radio statistics */
typedef struct {
	/** wifi radio (if multiple radio supported) */
	int radio;
	/** msecs the radio is awake (32 bits number accruing over time) */
	t_u32 on_time;
	/** msecs the radio is transmitting (32 bits number accruing over time)
	 */
	t_u32 tx_time;
	/**  TBD: num_tx_levels: number of radio transmit power levels */
	t_u32 reserved0;
	/** TBD: tx_time_per_levels: pointer to an array of radio transmit per
	 * power levels in msecs accured over time */
	/* t_u32 *reserved1;*/
	/** msecs the radio is in active receive (32 bits number accruing over
	 * time) */
	t_u32 rx_time;
	/** msecs the radio is awake due to all scan (32 bits number accruing
	 * over time) */
	t_u32 on_time_scan;
	/** msecs the radio is awake due to NAN (32 bits number accruing over
	 * time) */
	t_u32 on_time_nbd;
	/** msecs the radio is awake due to G?scan (32 bits number accruing over
	 * time) */
	t_u32 on_time_gscan;
	/** msecs the radio is awake due to roam?scan (32 bits number accruing
	 * over time) */
	t_u32 on_time_roam_scan;
	/** msecs the radio is awake due to PNO scan (32 bits number accruing
	 * over time) */
	t_u32 on_time_pno_scan;
	/** msecs the radio is awake due to HS2.0 scans and GAS exchange (32
	 * bits number accruing over time) */
	t_u32 on_time_hs20;
	/** number of channels */
	t_u32 num_channels;
	/** channel statistics */
	wifi_channel_stat channels[MAX_NUM_CHAN];
} wifi_radio_stat;

/** per rate statistics */
typedef struct {
	/** rate information */
	wifi_rate rate;
	/** number of successfully transmitted data pkts (ACK rcvd) */
	t_u32 tx_mpdu;
	/** number of received data pkts */
	t_u32 rx_mpdu;
	/** number of data packet losses (no ACK) */
	t_u32 mpdu_lost;
	/** total number of data pkt retries */
	t_u32 retries;
	/** number of short data pkt retries */
	t_u32 retries_short;
	/** number of long data pkt retries */
	t_u32 retries_long;
} wifi_rate_stat;

/** wifi peer type */
typedef enum {
	WIFI_PEER_STA,
	WIFI_PEER_AP,
	WIFI_PEER_P2P_GO,
	WIFI_PEER_P2P_CLIENT,
	WIFI_PEER_NAN,
	WIFI_PEER_TDLS,
	WIFI_PEER_INVALID,
} wifi_peer_type;

typedef struct bssload_info {
	/** station count */
	t_u16 sta_count;
	/** channel utilization */
	t_u16 chan_util;
	t_u8 PAD[4];
} bssload_info_t;

/** per peer statistics */
typedef struct {
	/** peer type (AP, TDLS, GO etc.) */
	wifi_peer_type type;
	/** mac address */
	t_u8 peer_mac_address[6];
	/** peer WIFI_CAPABILITY_XXX */
	t_u32 capabilities;
	/** STA count and channel utilization */
	bssload_info_t bssload;
	/** number of rates */
	t_u32 num_rate;
	/** per rate statistics, number of entries  = num_rate */
	wifi_rate_stat rate_stats[];
} wifi_peer_info;

/** per access category statistics */
typedef struct {
	/** access category (VI, VO, BE, BK) */
	mlan_wmm_ac_e ac;
	/** number of successfully transmitted unicast data pkts (ACK rcvd) */
	t_u32 tx_mpdu;
	/** number of received unicast mpdus */
	t_u32 rx_mpdu;
	/** number of succesfully transmitted multicast data packets */
	/** STA case: implies ACK received from AP for the unicast packet in
	 * which mcast pkt was sent */
	t_u32 tx_mcast;
	/** number of received multicast data packets */
	t_u32 rx_mcast;
	/** number of received unicast a-mpdus */
	t_u32 rx_ampdu;
	/** number of transmitted unicast a-mpdus */
	t_u32 tx_ampdu;
	/** number of data pkt losses (no ACK) */
	t_u32 mpdu_lost;
	/** total number of data pkt retries */
	t_u32 retries;
	/** number of short data pkt retries */
	t_u32 retries_short;
	/** number of long data pkt retries */
	t_u32 retries_long;
	/** data pkt min contention time (usecs) */
	t_u32 contention_time_min;
	/** data pkt max contention time (usecs) */
	t_u32 contention_time_max;
	/** data pkt avg contention time (usecs) */
	t_u32 contention_time_avg;
	/** num of data pkts used for contention statistics */
	t_u32 contention_num_samples;
} wifi_wmm_ac_stat;

/** interface statistics */
typedef struct {
	/** wifi interface */
	/* wifi_interface_handle iface;*/
	/** current state of the interface */
	mlan_interface_link_layer_info info;
	/** access point beacon received count from connected AP */
	t_u32 beacon_rx;
	/** Average beacon offset encountered (beacon_TSF - TBTT)
	 *    the average_tsf_offset field is used so as to calculate the
	 *    typical beacon contention time on the channel as well may be
	 *    used to debug beacon synchronization and related power consumption
	 * issue
	 */
	t_u64 average_tsf_offset;
	/** indicate that this AP typically leaks packets beyond the driver
	 * guard time */
	t_u32 leaky_ap_detected;
	/** average number of frame leaked by AP after frame with PM bit set was
	 * ACK'ed by AP */
	t_u32 leaky_ap_avg_num_frames_leaked;
	/** Guard time currently in force (when implementing IEEE power
	 * management based on frame control PM bit), How long driver waits
	 * before shutting down the radio and after receiving an ACK for a data
	 * frame with PM bit set)
	 */
	t_u32 leaky_ap_guard_time;
	/** access point mgmt frames received count from connected AP (including
	 * Beacon) */
	t_u32 mgmt_rx;
	/** action frames received count */
	t_u32 mgmt_action_rx;
	/** action frames transmit count */
	t_u32 mgmt_action_tx;
	/** access Point Beacon and Management frames RSSI (averaged) */
	t_s32 rssi_mgmt;
	/** access Point Data Frames RSSI (averaged) from connected AP */
	t_s32 rssi_data;
	/** access Point ACK RSSI (averaged) from connected AP */
	t_s32 rssi_ack;
	/** per ac data packet statistics */
	wifi_wmm_ac_stat ac[MAX_AC_QUEUES];
	/** number of peers */
	t_u32 num_peers;
	/** per peer statistics */
	wifi_peer_info peer_info[];
} wifi_iface_stat;

/** link layer stat configuration params */
typedef struct {
	/** threshold to classify the pkts as short or long */
	t_u32 mpdu_size_threshold;
	/** wifi statistics bitmap */
	t_u32 aggressive_statistics_gathering;
} wifi_link_layer_params;

/** wifi statistics bitmap  */
#define WIFI_STATS_RADIO 0x00000001 /** all radio statistics */
#define WIFI_STATS_RADIO_CCA                                                   \
	0x00000002 /** cca_busy_time (within radio statistics) */
#define WIFI_STATS_RADIO_CHANNELS                                              \
	0x00000004 /** all channel statistics (within radio statistics) */
#define WIFI_STATS_RADIO_SCAN                                                  \
	0x00000008 /** all scan statistics (within radio statistics) */
#define WIFI_STATS_IFACE 0x00000010 /** all interface statistics */
#define WIFI_STATS_IFACE_TXRATE                                                \
	0x00000020 /** all tx rate statistics (within interface statistics) */
#define WIFI_STATS_IFACE_AC                                                    \
	0x00000040 /** all ac statistics (within interface statistics) */
#define WIFI_STATS_IFACE_CONTENTION                                            \
	0x00000080 /** all contention (min, max, avg) statistics (within ac    \
		      statisctics) */

/** =========== Define Copied from HAL START =========== */
/** Ranging status */
typedef enum {
	RTT_STATUS_SUCCESS = 0,
	/** general failure status */
	RTT_STATUS_FAILURE = 1,
	/** target STA does not respond to request */
	RTT_STATUS_FAIL_NO_RSP = 2,
	/** request rejected. Applies to 2-sided RTT only */
	RTT_STATUS_FAIL_REJECTED = 3,
	RTT_STATUS_FAIL_NOT_SCHEDULED_YET = 4,
	/** timing measurement times out */
	RTT_STATUS_FAIL_TM_TIMEOUT = 5,
	/** Target on different channel, cannot range */
	RTT_STATUS_FAIL_AP_ON_DIFF_CHANNEL = 6,
	/** ranging not supported */
	RTT_STATUS_FAIL_NO_CAPABILITY = 7,
	/** request aborted for unknown reason */
	RTT_STATUS_ABORTED = 8,
	/** Invalid T1-T4 timestamp */
	RTT_STATUS_FAIL_INVALID_TS = 9,
	/** 11mc protocol failed */
	RTT_STATUS_FAIL_PROTOCOL = 10,
	/** request could not be scheduled */
	RTT_STATUS_FAIL_SCHEDULE = 11,
	/** responder cannot collaborate at time of request */
	RTT_STATUS_FAIL_BUSY_TRY_LATER = 12,
	/** bad request args */
	RTT_STATUS_INVALID_REQ = 13,
	/** WiFi not enabled */
	RTT_STATUS_NO_WIFI = 14,
	/** Responder overrides param info, cannot range with new params */
	RTT_STATUS_FAIL_FTM_PARAM_OVERRIDE = 15
} wifi_rtt_status;

/** RTT peer type */
typedef enum {
	RTT_PEER_AP = 0x1,
	RTT_PEER_STA = 0x2,
	RTT_PEER_P2P_GO = 0x3,
	RTT_PEER_P2P_CLIENT = 0x4,
	RTT_PEER_NAN = 0x5
} rtt_peer_type;

/** RTT Measurement Bandwidth */
typedef enum {
	WIFI_RTT_BW_5 = 0x01,
	WIFI_RTT_BW_10 = 0x02,
	WIFI_RTT_BW_20 = 0x04,
	WIFI_RTT_BW_40 = 0x08,
	WIFI_RTT_BW_80 = 0x10,
	WIFI_RTT_BW_160 = 0x20
} wifi_rtt_bw;

/** RTT Type */
typedef enum {
	RTT_TYPE_1_SIDED = 0x1,
	RTT_TYPE_2_SIDED = 0x2,
} wifi_rtt_type;

/** RTT configuration */
typedef struct {
	/** peer device mac address */
	t_u8 addr[MLAN_MAC_ADDR_LENGTH];
	/** 1-sided or 2-sided RTT */
	wifi_rtt_type type;
	/** optional - peer device hint (STA, P2P, AP) */
	rtt_peer_type peer;
	/** Required for STA-AP mode, optional for P2P, NBD etc. */
	wifi_channel_info channel;
	/** Time interval between bursts (units: 100 ms).
	 * Applies to 1-sided and 2-sided RTT multi-burst requests.
	 * Range: 0-31, 0: no preference by initiator (2-sided RTT) */
	t_u32 burst_period;
	/** Total number of RTT bursts to be executed. It will be
	 * specified in the same way as the parameter "Number of
	 * Burst Exponent" found in the FTM frame format. It
	 * applies to both: 1-sided RTT and 2-sided RTT. Valid
	 * values are 0 to 15 as defined in 802.11mc std.
	 * 0 means single shot
	 * The implication of this parameter on the maximum
	 * number of RTT results is the following:
	 * for 1-sided RTT: max num of RTT results =
	 * (2^num_burst)*(num_frames_per_burst)
	 * for 2-sided RTT: max num of RTT results =
	 * (2^num_burst)*(num_frames_per_burst - 1) */
	t_u32 num_burst;
	/** num of frames per burst. Minimum value = 1, Maximum value = 31
	 * For 2-sided this equals the number of FTM frames to be attempted in a
	 * single burst. This also equals the number of FTM frames that the
	 * initiator will request that the responder send in a single frame. */
	t_u32 num_frames_per_burst;
	/** number of retries for a failed RTT frame. Applies
	 * to 1-sided RTT only. Minimum value = 0, Maximum value = 3 */
	t_u32 num_retries_per_rtt_frame;

	/** following fields are only valid for 2-side RTT */
	/** Maximum number of retries that the initiator can retry an FTMR
	 * frame. Minimum value = 0, Maximum value = 3 */
	t_u32 num_retries_per_ftmr;
	/** 1: request LCI, 0: do not request LCI */
	t_u8 LCI_request;
	/** 1: request LCR, 0: do not request LCR */
	t_u8 LCR_request;
	/** Applies to 1-sided and 2-sided RTT. Valid values will
	 * be 2-11 and 15 as specified by the 802.11mc std for
	 * the FTM parameter burst duration. In a multi-burst
	 * request, if responder overrides with larger value,
	 * the initiator will return failure. In a single-burst
	 * request if responder overrides with larger value,
	 * the initiator will sent TMR_STOP to terminate RTT
	 * at the end of the burst_duration it requested. */
	t_u32 burst_duration;
	/** RTT preamble to be used in the RTT frames */
	wifi_preamble preamble;
	/** RTT BW to be used in the RTT frames */
	wifi_rtt_bw bw;
} wifi_rtt_config;

/** Format of information elements found in the beacon */
typedef struct {
	/** element identifier */
	t_u8 id;
	/** number of bytes to follow */
	t_u8 len;
	t_u8 data[];
} wifi_information_element;

/** RTT results */
typedef struct {
	/** device mac address */
	t_u8 addr[MLAN_MAC_ADDR_LENGTH];
	/** burst number in a multi-burst request */
	t_u32 burst_num;
	/** Total RTT measurement frames attempted */
	t_u32 measurement_number;
	/** Total successful RTT measurement frames */
	t_u32 success_number;
	/** Maximum number of "FTM frames per burst" supported by
	 * the responder STA. Applies to 2-sided RTT only.
	 * If reponder overrides with larger value:
	 * - for single-burst request initiator will truncate the
	 * larger value and send a TMR_STOP after receiving as
	 * many frames as originally requested.
	 * - for multi-burst request, initiator will return
	 * failure right away */
	t_u8 number_per_burst_peer;
	/** ranging status */
	wifi_rtt_status status;
	/** When status == RTT_STATUS_FAIL_BUSY_TRY_LATER,
	 * this will be the time provided by the responder as to
	 * when the request can be tried again. Applies to 2-sided
	 * RTT only. In sec, 1-31sec. */
	t_u8 retry_after_duration;
	/** RTT type */
	wifi_rtt_type type;
	/** average rssi in 0.5 dB steps e.g. 143 implies -71.5 dB */
	int rssi;
	/** rssi spread in 0.5 dB steps e.g. 5 implies 2.5 dB spread (optional)
	 */
	int rssi_spread;
	/** 1-sided RTT: TX rate of RTT frame.
	 * 2-sided RTT: TX rate of initiator's Ack in response to FTM frame. */
	wifi_rate tx_rate;
	/** 1-sided RTT: TX rate of Ack from other side.
	 * 2-sided RTT: TX rate of FTM frame coming from responder. */
	wifi_rate rx_rate;
	/** round trip time in picoseconds */
	t_s64 rtt;
	/** rtt standard deviation in picoseconds */
	t_s64 rtt_sd;
	/** difference between max and min rtt times recorded in picoseconds */
	t_s64 rtt_spread;
	/** distance in mm (optional) */
	int distance_mm;
	/** standard deviation in mm (optional) */
	int distance_sd_mm;
	/** difference between max and min distance recorded in mm (optional) */
	int distance_spread_mm;
	/** time of the measurement (in microseconds since boot) */
	t_s64 ts;
	/** in ms, actual time taken by the FW to finish one burst
	 * measurement. Applies to 1-sided and 2-sided RTT. */
	int burst_duration;
	/** Number of bursts allowed by the responder. Applies
	 * to 2-sided RTT only. */
	int negotiated_burst_num;
	/** for 11mc only */
	wifi_information_element *LCI;
	/** for 11mc only */
	wifi_information_element *LCR;
} wifi_rtt_result;

/** Preamble definition for bit mask used in wifi_rtt_capabilities */
#define PREAMBLE_LEGACY 0x1
#define PREAMBLE_HT 0x2
#define PREAMBLE_VHT 0x4

/** BW definition for bit mask used in wifi_rtt_capabilities */
#define BW_5_SUPPORT 0x1
#define BW_10_SUPPORT 0x2
#define BW_20_SUPPORT 0x4
#define BW_40_SUPPORT 0x8
#define BW_80_SUPPORT 0x10
#define BW_160_SUPPORT 0x20

/** RTT Capabilities */
typedef struct {
	/** if 1-sided rtt data collection is supported */
	t_u8 rtt_one_sided_supported;
	/** if ftm rtt data collection is supported */
	t_u8 rtt_ftm_supported;
	/** if initiator supports LCI request. Applies to 2-sided RTT */
	t_u8 lci_support;
	/** if initiator supports LCR request. Applies to 2-sided RTT */
	t_u8 lcr_support;
	/** bit mask indicates what preamble is supported by initiator */
	t_u8 preamble_support;
	/** bit mask indicates what BW is supported by initiator */
	t_u8 bw_support;
	/** if 11mc responder mode is supported */
	t_u8 responder_supported;
	/** draft 11mc spec version supported by chip. For instance,
	 * version 4.0 should be 40 and version 4.3 should be 43 etc. */
	t_u8 mc_version;
} wifi_rtt_capabilities;

/** API for setting LCI/LCR information to be provided to a requestor */
typedef enum {
	/** Not expected to change location */
	WIFI_MOTION_NOT_EXPECTED = 0,
	/** Expected to change location */
	WIFI_MOTION_EXPECTED = 1,
	/** Movement pattern unknown */
	WIFI_MOTION_UNKNOWN = 2,
} wifi_motion_pattern;

/** LCI information */
typedef struct {
	/** latitude in degrees * 2^25 , 2's complement */
	long latitude;
	/** latitude in degrees * 2^25 , 2's complement */
	long longitude;
	/** Altitude in units of 1/256 m */
	int altitude;
	/** As defined in Section 2.3.2 of IETF RFC 6225 */
	t_u8 latitude_unc;
	/** As defined in Section 2.3.2 of IETF RFC 6225 */
	t_u8 longitude_unc;
	/** As defined in Section 2.4.5 from IETF RFC 6225: */
	t_u8 altitude_unc;
	/** Following element for configuring the Z subelement */
	wifi_motion_pattern motion_pattern;
	/** floor in units of 1/16th of floor. 0x80000000 if unknown. */
	int floor;
	/** in units of 1/64 m */
	int height_above_floor;
	/** in units of 1/64 m. 0 if unknown */
	int height_unc;
} wifi_lci_information;

/** LCR information */
typedef struct {
	/** country code */
	char country_code[2];
	/** length of the info field */
	int length;
	/** Civic info to be copied in FTM frame */
	char civic_info[256];
} wifi_lcr_information;

/**
 * RTT Responder information
 */
typedef struct {
	wifi_channel_info channel;
	wifi_preamble preamble;
} wifi_rtt_responder;

/** =========== Define Copied from HAL END =========== */

#define MAX_RTT_CONFIG_NUM 10

/** RTT config params */
typedef struct wifi_rtt_config_params {
	t_u8 rtt_config_num;
	wifi_rtt_config rtt_config[MAX_RTT_CONFIG_NUM];
} wifi_rtt_config_params_t;

#define OID_RTT_REQUEST 0
#define OID_RTT_CANCEL 1

/** Pass RTT result element between mlan and moal */
typedef struct {
	/** element identifier  */
	t_u16 id;
	/** number of bytes to follow  */
	t_u16 len;
	/** data: fill with one wifi_rtt_result  */
	t_u8 data[];
} wifi_rtt_result_element;

/** station stats */
typedef struct _sta_stats {
	/** last_rx_in_msec */
	t_u64 last_rx_in_msec;
	/** rx_packets */
	t_u32 rx_packets;
	/** tx packets */
	t_u32 tx_packets;
	/** rx bytes */
	t_u32 rx_bytes;
	/** tx bytes */
	t_u32 tx_bytes;
} sta_stats;

#ifdef PRAGMA_PACK
#pragma pack(pop)
#endif

/** mlan_callbacks data structure */
typedef struct _mlan_callbacks {
	/** moal_get_fw_data */
	mlan_status (*moal_get_fw_data)(t_void *pmoal, t_u32 offset, t_u32 len,
					t_u8 *pbuf);
	mlan_status (*moal_get_vdll_data)(t_void *pmoal, t_u32 len, t_u8 *pbuf);
	/** moal_get_hw_spec_complete */
	mlan_status (*moal_get_hw_spec_complete)(t_void *pmoal,
						 mlan_status status,
						 pmlan_hw_info phw,
						 pmlan_bss_tbl ptbl);
	/** moal_init_fw_complete */
	mlan_status (*moal_init_fw_complete)(t_void *pmoal, mlan_status status);
	/** moal_shutdown_fw_complete */
	mlan_status (*moal_shutdown_fw_complete)(t_void *pmoal,
						 mlan_status status);
	/** moal_send_packet_complete */
	mlan_status (*moal_send_packet_complete)(t_void *pmoal,
						 pmlan_buffer pmbuf,
						 mlan_status status);
	/** moal_recv_complete */
	mlan_status (*moal_recv_complete)(t_void *pmoal, pmlan_buffer pmbuf,
					  t_u32 port, mlan_status status);
	/** moal_recv_packet */
	mlan_status (*moal_recv_packet)(t_void *pmoal, pmlan_buffer pmbuf);
	/** moal_recv_amsdu_packet */
	mlan_status (*moal_recv_amsdu_packet)(t_void *pmoal,
					      pmlan_buffer pmbuf);
	/** moal_recv_event */
	mlan_status (*moal_recv_event)(t_void *pmoal, pmlan_event pmevent);
	/** moal_ioctl_complete */
	mlan_status (*moal_ioctl_complete)(t_void *pmoal,
					   pmlan_ioctl_req pioctl_req,
					   mlan_status status);

	/** moal_alloc_mlan_buffer */
	mlan_status (*moal_alloc_mlan_buffer)(t_void *pmoal, t_u32 size,
					      ppmlan_buffer pmbuf);
	/** moal_free_mlan_buffer */
	mlan_status (*moal_free_mlan_buffer)(t_void *pmoal, pmlan_buffer pmbuf);

#ifdef USB
	/** moal_write_data_async */
	mlan_status (*moal_write_data_async)(t_void *pmoal, pmlan_buffer pmbuf,
					     t_u32 port);
#endif /* USB */
#if defined(SDIO) || defined(PCIE)
	/** moal_write_reg */
	mlan_status (*moal_write_reg)(t_void *pmoal, t_u32 reg, t_u32 data);
	/** moal_read_reg */
	mlan_status (*moal_read_reg)(t_void *pmoal, t_u32 reg, t_u32 *data);
#endif /* SDIO || PCIE */
	/** moal_write_data_sync */
	mlan_status (*moal_write_data_sync)(t_void *pmoal, pmlan_buffer pmbuf,
					    t_u32 port, t_u32 timeout);
	/** moal_read_data_sync */
	mlan_status (*moal_read_data_sync)(t_void *pmoal, pmlan_buffer pmbuf,
					   t_u32 port, t_u32 timeout);
	/** moal_malloc */
	mlan_status (*moal_malloc)(t_void *pmoal, t_u32 size, t_u32 flag,
				   t_u8 **ppbuf);
	/** moal_mfree */
	mlan_status (*moal_mfree)(t_void *pmoal, t_u8 *pbuf);
	/** moal_vmalloc */
	mlan_status (*moal_vmalloc)(t_void *pmoal, t_u32 size, t_u8 **ppbuf);
	/** moal_vfree */
	mlan_status (*moal_vfree)(t_void *pmoal, t_u8 *pbuf);
#ifdef PCIE
	/** moal_malloc_consistent */
	mlan_status (*moal_malloc_consistent)(t_void *pmoal, t_u32 size,
					      t_u8 **ppbuf, t_u64 *pbuf_pa);
	/** moal_mfree_consistent */
	mlan_status (*moal_mfree_consistent)(t_void *pmoal, t_u32 size,
					     t_u8 *pbuf, t_u64 buf_pa);
	/** moal_map_memory */
	mlan_status (*moal_map_memory)(t_void *pmoal, t_u8 *pbuf,
				       t_u64 *pbuf_pa, t_u32 size, t_u32 flag);
	/** moal_unmap_memory */
	mlan_status (*moal_unmap_memory)(t_void *pmoal, t_u8 *pbuf,
					 t_u64 buf_pa, t_u32 size, t_u32 flag);
#endif /* PCIE */
	/** moal_memset */
	t_void *(*moal_memset)(t_void *pmoal, t_void *pmem, t_u8 byte,
			       t_u32 num);
	/** moal_memcpy */
	t_void *(*moal_memcpy)(t_void *pmoal, t_void *pdest, const t_void *psrc,
			       t_u32 num);
	/** moal_memcpy_ext */
	t_void *(*moal_memcpy_ext)(t_void *pmoal, t_void *pdest,
				   const t_void *psrc, t_u32 num,
				   t_u32 dest_size);
	/** moal_memmove */
	t_void *(*moal_memmove)(t_void *pmoal, t_void *pdest,
				const t_void *psrc, t_u32 num);
	/** moal_memcmp */
	t_s32 (*moal_memcmp)(t_void *pmoal, const t_void *pmem1,
			     const t_void *pmem2, t_u32 num);
	/** moal_udelay */
	t_void (*moal_udelay)(t_void *pmoal, t_u32 udelay);
	/** moal_usleep_range */
	t_void (*moal_usleep_range)(t_void *pmoal, t_u32 min_delay,
				    t_u32 max_delay);
	/** moal_get_boot_ktime */
	mlan_status (*moal_get_boot_ktime)(t_void *pmoal, t_u64 *pnsec);
	/** moal_get_system_time */
	mlan_status (*moal_get_system_time)(t_void *pmoal, t_u32 *psec,
					    t_u32 *pusec);
	/** moal_init_timer*/
	mlan_status (*moal_init_timer)(t_void *pmoal, t_void **pptimer,
				       IN t_void (*callback)(t_void *pcontext),
				       t_void *pcontext);
	/** moal_free_timer */
	mlan_status (*moal_free_timer)(t_void *pmoal, t_void *ptimer);
	/** moal_start_timer*/
	mlan_status (*moal_start_timer)(t_void *pmoal, t_void *ptimer,
					t_u8 periodic, t_u32 msec);
	/** moal_stop_timer*/
	mlan_status (*moal_stop_timer)(t_void *pmoal, t_void *ptimer);
	/** moal_init_lock */
	mlan_status (*moal_init_lock)(t_void *pmoal, t_void **pplock);
	/** moal_free_lock */
	mlan_status (*moal_free_lock)(t_void *pmoal, t_void *plock);
	/** moal_spin_lock */
	mlan_status (*moal_spin_lock)(t_void *pmoal, t_void *plock);
	/** moal_spin_unlock */
	mlan_status (*moal_spin_unlock)(t_void *pmoal, t_void *plock);
	/** moal_print */
	t_void (*moal_print)(t_void *pmoal, t_u32 level, char *pformat, IN...);
	/** moal_print_netintf */
	t_void (*moal_print_netintf)(t_void *pmoal, t_u32 bss_index,
				     t_u32 level);
	/** moal_assert */
	t_void (*moal_assert)(t_void *pmoal, t_u32 cond);

	/** moal_hist_data_add */
	t_void (*moal_hist_data_add)(t_void *pmoal, t_u32 bss_index,
				     t_u16 rx_rate, t_s8 snr, t_s8 nflr,
				     t_u8 antenna);
	t_void (*moal_updata_peer_signal)(t_void *pmoal, t_u32 bss_index,
					  t_u8 *peer_addr, t_s8 snr, t_s8 nflr);
	mlan_status (*moal_get_host_time_ns)(t_u64 *time);
	t_u64 (*moal_do_div)(t_u64 num, t_u32 base);
	void (*moal_tp_accounting)(t_void *pmoal, t_void *buf,
				   t_u32 drop_point);
	void (*moal_tp_accounting_rx_param)(t_void *pmoal, unsigned int type,
					    unsigned int rsvd1);
	void (*moal_amsdu_tp_accounting)(t_void *pmoal, t_s32 delay,
					 t_s32 copy_delay);
	mlan_status (*moal_calc_short_ssid)(t_u8 *pssid, t_u32 ssid_len,
					    t_u32 *pshort_ssid);
} mlan_callbacks, *pmlan_callbacks;

/** Parameter unchanged, use MLAN default setting */
#define ROBUSTCOEX_GPIO_UNCHANGED 0
/** Parameter enabled, override MLAN default setting */
#define ROBUSTCOEX_GPIO_CFG 1

#if defined(SDIO)
/** Interrupt Mode SDIO */
#define INT_MODE_SDIO 0
/** Interrupt Mode GPIO */
#define INT_MODE_GPIO 1
/** New mode: GPIO-1 as a duplicated signal of interrupt as appear of SDIO_DAT1
 */
#define GPIO_INT_NEW_MODE 255
#endif

/** Parameter unchanged, use MLAN default setting */
#define MLAN_INIT_PARA_UNCHANGED 0
/** Parameter enabled, override MLAN default setting */
#define MLAN_INIT_PARA_ENABLED 1
/** Parameter disabled, override MLAN default setting */
#define MLAN_INIT_PARA_DISABLED 2

/** Control bit for stream 2X2 */
#define FEATURE_CTRL_STREAM_2X2 MBIT(0)
/** Control bit for DFS support */
#define FEATURE_CTRL_DFS_SUPPORT MBIT(1)
#ifdef USB
/** Control bit for winner check & not wait for FW ready event */
#define FEATURE_CTRL_USB_NEW_INIT MBIT(2)
#endif
/** Default feature control */
#define FEATURE_CTRL_DEFAULT 0xffffffff
/** Check if stream 2X2 enabled */
#define IS_STREAM_2X2(x) ((x)&FEATURE_CTRL_STREAM_2X2)
/** Check if DFS support enabled */
#define IS_DFS_SUPPORT(x) ((x)&FEATURE_CTRL_DFS_SUPPORT)
#ifdef USB
/** Check if winner check & not wait for FW ready event */
#define IS_USB_NEW_INIT(x) ((x)&FEATURE_CTRL_USB_NEW_INIT)
#endif

/*
#define DRV_MODE_NAN                 MBIT(4)
#define DRV_MODE_11P                 MBIT(5)
#define DRV_MODE_MAC80211            MBIT(6)
#define DRV_MODE_DFS                 MBIT(7)*/
#define DRV_MODE_MASK (MBIT(4) | MBIT(5) | MBIT(6) | MBIT(7))

/** mlan_device data structure */
typedef struct _mlan_device {
	/** MOAL Handle */
	t_void *pmoal_handle;
	/** BSS Attributes */
	mlan_bss_attr bss_attr[MLAN_MAX_BSS_NUM];
	/** Callbacks */
	mlan_callbacks callbacks;
#ifdef MFG_CMD_SUPPORT
	/** MFG mode */
	t_u32 mfg_mode;
#endif
#ifdef PCIE
	t_u16 ring_size;
#endif
#if defined(SDIO)
	/** SDIO interrupt mode (0: INT_MODE_SDIO, 1: INT_MODE_GPIO) */
	t_u32 int_mode;
	/** GPIO interrupt pin number */
	t_u32 gpio_pin;
#endif
#ifdef DEBUG_LEVEL1
	/** Driver debug bit masks */
	t_u32 drvdbg;
#endif
	/** allocate fixed buffer size for scan beacon buffer*/
	t_u32 fixed_beacon_buffer;
	/** SDIO MPA Tx */
	t_u32 mpa_tx_cfg;
	/** SDIO MPA Rx */
	t_u32 mpa_rx_cfg;
#ifdef SDIO
	/** host max block num */
	t_u32 max_blk_count;
	/** sdio blk size */
	t_u32 sdio_blk_size;
	/** SDIO Single port rx aggr */
	t_u8 sdio_rx_aggr_enable;
	/* see blk_queue_max_segment_size */
	t_u32 max_seg_size;
	/* see blk_queue_max_segments */
	t_u16 max_segs;
#endif
	/** Auto deep sleep */
	t_u32 auto_ds;
	/** Boot Time Config */
	t_u32 bootup_cal_ctrl;
	/** IEEE PS mode */
	t_u32 ps_mode;
	/** Max Tx buffer size */
	t_u32 max_tx_buf;
#if defined(STA_SUPPORT)
	/** 802.11d configuration */
	t_u32 cfg_11d;
#endif
	/** Feature control bitmask */
	t_u32 feature_control;
	/** enable/disable rx work */
	t_u8 rx_work;
	/** dev cap mask */
	t_u32 dev_cap_mask;
	/** oob independent reset */
	t_u32 indrstcfg;
	/** dtim interval */
	t_u16 multi_dtim;
	/** IEEE ps inactivity timeout value */
	t_u16 inact_tmo;
	/** card type */
	t_u16 card_type;
	/** card rev */
	t_u8 card_rev;
	/** Host sleep wakeup interval */
	t_u32 hs_wake_interval;
	/** GPIO to indicate wakeup source */
	t_u8 indication_gpio;
	/** Dynamic MIMO-SISO switch for hscfg*/
	t_u8 hs_mimo_switch;
	/** channel time and mode for DRCS*/
	t_u32 drcs_chantime_mode;
#ifdef USB
	/** Tx CMD endpoint address */
	t_u8 tx_cmd_ep;
	/** Rx CMD/EVT endpoint address */
	t_u8 rx_cmd_ep;

	/** Rx data endpoint address */
	t_u8 rx_data_ep;
	/** Tx data endpoint address */
	t_u8 tx_data_ep;
	/** Tx data second endpoint address */
	t_u8 tx_data2_ep;
#endif
	/** passive to active scan */
	t_u8 passive_to_active_scan;
	/** uap max supported station per chip */
	t_u8 uap_max_sta;
	/** wacp mode */
	t_u8 wacp_mode;
	/** custom Fw data */
	t_u32 fw_data_cfg;
	/** drv mode */
	t_u32 drv_mode;
	/** dfs w53 cfg */
	t_u8 dfs53cfg;
	/** dfs_offload */
	t_u8 dfs_offload;
	/** extend enhance scan */
	t_u8 ext_scan;
	/* mcs32 setting */
	t_u8 mcs32;
	/** second mac flag */
	t_u8 second_mac;
	/** napi */
	t_u8 napi;
	/** antcfg */
	t_u32 antcfg;
	/** dmcs */
	t_u8 dmcs;
	t_u8 pref_dbc;
	t_u32 reject_addba_req;

	t_u32 max_tx_pending;
	t_u16 tx_budget;
	t_u8 mclient_scheduling;
	t_u8 disable_11h_tpc;
	t_u8 tpe_ie_ignore;
} mlan_device, *pmlan_device;

/** MLAN API function prototype */
#define MLAN_API

/** Registration */
MLAN_API mlan_status mlan_register(pmlan_device pmdevice,
				   t_void **ppmlan_adapter);

/** Un-registration */
MLAN_API mlan_status mlan_unregister(t_void *padapter);

/** Firmware Downloading */
MLAN_API mlan_status mlan_dnld_fw(t_void *padapter, pmlan_fw_image pmfw);

/** Custom data pass API */
MLAN_API mlan_status mlan_set_init_param(t_void *padapter,
					 pmlan_init_param pparam);

/** Firmware Initialization */
MLAN_API mlan_status mlan_init_fw(t_void *padapter);

/** Firmware Shutdown */
MLAN_API mlan_status mlan_shutdown_fw(t_void *padapter);

/** Main Process */
MLAN_API mlan_status mlan_main_process(t_void *padapter);

/** Rx process */
mlan_status mlan_rx_process(t_void *padapter, t_u8 *rx_pkts);

/** Packet Transmission */
MLAN_API mlan_status mlan_send_packet(t_void *padapter, pmlan_buffer pmbuf);

#ifdef USB
/** mlan_write_data_async_complete */
MLAN_API mlan_status mlan_write_data_async_complete(t_void *padapter,
						    pmlan_buffer pmbuf,
						    t_u32 port,
						    mlan_status status);

/** Packet Reception */
MLAN_API mlan_status mlan_recv(t_void *padapter, pmlan_buffer pmbuf,
			       t_u32 port);
#endif /* USB */

/** Packet Reception complete callback */
MLAN_API mlan_status mlan_recv_packet_complete(t_void *padapter,
					       pmlan_buffer pmbuf,
					       mlan_status status);

/** handle amsdu deaggregated packet */
void mlan_process_deaggr_pkt(t_void *padapter, pmlan_buffer pmbuf, t_u8 *drop);

#if defined(SDIO) || defined(PCIE)
/** interrupt handler */
MLAN_API mlan_status mlan_interrupt(t_u16 msg_id, t_void *padapter);

#if defined(SYSKT)
/** GPIO IRQ callback function */
MLAN_API t_void mlan_hs_callback(t_void *pctx);
#endif /* SYSKT_MULTI || SYSKT */
#endif /* SDIO || PCIE */

MLAN_API t_void mlan_pm_wakeup_card(t_void *padapter, t_u8 keep_wakeup);

MLAN_API t_u8 mlan_is_main_process_running(t_void *adapter);
#ifdef PCIE
MLAN_API t_void mlan_set_int_mode(t_void *adapter, t_u32 int_mode,
				  t_u8 func_num);
#endif
/** mlan ioctl */
MLAN_API mlan_status mlan_ioctl(t_void *padapter, pmlan_ioctl_req pioctl_req);
/** mlan select wmm queue */
MLAN_API t_u8 mlan_select_wmm_queue(t_void *padapter, t_u8 bss_num, t_u8 tid);

/** mlan mask host interrupt */
MLAN_API mlan_status mlan_disable_host_int(t_void *padapter);
/** mlan unmask host interrupt */
MLAN_API mlan_status mlan_enable_host_int(t_void *padapter);
#ifdef PCIE
MLAN_API void mlan_process_pcie_interrupt_cb(t_void *pmadapter, int type);
#endif

#define CSI_SIGNATURE 0xABCD

/** secure add/sub **/
#ifndef SINT8_MAX
#define SINT8_MAX 0x7f
#endif

#ifndef SINT8_MIN
#define SINT8_MIN 0x80
#endif

#ifndef UINT8_MAX
#define UINT8_MAX 0xff
#endif

#ifndef SINT16_MAX
#define SINT16_MAX 0x7fff
#endif

#ifndef SINT16_MIN
#define SINT16_MIN 0x8000
#endif

#ifndef UINT16_MAX
#define UINT16_MAX 0xffff
#endif

#ifndef SINT32_MAX
#define SINT32_MAX 0x7fffffff
#endif

#ifndef SINT32_MIN
#define SINT32_MIN 0x80000000
#endif

#ifndef UINT32_MAX
#define UINT32_MAX 0xffffffff
#endif

#ifndef SINT64_MAX
#define SINT64_MAX 0x7fffffffffffffff
#endif

#ifndef SINT64_MIN
#define SINT64_MIN 0x8000000000000000
#endif

#ifndef UINT64_MAX
#define UINT64_MAX 0xffffffffffffffff
#endif

#ifndef PTR_MAX
#ifdef MLAN_64BIT
#define PTR_MAX UINT64_MAX
#else
#define PTR_MAX UINT32_MAX
#endif
#endif

#ifndef SINT_MAX
#ifdef MLAN_64BIT
#define SINT_MAX SINT64_MAX
#else
#define SINT_MAX SINT32_MAX
#endif
#endif

#ifndef SINT_MIN
#ifdef MLAN_64BIT
#define SINT_MIN SINT64_MIN
#else
#define SINT_MIN SINT32_MIN
#endif
#endif

#ifndef UINT_MAX
#ifdef MLAN_64BIT
#define UINT_MAX UINT64_MAX
#else
#define UINT_MAX UINT32_MAX
#endif
#endif

typedef enum {
	TYPE_SINT8,
	TYPE_UINT8,
	TYPE_SINT16,
	TYPE_UINT16,
	TYPE_SINT32,
	TYPE_UINT32,
	TYPE_SINT64,
	TYPE_UINT64,
	TYPE_PTR,
} data_type;

#endif /* !_MLAN_DECL_H_ */
