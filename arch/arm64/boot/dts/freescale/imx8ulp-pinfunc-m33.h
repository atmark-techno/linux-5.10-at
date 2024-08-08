// SPDX-License-Identifier: BSD-3-Clause
/*
 * Aliases to be used with gpio imx pinmux
 * The actual pin config comes from mcuxsdk:
 * core/devices/MIMX8UD7/drivers/fsl_iomuxc.h
 */

#ifndef __DTS_IMX8ULP_PINFUNC_M33_H
#define __DTS_IMX8ULP_PINFUNC_M33_H

/*!
 * @addtogroup iomuxc_driver
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @name Pin function ID
 * The pin function ID is a tuple of \<muxRegister muxMode inputRegister inputDaisy configRegister\>
 *
 * @{
 */
#define IOMUXC_PTA0_PTA0                 0x000
#define IOMUXC_PTA1_PTA1                 0x001
#define IOMUXC_PTA2_PTA2                 0x002
#define IOMUXC_PTA3_PTA3                 0x003
#define IOMUXC_PTA4_PTA4                 0x004
#define IOMUXC_PTA5_PTA5                 0x005
#define IOMUXC_PTA6_PTA6                 0x006
#define IOMUXC_PTA7_PTA7                 0x007
#define IOMUXC_PTA8_PTA8                 0x008
#define IOMUXC_PTA9_PTA9                 0x009
#define IOMUXC_PTA10_PTA10               0x00a
#define IOMUXC_PTA11_PTA11               0x00b
#define IOMUXC_PTA12_PTA12               0x00c
#define IOMUXC_PTA13_PTA13               0x00d
#define IOMUXC_PTA14_PTA14               0x00e
#define IOMUXC_PTA15_PTA15               0x00f
#define IOMUXC_PTA16_PTA16               0x010
#define IOMUXC_PTA17_PTA17               0x011
#define IOMUXC_PTA18_PTA18               0x012
#define IOMUXC_PTA19_PTA19               0x013
#define IOMUXC_PTA20_PTA20               0x014
#define IOMUXC_PTA21_PTA21               0x015
#define IOMUXC_PTA22_PTA22               0x016
#define IOMUXC_PTA23_PTA23               0x017
#define IOMUXC_PTA24_PTA24               0x018
#define IOMUXC_PTB0_PTB0                 0x100
#define IOMUXC_PTB1_PTB1                 0x101
#define IOMUXC_PTB2_PTB2                 0x102
#define IOMUXC_PTB3_PTB3                 0x103
#define IOMUXC_PTB4_PTB4                 0x104
#define IOMUXC_PTB5_PTB5                 0x105
#define IOMUXC_PTB6_PTB6                 0x106
#define IOMUXC_PTB7_PTB7                 0x107
#define IOMUXC_PTB8_PTB8                 0x108
#define IOMUXC_PTB9_PTB9                 0x109
#define IOMUXC_PTB10_PTB10               0x10a
#define IOMUXC_PTB11_PTB11               0x10b
#define IOMUXC_PTB12_PTB12               0x10c
#define IOMUXC_PTB13_PTB13               0x10d
#define IOMUXC_PTB14_PTB14               0x10e
#define IOMUXC_PTB15_PTB15               0x10f
#define IOMUXC_PTC0_PTC0                 0x200
#define IOMUXC_PTC1_PTC1                 0x201
#define IOMUXC_PTC2_PTC2                 0x202
#define IOMUXC_PTC3_PTC3                 0x203
#define IOMUXC_PTC4_PTC4                 0x204
#define IOMUXC_PTC5_PTC5                 0x205
#define IOMUXC_PTC6_PTC6                 0x206
#define IOMUXC_PTC7_PTC7                 0x207
#define IOMUXC_PTC8_PTC8                 0x208
#define IOMUXC_PTC9_PTC9                 0x209
#define IOMUXC_PTC10_PTC10               0x20a
#define IOMUXC_PTC11_PTC11               0x20b
#define IOMUXC_PTC12_PTC12               0x20c
#define IOMUXC_PTC13_PTC13               0x20d
#define IOMUXC_PTC14_PTC14               0x20e
#define IOMUXC_PTC15_PTC15               0x20f
#define IOMUXC_PTC16_PTC16               0x210
#define IOMUXC_PTC17_PTC17               0x211
#define IOMUXC_PTC18_PTC18               0x212
#define IOMUXC_PTC19_PTC19               0x213
#define IOMUXC_PTC20_PTC20               0x214
#define IOMUXC_PTC21_PTC21               0x215
#define IOMUXC_PTC22_PTC22               0x216
#define IOMUXC_PTC23_PTC23               0x217
/*@}*/

#define IOMUXC_PCR_PS_MASK        (0x1U)
#define IOMUXC_PCR_PS_SHIFT       (0U)
#define IOMUXC_PCR_PS(x)          (((uint32_t)(((uint32_t)(x)) << IOMUXC_PCR_PS_SHIFT)) & IOMUXC_PCR_PS_MASK)
#define IOMUXC_PCR_PE_MASK        (0x2U)
#define IOMUXC_PCR_PE_SHIFT       (1U)
#define IOMUXC_PCR_PE(x)          (((uint32_t)(((uint32_t)(x)) << IOMUXC_PCR_PE_SHIFT)) & IOMUXC_PCR_PE_MASK)
#define IOMUXC_PCR_SRE_MASK       (0x4U)
#define IOMUXC_PCR_SRE_SHIFT      (2U)
#define IOMUXC_PCR_SRE(x)         (((uint32_t)(((uint32_t)(x)) << IOMUXC_PCR_SRE_SHIFT)) & IOMUXC_PCR_SRE_MASK)
#define IOMUXC_PCR_ODE_MASK       (0x20U)
#define IOMUXC_PCR_ODE_SHIFT      (5U)
#define IOMUXC_PCR_ODE(x)         (((uint32_t)(((uint32_t)(x)) << IOMUXC_PCR_ODE_SHIFT)) & IOMUXC_PCR_ODE_MASK)
#define IOMUXC_PCR_DSE_MASK       (0x40U)
#define IOMUXC_PCR_DSE_SHIFT      (6U)
#define IOMUXC_PCR_DSE(x)         (((uint32_t)(((uint32_t)(x)) << IOMUXC_PCR_DSE_SHIFT)) & IOMUXC_PCR_DSE_MASK)
#define IOMUXC_PCR_MUX_MODE_MASK  (0xF00U)
#define IOMUXC_PCR_MUX_MODE_SHIFT (8U)
#define IOMUXC_PCR_MUX_MODE(x)    (((uint32_t)(((uint32_t)(x)) << IOMUXC_PCR_MUX_MODE_SHIFT)) & IOMUXC_PCR_MUX_MODE_MASK)
#define IOMUXC_PCR_LK_MASK        (0x8000U)
#define IOMUXC_PCR_LK_SHIFT       (15U)
#define IOMUXC_PCR_LK(x)          (((uint32_t)(((uint32_t)(x)) << IOMUXC_PCR_LK_SHIFT)) & IOMUXC_PCR_LK_MASK)
#define IOMUXC_PCR_IBE_MASK       (0x10000U)
#define IOMUXC_PCR_IBE_SHIFT      (16U)
#define IOMUXC_PCR_IBE(x)         (((uint32_t)(((uint32_t)(x)) << IOMUXC_PCR_IBE_SHIFT)) & IOMUXC_PCR_IBE_MASK)
#define IOMUXC_PCR_OBE_MASK       (0x20000U)
#define IOMUXC_PCR_OBE_SHIFT      (17U)
#define IOMUXC_PCR_OBE(x)         (((uint32_t)(((uint32_t)(x)) << IOMUXC_PCR_OBE_SHIFT)) & IOMUXC_PCR_OBE_MASK)
#define IOMUXC_PCR_DFE_MASK       (0x100000U)
#define IOMUXC_PCR_DFE_SHIFT      (20U)
#define IOMUXC_PCR_DFE(x)         (((uint32_t)(((uint32_t)(x)) << IOMUXC_PCR_DFE_SHIFT)) & IOMUXC_PCR_DFE_MASK)
#define IOMUXC_PCR_DFCS_MASK      (0x200000U)
#define IOMUXC_PCR_DFCS_SHIFT     (21U)
#define IOMUXC_PCR_DFCS(x)        (((uint32_t)(((uint32_t)(x)) << IOMUXC_PCR_DFCS_SHIFT)) & IOMUXC_PCR_DFCS_MASK)
#define IOMUXC_PCR_DFD_MASK       (0x7C00000U)
#define IOMUXC_PCR_DFD_SHIFT      (22U)
#define IOMUXC_PCR_DFD(x)         (((uint32_t)(((uint32_t)(x)) << IOMUXC_PCR_DFD_SHIFT)) & IOMUXC_PCR_DFD_MASK)

#define IOMUXC_PSMI_SSS_MASK  (0xFU)
#define IOMUXC_PSMI_SSS_SHIFT (0U)
#define IOMUXC_PSMI_SSS(x)    (((uint32_t)(((uint32_t)(x)) << IOMUXC_PSMI_SSS_SHIFT)) & IOMUXC_PSMI_SSS_MASK)
#define IOMUXC_PSMI_INV_MASK  (0x8000U)
#define IOMUXC_PSMI_INV_SHIFT (15U)
#define IOMUXC_PSMI_INV(x)    (((uint32_t)(((uint32_t)(x)) << IOMUXC_PSMI_INV_SHIFT)) & IOMUXC_PSMI_INV_MASK)

#endif /* __DTS_IMX8ULP_PINFUNC_M33_H */
