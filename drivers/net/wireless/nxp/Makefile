# SPDX-License-Identifier: GPL-2.0
#
# Copyright (c) 2022 Atmark techno
#
# quick and dirty adaptation with only options we care about
# trying to turn any of these off quickly leads to build failure
# so did not try too hard.

obj-$(CONFIG_MLAN) += mlan.o moal.o

# version
ccflags-y += -DMLAN_RELEASE_VERSION='"540.p33"' -DFPNUM='"92"'

# CONFIG_DEBUG 1
ccflags-y += -DDEBUG_LEVEL1

# CONFIG_IMX_SUPPORT
ccflags-y += -DIMX_SUPPORT

# CONFIG_DUMP_TO_PROC
ccflags-y += -DDUMP_TO_PROC

# CONFIG_SDIO_SUSPEND_RESUME
ccflags-y += -DSDIO_SUSPEND_RESUME

# CONFIG_SDIO
ccflags-y += -DSDIO -DSDIO_MMC
mlan-y += mlan/mlan_sdio.o
moal-y += mlinux/moal_sdio_mmc.o

# CONFIG_STA_SUPPORT
ccflags-y += -DSTA_SUPPORT
mlan-y += mlan/mlan_meas.o \
		mlan/mlan_scan.o \
		mlan/mlan_sta_ioctl.o \
		mlan/mlan_sta_rx.o \
		mlan/mlan_sta_tx.o \
		mlan/mlan_sta_event.o \
		mlan/mlan_sta_cmd.o \
		mlan/mlan_sta_cmdresp.o \
		mlan/mlan_join.o

# CONFIG_STA_WEXT
ccflags-y += -DSTA_WEXT
moal-y += mlinux/moal_priv.o \
		mlinux/moal_wext.o

# CONFIG_STA_CFG80211
ccflags-y += -DSTA_CFG80211
moal-y += mlinux/moal_cfg80211.o \
		mlinux/moal_cfg80211_util.o \
		mlinux/moal_sta_cfg80211.o

# CONFIG_UAP_SUPPORT
ccflags-y += -DUAP_SUPPORT
mlan-y += mlan/mlan_uap_ioctl.o \
		mlan/mlan_uap_cmdevent.o \
		mlan/mlan_uap_txrx.o
moal-y += mlinux/moal_uap.o

# CONFIG_UAP_WEXT
moal-y += mlinux/moal_uap_priv.o \
		mlinux/moal_uap_wext.o
ccflags-y += -DUAP_WEXT

# CONFIG_UAP_CFG80211
ccflags-y += -DUAP_CFG80211
moal-y += mlinux/moal_cfg80211.o \
		mlinux/moal_cfg80211_util.o \
		mlinux/moal_uap_cfg80211.o

# CONFIG_WIFI_DIRECT_SUPPORT
ccflags-y += -DWIFI_DIRECT_SUPPORT

# CONFIG_REASSOCIATION
ccflags-y += -DREASSOCIATION

# CONFIG_MFG_CMD_SUPPORT
ccflags-y += -DMFG_CMD_SUPPORT

# CONFIG_DFS_TESTING_SUPPORT
ccflags-y += -DDFS_TESTING_SUPPORT

# CONFIG_PCIE9098 (88W9098)
ccflags-y += -DPCIE9098

# CONFIG_SD9177 (IW612)
ccflags-y += -DSD9177

# CONFIG_PCIE
ccflags-y += -DPCIE
moal-y += mlinux/moal_pcie.o
mlan-y +=  mlan/mlan_pcie.o

# CONFIG_PROC_FS
moal-y += mlinux/moal_proc.o \
		mlinux/moal_debug.o

# CONFIG_64BIT
ccflags-y += -DMLAN_64BIT

# non-option
ccflags-y += -DLINUX
moal-y += mlinux/moal_main.o \
                mlinux/moal_ioctl.o \
                mlinux/moal_shim.o \
                mlinux/moal_eth_ioctl.o \
                mlinux/moal_init.o
mlan-y += mlan/mlan_shim.o mlan/mlan_init.o \
                mlan/mlan_txrx.o \
                mlan/mlan_cmdevt.o mlan/mlan_misc.o \
                mlan/mlan_cfp.o \
                mlan/mlan_module.o \
		mlan/mlan_wmm.o \
		mlan/mlan_11n_aggr.o \
		mlan/mlan_11n_rxreorder.o \
		mlan/mlan_11n.o \
		mlan/mlan_11ac.o \
		mlan/mlan_11ax.o \
		mlan/mlan_11d.o \
		mlan/mlan_11h.o
