# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the video capture/playback device drivers.
#

obj-$(CONFIG_VIDEO_ASPEED)		+= aspeed-video.o
obj-$(CONFIG_VIDEO_CADENCE)		+= cadence/
obj-$(CONFIG_VIDEO_VIA_CAMERA) += via-camera.o
obj-$(CONFIG_VIDEO_CAFE_CCIC) += marvell-ccic/
obj-$(CONFIG_VIDEO_MMP_CAMERA) += marvell-ccic/

obj-$(CONFIG_VIDEO_OMAP3)	+= omap3isp/
obj-$(CONFIG_VIDEO_PXA27x)	+= pxa_camera.o

obj-$(CONFIG_VIDEO_VIU) += fsl-viu.o

obj-y	+= ti-vpe/

obj-$(CONFIG_VIDEO_MX2_EMMAPRP)		+= mx2_emmaprp.o
obj-$(CONFIG_VIDEO_CODA)		+= coda/

obj-$(CONFIG_VIDEO_IMX_PXP)		+= imx-pxp.o

obj-$(CONFIG_VIDEO_MEM2MEM_DEINTERLACE)	+= m2m-deinterlace.o

obj-$(CONFIG_VIDEO_MUX)			+= video-mux.o

obj-$(CONFIG_VIDEO_S3C_CAMIF)		+= s3c-camif/
obj-$(CONFIG_VIDEO_SAMSUNG_EXYNOS4_IS)	+= exynos4-is/
obj-$(CONFIG_VIDEO_SAMSUNG_S5P_JPEG)	+= s5p-jpeg/
obj-$(CONFIG_VIDEO_SAMSUNG_S5P_MFC)	+= s5p-mfc/

obj-$(CONFIG_VIDEO_SAMSUNG_S5P_G2D)	+= s5p-g2d/
obj-$(CONFIG_VIDEO_SAMSUNG_EXYNOS_GSC)	+= exynos-gsc/

obj-$(CONFIG_VIDEO_STI_BDISP)		+= sti/bdisp/
obj-$(CONFIG_VIDEO_STI_HVA)		+= sti/hva/
obj-$(CONFIG_DVB_C8SECTPFE)		+= sti/c8sectpfe/

obj-$(CONFIG_VIDEO_STI_DELTA)		+= sti/delta/

obj-y					+= stm32/

obj-y					+= davinci/

obj-$(CONFIG_VIDEO_SH_VOU)		+= sh_vou.o

obj-$(CONFIG_VIDEO_RCAR_DRIF)		+= rcar_drif.o
obj-$(CONFIG_VIDEO_RENESAS_CEU)		+= renesas-ceu.o
obj-$(CONFIG_VIDEO_RENESAS_FCP)		+= rcar-fcp.o
obj-$(CONFIG_VIDEO_RENESAS_FDP1)	+= rcar_fdp1.o
obj-$(CONFIG_VIDEO_RENESAS_JPU)		+= rcar_jpu.o
obj-$(CONFIG_VIDEO_RENESAS_VSP1)	+= vsp1/

obj-$(CONFIG_VIDEO_ROCKCHIP_RGA)	+= rockchip/rga/

obj-y	+= omap/

obj-$(CONFIG_VIDEO_AM437X_VPFE)		+= am437x/

obj-$(CONFIG_VIDEO_XILINX)		+= xilinx/

obj-$(CONFIG_VIDEO_RCAR_VIN)		+= rcar-vin/
obj-y += renesas/rzg2l-cru/

obj-$(CONFIG_VIDEO_ATMEL_ISC)		+= atmel/
obj-$(CONFIG_VIDEO_ATMEL_ISI)		+= atmel/

obj-$(CONFIG_VIDEO_STM32_DCMI)		+= stm32/

obj-$(CONFIG_VIDEO_MX8_CAPTURE)		+= imx8/
obj-$(CONFIG_VIDEO_MXC_CAPTURE)		+= mxc/capture/
obj-$(CONFIG_VIDEO_MXC_OUTPUT)		+= mxc/output/

obj-$(CONFIG_VIDEO_MEDIATEK_VPU)	+= mtk-vpu/

obj-$(CONFIG_VIDEO_MEDIATEK_VCODEC)	+= mtk-vcodec/

obj-$(CONFIG_VIDEO_MEDIATEK_MDP)	+= mtk-mdp/

obj-$(CONFIG_VIDEO_MEDIATEK_JPEG)	+= mtk-jpeg/

obj-$(CONFIG_VIDEO_QCOM_CAMSS)		+= qcom/camss/

obj-$(CONFIG_VIDEO_QCOM_VENUS)		+= qcom/venus/

obj-y					+= sunxi/
