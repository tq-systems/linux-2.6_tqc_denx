#
# Makefile for the Freescale PowerPC 512x linux kernel.
#
obj-y				:= clock.o mpc5121_usb.o mpc5121_pscgpio.o
obj-$(CONFIG_PCI)		+= pci.o
obj-$(CONFIG_SPI)		+= mpc5121_spi.o
obj-$(CONFIG_MPC5121_ADS)	+= mpc5121_ads.o mpc5121ads_cpld.o
obj-$(CONFIG_PM)		+= mpc512x_pm.o
obj-$(CONFIG_MPC5121_PM_TEST)	+= mpc512x_pm_test.o
