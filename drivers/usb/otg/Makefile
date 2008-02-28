#
# Makefile for USB OTG controller  driver
#
obj-$(CONFIG_USB_OTG)		+= usb.o

fsl_usb2_otg-objs		:= fsl_otg.o otg_fsm.o
ifneq ($(CONFIG_USB),)
obj-m				+= fsl_usb2_otg.o
endif

