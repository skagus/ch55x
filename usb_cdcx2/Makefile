TARGET = usb_device_cdcx2

# Adjust the XRAM location and size to leave space for the USB DMA buffers
XRAM_SIZE = 0x0200
XRAM_LOC = 0x0200

EXTRA_FLAGS = --opt-code-speed

C_FILES = \
	usb_cdc2x.c \
	logic.c \
	../include/debug.c \
	../include/touchkey.c
pre-flash:
	

include ../Makefile.include
