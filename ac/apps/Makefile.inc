THIS_DIR := $(dir $(lastword $(MAKEFILE_LIST)))
DFU ?= dfu-util
BOARD_USB_VENDOR_ID  := 1EAF
BOARD_USB_PRODUCT_ID := 0003
SUPPORT_PATH = $(THIS_DIR)../syrup/libmaple/support/
