include ../Makefile.inc

upload:
	$(SUPPORT_PATH)/scripts/reset.py && \
		sleep 1                  && \
		$(DFU) -a1 -d $(BOARD_USB_VENDOR_ID):$(BOARD_USB_PRODUCT_ID) -D application.bin -R
