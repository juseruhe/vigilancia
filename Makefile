############################################################################
# apps/examples/lora_rx/Makefile
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = $(CONFIG_EXAMPLES_LORA_RX_PROGNAME)
PRIORITY  = $(CONFIG_EXAMPLES_LORA_RX_PRIORITY)
STACKSIZE = $(CONFIG_EXAMPLES_LORA_RX_STACKSIZE)
MODULE    = $(CONFIG_EXAMPLES_LORA_RX)

MAINSRC = lora_rx_main.c
CSRCS   = sx1278.c ssd1306.c

include $(APPDIR)/Application.mk
