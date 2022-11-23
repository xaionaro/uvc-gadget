CROSS_COMPILE	?= 
ARCH		?= x86
KERNEL_DIR	?= /usr/src/linux

LIBAV=`pkg-config --libs libavformat libavutil libavcodec`

CC		:= $(CROSS_COMPILE)gcc
KERNEL_INCLUDE	:= -I$(KERNEL_DIR)/include -I$(KERNEL_DIR)/arch/$(ARCH)/include
CFLAGS		:= -std=gnu99 -Wall -Wextra -g $(KERNEL_INCLUDE) -O2
LDFLAGS		:= -g $(LIBAV)

all: uvc-gadget

uvc-gadget: uvc-gadget.o
	$(CC) -o $@ $^ $(LDFLAGS)

clean:
	rm -f *.o
	rm -f uvc-gadget
