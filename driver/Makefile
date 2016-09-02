###############################################################################
#
# Filename: 	Makefile
#
# Description:	Makefile for Demo driver
#
#
# Copyright 2000-2009 Concurrent Technologies.
#
###############################################################################

# Defaults

src ?= .
obj ?= .

KERNEL_VERSION := $(shell uname -r)
KERNEL_MODLIB := /lib/modules/$(KERNEL_VERSION)
KERNEL_SOURCES := $(shell test -d $(KERNEL_MODLIB)/source && echo $(KERNEL_MODLIB)/source || echo $(KERNEL_MODLIB)/build)
KERNEL_OUTPUT := $(KERNEL_SOURCES)

KERNEL := kernel-$(KERNEL_VERSION)

# Select module build process, depending on kernel version
ifeq ($(findstring 2.4.,$(KERNEL_VERSION)),2.4.)
USE_KBUILD := FALSE
else
USE_KBUILD := TRUE
endif


TARGET := universeIIdrv

ifeq ($(USE_KBUILD),FALSE)
OBJS := $(TARGET).o
SRC := universe_II.c
endif

CC 	= gcc
AR	= ar
LD	= ld
INCLUDE = $(KERNEL_SOURCES)/include/

ifeq ($(USE_KBUILD),FALSE)
CFLAGS = -O2 -Wall -DMODULE -D__KERNEL__ -I./ -I$(INCLUDE)
endif

all: prepare vmeall

prepare:
ifeq ($(USE_KBUILD),FALSE)


$(TARGET).o: $(SRC:.c=.o)
	$(LD) -r $^ -o $@

$(KERNEL):
	mkdir $(KERNEL)

$(KERNEL)/$(TARGET).o: $(TARGET).o
	cp $(TARGET).o $(KERNEL)

kernel: $(KERNEL) $(KERNEL)/$(TARGET).o

vmeall: kernel

clean:
	rm -f *.o *.a *~ core .depend
	rm -f $(KERNEL)/*.o
else

VME_GLUE_OBJS := universe_II.o

ifneq ($(KERNELRELEASE),)
obj-m	:= $(TARGET).o
$(TARGET)-objs :=  $(VME_GLUE_OBJS)
else
KDIR	:= $(KERNEL_SOURCES)
PWD	:= $(shell pwd)
default:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules
endif

$(KERNEL):
	mkdir $(KERNEL)

vmeall: $(KERNEL)
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) $(KBUILD_OPTIONS) modules
	@if ! [ -f $(TARGET).ko ]; then \
	  echo "$(TARGET).ko failed to build!"; \
	  exit 1; \
	fi
	cp $(TARGET).ko $(KERNEL)

clean:
	rm -f *.o *.a *.ko .*.cmd *.mod.c
	rm -f -r .tmp_versions
	rm -f $(KERNEL)/*.ko
endif
