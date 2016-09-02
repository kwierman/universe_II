###############################################################################
#
# Filename: 	Makefile
# 
# Description:	Makefile for Linux VME device driver
#
# $Revision: 1.8 $
#
# $Date: 2015-04-02 14:22:52 $
#
# $Source: /home/cvs/cvsroot/Linuxvme4/linuxvmeen/Makefile,v $
#
# Copyright 2000-2005 Concurrent Technologies.
#
###############################################################################

# Defaults
POWER_PC = FALSE
MULTI_PROCESS = TRUE

src ?= .
obj ?= .

CCT_MODULES := cct_modules

ifeq ($(POWER_PC),TRUE)
KERNEL_VERSION := 2.4.20
KERNEL_SOURCES := $(shell test -d /opt/VP754/LINUX_20x && echo /opt/VP754/LINUX_20x || echo /usr/src/linux)
KERNEL_OUTPUT := $(KERNEL_SOURCES)
else
KERNEL_VERSION := $(shell uname -r)
KERNEL_MODLIB := /lib/modules/$(KERNEL_VERSION)
KERNEL_SOURCES := $(shell test -d $(KERNEL_MODLIB)/source && echo $(KERNEL_MODLIB)/source || echo $(KERNEL_MODLIB)/build)
KERNEL_OUTPUT := $(KERNEL_SOURCES)
endif


ifeq ($(POWER_PC),TRUE)
KERNEL := kernel-ppc
else
KERNEL := kernel-$(KERNEL_VERSION)
endif

# Select module build process, depending on kernel version
ifeq ($(findstring 2.4.,$(KERNEL_VERSION)),2.4.)
USE_KBUILD := FALSE
LIBVERSION := libvmedriver24
else
USE_KBUILD := TRUE
LIBVERSION := libvmedriver26
endif

GCC_MAJOR_VERSION := $(shell gcc -dumpversion | cut -f1 -d.)
GCC_MINOR_VERSION := $(shell gcc -dumpversion | cut -f2 -d.)
GCC_SUBMINOR_VERSION := $(shell gcc -dumpversion | cut -f3 -d.)

TARGET := vmedriver
ifeq ($(USE_KBUILD),FALSE)
OBJS := $(TARGET).o
SRC := vme_driver.c vme_os.c vme_proc.c vme_krnapi.c
endif

ifeq ($(POWER_PC),TRUE)
CC 	= ppc_74xx-gcc
AR	= ppc_74xx-ar
LD	= ppc_74xx-ld
INCLUDE = $(KERNEL_SOURCES)/include/
else
CC 	= gcc
AR	= ar
LD	= ld
INCLUDE = $(KERNEL_SOURCES)/include/
endif

ifeq ($(USE_KBUILD),FALSE)
CFLAGS = -O2 -Wall -DMODULE -DEXPORT_SYMTAB -D__KERNEL__ -DLINUXOS -I./ -I$(INCLUDE) -fno-common
ifeq ($(shell test $(GCC_MAJOR_VERSION) -gt 4; echo $$?), 0)
	CFLAGS += -Wno-date-time
endif
ifeq ($(shell test $(GCC_MAJOR_VERSION) -eq 4; echo $$?), 0)
ifeq ($(shell test $(GCC_MINOR_VERSION) -ge 9; echo $$?), 0)
	CFLAGS += -Wno-date-time
endif
endif
else
EXTRA_CFLAGS += -DLINUXOS
ifeq ($(shell test $(GCC_MAJOR_VERSION) -gt 4; echo $$?), 0)
	EXTRA_CFLAGS += -Wno-date-time
endif
ifeq ($(shell test $(GCC_MAJOR_VERSION) -eq 4; echo $$?), 0)
ifeq ($(shell test $(GCC_MINOR_VERSION) -ge 9; echo $$?), 0)
	EXTRA_CFLAGS += -Wno-date-time
endif
endif
endif

SYSTEST := sh $(src)/systest.sh "$(CC)" $(KERNEL_SOURCES) $(KERNEL_OUTPUT)

# The remap_page_range check became necessary with the introduction of the five
# argument version and the replacement remap_pfn_range function. 

ifeq ($(shell $(SYSTEST) remap_pfn_range), 1)
  ifeq ($(USE_KBUILD),FALSE)
    CFLAGS += -DREMAP_PFN_RANGE
  else
    EXTRA_CFLAGS += -DREMAP_PFN_RANGE
  endif
else
  REMAP_PAGE_RANGE := $(shell $(SYSTEST) remap_page_range)

  ifeq ($(REMAP_PAGE_RANGE), 4)
    ifeq ($(USE_KBUILD),FALSE)
      CFLAGS += -DREMAP_PAGE_RANGE_4
    else
      EXTRA_CFLAGS += -DREMAP_PAGE_RANGE_4
    endif
  endif
endif

ifeq ($(MULTI_PROCESS),TRUE)
  ifeq ($(USE_KBUILD),FALSE)
    CFLAGS += -DMULTI_PROCESS
  else
    EXTRA_CFLAGS += -DMULTI_PROCESS
  endif
endif

all: prepare vmeall

prepare:
ifndef LIBVERSION
	@echo This kernel version is not supported!; exit 1;
endif


ifeq ($(USE_KBUILD),FALSE)

depend .depend dep:
	$(CC) $(CFLAGS) -M *.c > $@

$(TARGET).o: $(SRC:.c=.o) $(LIBVERSION).a
	$(LD) -r $^ -o $@

$(KERNEL):
	mkdir $(KERNEL)

$(KERNEL)/$(TARGET).o: $(TARGET).o
	cp $(TARGET).o $(KERNEL)

$(LIBVERSION).a:
	cp $(src)/$(CCT_MODULES)/$(LIBVERSION).a $(LIBVERSION).a

kernel: $(KERNEL) $(KERNEL)/$(TARGET).o

vmeall: .depend kernel

clean:
	rm -f *.o *.a *~ core .depend
	rm -f $(KERNEL)/*.o
else

VME_CORE_OBJS := $(LIBVERSION).a 
VME_GLUE_OBJS := vme_driver.o vme_os.o vme_proc.o vme_krnapi.o

$(obj)/$(LIBVERSION).a:
	cp $(src)/$(CCT_MODULES)/$(LIBVERSION).a $(obj)/$(LIBVERSION).a

ifneq ($(KERNELRELEASE),)
obj-m	:= $(TARGET).o
$(TARGET)-objs :=  $(VME_GLUE_OBJS) $(VME_CORE_OBJS)
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



