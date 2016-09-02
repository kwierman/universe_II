###############################################################################
#
# Filename: 	Makefile
#
# Description:	Makefile for Example Program.
#
# $Revision: 1.8 $
#
# $Date: 2014-12-17 09:14:53 $
#
# $Source: /home/cvs/cvsroot/Linuxvme4/linuxvmeen_examples/Makefile,v $
#
# Copyright 2000 - 2005 Concurrent Technologies.
#
###############################################################################

# Defaults

OS64BIT = TRUE
COMPATLIB = FALSE
# To enable compatibility library for 32bit applications on 64bit OS
# set OS64BIT to FALSE and COMPATLIB to TRUE
# For 64 bit native application OS64BIT to TRUE and COMPATLIB to FALSE

CC 	= g++
LIBPATHSHARED = ../api/
LIBPATH = ../api/
ifeq ($(COMPATLIB),FALSE)
LIBEN	= -luniverse
else
LIBEN	= -luniverse
endif

INCLUDE = ../api/ -I../driver/
RPATH:=$(shell cd $(LIBPATHSHARED); pwd)

ifeq ($(OS64BIT),FALSE)
CFLAGS  = -Wall -m32 -I $(INCLUDE)
else
CFLAGS  = -Wall -m64 -DOS64BIT -I $(INCLUDE)
endif

all: testDevice

testDevice: testDevice.cc
	$(CC) -Wl,-rpath=$(RPATH) $(CFLAGS) $^ -o $@ -L $(RPATH) $(LIBEN)


# example10: example10.c
#	$(CC) $(CFLAGS) $^ -o $@ -L $(LIBPATH) $(LIBEN)

clean:
	rm -f testDevice
