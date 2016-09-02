###############################################################################
#
# Filename: 	Makefile
# 
# Description:	Makefile for Example Program.
#
# Copyright 2000 - 2009 Concurrent Technologies.
#
###############################################################################

# Defaults

CC 	= gcc
INCLUDE = ../example_driver/
CFLAGS  = -Wall -I$(INCLUDE)

all: example_api example_appc

example_appc: example_appc.c
	$(CC) $(CFLAGS) $^ -o $@

example_api: example_api.c
	$(CC) $(CFLAGS) $^ -o $@ 

clean:
	rm -f *.o example_api example_appc
