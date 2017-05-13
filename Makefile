# Change TOPDIR to directory that contains STM32Library submodule
TOPDIR=.

# Change OUTPUT to whatever final bin, hex, and elf files should be called
OUTPUT=main

MODULEDIR=$(TOPDIR)/STM32Library
include $(MODULEDIR)/Makefile_defines
include $(MODULEDIR)/Makefile_std_rules