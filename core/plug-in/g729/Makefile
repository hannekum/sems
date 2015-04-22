plug_in_name = g729
COREPATH =../..

G729_INC=
#/usr/include/
G729_LIB = -lbcg729

module_ldflags = $(G729_LIB)
module_cflags  = -ansi # -DNOFPU

ifdef NOFPU
	module_cflags += -DNOFPU
endif

include ../Makefile.audio_module
