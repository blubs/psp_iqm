PSPSDK=$(shell psp-config --pspsdk-path)
TARGET = main
OBJS = main.o $(PSPSDK)/samples/gu/common/callbacks.o $(PSPSDK)/samples/gu/common/vram.o

INCDIR = $(PSPSDK)/samples/gu/common/
CFLAGS = -Wall
CXXFLAGS = $(CFLAGS) -fno-exceptions -fno-rtti
ASFLAGS = $(CFLAGS)

LIBDIR = 
LDFLAGS = 
LIBS = -lpspgum -lpspgu

EXTRA_TARGETS = EBOOT.PBP
PSP_EBOOT_TITLE = IQM Test Project

include $(PSPSDK)/lib/build.mak
# include $(PSPSDK)/samples/gu/common/vram.h
# include $(PSPSDK)/samples/gu/common/callbacks.h