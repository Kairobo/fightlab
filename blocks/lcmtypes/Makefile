# Construct the pathnames to the C types
LCMTYPES_C := $(shell ls *.c)
LCMTYPES_O = $(LCMTYPES_C:%.c=%.o)
CC = gcc
CFLAGS = -g `pkg-config --cflags lcm`

ALL = $(LCMTYPES_O)

all: $(ALL)

%.o: %.c
	$(CC) $(CFLAGS) -c $^ -o $@

clean:
	rm -f *.c *.h *.o

