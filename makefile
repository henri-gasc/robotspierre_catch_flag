CC = arm-linux-gnueabi-gcc
FLAGS = -Wall -Wextra -Wpedantic
OUT = project_os
SOURCES = test.c
LIB = ev3dev-c/lib/libev3dev-c.a

.PHONY: default all build clean send

default: all

all: build send

build: $(OUT)

send: build
	scp $(OUT) robot@ev3dev.local

$(OUT): $(LIB) $(SOURCES)
	$(CC) $(FLAGS) -o $(OUT) $(SOURCES) -Lev3dev-c/lib -lev3dev-c

$(LIB):
	make -C ev3dev-c/source/ev3
