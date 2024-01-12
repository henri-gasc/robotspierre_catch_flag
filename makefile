CC = arm-linux-gnueabi-gcc
FLAGS = -Wall -Wextra -Wpedantic
OUT = project_os
SOURCES = main.c
LIB = ev3dev-c/lib/libev3dev-c.a
IP = 14.185

.PHONY: default all build clean send

default: all

all: build-docker send

build-docker:
	docker run --rm -it -h ev3 -v ./:/src -w /src ev3cc /usr/bin/make build

build: $(OUT)

send:
	scp $(OUT) robot@192.168.$(IP):/home/robot

$(OUT): $(LIB) $(SOURCES)
	$(CC) $(FLAGS) -o $(OUT) $(SOURCES) -Lev3dev-c/lib -lev3dev-c

$(LIB):
	make -C ev3dev-c/source/ev3
