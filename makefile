CC = arm-linux-gnueabi-gcc
FLAGS = -Wall -Wextra -Wpedantic
OUT = project_os
SOURCES = main.c
LIB = ev3dev-c/lib/libev3dev-c.a
IP = 235.185


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

angle: examples/angle.c
	docker run --rm -it -h ev3 -v ./:/src -w /src ev3cc $(CC) $(FLAGS) examples/angle.c -o bin/angle -Lev3dev-c/lib -lev3dev-c
	scp bin/angle robot@192.168.$(IP):/home/robot

motor_button: examples/motor_button.c
	docker run --rm -it -h ev3 -v ./:/src -w /src ev3cc $(CC) $(FLAGS) examples/motor_button.c -o bin/motor_button -Lev3dev-c/lib -lev3dev-c
	scp bin/motor_button robot@192.168.$(IP):/home/robot

move_and_catch: examples/move_and_catch.c
	docker run --rm -it -h ev3 -v ./:/src -w /src ev3cc $(CC) $(FLAGS) examples/move_and_catch.c -o bin/move_and_catch -Lev3dev-c/lib -lev3dev-c
	scp bin/move_and_catch robot@192.168.$(IP):/home/robot

move_straight: examples/move_straight.c
	docker run --rm -it -h ev3 -v ./:/src -w /src ev3cc $(CC) $(FLAGS) examples/move_straight.c -o bin/move_straight -Lev3dev-c/lib -lev3dev-c
	scp bin/move_straight robot@192.168.$(IP):/home/robot

set_gyro_to_zero: examples/set_gyro_to_zero.c
	docker run --rm -it -h ev3 -v ./:/src -w /src ev3cc $(CC) $(FLAGS) examples/set_gyro_to_zero.c -o bin/set_gyro_to_zero -Lev3dev-c/lib -lev3dev-c
	scp bin/set_gyro_to_zero robot@192.168.$(IP):/home/robot

sonar: examples/sonar.c
	docker run --rm -it -h ev3 -v ./:/src -w /src ev3cc $(CC) $(FLAGS) examples/sonar.c -o bin/sonar -Lev3dev-c/lib -lev3dev-c
	scp bin/sonar robot@192.168.$(IP):/home/robot

print_color: examples/print_color.c
	docker run --rm -it -h ev3 -v ./:/src -w /src ev3cc $(CC) $(FLAGS) examples/print_color.c -o bin/print_color -Lev3dev-c/lib -lev3dev-c
	scp bin/print_color robot@192.168.$(IP):/home/robot

test_gyro: examples/test_gyro.c
	docker run --rm -it -h ev3 -v ./:/src -w /src ev3cc $(CC) $(FLAGS) examples/test_gyro.c -o bin/test_gyro -Lev3dev-c/lib -lev3dev-c
	scp bin/test_gyro robot@192.168.$(IP):/home/robot

catch_flag: examples/catch_flag.c
	docker run --rm -it -h ev3 -v ./:/src -w /src ev3cc $(CC) $(FLAGS) examples/catch_flag.c -o bin/catch_flag -Lev3dev-c/lib -lev3dev-c
	scp bin/catch_flag robot@192.168.$(IP):/home/robot
