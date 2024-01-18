#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "../include/ev3.h"
#include "../include/ev3_sensor.h"
#include "../include/ev3_tacho.h"

#define Sleep(msec) usleep((msec) * 1000)
#define PORT_A 65
#define PORT_B 66
#define PORT_C 67
#define PORT_D 68

const char *port_name[] = {"A", "B", "C", "D"};
const char *color[] = {"?",      "BLACK", "BLUE",  "GREEN",
                       "YELLOW", "RED",   "WHITE", "BROWN"};

#define COLOR_COUNT ((int)(sizeof(color) / sizeof(color[0])))

int MIN(int a, int b) {
    if (a <= b) {
        return a;
    } else {
        return b;
    }
}

int is_motor_here(int port, uint8_t *sn, char *text_yes, char *text_no) {
    int status = 1;
    if (!ev3_search_tacho_plugged_in(port, 0, sn, 0)) {
        printf("%s\n", text_no);
        status = 0;
    } else {
        printf("%s\n", text_yes);
    }
    return status;
}

int get_min_maxspeed(uint8_t sn_1, uint8_t sn_2, uint8_t sn_3) {
    int max_speed;
    int temp;
    if (get_tacho_max_speed(sn_1, &max_speed) == 0) {
        printf("Could not read the maximum speed for first arg\n");
        max_speed = -1;
    }
    if (get_tacho_max_speed(sn_2, &temp) == 0) {
        printf("Could not read the maximum speed for second arg\n");
        max_speed = -2;
    }
    max_speed = MIN(max_speed, temp);
    if (get_tacho_max_speed(sn_3, &temp) == 0) {
        printf("Could not read the maximum speed for third arg\n");
        max_speed = -3;
    }
    return MIN(max_speed, temp);
}

void motor_state_time(uint8_t sn, int speed, int time) {
    set_tacho_stop_action_inx(sn, TACHO_COAST);
    set_tacho_speed_sp(sn, speed);
    set_tacho_time_sp(sn, time);
    set_tacho_command_inx(sn, TACHO_RUN_TIMED);
}

void motor_state(uint8_t sn, int speed) { motor_state_time(sn, speed, 100); }

void move_forward(uint8_t left, uint8_t right, int speed, int time) {
    motor_state_time(left, speed, time);
    motor_state_time(right, speed, time);
}

void catch_flag(uint8_t sn_wheel_left, uint8_t sn_wheel_right, uint8_t sn_clamp,
                int speed) {
    move_forward(sn_wheel_left, sn_wheel_right, speed, 500);
    motor_state_time(sn_clamp, -speed, 1000);
    Sleep(1000);
    move_forward(sn_wheel_left, sn_wheel_right, 0, 100);
    motor_state_time(sn_clamp, speed, 1000);
    Sleep(1000);
    motor_state(sn_clamp, 0);
}

int main(void) {
    uint8_t sn_wheel_left;
    uint8_t sn_wheel_right;
    uint8_t sn_clamp;
    uint8_t sn_color;
    int val_color;
    char s[256];

    printf("Test move and catch\n");

    if (ev3_init() == -1)
        return 1;
    printf("Waiting tacho is plugged...\n");

    while ((ev3_tacho_init() < 1) & (ev3_sensor_init() < 1)) {
        Sleep(1000);
    }

    printf("Found tacho motors:\n");
    for (int i = 0; i < DESC_LIMIT; i++) {
        if (ev3_tacho[i].type_inx != TACHO_TYPE__NONE_) {
            printf("  type = %s\n", ev3_tacho_type(ev3_tacho[i].type_inx));
            printf("  port = %s\n", ev3_tacho_port_name(i, s));
            printf("  port = %d %d\n", ev3_tacho_desc_port(i),
                   ev3_tacho_desc_extport(i));
        }
    }

    int port_wheel_left = PORT_A;
    int port_wheel_right = PORT_B;
    int port_clamp = PORT_C;
    int count = 0;
    bool quit = false;

    if (ev3_search_sensor(LEGO_EV3_COLOR, &sn_color, 0)) {
        printf("Found the touch sensor\n");
    } else {
        printf("Could not find touch sensor. Cannot continue\n");
        return 2;
    }
    if (!is_motor_here(port_wheel_left, &sn_wheel_left, "Found the left wheel",
                       "Could not find the left wheel")) {
        return 3;
    }
    if (!is_motor_here(port_wheel_right, &sn_wheel_right,
                       "Found the right wheel",
                       "Could not find the right wheel")) {
        return 4;
    }
    if (!is_motor_here(port_clamp, &sn_clamp, "Found the clamp",
                       "Could not find the clamp")) {
        return 5;
    }
    int max_speed = get_min_maxspeed(sn_wheel_left, sn_wheel_right, sn_clamp);
    if (max_speed < 0) {
        return max_speed;
    }

    bool move = true;
    int speed_move = max_speed / 4;

    while (!quit) {
        if (move) {
            move_forward(sn_wheel_left, sn_wheel_right, speed_move, 100);
        }
        if (!get_sensor_value(0, sn_color, &val_color) || (val_color < 0) ||
            (val_color >= COLOR_COUNT)) {
            val_color = 0;
            count = 0;
        }
        if (val_color != 0) {
            count++;
        }
        if (count >= 2) { // Change color
            catch_flag(sn_wheel_left, sn_wheel_right, sn_clamp,
                       speed_move);
            quit = true;
        }
    }

    ev3_uninit();
    return 0;
}
