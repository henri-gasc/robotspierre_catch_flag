#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "include/ev3.h"
#include "include/ev3_sensor.h"
#include "include/ev3_tacho.h"

#define Sleep(msec) usleep((msec) * 1000)
#define PORT_A 65
#define PORT_B 66
#define PORT_C 67
#define PORT_D 68

#define DEFAULT_TIME 50

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

int is_motor_here(int port, uint8_t* sn, char* text_yes, char* text_no) {
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

bool found_red(uint8_t sn) {
    float red;
    float green;
    float blue;
    if (!get_sensor_value0(sn, &red)) {
        red = 1020;
    }
    if (!get_sensor_value1(sn, &green)) {
        green = 1020;
    }
    if (!get_sensor_value2(sn, &blue)) {
        blue = 1020;
    }
    if ((red > 10) && (red > green) && (red > blue)) {
        return true;
    } else {
        return false;
    }
}

void motor_state_time(uint8_t sn, int speed, int time) {
    set_tacho_stop_action_inx(sn, TACHO_COAST);
    set_tacho_speed_sp(sn, speed);
    set_tacho_time_sp(sn, time);
    set_tacho_command_inx(sn, TACHO_RUN_TIMED);
}

void stop_motor(uint8_t sn) {
    set_tacho_command_inx(sn, TACHO_STOP);
}

void motor_state(uint8_t sn, int speed) {
    motor_state_time(sn, speed, DEFAULT_TIME);
}

void move_forward(uint8_t left, uint8_t right, int speed_left, int speed_right, int time) {
    motor_state_time(left, speed_left, time);
    motor_state_time(right, speed_right, time);
}

void turn_left(uint8_t left, uint8_t right, int speed, int time) {
    motor_state_time(left, - speed, time);
    motor_state_time(right, speed, time);
}

void turn_right(uint8_t left, uint8_t right, int speed, int time) {
    turn_left(right, left, speed, time);
}

void catch_flag(uint8_t sn_wheel_left, uint8_t sn_wheel_right, uint8_t sn_clamp, int speed) {
    move_forward(sn_wheel_left, sn_wheel_right, speed, speed, 500);
    motor_state_time(sn_clamp, -speed, 1000);
    Sleep(1000);
    move_forward(sn_wheel_left, sn_wheel_right, 0, 0, DEFAULT_TIME);
    motor_state_time(sn_clamp, speed, 1000);
    Sleep(1000);
    motor_state(sn_clamp, 0);
}

int main(void) {
    uint8_t sn_wheel_left;
    uint8_t sn_wheel_right;
    uint8_t sn_clamp;
    uint8_t sn_color;
    uint8_t sn_sonar;
    uint8_t sn_gyro;
    float default_gyro;
    float val_sonar;
    float val_gyro;
    int val_color;
    int count = 0;

    printf("Test move and catch\n");

    if (ev3_init() == -1)
        return 1;
    printf("Waiting tacho is plugged...\n");

    while ((ev3_tacho_init() < 1) & (ev3_sensor_init() < 1)) {
        Sleep(1000);
    }

    int port_wheel_left = PORT_A;
    int port_wheel_right = PORT_B;
    int port_clamp = PORT_C;
    bool quit = false;

    if (ev3_search_sensor(LEGO_EV3_COLOR, &sn_color, 0)) {
        printf("Found the color sensor\n");
        set_sensor_mode_inx(sn_color, LEGO_EV3_COLOR_RGB_RAW);
    } else {
        printf("Could not find color sensor. Cannot continue\n");
        return 2;
    }
    if (ev3_search_sensor(LEGO_EV3_US, &sn_sonar,0)) {
        printf("Found the sonar\n");
    } else {
        printf("Could not find the sonar\n");
        return 2;
    }
    if (ev3_search_sensor(LEGO_EV3_GYRO, &sn_gyro,0)) {
        printf("Found the gyroscope\n");
    } else {
        printf("Could not find the gyroscope\n");
        return 2;
    }
    if (!is_motor_here(port_wheel_left, &sn_wheel_left, "Found the left wheel", "Could not find the left wheel")) {
        return 3;
    }
    if (!is_motor_here(port_wheel_right, &sn_wheel_right, "Found the right wheel", "Could not find the right wheel")) {
        return 4;
    }
    if (!is_motor_here(port_clamp, &sn_clamp, "Found the clamp", "Could not find the clamp")) {
        return 5;
    }
    int max_speed = get_min_maxspeed(sn_wheel_left, sn_wheel_right, sn_clamp);
    if (max_speed < 0) {
        return max_speed;
    }

    get_sensor_value0(sn_gyro, &default_gyro);

    bool move = true;
    bool turn = false;
    bool turned = false;
    int speed_move_default = max_speed / 4;
    int speed_right = speed_move_default;
    int speed_left = speed_move_default;
    int diff = 0;
    int count_turned = 1;
    float mul = 1;
    int action = 0;

    while (!quit) {
        if ((action == 0) | (action == 2)) {
            get_sensor_value0(sn_gyro, &val_gyro);
            diff = (int) (default_gyro - val_gyro) % 360;
            mul = 1 + (float) abs(diff) / 3;
            // printf("\r%3d", diff);
            // fflush(stdout);
            if (!turned && (diff != 0)) {
                if (diff > 0) {
                    speed_left  = mul * speed_move_default;
                    speed_right = 1.0 * speed_move_default;
                } else if (diff < 0) {
                    speed_left  = 1.0 * speed_move_default;
                    speed_right = mul * speed_move_default;
                }
            } else {
                speed_right = speed_move_default;
                speed_left  = speed_move_default;
            }
            // if (turn) {
            //     turn_right(sn_wheel_left, sn_wheel_right, 2 * speed_move_default, DEFAULT_TIME);
            // } else {
                move_forward(sn_wheel_left, sn_wheel_right, speed_left, speed_right, DEFAULT_TIME);
            // }
        }
        if (action == 2) {
            if (!get_sensor_value(0, sn_color, &val_color) || (val_color < 0) || (val_color >= COLOR_COUNT)) {
                val_color = 0;
                count = 0;
            }
            if (val_color != 0) {
                count++;
            }
            if (count >= 2) { // Red
                catch_flag(sn_wheel_left, sn_wheel_right, sn_clamp, speed_move_default);
                // turn = true;
                quit = true;
            }
        }
        if (!get_sensor_value0(sn_sonar, &val_sonar)) {
            val_sonar = -1;
        }
        if (val_sonar > 0) {
            // if (val_sonar < 100) {
            //     move_forward(sn_wheel_left, sn_wheel_right, - speed_left, speed_right, DEFAULT_TIME);
            //     Sleep(DEFAULT_TIME);
            // printf("\r%d, %f, ", move, val_sonar);
            printf("\n%d", action);
            // fflush(stdout);
            if (action == 0) {
                if (val_sonar < 150) {
                    action = 1;
                    while (val_gyro >= (default_gyro - 45)) {
                        printf("\r%f, %f", default_gyro, val_gyro);
                        fflush(stdout);
                        turn_left(sn_wheel_left, sn_wheel_right, 2 * speed_move_default, DEFAULT_TIME);
                        get_sensor_value0(sn_gyro, &val_gyro);
                    }
                }
            } else if (action == 1) {
                if (val_sonar < 100) {
                    action = 2;
                    while (val_gyro >= (default_gyro - 80)) {
                        printf("\r%f, %f", default_gyro, val_gyro);
                        fflush(stdout);
                        turn_left(sn_wheel_left, sn_wheel_right, 2 * speed_move_default, DEFAULT_TIME);
                        get_sensor_value0(sn_gyro, &val_gyro);
                    }
                    get_sensor_value0(sn_gyro, &default_gyro);
                } else {
                    move_forward(sn_wheel_left, sn_wheel_right, speed_left, speed_right, DEFAULT_TIME);
                }
            }
            // if (val_sonar < 200) {
            //     get_sensor_value0(sn_gyro, &val_gyro);
            //     // while (val_gyro != default_gyro - 90) {
            //     //     printf("\r%f, %f", default_gyro, val_gyro);
            //     //     fflush(stdout);
            //     //     turn_left(sn_wheel_left, sn_wheel_right, 2 * speed_move_default, DEFAULT_TIME);
            //     //     get_sensor_value0(sn_gyro, &val_gyro);
            //     // }
            //     // move_forward(sn_wheel_left, sn_wheel_right, 0, 0, DEFAULT_TIME);
            //     move = false;
            //     turned = true;
            //     count_turned++;
            // } else {
            //     move = true;
            // }
        }
        // if (val_sonar > 500) {
        //     move = true;
        // }
    }

    stop_motor(sn_wheel_left);
    stop_motor(sn_wheel_right);
    ev3_uninit();
    return 0;
}
