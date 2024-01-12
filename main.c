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
#define DISTANCE_STOP 50

const int port_wheel_left = PORT_A;
const int port_wheel_right = PORT_B;
const int port_clamp = PORT_C;

// int max_speed = 1000;
float val_sonar = -1;
float previous_sonar = -1;
uint8_t sn_sonar;
uint8_t sn_wheel_left;
uint8_t sn_wheel_right;
uint8_t sn_clamp;
uint8_t sn_color;
uint8_t sn_gyro;

float update_sonar(void) {
    get_sensor_value0(sn_sonar, &val_sonar);
    if (previous_sonar == -1) {
        previous_sonar = val_sonar;
    }
    // if (previous_sonar > val_sonar + 100) {
    //     return previous_sonar;
    // }
    float new_val = (val_sonar + previous_sonar) / 2;
    previous_sonar = val_sonar;
    return new_val;
}

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

void move_straight(int speed_default, int time, float default_gyro) {
    float val_gyro;
    float speed_left;
    float speed_right;
    get_sensor_value0(sn_gyro, &val_gyro);
    int diff = (int) (default_gyro - val_gyro) % 360;
    float mul = 1 + (float) abs(diff) / 10;
    if (diff != 0) {
        if (diff > 0) {
            speed_left  = mul * speed_default;
            speed_right = 1.0 * speed_default;
        } else if (diff < 0) {
            speed_left  = 1.0 * speed_default;
            speed_right = mul * speed_default;
        }
    } else {
        speed_right = speed_default;
        speed_left  = speed_default;
    }
    motor_state_time(sn_wheel_left, speed_left, time);
    motor_state_time(sn_wheel_right, speed_right, time);
}

void move_forward(int speed_left, int speed_right, int time) {
    motor_state_time(sn_wheel_left, speed_left, time);
    motor_state_time(sn_wheel_right, speed_right, time);
}

void turn_left(int speed, int time) {
    motor_state_time(sn_wheel_left, 0, time);
    motor_state_time(sn_wheel_right, speed, time);
}

void turn_right(int speed, int time) {
    motor_state_time(sn_wheel_left, speed, time);
    motor_state_time(sn_wheel_right, 0, time);
}

/**
 * @brief Open the clamp
 * 
 * @param sn_clamp the uint8_t of the clamp
 * @param speed the speed > 0 to open the clamp
 */
void open_clamp(float speed, int time) {
    motor_state_time(sn_clamp, - speed, time);
}

void close_clamp(float speed, int time) {
    open_clamp(- speed, time);
}

void catch_flag(int speed) {
    move_forward(speed, speed, 500);
    open_clamp(speed, 1000);
    Sleep(1000);
    move_forward(0, 0, DEFAULT_TIME);
    close_clamp(speed, 1000);
    Sleep(1000);
}

int init_robot() {
    if (ev3_init() == -1)
        return 1;
    printf("Waiting tacho is plugged...\n");
    while ((ev3_tacho_init() < 1) & (ev3_sensor_init() < 1)) {
        Sleep(1000);
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
    return 0;
}

int main(void) {
    float default_gyro;
    float val_gyro;
    int val_color;
    int count = 0;
    bool quit = false;
    int status;

    if ((status = init_robot())) {
        return status;
    }

    int max_speed = get_min_maxspeed(sn_wheel_left, sn_wheel_right, sn_clamp);
    if (max_speed < 0) {
        return max_speed;
    }

    get_sensor_value0(sn_gyro, &default_gyro);
    const float default_gyro_start = default_gyro;

    bool move = true;
    bool turn = false;
    bool turned = false;
    int speed_move_default = max_speed / 4;
    int speed_right = speed_move_default;
    int speed_left = speed_move_default;
    int action = 0;
    float sonar = 0;
    bool can_catch = false;
    bool allow_quit = true;
    get_sensor_value0(sn_gyro, &val_gyro);

    while (!quit) {
        if ((action == 1) || (action == 2) | (action == 4) | (action == 5)) {
            // if (turn) {
            //     turn_right(sn_wheel_left, sn_wheel_right, 2 * speed_move_default, DEFAULT_TIME);
            // } else {
                move_forward(speed_left, speed_right, DEFAULT_TIME);
            // }
        }
        if ((action == 0) || (action == 6)) {
            get_sensor_value0(sn_sonar, &val_sonar);
            sonar = val_sonar;
        } else {
            sonar = update_sonar();
        }
        if (!sonar) {
            continue;
        }
        printf("\r%3.0f", sonar);
        fflush(stdout);

        // Phase 1
        if (action == 0) {
            while (val_gyro <= (default_gyro_start + 45)) {
                turn_right(2 * speed_move_default, DEFAULT_TIME);
                get_sensor_value0(sn_gyro, &val_gyro);
            }
            // get_sensor_value0(sn_gyro, &default_gyro);
            action = 1;
            printf("Phase 1");
        }
        if (sonar > 0) {
            if (sonar >= DISTANCE_STOP) {
                allow_quit = false;
            }
            if ((sonar < DISTANCE_STOP) && allow_quit) {
                quit = true;
            }

            // Phase 3
            else if (action == 1) {
                if (sonar < 230) {
                    while (val_gyro >= default_gyro_start) {
                        turn_left(2 * speed_move_default, DEFAULT_TIME);
                        get_sensor_value0(sn_gyro, &val_gyro);
                    }
                    action = 3;
                    printf("Phase 3\n");
                } else {
                    move_straight(speed_move_default, speed_move_default, default_gyro_start);
                }
            } else if (action == 3) {
                if (sonar < 250) {
                    get_sensor_value0(sn_gyro, &val_gyro);
                    while (val_gyro >= (default_gyro_start - 50)) {
                        // printf("\r%f, %f", default_gyro, val_gyro);
                        // fflush(stdout);
                        turn_left(2 * speed_move_default, DEFAULT_TIME);
                        get_sensor_value0(sn_gyro, &val_gyro);
                    }
                    action = 5;
                    printf("Phase 5\n");
                    get_sensor_value0(sn_gyro, &default_gyro);
                    Sleep(500);
                } else {
                    move_straight(speed_move_default, DEFAULT_TIME, default_gyro_start);
                }
            } else if (action == 5) {
                if (sonar < 200) {
                    while (val_gyro >= (default_gyro - 80)) {
                        // printf("\r%f, %f", default_gyro, val_gyro);
                        // fflush(stdout);
                        turn_left(2 * speed_move_default, DEFAULT_TIME);
                        get_sensor_value0(sn_gyro, &val_gyro);
                    }
                    action = 6;
                    printf("Phase 6\n");
                } else if ((sonar < 550) && (sonar > 450)) {
                    if (can_catch) {
                        catch_flag(speed_move_default);
                        get_sensor_value0(sn_gyro, &default_gyro);
                    }
                } else if (sonar <= 450) {
                    close_clamp(speed_move_default, 1000);
                    set_tacho_command_inx(sn_clamp, TACHO_RUN_FOREVER);
                } else {
                    if (sonar > 600) {
                        can_catch = true;
                        open_clamp(speed_move_default, 1000);
                    }
                    move_forward(speed_left, speed_right, DEFAULT_TIME);
                }
            } else if (action == 6) {
                move_straight(speed_move_default * 2, DEFAULT_TIME, default_gyro_start - 180);
                if (sonar < DISTANCE_STOP) {
                    move_forward(0, 0, DEFAULT_TIME);
                    Sleep(5000); // Wait 5 five seconds 
                    allow_quit = true;
                }
            }
        }
    }

    stop_motor(sn_wheel_left);
    stop_motor(sn_wheel_right);
    stop_motor(sn_clamp);
    ev3_uninit();
    return 0;
}
