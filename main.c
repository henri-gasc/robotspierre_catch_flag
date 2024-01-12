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
float previous_sonar = -1;
float val_sonar = -1;
uint8_t sn_sonar;
uint8_t sn_wheel_left;
uint8_t sn_wheel_right;
uint8_t sn_clamp;
uint8_t sn_color;
uint8_t sn_gyro;

/**
 * @brief Return the value of the sonar after some filtering
 * 
 * @return float the value of the sonar
 */
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

/**
 * @brief Update IN PLACE gyro_now and return the value
 * 
 * @return float gyro_now
 */
int update_gyro() {
    int val;
    get_sensor_value(0, sn_gyro, &val);
    // gyro_now = (int) gyro_now % 360;
    return val;
}

/**
 * @brief Return the minimum between a and b
 * 
 * @param a the first value
 * @param b the second value
 * @return int the minimum
 */
int MIN(int a, int b) {
    if (a <= b) {
        return a;
    } else {
        return b;
    }
}

/**
 * @brief Test for the presence of a motor on the specified port
 * 
 * @param port the port the motor should connected to
 * @param sn the address of the sn
 * @param text_yes the text to print when the motor is found
 * @param text_no the text to print when the motor is not found
 * @return bool if the motor was found or not
 */
bool is_motor_here(int port, uint8_t* sn, char* text_yes, char* text_no) {
    bool status = true;
    if (!ev3_search_tacho_plugged_in(port, 0, sn, 0)) {
        printf("%s\n", text_no);
        status = false;
    } else {
        printf("%s\n", text_yes);
    }
    return status;
}

/**
 * @brief Get the minimum of the maximum speed the motors can run at
 * 
 * @param sn_1 the first motor
 * @param sn_2 the second motor
 * @param sn_3 the thrid motor
 * @return int the speed (< 0 if error)
 */
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

/**
 * @brief set the motor to the speed for the set time
 * 
 * @param sn the motor
 * @param speed the speed
 * @param time the time
 */
void motor_state_time(uint8_t sn, int speed, int time) {
    set_tacho_stop_action_inx(sn, TACHO_COAST);
    set_tacho_speed_sp(sn, speed);
    set_tacho_time_sp(sn, time);
    set_tacho_command_inx(sn, TACHO_RUN_TIMED);
}

/**
 * @brief Tell the motor to stop
 * 
 * @param sn the motor
 */
void stop_motor(uint8_t sn) {
    set_tacho_command_inx(sn, TACHO_STOP);
}

/**
 * @brief Set the motor to run with the default time
 * 
 * @param sn the motor
 * @param speed the time
 */
void motor_state(uint8_t sn, int speed) {
    motor_state_time(sn, speed, DEFAULT_TIME);
}

/**
 * @brief Set the two wheel to run at the specified speeds
 * 
 * @param speed_left the speed of the left wheel
 * @param speed_right the speed of the right wheel
 * @param time the time the motor should turn for
 */
void move_forward(int speed_left, int speed_right, int time) {
    motor_state_time(sn_wheel_left, speed_left, time);
    motor_state_time(sn_wheel_right, speed_right, time);
}

/**
 * @brief Move the robot in a straigh line accoring to gyro_ref
 * 
 * @param speed_default the baseline speed
 * @param time the time the motor should turn
 * @param gyro_ref the value of reference for the gyroscope
 */
// void move_straight(int speed_default, int time, float gyro_ref) {
//     float speed_left;
//     float speed_right;
//     // int val = update_gyro();
//     float val;
//     get_sensor_value0(sn_gyro, &val);
//     // get_sensor_value0(sn_gyro, &val);
//     // gyro_now = update_gyro();
//     int diff = (int) (gyro_ref - val) % 360;
//     printf("%d: %3.0f - %3.0f = %d\n", sn_gyro, gyro_ref, val, diff);
//     if (diff != 0) {
//         float mul = 1 + (float) abs(diff) / 10;
//         printf("%f\n", mul);
//         if (diff > 0) {
//             speed_left  = mul * speed_default;
//             speed_right = 1.0 * speed_default;
//         } else if (diff < 0) {
//             speed_left  = 1.0 * speed_default;
//             speed_right = mul * speed_default;
//         }
//     } else {
//         speed_right = speed_default;
//         speed_left  = speed_default;
//     }
//     move_forward(speed_left, speed_right, time);
// }
void move_straight(int speed_default, int time, float default_gyro) {
    float val_gyro;
    float speed_left;
    float speed_right;
    get_sensor_value0(sn_gyro, &val_gyro);
    int diff = (int) (default_gyro - val_gyro) % 360;
    if (diff != 0) {
        float mul = 1 + (float) abs(diff) / 5;
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
    move_forward(speed_left, speed_right, time);
}

/**
 * @brief Turn to the left
 * 
 * @param speed the speed
 * @param time the time
 */
void turn_left(int speed, int time) {
    motor_state_time(sn_wheel_left, 0, time);
    motor_state_time(sn_wheel_right, speed, time);
}

/**
 * @brief Turn to the right
 * 
 * @param speed the speed
 * @param time the time
 */
void turn_right(int speed, int time) {
    motor_state_time(sn_wheel_left, speed, time);
    motor_state_time(sn_wheel_right, 0, time);
}

/**
 * @brief Open the clamp
 * 
 * @param speed the speed > 0 to open the clamp
 * @param time the time
 */
void open_clamp(float speed, int time) {
    motor_state_time(sn_clamp, - speed, time);
}

/**
 * @brief Close the clamp
 * 
 * @param speed the speed
 * @param time the time
 */
void close_clamp(float speed, int time) {
    open_clamp(- speed, time);
}

/**
 * @brief Function that ease catching the flag
 * 
 * @param speed the speed
 */
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
    bool quit = false;
    int status;

    if ((status = init_robot())) {
        return status;
    }

    int max_speed = get_min_maxspeed(sn_wheel_left, sn_wheel_right, sn_clamp);
    if (max_speed < 0) {
        return max_speed;
    }

    float gyro_now;
    get_sensor_value0(sn_gyro, &gyro_now);
    const float gyro_val_start = gyro_now;
    const float first_angle = gyro_val_start + 45;
    const float second_angle = gyro_val_start;
    const float third_angle = gyro_val_start - 45;
    const float fourth_angle = gyro_val_start - 80;
    const float fifth_angle = gyro_val_start - 180;

    int speed_move_default = max_speed / 4;
    int speed_right = speed_move_default;
    int speed_left = speed_move_default;
    int action = 0;
    float sonar = 0;
    bool can_catch = false;
    bool allow_quit = false;

    while (!quit) {
        if ((action == 1) || (action == 5)) {
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
        // printf("\r%3.0f", sonar);
        // fflush(stdout);

        // Phase 1
        if (action == 0) {
            while (gyro_now <= (gyro_val_start + 45)) {
                turn_right(2 * speed_move_default, DEFAULT_TIME);
                get_sensor_value0(sn_gyro, &gyro_now);
            }
            // get_sensor_value0(sn_gyro, &default_gyro);
            action = 1;
            printf("Phase 1");
        }
        if (sonar > 0) {
            if (sonar >= DISTANCE_STOP) {
                allow_quit = false;
            }
            if ((sonar <= DISTANCE_STOP) && allow_quit) {
                quit = true;
            }

            // Phase 3
            else if (action == 1) {
                if (sonar < 230) {
                    while (gyro_now >= gyro_val_start) {
                        turn_left(2 * speed_move_default, DEFAULT_TIME);
                        get_sensor_value0(sn_gyro, &gyro_now);
                    }
                    action = 3;
                    printf("Phase 3\n");
                } else {
                    move_straight(speed_move_default, speed_move_default, gyro_val_start);
                }
            } else if (action == 3) {
                if (sonar < 250) {
                    get_sensor_value0(sn_gyro, &gyro_now);
                    while (gyro_now >= (gyro_val_start - 50)) {
                        // printf("\r%f, %f", default_gyro, val_gyro);
                        // fflush(stdout);
                        turn_left(2 * speed_move_default, DEFAULT_TIME);
                        get_sensor_value0(sn_gyro, &gyro_now);
                    }
                    action = 5;
                    printf("Phase 5\n");
                    Sleep(500);
                } else {
                    move_straight(speed_move_default, DEFAULT_TIME, second_angle);
                }
            } else if (action == 5) {
                if (sonar < 200) {
                    while (gyro_now >= (gyro_val_start - 180)) {
                        // printf("\r%f, %f", default_gyro, val_gyro);
                        // fflush(stdout);
                        turn_left(2 * speed_move_default, DEFAULT_TIME);
                        get_sensor_value0(sn_gyro, &gyro_now);
                    }
                    action = 6;
                    printf("Phase 6\n");
                } else if ((sonar < 550) && (sonar > 450)) {
                    if (can_catch) {
                        catch_flag(speed_move_default);
                        get_sensor_value0(sn_gyro, &gyro_now);
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
                move_straight(speed_move_default * 2, DEFAULT_TIME, gyro_val_start - 180);
                if (sonar <= DISTANCE_STOP) {
                    move_forward(0, 0, DEFAULT_TIME);
                    Sleep(1000); // Wait 5 five seconds 
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
