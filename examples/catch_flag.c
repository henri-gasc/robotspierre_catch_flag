#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "../include/ev3.h"
#include "../include/ev3_sensor.h"
#include "../include/ev3_tacho.h"

#define Sleep(msec) usleep((msec) * 1000)
#define PORT_A 65
#define PORT_B 66
#define PORT_C 67
#define PORT_D 68

#define DEFAULT_TIME 50
#define DISTANCE_STOP 50

// For the brick
const int port_wheel_left = PORT_A;
const int port_wheel_right = PORT_B;
const int port_clamp = PORT_C;
uint8_t sn_sonar;
uint8_t sn_wheel_left;
uint8_t sn_wheel_right;
uint8_t sn_clamp;
uint8_t sn_color;
uint8_t sn_gyro;

// Variables that change over the course of the program
int action = 0;
float previous_sonar = -1;
float val_sonar = -1;
int gyro_now = -1;
time_t start_4;

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
    get_sensor_value(0, sn_gyro, &gyro_now);
    // gyro_now = (int) gyro_now % 360;
    return gyro_now;
}


const char *color[] = {"?",      "BLACK", "BLUE",  "GREEN",
                       "YELLOW", "RED",   "WHITE", "BROWN"};
#define COLOR_COUNT ((int)(sizeof(color) / sizeof(color[0])))

int get_color_from_sensor() {
    int val = 0;

    if (ev3_search_sensor(LEGO_EV3_COLOR, &sn_color, 0)) {
        if (!get_sensor_value(0, sn_color, &val) || (val < 0) || (val >= COLOR_COUNT)) {
            val = 0;
        }
    }
    printf("%s\n", color[val]);
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
void move_straight(int speed_default, int time, float default_gyro) {
    float speed_left;
    float speed_right;

    update_gyro();
    int diff = (int) (default_gyro - gyro_now) % 360;
    if (diff != 0) {
        float mul = 1 + (float) abs(diff) / 5;
        if (diff > 0) {
            speed_left  = mul * speed_default;
            speed_right = 1.0 * speed_default;
        } else if (diff < 0) {
            speed_left  = 1.0 * speed_default;
            speed_right = mul * speed_default;
        }
        // printf("%f, %f, %f\n", mul, speed_left, speed_right);
    } else {
        speed_right = speed_default;
        speed_left  = speed_default;
    }
    move_forward(speed_left, speed_right, time);
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
bool catch_flag(int speed) {
    open_clamp(speed, 2000);
    Sleep(500);
    move_forward(speed, speed, 750);
    Sleep(1500);
    move_forward(0, 0, DEFAULT_TIME);
    close_clamp(speed, 2000);
    Sleep(2000);
    int compt = 0;
    for (int i = 0; i < 6; i++) {
        int color = get_color_from_sensor();
        if ((color == 0) && (color != 1)) {
            compt++;
        }
        Sleep(1000);
    }
    return compt == 6;
}

/**
 * @brief Initialize the motors and sensors of the robot
 * 
 * @return int the success status
 */
int init_robot(void) {
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
        return 3;
    }
    if (ev3_search_sensor(LEGO_EV3_COLOR, &sn_color, 0)) {
            printf("Found the color sensor\n");
    } else {
        printf("Could not find color sensor\n");
        return 4;
    }
    if (!is_motor_here(port_wheel_left, &sn_wheel_left, "Found the left wheel", "Could not find the left wheel")) {
        return 5;
    }
    if (!is_motor_here(port_wheel_right, &sn_wheel_right, "Found the right wheel", "Could not find the right wheel")) {
        return 6;
    }
    if (!is_motor_here(port_clamp, &sn_clamp, "Found the clamp", "Could not find the clamp")) {
        return 7;
    }
    return 0;
}

int main(void) {
    int status;
    if ((status = init_robot())) {
        return status;
    }

    int max_speed = get_min_maxspeed(sn_wheel_left, sn_wheel_right, sn_clamp);
    if (max_speed < 0) {
        return max_speed;
    }

    const int speed_move_default = max_speed / 4;
    const int speed_clamp = max_speed / 6;

    const float gyro_val_start = update_gyro();

    bool quit = false;

    while (!quit) {
        move_straight(speed_move_default, DEFAULT_TIME, gyro_val_start);
        quit = catch_flag(speed_clamp);
    }

    stop_motor(sn_wheel_left);
    stop_motor(sn_wheel_right);
    stop_motor(sn_clamp);
    ev3_uninit();
    return 0;
}
