#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
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
long long start_4;
pid_t sound_pid;
int step = 0;

/**
 * @brief Shamelessly taken from https://stackoverflow.com/a/44896326
 *
 * @return long long the current time in milliseconds
 */
long long timeInMilliseconds(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (((long long)tv.tv_sec) * 1000) + (tv.tv_usec / 1000);
}

/**
 * @brief Return the value of the sonar after some filtering, if the value is
 * set to -1, return the previous value
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
    float new_val = (val_sonar + previous_sonar) / 2; // To avoid interferences
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

#define COLOR_COUNT                                                            \
    ((int)(sizeof(color) / sizeof(color[0]))) // Number of colors in the array

/**
 * @brief Retrieves the color from a sensor.
 *
 * This function checks if a LEGO EV3 color sensor is connected. If it is, it
 * retrieves the sensor value. If the sensor value is not retrievable, or if it
 * is outside the valid range (0 to COLOR_COUNT-1), it defaults to 0. It then
 * returns the color corresponding to the sensor value from the color array. If
 * the sensor is not connected, it returns the first color in the color array.
 *
 * @return const char* The color corresponding to the sensor value, or the first
 * color in the array if the sensor is not connected or the sensor value is
 * invalid.
 */
int get_color_from_sensor(void) {
    int val = 0;
    if (ev3_search_sensor(LEGO_EV3_COLOR, &sn_color, 0)) {
        if (!get_sensor_value(0, sn_color, &val) || (val < 0) ||
            (val >= COLOR_COUNT)) {
            val = 0;
        }
    }
    return val;
}

/**
 * @brief Increment the action counter by 1
 * action is a global variable
 *
 */
void change_action(void) {
    action++; // Increment the action counter
    printf("Action %d\n", action);
    printf("%ld\n", time(NULL));
}

/**
 * @brief Force the action counter to be this value
 * action is a global variable
 *
 * @param new_action the new value for the action counter
 */
void override_action(int new_action) {
    printf("Force the change of action from %d to %d\n", action, new_action);
    action = new_action; // Force the action counter to be this value
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
bool is_motor_here(int port, uint8_t *sn, char *text_yes, char *text_no) {
    bool status = true;
    if (!ev3_search_tacho_plugged_in(port, 0, sn,
                                     0)) { // Check if the motor is connected
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
    max_speed = MIN(max_speed, temp); // Get the minimum of the two first motors
    if (get_tacho_max_speed(sn_3, &temp) == 0) {
        printf("Could not read the maximum speed for third arg\n");
        max_speed = -3;
    }
    return MIN(max_speed, temp); // Get the minimum of the three motors (the
                                 // minimum of the two first and the third)
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
void stop_motor(uint8_t sn) { set_tacho_command_inx(sn, TACHO_STOP); }

/**
 * @brief Set the motor to run with the default time
 *
 * @param sn the motor
 * @param speed the time
 */
void motor_state(uint8_t sn, int speed) {
    motor_state_time(sn, speed, DEFAULT_TIME); // Set the motor to run for 50 ms
}

/**
 * @brief Set the two wheel to run at the specified speeds
 *
 * @param speed_left the speed of the left wheel
 * @param speed_right the speed of the right wheel
 * @param time the time the motor should turn for
 */
void move_forward(int speed_left, int speed_right, int time) {
    motor_state_time(sn_wheel_left, speed_left,
                     time); // Set the left wheel to run
    motor_state_time(sn_wheel_right, speed_right,
                     time); // Set the right wheel to run
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

    // This part of the code is used to correct the trajectory of the robot.
    // The two motor are not synchronized and the robot tend to turn to the left
    // so we rectify that
    update_gyro();
    int diff = (int)(default_gyro - gyro_now) % 360;
    if (diff != 0) {
        float mul = 1 + (float)abs(diff) / 10;
        if (diff > 0) {
            speed_left = mul * speed_default;
            speed_right = 1.0 * speed_default;
        } else if (diff < 0) {
            speed_left = 1.0 * speed_default;
            speed_right = mul * speed_default;
        }
        // printf("%f, %f, %f\n", mul, speed_left, speed_right);
    } else {
        speed_right = speed_default;
        speed_left = speed_default;
    }
    move_forward(speed_left, speed_right, time);
}

/**
 * @brief Move straight for a set amount of time
 * 
 * @param milliseconds the time we will move straight for
 * @param reference_angle the reference angle we will use
 * @param speed_default the speed at which it will move
 */
void move_straight_for(int milliseconds, float reference_angle,
                       int speed_default) {
    long long start = timeInMilliseconds();
    long long now = start;
    while ((now - start) < milliseconds) {
        move_straight(speed_default, DEFAULT_TIME, reference_angle);
        now = timeInMilliseconds();
    }
    stop_motor(sn_wheel_left);
    stop_motor(sn_wheel_right);
}

/**
 * @brief Turn to the left but the robot also move forward
 *
 * @param speed the speed
 * @param time the time
 */
void turn_left(int speed, int time) {
    motor_state_time(sn_wheel_left, 0, time); // only one wheel turn
    motor_state_time(sn_wheel_right, speed, time);
}

/**
 * @brief Turn to the right but the robot also move forward
 *
 * @param speed the speed
 * @param time the time
 */
void turn_right(int speed, int time) {
    motor_state_time(sn_wheel_left, speed, time); // same as before
    motor_state_time(sn_wheel_right, 0, time);
}

/**
 * @brief Turn to the right without moving forward
 *
 * @param speed the speed
 * @param time the time
 */
void turn_right_in_place(int speed, int time) {
    motor_state_time(sn_wheel_left, speed, time);
    motor_state_time(sn_wheel_right, -speed, time);
}

/**
 * @brief Open the clamp
 *
 * @param speed the speed > 0 to open the clamp
 * @param time the time
 */
void open_clamp(float speed, int time) {
    motor_state_time(sn_clamp, -speed, time);
}

/**
 * @brief Close the clamp
 *
 * @param speed the speed
 * @param time the time
 */
void close_clamp(float speed,
                 int time) {
    open_clamp(- speed, time); // We just reverse the speed
}

/**
 * @brief Function that ease catching the flag
 * We start by opening the clamp, then we move a little forward. The we close the clamp and get the color sensor value. If the sensor cannot see a color, it means there is the flag in front of it.
 *
 * @param speed the speed
 */
bool catch_flag(int speed, float ref_angle) {
    open_clamp(speed, 2000);
    Sleep(500);
    move_straight_for(500, ref_angle, speed);
    Sleep(1000);
    close_clamp(speed, 2000);
    Sleep(2000);
    int num = 6;
    int compt = 0;
    for (int i = 0; i < num; i++) {
        int k = get_color_from_sensor();
        printf("\r%6s", color[k]);
        fflush(stdout);
        if (k == 0) {
            compt++;
        }
        Sleep(250);
    }
    return compt == num;
}

/**
 * @brief Turn to 
 * 
 * @param speed 
 * @param gyro_ref 
 * @param marge 
 */
void turn_to(int speed, float gyro_ref, int marge) {
    update_gyro();
    bool quit = false;
    gyro_ref = (int)gyro_ref;
    float diff;
    while (!quit) {
        gyro_now = gyro_now;
        // Should be better
        // diff = ((int) (gyro_ref - gyro_now) % 360) - 180;
        diff = gyro_ref - gyro_now;
        if (diff > 0) {
            turn_right(speed, DEFAULT_TIME);
        } else if (diff < 0) {
            turn_left(speed, DEFAULT_TIME);
        }
        update_gyro();
        quit = (-marge <= diff) && (diff <= marge);
    }
}

void bypass_obstacle(int speed, float reference_angle, bool obstacle) {
    // If we want to be sure there is no longer an opponent in front
    // Sleep(3000);
    // update_sonar();
    // if (val_sonar >= 200) {
    //     printf("There is no opponent\n");
    //     return;
    // }
    if (obstacle) {
        move_straight_for(2000, reference_angle, -speed);
    }
    turn_to(speed, reference_angle - 90, 1);
    update_sonar();
    while (val_sonar >= 270) {
        move_straight(2 * speed, DEFAULT_TIME, reference_angle - 90);
        update_sonar();
    }
    turn_to(speed, reference_angle, 1);
    int time_forward = 1;
    if (obstacle) {
        time_forward++;
    }
    long long start = timeInMilliseconds();
    long long now = start;
    update_sonar();
    while ((now - start < time_forward * 1000) && (val_sonar > 300)) {
        move_straight(2 * speed, DEFAULT_TIME, reference_angle);
        now = timeInMilliseconds();
        update_sonar();
    }
    while (val_sonar < 270) {
        move_forward(-speed, -speed, DEFAULT_TIME);
        update_sonar();
    }
    turn_to(speed, reference_angle + 90, 1);
    update_sonar();
    while (val_sonar >= 270) {
        move_straight(2 * speed, DEFAULT_TIME, reference_angle + 90);
        update_sonar();
    }
    turn_to(speed, reference_angle, 1);
    update_sonar();
    while (val_sonar < 270) {
        move_forward(-speed, -speed, DEFAULT_TIME);
        update_sonar();
    }
}

void bypass_back(int speed, float reference_angle, bool obstacle) {
    if (obstacle) {
        move_straight_for(2000, reference_angle, -2 * speed);
    }
    turn_to(speed, reference_angle - 90, 1);
    update_sonar();
    while (val_sonar >= 300) {
        move_straight(2 * speed, DEFAULT_TIME, reference_angle - 90);
        update_sonar();
    }
    turn_to(speed, reference_angle, 1);
    update_sonar();
    while (val_sonar < 300) {
        move_forward(-speed, -speed, DEFAULT_TIME);
        update_sonar();
    }
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

    if (ev3_search_sensor(LEGO_EV3_US, &sn_sonar, 0)) {
        printf("Found the sonar\n");
    } else {
        printf("Could not find the sonar\n");
        return 2;
    }
    if (ev3_search_sensor(LEGO_EV3_GYRO, &sn_gyro, 0)) {
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
    if (!is_motor_here(port_wheel_left, &sn_wheel_left, "Found the left wheel",
                       "Could not find the left wheel")) {
        return 5;
    }
    if (!is_motor_here(port_wheel_right, &sn_wheel_right,
                       "Found the right wheel",
                       "Could not find the right wheel")) {
        return 6;
    }
    if (!is_motor_here(port_clamp, &sn_clamp, "Found the clamp",
                       "Could not find the clamp")) {
        return 7;
    }
    return 0;
}

void *thread_play_sound() {
    sound_pid = fork();

    if (step == 0) {
        step = 1;
        if (sound_pid == 0) { // This block will be run by the child process
            execlp("aplay", "aplay", "dubstep.wav", NULL);
        }
    }
    if (step == 1) {
        if (sound_pid == 0) { // This block will be run by the child process
            execlp("/bin/sh", "/bin/sh", "-c",
                   "espeak \"viva la revolution\" --stdout -v spanish | aplay",
                   NULL);
        }
    }
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

    const int speed_move_default = max_speed / 3;
    const int speed_return = speed_move_default;
    const int speed_clamp = max_speed / 5;
    const int speed_right = speed_move_default;
    const int speed_left = speed_move_default;

    // Action 0: Turn to 45° the right
    // Action 1: starting position -> wall, turn to start orientation
    // Action 2: wall right -> wall other side, turn 90° to the left
    // Action 3: move straigh while closing the clamp, turn 90° to the left
    // Action 4: Speed to our camp, turn 90° to the left
    // Action 5: Move forward, open clamp
    // Action 10: Did not found the flag during action 3, go back to the other
    // side and try again

    /* Here are the angle the robot should follow for all phases */
    // const float gyro_val_start = turn_until_min(speed_clamp, DEFAULT_TIME);
    const float gyro_val_start = update_gyro();
    const float first_angle = gyro_val_start + 45;
    const float second_angle = gyro_val_start - 2;
    const float third_angle = gyro_val_start - 90;
    const float fourth_angle = gyro_val_start - 182;
    const float fifth_angle = gyro_val_start - 290;
    const float tenth_angle = gyro_val_start - 270;

    float ref_angle_fourth_phase = fourth_angle;

    printf("%f, %f, %f, %f, %f, %f\n", gyro_val_start, first_angle,
           second_angle, third_angle, fourth_angle, fifth_angle);

    float sonar = 0;
    bool quit = false;
    bool can_catch = true;
    bool allow_quit = false;
    // bool entered = false;
    long long start = timeInMilliseconds();
    long long now = start;

    while (!quit) {
        if ((action == 0) || (action == 4)) {
            get_sensor_value0(sn_sonar, &val_sonar);
            sonar = val_sonar;
        } else {
            sonar = update_sonar();
        }
        if (!sonar) {
            continue;
        }

        // Phase 0
        if (action == 0) {
            turn_to(2 * speed_move_default, first_angle, 1);
            change_action();
        } else if (sonar > 0) {
            if (sonar >= DISTANCE_STOP) {
                allow_quit = false;
            }
            if ((sonar <= DISTANCE_STOP) && allow_quit) {
                quit = true;
            }

            // Phase 1
            else if (action == 1) {
                if (sonar < 280) {
                    turn_to(speed_move_default, second_angle, 1);
                    change_action();
                } else {
                    move_straight(speed_move_default, DEFAULT_TIME,
                                  first_angle);
                }
                // Phase 2
            } else if (action == 2) {
                if (sonar < 230) {
                    now = timeInMilliseconds();
                    long long diff = now - start;
                    printf("turning: %lld\n", diff);
                    if (diff < 10000) {
                        bypass_obstacle(speed_move_default, gyro_val_start,
                                        (diff < 7000) && (diff > 5000));
                    } else {
                        turn_to(speed_clamp, third_angle, 0);
                        now = timeInMilliseconds();
                        while (start + 20000 > now) {
                            printf("\rMoving again in %2lld",
                                   20 - (now - start) / 1000);
                            fflush(stdout);
                            Sleep(500);
                            now = timeInMilliseconds();
                        }
                        printf("\rStarting now !           \n");
                        change_action();
                        sonar = update_sonar();
                    }
                } else {
                    move_straight(speed_move_default, DEFAULT_TIME,
                                  second_angle);
                }
                // Phase 3
            } else if (action == 3) {
                if (sonar < 240) {
                    turn_to(speed_clamp, fourth_angle, 0);
                    if (!can_catch) {
                        // if (entered && !can_catch) {
                        set_tacho_command_inx(sn_clamp, TACHO_RUN_FOREVER);
                        change_action();
                    } else { // We did not found the flag
                        move_straight_for(1000, fourth_angle,
                                          speed_move_default);
                        turn_to(speed_move_default, tenth_angle, 1);
                        override_action(10);
                    }
                    start_4 = timeInMilliseconds();
                } else if ((sonar < 540) && (sonar > 430) && can_catch) {
                    can_catch = !catch_flag(speed_clamp, third_angle);
                    if (!can_catch) {
                        printf("\rFOUND THE FLAG!!! FOUND THE FLAG!!!\n");
                        pthread_t sound_thread;
                        pthread_create(&sound_thread, NULL, thread_play_sound,
                                       NULL);
                        pthread_detach(sound_thread);
                    }
                } else if (sonar <= 430) {
                    close_clamp(speed_clamp, 1000);
                    move_straight(speed_move_default, DEFAULT_TIME,
                                  third_angle);
                    // set_tacho_command_inx(sn_clamp, TACHO_RUN_FOREVER);
                } else {
                    if (sonar > 600) {
                        // can_catch = true;
                        open_clamp(speed_move_default, 1000);
                    }
                    // entered = true;
                    move_straight(speed_left, speed_right, third_angle);
                }
            } else if (action == 4) {
                now = timeInMilliseconds();
                // printf("%ld\n", now);
                move_straight(speed_return, DEFAULT_TIME,
                              ref_angle_fourth_phase);
                if (sonar <= DISTANCE_STOP) {
                    move_forward(0, 0, DEFAULT_TIME);
                    Sleep(1000); // Wait 1 five seconds
                    allow_quit = true;
                } else if ((sonar <= 250) && (now - start_4 < 8500)) {
                    // printf("Changing angle from %f to ",
                    // ref_angle_fourth_phase);
                    ref_angle_fourth_phase = fourth_angle + 12;
                    // printf("%f\n", ref_angle_fourth_phase);
                    bypass_back(speed_move_default, ref_angle_fourth_phase,
                                now - start < 6000);
                } else if (sonar <= 210) {
                    stop_motor(sn_clamp);
                    turn_to(speed_return, fifth_angle, 1);
                    move_forward(0, 0, DEFAULT_TIME);
                    open_clamp(speed_clamp, 2000);
                    Sleep(500);
                    turn_right_in_place(speed_clamp, 1000);
                    Sleep(1500);
                    change_action();
                    quit = true;
                }
            } else if (action == 10) {
                if (val_sonar <= 100) {
                    move_straight_for(500, tenth_angle, -speed_move_default);
                } else if (val_sonar <= 250) {
                    turn_to(speed_move_default, second_angle, 1);
                    override_action(2);
                } else {
                    move_straight(2 * speed_move_default, DEFAULT_TIME,
                                  tenth_angle);
                }
            }
        }
    }

    kill(sound_pid, SIGTERM); // Stop the current sound
    thread_play_sound();      // Play a new sound

    stop_motor(sn_wheel_left);
    stop_motor(sn_wheel_right);
    stop_motor(sn_clamp);
    ev3_uninit();
    return 0;
}
