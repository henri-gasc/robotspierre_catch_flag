#include "include/ev3.h"
#include "include/ev3_sensor.h"
#include "include/ev3_tacho.h"
#include <stdio.h>
#include <unistd.h>

#define Sleep(msec) usleep((msec) * 1000)
#define DEFAULT_TIME 50

uint8_t sn_gyro;
uint8_t sn_wheel_left;
uint8_t sn_wheel_right;

const int port_wheel_left = 65;
const int port_wheel_right = 66;

void motor_state_time(uint8_t sn, int speed, int time) {
  set_tacho_stop_action_inx(sn, TACHO_COAST);
  set_tacho_speed_sp(sn, speed);
  set_tacho_time_sp(sn, time);
  set_tacho_command_inx(sn, TACHO_RUN_TIMED);
}

void turn_left(int speed, int time) {
  motor_state_time(sn_wheel_left, -speed, time);
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
  motor_state_time(sn_wheel_right, -speed, time);
}

void turn_to(int speed, float gyro_ref) {
  // int gyro_now = update_gyro();
  float gyro_now;
  get_sensor_value0(sn_gyro, &gyro_now);
  bool quit = false;
  float diff;
  while (!quit) {
    diff = gyro_ref - gyro_now;
    printf("\r%6.0f", gyro_now);
    fflush(stdout);
    if (diff > 0) {
      turn_right(speed, DEFAULT_TIME);
    } else if (diff < 0) {
      turn_left(speed, DEFAULT_TIME);
    }
    // gyro_now = update_gyro();
    get_sensor_value0(sn_gyro, &gyro_now);
    quit = (gyro_ref - 1 <= gyro_now) && (gyro_now <= gyro_ref + 1);
  }
}

bool is_motor_here(int port, uint8_t *sn, char *text_yes, char *text_no) {
  bool status = true;
  if (!ev3_search_tacho_plugged_in(port, 0, sn, 0)) {
    printf("%s\n", text_no);
    status = false;
  } else {
    printf("%s\n", text_yes);
  }
  return status;
}

int main(void) {
  int i;
  uint8_t sn;
  FLAGS_T state;
  char s[256];
  float val;
  float value;
  uint32_t n, ii;

  while ((ev3_tacho_init() < 1) & (ev3_sensor_init() < 1)) {
    Sleep(1000);
  }

  if (!is_motor_here(port_wheel_left, &sn_wheel_left, "Found the left wheel",
                     "Could not find the left wheel")) {
    return 3;
  }
  if (!is_motor_here(port_wheel_right, &sn_wheel_right, "Found the right wheel",
                     "Could not find the right wheel")) {
    return 4;
  }

  printf("*** ( EV3 ) Hello! ***\n");

  int default_val;
  if (ev3_search_sensor(LEGO_EV3_GYRO, &sn_gyro, 0)) {
    printf("Found the gyroscope\n");
  } else {
    printf("Could not find the gyroscope\n");
    return 2;
  }
  get_sensor_value(0, sn_gyro, &default_val);

  // Run all sensors
  ev3_sensor_init();

  turn_to(500, 0);
  while (1) {
    get_sensor_value0(sn_gyro, &val);
    printf("\r%6.0f\n", val);
    fflush(stdout);
    Sleep(100);
  }

  ev3_uninit();
  printf("*** ( EV3 ) Bye! ***\n");

  return (0);
}
