#include "include/ev3.h"
#include "include/ev3_port.h"
#include "include/ev3_sensor.h"
#include "include/ev3_tacho.h"
#include <stdio.h>
#include <stdlib.h>

#include <unistd.h>
#define Sleep(msec) usleep((msec) * 1000)

const char const *color[] = {"?",      "BLACK", "BLUE",  "GREEN",
                             "YELLOW", "RED",   "WHITE", "BROWN"};
#define COLOR_COUNT ((int)(sizeof(color) / sizeof(color[0])))

int main(void) {
  int i;
  uint8_t sn_color;
  char s[256];
  int val;
  uint32_t n, ii;
  if (ev3_init() == -1) {
    return (1);
  }

  printf("*** ( EV3 ) Hello! ***\n");

  // Run all sensors
  ev3_sensor_init();

  printf("Found sensors:\n");
  for (i = 0; i < DESC_LIMIT; i++) {
    if (ev3_sensor[i].type_inx != SENSOR_TYPE__NONE_) {
      printf("  type = %s\n", ev3_sensor_type(ev3_sensor[i].type_inx));
      printf("  port = %s\n", ev3_sensor_port_name(i, s));
      if (get_sensor_mode(i, s, sizeof(s))) {
        printf("  mode = %s\n", s);
      }
      if (get_sensor_num_values(i, &n)) {
        for (ii = 0; ii < n; ii++) {
          if (get_sensor_value(ii, i, &val)) {
            printf("  value%d = %d\n", ii, val);
          }
        }
      }
    }
  }
  for (;;) {
    if (ev3_search_sensor(LEGO_EV3_COLOR, &sn_color, 0)) {
      printf("COLOR sensor is found, reading COLOR...\n");
      if (!get_sensor_value(0, sn_color, &val) || (val < 0) ||
          (val >= COLOR_COUNT)) {
        val = 0;
      }
      printf("\r(%s) \n", color[val]);
      fflush(stdout);
    }
    Sleep(200);
  }

  ev3_uninit();
  printf("*** ( EV3 ) Bye! ***\n");

  return (0);
}
