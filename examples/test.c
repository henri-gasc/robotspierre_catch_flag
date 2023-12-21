#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>

#include "include/ev3.h"
#include "include/ev3_sensor.h"
#include "include/ev3_tacho.h"

#define Sleep(msec) usleep((msec) * 1000)
#define PORT_A 65
#define PORT_B 66
#define PORT_C 67
#define PORT_D 68

const char *port_name[] = {"A", "B", "C", "D"};
const char *color[] = {"?",      "BLACK", "BLUE",  "GREEN",
                       "YELLOW", "RED",   "WHITE", "BROWN"};

#define COLOR_COUNT ((int)(sizeof(color) / sizeof(color[0])))

static bool _check_pressed(uint8_t sn) {
    int val;
    if (sn == SENSOR__NONE_) {
        return ev3_read_keys((uint8_t *)&val) && (val & EV3_KEY_UP);
    }
    return get_sensor_value(0, sn, &val) && (val != 0);
}

int main(void) {
    uint8_t sn;
    FLAGS_T state;
    uint8_t sn_touch;
    char s[256];

    printf("Test grad & release with touch sensor\n");

    if (ev3_init() == -1)
        return 1;
    printf("Waiting tacho is plugged...\n");

    while ((ev3_tacho_init() < 1) & (ev3_sensor_init() < 1)) {
        Sleep(1000);
    }

    printf("*** ( EV3 ) Hello! ***\n");

    printf("Found tacho motors:\n");
    for (int i = 0; i < DESC_LIMIT; i++) {
        if (ev3_tacho[i].type_inx != TACHO_TYPE__NONE_) {
        printf("  type = %s\n", ev3_tacho_type(ev3_tacho[i].type_inx));
        printf("  port = %s\n", ev3_tacho_port_name(i, s));
        printf("  port = %d %d\n", ev3_tacho_desc_port(i),
                ev3_tacho_desc_extport(i));
        }
    }

    int motor_port = PORT_A;
    bool pressed = false;
    bool quit = false;
    int motor_state = 0;

    if (ev3_search_sensor(LEGO_EV3_TOUCH, &sn_touch, 0)) {
        printf("Found the touch sensor\n");
    } else {
        printf("Could not find touch sensor. Cannot continue\n");
        return 2;
    }
    if (!ev3_search_tacho_plugged_in(motor_port, 0, &sn, 0)) {
        printf("No motor plugged in port %s.\nIs it really in ?\n", port_name[motor_port - PORT_A]);
        return 3;
    } else {
        printf("Found the motor in port %s\n", port_name[motor_port - PORT_A]);
    }
    int max_speed;
    if (get_tacho_max_speed(sn, &max_speed) == 0) {
        printf("Could not read the maximum speed\n");
        return 4;
    } else {
        printf("max_speed = %d\n", max_speed);
    }

    while (!quit) {
        if (_check_pressed(sn_touch) & !pressed) {
            motor_state = (motor_state + 1) % 3;
            pressed = true;
            // printf("Changed motor_state (now %d)\n", motor_state);
        } else if (pressed & !_check_pressed(sn_touch)) {
            pressed = false;
            // printf("You released the button\n");
        }

        set_tacho_stop_action_inx( sn, TACHO_COAST );
        set_tacho_speed_sp( sn, (motor_state - 1) * max_speed / 2 );
        set_tacho_time_sp( sn, 100 );
        set_tacho_command_inx( sn, TACHO_RUN_TIMED );
    }

    ev3_uninit();
    return 0;
}
