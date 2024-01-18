#include "../include/ev3.h"
#include "../include/ev3_sensor.h"
#include <stdio.h>
#include <unistd.h>
#define Sleep(msec) usleep((msec) * 1000)

int main(void) {
    int i;
    uint8_t sn;
    FLAGS_T state;
    uint8_t sn_sonar;
    char s[256];
    int val;
    float value;
    uint32_t n, ii;

    while (ev3_sensor_init() < 1) {
        Sleep(1000);
    }

    printf("*** ( EV3 ) Hello! ***\n");

    int default_val;
    if (ev3_search_sensor(LEGO_EV3_US, &sn_sonar, 0)) {
        printf("Found the sonar\n");
    } else {
        printf("Could not find the sonar\n");
        return 2;
    }
    get_sensor_value(0, sn_sonar, &default_val);

    // Run all sensors
    ev3_sensor_init();

    while (1) {
        get_sensor_value(0, sn_sonar, &val);
        printf("\r%d", val);
        fflush(stdout);
        Sleep(100);
    }

    ev3_uninit();
    printf("*** ( EV3 ) Bye! ***\n");

    return (0);
}
