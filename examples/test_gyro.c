#include "../include/ev3.h"
#include "../include/ev3_sensor.h"
#include <stdio.h>
#include <unistd.h>
#define Sleep(msec) usleep((msec) * 1000)

int main(void) {
    int i;
    uint8_t sn;
    FLAGS_T state;
    uint8_t sn_gyro;
    char s[256];
    int val_1;
    int val_2;
    float value;
    uint32_t n, ii;

    while (ev3_sensor_init() < 1) {
        Sleep(1000);
    }

   

    
    if (ev3_search_sensor(LEGO_EV3_GYRO, &sn_gyro, 0)) {
        printf("Found the gyroscope\n");
    } else {
        printf("Could not find the gyroscope\n");
        return 2;
    }


    // Run all sensors
    ev3_sensor_init();

    
    get_sensor_value(0, sn_gyro, &val_1);

    fflush(stdout);
    Sleep(3000);

    get_sensor_value(0, sn_gyro, &val_2)
    fflush(stdout);

    if (val_1 == val_2){
        printf("The sensor is working\n");
    }
    else {
        printf("The sensor is inconsistent");
    }
    

    ev3_uninit();
    


    return (0);
}
