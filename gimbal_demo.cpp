#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <cstdint>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <sstream>
#include <string.h> 

// Gimbal Includes
#include "gimbal.h"

//#define DBG

using namespace std;


int main(int argc, char **argv)
{
    
    Gimbal my_gimbal;

    my_gimbal.connect_gimbal();

    my_gimbal.engage_gimbal();

    my_gimbal.set_gimbal_speed(0, 10, 0);
    usleep(2000000);

    my_gimbal.set_gimbal_angle(0.0, 0.0, 0.0);
    usleep(1000000);


    my_gimbal.disconnect_gimbal();

    return 0;
}

