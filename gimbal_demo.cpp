#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <cstdint>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <sstream>
#include <string.h> 

// Gimbal Include
#include "gimbal.h"

using namespace std;

int main(int argc, char **argv)
{
    // Create gimbal instance
    Gimbal my_gimbal;

    // Connect
    my_gimbal.connect_gimbal();

    // Or Connect like that
    //my_gimbal.connect_gimbal("/dev/ttyUSB1");

    // Engage
    my_gimbal.engage_gimbal();

    // You can set speed like that
    my_gimbal.set_gimbal_speed(0, 10, 0);
    
    // Wait 2 seconds (The gimbal is still moving)
    usleep(2000000);

    // Set angle relative to frame like that
    my_gimbal.set_gimbal_angle(0.0, 0.0, 0.0);
    
    // Wait a while
    usleep(1000000);

    // And you can disconnect like that
    my_gimbal.disconnect_gimbal();

    return 0;
}

