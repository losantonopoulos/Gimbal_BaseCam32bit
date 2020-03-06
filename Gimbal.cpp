#include "gimbal.h"

// Makes Life easier
using namespace std;

// We need the Termios library to communicate
struct 	termios tio;
int 	tty_fd;
fd_set 	rdset;

/*
*
*/
void Gimbal::INFO(const char *msg)
{
    cout << msg << endl;
}

/*
*
*/
int16_t Gimbal::constrain(int16_t value, int16_t min_val, int16_t max_val)
{
    if(value < min_val){
        return min_val;
    }else if(value > max_val){
        return max_val;
    }else{
        return value;
    }

    return value;
}

/*
*
*/
int Gimbal::constrain(int value, int min_val, int max_val)
{
    if(value < min_val){
        return min_val;
    }else if(value > max_val){
        return max_val;
    }else{
        return value;
    }

    return value;
}

/*
*
*/
double Gimbal::constrain(double value, double min_val, double max_val)
{
    if(value < min_val){
        return min_val;
    }else if(value > max_val){
        return max_val;
    }else{
        return value;
    }

    return value;
}

/*
*
*/
bool Gimbal::engage_gimbal()
{
    INFO("Engage gimbal");

    int n_bytes = 0;

    uint8_t buffer_out[46];

    // Setup request

    uint16_t checksum = 0x0;

    buffer_out[0]   = 0x3e; // Start      
    buffer_out[1]   = 0x5a; // Type
    buffer_out[2]   = 41;   // Payload size
    buffer_out[3]   = (uint16_t)(buffer_out[2] + buffer_out[1]) % 256; // Head checksum

    // Timeout ms
    checksum += buffer_out[4]    = 0x0;
    checksum += buffer_out[5]    = 0x0;

    // Priorities 
    checksum += buffer_out[6]    = 0x0;
    checksum += buffer_out[7]    = 0x0;
    checksum += buffer_out[8]    = 0x0;
    checksum += buffer_out[9]    = 0x0;
    checksum += buffer_out[10]   = 0x1;

    // Axis 1  
    checksum += buffer_out[11]   = 0x0;
    checksum += buffer_out[12]   = 0x0;
    checksum += buffer_out[13]   = 0x0;
    checksum += buffer_out[14]   = 0x0;
    checksum += buffer_out[15]   = 0x0;
    checksum += buffer_out[16]   = 0x0;
    checksum += buffer_out[17]   = 0x0;

    // Axis 2
    checksum += buffer_out[18]   = 0x0;
    checksum += buffer_out[19]   = 0x0;
    checksum += buffer_out[20]   = 0x0;
    checksum += buffer_out[21]   = 0x0;
    checksum += buffer_out[22]   = 0x0;
    checksum += buffer_out[23]   = 0x0;
    checksum += buffer_out[24]   = 0x0;

    // Axis 3
    checksum += buffer_out[25]   = 0x0;
    checksum += buffer_out[26]   = 0x0;
    checksum += buffer_out[27]   = 0x0;
    checksum += buffer_out[28]   = 0x0;
    checksum += buffer_out[29]   = 0x0;
    checksum += buffer_out[30]   = 0x0;
    checksum += buffer_out[31]   = 0x0;
    

    // RC EXPO RATE
    checksum += buffer_out[32]   = 0x0;

    // FLAGS
    checksum += buffer_out[33]   = 0x1; // 
    checksum += buffer_out[34]   = 0x0;

    // RESERVED
    for(int i=35; i<45; i++)        checksum += buffer_out[i] = 0x0;

    buffer_out[45]  = checksum % 256;    

    n_bytes = write(tty_fd, &buffer_out, 46);   // Request

    if(n_bytes == 46){

        //INFO("Gimbal Engaged");

        return true;

    }

    //INFO("Could not engage gimbal...");
    
    return false;
}

/*
*
*/
bool Gimbal::disengage_gimbal()
{

    int n_bytes = 0;

    uint8_t buffer_out[46];

    // Setup request

    uint16_t checksum = 0x0;

    buffer_out[0]   = 0x3e; // Start      
    buffer_out[1]   = 0x5a; // Type
    buffer_out[2]   = 41;   // Payload size
    buffer_out[3]   = (uint16_t)(buffer_out[2] + buffer_out[1]) % 256; // Head checksum

    // Timeout ms
    checksum += buffer_out[4]    = 0x0;
    checksum += buffer_out[5]    = 0x0;

    // Priorities 
    checksum += buffer_out[6]    = 0x0;
    checksum += buffer_out[7]    = 0x1;
    checksum += buffer_out[8]    = 0x0;
    checksum += buffer_out[9]    = 0x0;
    checksum += buffer_out[10]   = 0x0;

    // Axis 1  
    checksum += buffer_out[11]   = 0x0;
    checksum += buffer_out[12]   = 0x0;
    checksum += buffer_out[13]   = 0x0;
    checksum += buffer_out[14]   = 0x0;
    checksum += buffer_out[15]   = 0x0;
    checksum += buffer_out[16]   = 0x0;
    checksum += buffer_out[17]   = 0x0;

    // Axis 2
    checksum += buffer_out[18]   = 0x0;
    checksum += buffer_out[19]   = 0x0;
    checksum += buffer_out[20]   = 0x0;
    checksum += buffer_out[21]   = 0x0;
    checksum += buffer_out[22]   = 0x0;
    checksum += buffer_out[23]   = 0x0;
    checksum += buffer_out[24]   = 0x0;

    // Axis 3
    checksum += buffer_out[25]   = 0x0;
    checksum += buffer_out[26]   = 0x0;
    checksum += buffer_out[27]   = 0x0;
    checksum += buffer_out[28]   = 0x0;
    checksum += buffer_out[29]   = 0x0;
    checksum += buffer_out[30]   = 0x0;
    checksum += buffer_out[31]   = 0x0;
    
    // RC EXPO RATE
    checksum += buffer_out[32]   = 0x0;

    // FLAGS
    checksum += buffer_out[33]   = 0x1; // 
    checksum += buffer_out[34]   = 0x0;

    // RESERVED
    for(int i=35; i<45; i++)        checksum += buffer_out[i] = 0x0;

    buffer_out[45]  = checksum % 256;    
    
    n_bytes = write(tty_fd,&buffer_out, 46);   // Request

    if(n_bytes != 46) return false;

    buffer_out[0]   = 0x3E; // Start      
    buffer_out[1]   = 0x43; // Type
    buffer_out[2]   = 15;  // Payload size
    buffer_out[3]   = (uint16_t)(buffer_out[2] + buffer_out[1]) % 256; // Head checksum

    checksum = 0;

    checksum += buffer_out[4]    = 0x0;     // MODE SPEED (RATE)
    checksum += buffer_out[5]    = 0x0;     // MODE SPEED (RATE)
    checksum += buffer_out[6]    = 0x0;     // MODE SPEED (RATE)

    // Axis 1
    checksum += buffer_out[7]    = 0x0;
    checksum += buffer_out[8]    = 0x0;
    checksum += buffer_out[9]    = 0x0;
    checksum += buffer_out[10]   = 0x0;

    // Axis 2
    checksum += buffer_out[11]   = 0x0;
    checksum += buffer_out[12]   = 0x0;
    checksum += buffer_out[13]   = 0x0;
    checksum += buffer_out[14]   = 0x0;

    // Axis 3
    checksum += buffer_out[15]   = 0x0;
    checksum += buffer_out[16]   = 0x0;
    checksum += buffer_out[17]   = 0x0;
    checksum += buffer_out[18]   = 0x0;

    buffer_out[19]  = checksum % 256;    

    n_bytes = write(tty_fd, &buffer_out, 20);   // Request

    if(n_bytes != 20) return false;
    return true;
}

/*
*
*/
void Gimbal::set_gimbal_speed(double roll_speed, double pitch_speed, double yaw_speed)
{
    int n_bytes = 0;

    uint8_t buffer_out[20];

    // Setup request

    uint16_t checksum = 0x0;

    buffer_out[0]   = 0x3E; // Start      
    buffer_out[1]   = 0x43; // Type
    buffer_out[2]   = 15;  // Payload size
    buffer_out[3]   = (uint16_t)(buffer_out[2] + buffer_out[1]) % 256; // Head checksum

    int16_t roll_rate     = (int16_t)(roll_speed  / GIMBAL_ANGLE_RATE_UNIT);
    int16_t pitch_rate    = (int16_t)(pitch_speed / GIMBAL_ANGLE_RATE_UNIT);
    int16_t yaw_rate      = (int16_t)(yaw_speed   / GIMBAL_ANGLE_RATE_UNIT); 

    roll_speed  = constrain(roll_speed, GIMBAL_MIN_ROLL_SPEED, GIMBAL_MAX_ROLL_SPEED);

    pitch_speed = constrain(pitch_speed, GIMBAL_MIN_PITCH_SPEED, GIMBAL_MAX_PITCH_SPEED);

    yaw_speed   = constrain(yaw_speed, GIMBAL_MIN_YAW_SPEED, GIMBAL_MAX_YAW_SPEED);

    //cout << "Roll: " << roll_rate << "\tPitch: " << pitch_rate << "\tYaw: " << yaw_rate << endl;

    checksum += buffer_out[4]    = 0x1;     // MODE SPEED (RATE)
    checksum += buffer_out[5]    = 0x1;     // MODE SPEED (RATE)
    checksum += buffer_out[6]    = 0x1;     // MODE SPEED (RATE)

    // Axis 1
    checksum += buffer_out[7]    = (uint8_t)roll_rate;
    checksum += buffer_out[8]    = (uint8_t)(roll_rate >> 8);
    checksum += buffer_out[9]    = 0x0;
    checksum += buffer_out[10]   = 0x0;

    // Axis 2
    checksum += buffer_out[11]   = (uint8_t)pitch_rate;
    checksum += buffer_out[12]   = (uint8_t)(pitch_rate >> 8);
    checksum += buffer_out[13]   = 0x0;
    checksum += buffer_out[14]   = 0x0;

    // Axis 3
    checksum += buffer_out[15]   = (uint8_t)yaw_rate;
    checksum += buffer_out[16]   = (uint8_t)(yaw_rate >> 8);
    checksum += buffer_out[17]   = 0x0;
    checksum += buffer_out[18]   = 0x0;

    buffer_out[19]  = checksum % 256;    

    n_bytes = write(tty_fd,&buffer_out, 20);   // Request

    if(n_bytes == 20){
        ;
    }

}

/*
*
*/
void Gimbal::set_gimbal_angle(double roll_angle, double pitch_angle, double yaw_angle)
{
    int n_bytes = 0;

    uint8_t buffer_out[20];

    // Setup request

    uint16_t checksum = 0x0;

    buffer_out[0]   = 0x3E; // Start      
    buffer_out[1]   = 0x43; // Type
    buffer_out[2]   = 15;  // Payload size
    buffer_out[3]   = (uint16_t)(buffer_out[2] + buffer_out[1]) % 256; // Head checksum

    roll_angle  = constrain(roll_angle, GIMBAL_MIN_ROLL_ANGLE, GIMBAL_MAX_ROLL_ANGLE);
    
    pitch_angle = constrain(pitch_angle, GIMBAL_MIN_PITCH_ANGLE, GIMBAL_MAX_PITCH_ANGLE);
    
    yaw_angle   = constrain(yaw_angle, GIMBAL_MIN_YAW_ANGLE, GIMBAL_MAX_YAW_ANGLE);
    
    int16_t roll     = (int16_t)(roll_angle  / GIMBAL_ANGLE_UNIT);
    int16_t pitch    = (int16_t)(pitch_angle / GIMBAL_ANGLE_UNIT);
    int16_t yaw      = (int16_t)(yaw_angle   / GIMBAL_ANGLE_UNIT); 

    //cout << "Roll: " << roll << "\tPitch: " << pitch << "\tYaw: " << yaw << endl;

    checksum += buffer_out[4]   = 0x2;     // MODE ANGLE
    checksum += buffer_out[5]   = 0x2;     // MODE ANGLE
    checksum += buffer_out[6]   = 0x2;     // MODE ANGLE

    // Axis 1
    checksum += buffer_out[7]   = 0x0;
    checksum += buffer_out[8]   = 0x0;
    checksum += buffer_out[9]   = (uint8_t)roll;
    checksum += buffer_out[10]  = (uint8_t)(roll >> 8);

    // Axis 2
    checksum += buffer_out[11]  = 0x0;
    checksum += buffer_out[12]  = 0x0;
    checksum += buffer_out[13]  = (uint8_t)pitch;
    checksum += buffer_out[14]  = (uint8_t)(pitch >> 8);

    // Axis 3
    checksum += buffer_out[15]  = 0x0;
    checksum += buffer_out[16]  = 0x0;
    checksum += buffer_out[17]  = (uint8_t)yaw;
    checksum += buffer_out[18]  = (uint8_t)(yaw >> 8);

    buffer_out[19]  = checksum % 256;    

    n_bytes = write(tty_fd,&buffer_out, 20);   // Request

    if(n_bytes == 20){
        ; 
    }
    
}

/*
*
*/
void Gimbal::set_gimbal_angle_speed(double roll_speed,  double roll_angle, 
                            double pitch_speed, double pitch_angle,
                            double yaw_speed,   double yaw_angle)
{
    int n_bytes = 0;

    uint8_t buffer_out[20];

    // Setup request
    uint16_t checksum = 0x0;

    buffer_out[0]   = 0x3E; // Start      
    buffer_out[1]   = 0x43; // Type
    buffer_out[2]   = 15;  // Payload size
    buffer_out[3]   = (uint16_t)(buffer_out[2] + buffer_out[1]) % 256; // Head checksum

    roll_angle  = constrain(roll_angle, GIMBAL_MIN_ROLL_ANGLE, GIMBAL_MAX_ROLL_ANGLE);
    roll_speed  = constrain(roll_speed, GIMBAL_MIN_ROLL_SPEED, GIMBAL_MAX_ROLL_SPEED);

    pitch_angle = constrain(pitch_angle, GIMBAL_MIN_PITCH_ANGLE, GIMBAL_MAX_PITCH_ANGLE);
    pitch_speed = constrain(pitch_speed, GIMBAL_MIN_PITCH_SPEED, GIMBAL_MAX_PITCH_SPEED);

    yaw_angle   = constrain(yaw_angle, GIMBAL_MIN_YAW_ANGLE, GIMBAL_MAX_YAW_ANGLE);
    yaw_speed   = constrain(yaw_speed, GIMBAL_MIN_YAW_SPEED, GIMBAL_MAX_YAW_SPEED);

    int16_t roll_rate   = (int16_t)(roll_speed  / GIMBAL_ANGLE_RATE_UNIT);
    int16_t pitch_rate  = (int16_t)(pitch_speed / GIMBAL_ANGLE_RATE_UNIT);
    int16_t yaw_rate    = (int16_t)(yaw_speed   / GIMBAL_ANGLE_RATE_UNIT); 

    int16_t roll_ang    = (int16_t)(roll_angle  / GIMBAL_ANGLE_RATE_UNIT);
    int16_t pitch_ang   = (int16_t)(pitch_angle / GIMBAL_ANGLE_RATE_UNIT);
    int16_t yaw_ang     = (int16_t)(yaw_angle   / GIMBAL_ANGLE_RATE_UNIT);

    checksum += buffer_out[4]    = 0x2;     // MODE ANGLE
    checksum += buffer_out[5]    = 0x2;     // MODE ANGLE
    checksum += buffer_out[6]    = 0x2;     // MODE ANGLE

    // Axis 1
    checksum += buffer_out[7]    = (uint8_t) roll_rate;
    checksum += buffer_out[8]    = (uint8_t)(roll_rate >> 8);
    checksum += buffer_out[9]    = (uint8_t) roll_ang;
    checksum += buffer_out[10]   = (uint8_t)(roll_ang >> 8);

    // Axis 2
    checksum += buffer_out[11]   = (uint8_t) pitch_rate;
    checksum += buffer_out[12]   = (uint8_t)(pitch_rate >> 8);
    checksum += buffer_out[13]   = (uint8_t) pitch_ang;
    checksum += buffer_out[14]   = (uint8_t)(pitch_ang >> 8);

    // Axis 3
    checksum += buffer_out[15]   = (uint8_t) yaw_rate;
    checksum += buffer_out[16]   = (uint8_t)(yaw_rate >> 8);
    checksum += buffer_out[17]   = (uint8_t) yaw_ang;
    checksum += buffer_out[18]   = (uint8_t)(yaw_ang >> 8);

    buffer_out[19]  = checksum % 256;    

    n_bytes = write(tty_fd,&buffer_out, 20);   // Request

    if(n_bytes == 20){

        ;
        //INFO("Set speed...");
        
    }

}

/*
*
*/
bool Gimbal::connect_gimbal()
{
    
    const char *port = DEFAULT_PORT;

    INFO("Connecting to Gimbal @");
    INFO(port);

    memset(&tio, 0, sizeof(tio));
    
    tio.c_iflag =   0;
    tio.c_oflag =   0;
    tio.c_cflag =   CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more information
    tio.c_lflag =   0;
    tio.c_cc[VMIN]  =   1;
    tio.c_cc[VTIME] =   1;

    tty_fd = open(port, O_RDWR);        // O_NONBLOCK might override VMIN and VTIME, so read() may return immediately. | O_NONBLOCK
    
    cfsetospeed(&tio, B115200);  // 115200 baud
    cfsetispeed(&tio, B115200);  // 115200 baud

    tcsetattr(tty_fd, TCSANOW, &tio); // return 0 for successfull connection establish. -1 for non

    return true;
}

/*
*
*/
bool Gimbal::connect_gimbal(const char *port)
{

    INFO("Connecting to Gimbal @");
    INFO(port);

    memset(&tio, 0, sizeof(tio));
    
    tio.c_iflag =   0;
    tio.c_oflag =   0;
    tio.c_cflag =   CS8|CREAD|CLOCAL;    // 8n1, see termios.h for more information
    tio.c_lflag =   0;
    tio.c_cc[VMIN]  =   1;
    tio.c_cc[VTIME] =   1;

    tty_fd = open(port, O_RDWR);        // O_NONBLOCK might override VMIN and VTIME, so read() may return immediately. | O_NONBLOCK
    
    cfsetospeed(&tio, B115200);  // 115200 baud
    cfsetispeed(&tio, B115200);  // 115200 baud

    tcsetattr(tty_fd, TCSANOW, &tio); // return 0 for successfull connection establish. -1 for non

    return true;
}

/*
*
*/
void Gimbal::disconnect_gimbal()
{
    INFO("Disconnecting Gimbal");

    // Make sure speed is set to zero
    set_gimbal_speed(0, 0, 0);

    // Disengaging gimbal
    disengage_gimbal();

    // Dropping connection
    close(tty_fd); 
}