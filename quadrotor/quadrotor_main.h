#include <lcm/lcm.h>

#include "../robocape/src/usefulincludes.h"
#include "../robocape/src/robocape.h"
#include "../lcmtypes/pose_xyzrpy_t.h"
#include "../lcmtypes/pose_list_t.h"
#include "../lcmtypes/channels_t.h"
#include "../lcmtypes/cfg_data_frequency_t.h"
//TODO: include other lcm types as needed


#define     SAMPLE_RATE_HZ          100   // imu read speed
#define     DT                      0.01  // 1/sample_rate
#define     LCM_HZ                  30
#define     PRINTF_HZ               10



static const char BLOCKS_RX_CHANNEL[] = "CHANNELS_1_RX";
static const char BLOCKS_TX_CHANNEL[] = "CHANNELS_1_TX";
static const char OPTI_CHANNEL[] = "QUADROTOR_POSE_CHANNEL";

typedef struct current_state_t{
    int autonomous_mode;    //Stores current mode, 1 for autonomous

    float Opti_dt;          //seconds from last message
    uint64_t Q_timestamp;   //timestamp of optitrack message
    float Q_xpos;           // X position of Quadrotor
    float Q_ypos;           // Y position of Quadrotor
    float Q_zpos;           // Z position of Quadrotor
    float Q_xdot;           // X velocity of Quadrotor
    float Q_ydot;           // Y velocity of Quadrotor
    float Q_zdot;           // Z velocity of Quadrotor
    float Q_roll;           // roll of Quadrotor
    float Q_pitch;          // pitch of Quadrotor
    float Q_yaw;            // yaw of Quadrotor
    float Q_yawdot;         // rotational velocity of Quadrotor

    int RC_cmds[8];         //RC stick commands

    uint64_t IMU_timestamp; //IMU Data
    float IMU_roll;
    float IMU_pitch;
    float IMU_yaw;
    
    //TODO: Add other state variables as needed
} current_state_t;

// Global data
// #define EXTERN in your main() .c file
// #define EXTERN extern in all other .c files
// Global Variables
EXTERN imu_data_t imu_data;
// LCM Global Variables
EXTERN lcm_t * lcm;
EXTERN pthread_mutex_t state_mutex;
EXTERN current_state_t state;

// IMU interrupt routine
int read_imu();

//threads
void* printf_loop(void* ptr);
void* lcm_publish_loop(void* ptr);
void* lcm_subscribe_loop(void* ptr);

void auto_control(float *pose, float *set_points, int16_t *channels_ptr);
void channels_handler(const lcm_recv_buf_t *rbuf, const char *channel,
              const channels_t *msg, void *userdata);
void optitrack_handler(const lcm_recv_buf_t *rbuf, const char *channel,
              const pose_list_t *msg, void *userdata);