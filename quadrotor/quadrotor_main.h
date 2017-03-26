#include <lcm/lcm.h>

#include "../robocape/src/usefulincludes.h"
#include "../robocape/src/robocape.h"
#include "../lcmtypes/pose_xyzrpy_t.h"
#include "../lcmtypes/channels_t.h"
//TODO: include other lcm types as needed


#define     SAMPLE_RATE_HZ          100   // imu read speed
#define     DT                      0.01  // 1/sample_rate
#define     LCM_HZ                  30
#define     PRINTF_HZ               10

// Global Variables
imu_data_t imu_data;

// LCM Global Variables
lcm_t * lcm = NULL;
static const char BLOCKS_CHANNEL[] = "";
static const char QUAD_OPTI_CHANNEL[] = "QUADROTOR_POSE_CHANNEL";
static const char PICKUP_OPTI_CHANNEL[] = "PICKUP_POSE_CHANNEL";
static const char DROPOFF_OPTI_CHANNEL[] = "DROPOFF_POSE_CHANNEL";

typedef struct current_state_t{
    float Q_xpos;           // X position of Quadrotor
    float Q_ypos;           // Y position of Quadrotor
    float Q_zpos;           // Z position of Quadrotor
    float Q_roll;           // roll of Quadrotor
    float Q_pitch;          // pitch of Quadrotor
    float Q_yaw;            // yaw of Quadrotor

    int RC_thrust;          // Pilot CTL raw thrust
    int RC_roll;            // Pilot CTL raw roll
    int RC_pitch;           // Pilot CTL raw pitch
    int RC_yaw;             // Pilot CTL raw yaw
    int RC_auto;            // Pilot CTL raw autonomous switch

    uint64_t IMU_timestamp; //IMU Data
    float IMU_roll;
    float IMU_pitch;
    float IMU_yaw;
    
    //TODO: Add other state variables as needed
} current_state_t;

current_state_t state;

// IMU interrupt routine
int read_imu();

//threads
void* printf_loop(void* ptr);
void* lcm_publish_loop(void* ptr);
void* lcm_subscribe_loop(void* ptr);