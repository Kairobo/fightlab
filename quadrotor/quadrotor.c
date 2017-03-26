/*******************************************************************************
*
*	quadrotor.c
*   Template for ROB550 FlightLab W2017
*
*	pgaskell@umich.edu
*******************************************************************************/

#include <lcm/lcm.h>

#include "../robocape/src/usefulincludes.h"
#include "../robocape/src/robocape.h"
#include "../lcmtypes/pose_xyzrpy_t.h"
#include "../lcmtypes/channels_t.h"
//TODO: include other lcm types as needed


#define 	SAMPLE_RATE_HZ 			100	  // imu read speed
#define 	DT 						0.01  // 1/sample_rate
#define		LCM_HZ				    30
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


int main(int argc, char *argv[]){
	
    //initialize robocape
	initialize_cape();
	set_cpu_frequency(FREQ_1000MHZ);

    //initialize LCM
    lcm = lcm_create(NULL);
    printf("Starting LCM Threads\n");
    //start lcm publish thread
    pthread_t  lcm_publish_thread;
    pthread_t  lcm_subscribe_thread;
    pthread_create(&lcm_publish_thread, NULL, lcm_publish_loop, (void*) NULL);
    pthread_create(&lcm_subscribe_thread, NULL, lcm_subscribe_loop, (void*) NULL);

    // start printf_thread if running from a terminal
    // if it was started as a background process then don't bother
    if(isatty(fileno(stdout))){
        pthread_t  printf_thread;
        pthread_create(&printf_thread, NULL, printf_loop, (void*) NULL);
    }

	// set up IMU configuration
	imu_config_t imu_config = get_default_imu_config();
	imu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	imu_config.orientation = ORIENTATION_Z_UP;

    printf("Starting IMU Thread\n");
	// start imu
	if(initialize_imu_dmp(&imu_data, imu_config)){
		printf("ERROR: can't talk to IMU\n");
		return -1;
	}

    printf("Starting Blocks Thread\n");
    channels_t_subscribe(lcm, "CHANNELS_1_RX", channels_handler, lcm);


	//attach control routine to imu interrupt
	set_imu_interrupt_func(&read_imu);
	
	// start in the RUNNING state
	set_state(RUNNING);

	// Keep Running until state changes to EXITING
	while(get_state()!=EXITING){
		// always sleep at some point
		usleep(10000);
	}
	
	cleanup_cape(); // exit cleanly
	set_cpu_frequency(FREQ_ONDEMAND);
	return 0;
}


/*******************************************************************************
* read_imu() IMU interrupt routine to state variables
* Called at SAMPLE_RATE_HZ
*******************************************************************************/
int read_imu(){

	/******************************************************************
	* STATE_ESTIMATION
	* read IMU and fill lcm message
	******************************************************************/
    state.IMU_timestamp = micros_since_epoch();
	state.IMU_pitch = imu_data.dmp_TaitBryan[TB_PITCH_X];
    state.IMU_roll = imu_data.dmp_TaitBryan[TB_ROLL_Y];
    state.IMU_yaw = imu_data.dmp_TaitBryan[TB_YAW_Z];
	/*************************************************************
	* check for various exit conditions AFTER state estimate
	***************************************************************/
	if(get_state() == EXITING){
		return 0;
	}
	return 1;
}

/*******************************************************************************
* lcm publish_loop() 
*
* prints diagnostics to console
* this only gets started if executing from terminal
*******************************************************************************/

void* lcm_publish_loop(void* ptr){
	while(get_state()!=EXITING){
		//publish lcm messages here, always publishes the latest data
		usleep(1000000 / LCM_HZ);
	}
	return NULL;
}


/*******************************************************************************
* lcm subscribe_loop() 
*
* prints diagnostics to console
* this only gets started if executing from terminal
*******************************************************************************/
void *lcm_subscribe_loop(void *data){
    //Set data frequency of blocks to 100Hz
    cfg_data_frequency_t cfg_data_frequency;
    cfg_data_frequency.hz = (uint8_t) 100;
    cfg_data_frequency_t_publish(lcm, "CFG_DATA_FREQUENCY_1_TX", &cfg_data_frequency);
    cfg_data_frequency_t_publish(lcm, "CFG_DATA_FREQUENCY_1_RX", &cfg_data_frequency);
    //Subscribe to blocks channels lcm channel
    channels_t_subscribe(lcm, "CHANNELS_1_RX", channels_handler, lcm);
    while(1){
      lcm_handle_timeout(lcm, 1);
    }   
    lcm_destroy(lcm);
    return 0;
}


/*******************************************************************************
* printf_loop() 
*
* prints diagnostics to console
* this only gets started if executing from terminal
*******************************************************************************/
void* printf_loop(void* ptr){
    state_t last_state, new_state; // keep track of last state 
    while(get_state()!=EXITING){
        new_state = get_state();
        // check if this is the first time since being paused
        if(new_state==RUNNING && last_state!=RUNNING){
            printf("\nRUNNING\n");
            printf("\n");
            printf("   X   |");
            printf("   Y   |");
            printf("   Z   |");
            printf("  ROLL |");
            printf(" PITCH |");
            printf("  YAW  |");
            printf("\n");

        }
        else if(new_state==PAUSED && last_state!=PAUSED){
            printf("\nPAUSED\n");
        }
        last_state = new_state;
        
        if(new_state == RUNNING){   
            printf("\r");
            //Add Print stattements here, do not follow with /n
            printf("%6.2f |", state.Q_xpos);
            printf("%6.2f |", state.Q_ypos);
            printf("%6.2f |", state.Q_zpos);
            printf("%6.2f |", state.IMU_roll);
            printf("%6.2f |", state.IMU_pitch);
            printf("%6.2f |", state.IMU_yaw);

            fflush(stdout);
        }
        usleep(1000000 / PRINTF_HZ);
    }
    return NULL;
} 