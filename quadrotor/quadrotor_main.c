/*******************************************************************************
*
*	quadrotor.c
*   Template for ROB550 FlightLab W2017
*
*	pgaskell@umich.edu
*******************************************************************************/
#define EXTERN
#include "quadrotor_main.h"

int main(int argc, char *argv[]){
	
    //initialize robocape
	initialize_cape();
	set_cpu_frequency(FREQ_1000MHZ);

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
    //attach control routine to imu interrupt
    set_imu_interrupt_func(&read_imu);

    //initialize state mutex
    pthread_mutex_init(&state_mutex, NULL);

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
        printf("Starting Printf Thread\n");
        pthread_t  printf_thread;
        pthread_create(&printf_thread, NULL, printf_loop, (void*) NULL);
    }

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
    pthread_mutex_lock(&state_mutex);
    state.IMU_timestamp = micros_since_epoch();
	state.IMU_pitch = imu_data.dmp_TaitBryan[TB_PITCH_X];
    state.IMU_roll = imu_data.dmp_TaitBryan[TB_ROLL_Y];
    state.IMU_yaw = imu_data.dmp_TaitBryan[TB_YAW_Z];
    pthread_mutex_unlock(&state_mutex);
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
		//publish you own lcm messages here
        //use this to record what you want in lcm logger
        //always publishes the latest data
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
    //Subscribe to lcm channels here
    channels_t_subscribe(lcm, BLOCKS_RX_CHANNEL, channels_handler, lcm);
    pose_list_t_subscribe(lcm, OPTI_CHANNEL, optitrack_handler, lcm);
    
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
                                      //   |
            printf("       OPTITRACK       |");
            printf("          IMU          |");
            printf("        BLOCKS          |");
            printf("\n");
            printf("   X   |");
            printf("   Y   |");
            printf("   Z   |");
            printf("  ROLL |");
            printf(" PITCH |");
            printf("  YAW  |");
            printf("THST|");
            printf("ROLL|");
            printf("PTCH|");
            printf("YAW |");
            printf("AUTO|");
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
            printf("%4d|",state.RC_cmds[0]);
            printf("%4d|",state.RC_cmds[1]);
            printf("%4d|",state.RC_cmds[2]);
            printf("%4d|",state.RC_cmds[3]);
            printf("%4d|",state.RC_cmds[7]);
            fflush(stdout);
        }
        usleep(1000000 / PRINTF_HZ);
    }
    return NULL;
} 