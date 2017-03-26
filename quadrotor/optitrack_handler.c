// Handler function to parse Optitrack lcm messages
#define EXTERN extern
#include "quadrotor_main.h"

////////////////////////////////////////////////////////////////////////

void optitrack_handler(const lcm_recv_buf_t *rbuf, const char *channel,
              const pose_list_t *msg, void *userdata){

    //lock mutex state while editing
    pthread_mutex_lock(&state_mutex);
    state.Opti_dt = (msg->Q_pose.utime - state.Q_timestamp)/1000000.0;
    state.Q_timestamp = msg->Q_pose.utime;
    state.Q_xdot = (state.Q_xpos - msg->Q_pose.x)/state.Opti_dt;
    state.Q_ydot = (state.Q_ypos - msg->Q_pose.y)/state.Opti_dt;
    state.Q_zdot = (state.Q_zpos - msg->Q_pose.z)/state.Opti_dt;
    state.Q_xpos = msg->Q_pose.x;
    state.Q_ypos = msg->Q_pose.y;
    state.Q_zpos = msg->Q_pose.z;
    state.Q_roll = msg->Q_pose.roll;
    state.Q_pitch = msg->Q_pose.pitch;
    state.Q_yawdot = (state.Q_yaw - msg->Q_pose.yaw)/state.Opti_dt;
    state.Q_yaw = msg->Q_pose.yaw;
    pthread_mutex_unlock(&state_mutex);
}