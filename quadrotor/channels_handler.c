// Handler function to either pass through RX commands to Naza or else
// copy computer (autonomous control) commands through to Naza.
#define EXTERN extern
#include "quadrotor_main.h"

////////////////////////////////////////////////////////////////////////

void channels_handler(const lcm_recv_buf_t *rbuf, const char *channel,
              const channels_t *msg, void *userdata)
{
  // create a copy of the received message
  channels_t new_msg;
  new_msg.utime = msg->utime;
  new_msg.num_channels = msg->num_channels;
  new_msg.channels = (int16_t*) malloc(msg->num_channels*sizeof(int16_t));

  //copy stick commands into new message and to curent state
  pthread_mutex_lock(&state_mutex);
  for(int i = 0; i < msg->num_channels; i++){
    state.RC_cmds[i] = msg->channels[i];
    new_msg.channels[i] = msg->channels[i];
  }
  // Copy state to local state struct to minimize mutex lock time
  struct current_state_t localstate;
  localstate = state;
  pthread_mutex_unlock(&state_mutex);

  // Decide whether or not to edit the motor message prior to sending it...
  float pose[8], set_points[8];
  // Grab pose form current state here
  if(state.autonomous_mode == 1){
    
    //get current quadrotor position, orientation, and relevant velocities
    pose[0] = localstate.Q_xpos;
    pose[1] = localstate.Q_ypos;
    pose[2] = localstate.Q_zpos;
    pose[3] = localstate.Q_yaw;
    pose[4] = localstate.Q_xdot;
    pose[5] = localstate.Q_ydot;
    pose[6] = localstate.Q_zdot;
    pose[7] = localstate.Q_yawdot;

    //simple position hold, update later to handle movment
    set_points[0] = 0.0;
    set_points[1] = 0.0;
    set_points[2] = -1.0;
    set_points[3] = 0.0;
    set_points[4] = 0.0;
    set_points[5] = 0.0;
    set_points[6] = 0.0;
    set_points[7] = 0.0;

    auto_control(pose, set_points, new_msg.channels);
    printf("\rAUTONOMOUS ON                        ");

  } else{
    // pass user commands through without modifying
    printf("\rMANUAL                               ");
  }

  // send lcm message to motors
  channels_t_publish((lcm_t *) userdata, BLOCKS_TX_CHANNEL, &new_msg);
}