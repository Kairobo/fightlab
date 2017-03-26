// Handler function to either pass through RX commands to Naza or else
// copy computer (autonomous control) commands through to Naza.
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
  for(int i = 0; i < msg->num_channels; i++){
    new_msg.channels[i] = msg->channels[i];
  }

  // Copy state to local state struct to minimize mutex lock time
  struct state localstate;
  pthread_mutex_lock(&state_mutex);
  memcpy(&localstate, state, sizeof(struct state));
  pthread_mutex_unlock(&state_mutex);

  // Decide whether or not to edit the motor message prior to sending it
  // set_points[] array is specific to geofencing.  You need to add code 
  // to compute them for our FlightLab application!!!
  float pose[8], set_points[8];
  if(localstate.fence_on == 1){
    for(int i = 0; i < 8; i++){
      pose[i] = (float) localstate.pose[i];
      set_points[i] = localstate.set_points[i];
    }

    auto_control(pose, set_points, new_msg.channels);
    printf("AUTONOMOUS ON\n");

  } else{
    // pass user commands through without modifying
    printf("MANUAL\n");
  }

  // send lcm message to motors
  channels_t_publish((lcm_t *) userdata, BLOCKS_TX_CHANNEL, &new_msg);

  // Save received (msg) and modified (new_msg) command data to file.
  // NOTE:  Customize as needed (set_points[] is for geofencing)
  fprintf(block_txt,"%"PRId64",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f\n",
      utime_now(),msg->channels[0],msg->channels[1],msg->channels[2],
      msg->channels[3], msg->channels[7],
      new_msg.channels[0],new_msg.channels[1],new_msg.channels[2], 
      new_msg.channels[3],new_msg.channels[7],
      set_points[0],set_points[1],set_points[2],
      set_points[3],set_points[4],set_points[5],set_points[6],
      set_points[7]);
  fflush(block_txt);
}