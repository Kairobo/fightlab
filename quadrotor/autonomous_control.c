//
// auto_control:  Function to generate autonomous control PWM outputs.
// 
#define EXTERN extern
#include "quadrotor_main.h"

// Define outer loop controller 
// PWM signal limits and neutral (baseline) settings

// THRUST
#define thrust_PWM_up 1575 // Upper saturation PWM limit.
#define thrust_PWM_base 1500 // Zero z_vela PWM base value. 
#define thrust_PWM_down 1425 // Lower saturation PWM limit. 
  
// ROLL
#define roll_PWM_left 1620  // Left saturation PWM limit.
#define roll_PWM_base 1500  // Zero roll_dot PWM base value. 
#define roll_PWM_right 1380 //Right saturation PWM limit. 

// PITCH
#define pitch_PWM_forward 1620  // Forward direction saturation PWM limit.
#define pitch_PWM_base 1500 // Zero pitch_dot PWM base value. 
#define pitch_PWM_backward 1380 // Backward direction saturation PWM limit. 

// YAW
#define yaw_PWM_ccw 1575 // Counter-Clockwise saturation PWM limit (ccw = yaw left).
#define yaw_PWM_base 1500 // Zero yaw_dot PWM base value. 
#define yaw_PWM_cw 1425 // Clockwise saturation PWM limit (cw = yaw right). 

// Outer loop controller to generate PWM signals for the Naza-M autopilot
void auto_control(float *pose, float *set_points, int16_t* channels_ptr)
{  
  // pose (size 8):  actual {x, y , alt, yaw, xdot, ydot, altdot, yawdot}
  // set_points (size 8):  reference state (you need to set this!) 
  //                       {x, y, alt, yaw, xdot, ydot, altdot, yawdot} 

  // channels_ptr (8-element array of PWM commands to generate in this function)
  // Channels for you to set:
  // [0] = thrust
  // [1] = roll
  // [2] = pitch
  // [3] = yaw
}