/*
  foward_kinematics for 4 DOF robotic arm
*/

#ifndef foward_kinematics_h
#define foward_kinematics_h

#define l1 0.205
#define l2 0.257
#define l3 0.265
#define l4 0.220
#define pi 3.1416

void qpos_to_cartesian(float *pose, const float (&act_q)[4]) 
{ // Convert q-positions to cartesian coordinates (+ gripper orientation (4 DOF))
  
    pose[0]=cos(act_q[0]+pi/2)*(l2*cos(act_q[1])+(l3*cos(act_q[1]+act_q[2]))+(l4*cos(act_q[1]+act_q[2]+act_q[3])));
    pose[1]=sin(act_q[0]+pi/2)*(l2*cos(act_q[1])+(l3*cos(act_q[1]+act_q[2]))+(l4*cos(act_q[1]+act_q[2]+act_q[3])));
    pose[2]=(l1)+(l2*sin(act_q[1]))+(l3*sin(act_q[1]+act_q[2]))+(l4*sin(act_q[1]+act_q[2]+act_q[3]));
    pose[3]=act_q[1]+act_q[2]+act_q[3];
}

#endif
