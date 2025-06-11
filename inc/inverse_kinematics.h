/*
  Inverse_kinematics for 4 DOF robotic arm based in inverse Jacobian method

  The function provide articular velocities (wq) receiving the actual pose of the gripper and the 
  cartesian velocities (X, Y, Z and orientation) stablished by the higher level controller (path planner involved)  
*/

#ifndef foward_kinematics_h
#define foward_kinematics_h

#define l1 0.205
#define l2 0.257
#define l3 0.265
#define l4 0.220
#define pi 3.1416

void inv_Jacobian(float *iJ, const float (&act_q)[4])
{
  float J[16]; //Jacobian (linear indexing)
  
  J[0]=-sin(act_q[0]+pi/2)*((l2*cos(act_q[1]))+(l3*cos(act_q[1]+act_q[2]))+(l4*cos(act_q[1]+act_q[2]+act_q[3])));                                     // J(1,1)
  J[1]=cos(act_q[0]+pi/2)*((l2*cos(act_q[1]))+(l3*cos(act_q[1]+act_q[2]))+(l4*cos(act_q[1]+act_q[2]+act_q[3])));                                      // J(2,1)
  J[2]=0;                                                                                                                                             // J(3,1)
  J[3]=0;                                                                                                                                             // J(4,1)
  J[4]=-(sin(act_q[1])*cos(act_q[0]+pi/2)*l2)-(sin(act_q[1]+act_q[2])*cos(act_q[0]+pi/2)*l3)-(sin(act_q[1]+act_q[2]+act_q[3])*cos(act_q[0]+pi/2)*l4); // J(1,2)
  J[5]=-(sin(act_q[1])*sin(act_q[0]+pi/2)*l2)-(sin(act_q[1]+act_q[2])*sin(act_q[0]+pi/2)*l3)-(sin(act_q[1]+act_q[2]+act_q[3])*sin(act_q[0]+pi/2)*l4); // J(2,2)
  J[6]=(l2*cos(act_q[1]))+(l3*cos(act_q[1]+act_q[2]))+(l4*cos(act_q[1]+act_q[2]+act_q[3]));                                            // J(3,2)
  J[7]=1;                                                                                                                               // J(4,2)
  J[8]=-(sin(act_q[1]+act_q[2])*cos(act_q[0]+pi/2)*l3)-(sin(act_q[1]+act_q[2]+act_q[3])*cos(act_q[0]+pi/2)*l4);                                   // J(1,3)
  J[9]=-(sin(act_q[1]+act_q[2])*sin(act_q[0]+pi/2)*l3)-(sin(act_q[1]+act_q[2]+act_q[3])*sin(act_q[0]+pi/2)*l4);                                   // J(2,3)
  J[10]=(l3*cos(act_q[1]+act_q[2]))+(l4*cos(act_q[1]+act_q[2]+act_q[3]));                                                              // J(3,3)
  J[11]=1;                                                                                                                              // J(4,3)
  J[12]=-(sin(act_q[1]+act_q[2]+act_q[3])*cos(act_q[0]+pi/2)*l4);                                                                            // J(1,4)
  J[13]=-(sin(act_q[1]+act_q[2]+act_q[3])*sin(act_q[0]+pi/2)*l4);                                                                            // J(2,4)
  J[14]=(l4*cos(act_q[1]+act_q[2]+act_q[3]));                                                                                          // J(3,4)
  J[15]=1;                                                                                                                              // J(4,4)

  float detJ=J[0]*((J[5]*J[10])+(J[6]*J[13])+(J[9]*J[14])-(J[13]*J[10])-(J[14]*J[5])-(J[9]*J[6]))-J[1]*((J[4]*J[10])+(J[6]*J[12])+(J[8]*J[14])-(J[12]*J[10])-(J[14]*J[4])-(J[8]*J[6]));

  float Adj_J_trasp[16]; //linear indexing
  Adj_J_trasp[0]=(J[5]*J[10])+(J[9]*J[14])+(J[13]*J[6])-(J[10]*J[13])-(J[14]*J[5])-(J[6]*J[9]);
  Adj_J_trasp[1]=-1*((J[1]*J[10])-(J[1]*J[14]));
  Adj_J_trasp[2]=(J[1]*J[6])-(J[14]*J[1]);
  Adj_J_trasp[3]=-1*((J[1]*J[6])-(J[10]*J[1]));
  Adj_J_trasp[4]=-1*((J[4]*J[10])+(J[8]*J[14])+(J[12]*J[6])-(J[10]*J[12])-(J[14]*J[4])-(J[6]*J[8]));
  Adj_J_trasp[5]=(J[0]*J[10])-(J[14]*J[0]);
  Adj_J_trasp[6]=-1*((J[0]*J[6])-(J[14]*J[0]));
  Adj_J_trasp[7]=(J[0]*J[6])-(J[10]*J[0]);
  Adj_J_trasp[8]=(J[4]*J[9])+(J[8]*J[13])+(J[12]*J[5])-(J[9]*J[12])-(J[13]*J[4])-(J[5]*J[8]);
  Adj_J_trasp[9]=-1*((J[0]*J[9])+(J[12]*J[1])-(J[13]*J[0])-(J[1]*J[8]));
  Adj_J_trasp[10]=(J[0]*J[5])+(J[12]*J[1])-(J[13]*J[0])-(J[1]*J[4]);
  Adj_J_trasp[11]=-1*((J[0]*J[5])+(J[8]*J[1])-(J[9]*J[0])-(J[1]*J[4]));
  Adj_J_trasp[12]=-1*((J[4]*J[9]*J[14])+(J[8]*J[13]*J[6])+(J[12]*J[5]*J[10])-(J[6]*J[9]*J[12])-(J[10]*J[13]*J[4])-(J[14]*J[5]*J[8]));
  Adj_J_trasp[13]=(J[0]*J[9]*J[14])+(J[12]*J[1]*J[10])-(J[10]*J[13]*J[0])-(J[14]*J[1]*J[8]);
  Adj_J_trasp[14]=-1*((J[0]*J[5]*J[14])+(J[12]*J[1]*J[6])-(J[6]*J[13]*J[0])-(J[14]*J[1]*J[4]));
  Adj_J_trasp[15]=(J[0]*J[5]*J[10])+(J[8]*J[1]*J[6])-(J[6]*J[9]*J[0])-(J[10]*J[1]*J[4]);

  for (int i=0;i<=15;i++) iJ[i]=Adj_J_trasp[i]/detJ; 
}

void inv_kinema(float *wq, const float (&act_q)[4], const float (&carts_vel)[4]) 
{ 
  float iJ[16]; //linear indexing like Matlab
  
  inv_Jacobian(&iJ[0], act_q);
  //wq=inv_J*carts_vel 
  wq[0]=(iJ[0]*carts_vel[0])+(iJ[4]*carts_vel[1])+(iJ[8]*carts_vel[2])+(iJ[12]*carts_vel[3]);
  wq[1]=(iJ[1]*carts_vel[0])+(iJ[5]*carts_vel[1])+(iJ[9]*carts_vel[2])+(iJ[13]*carts_vel[3]);
  wq[2]=(iJ[2]*carts_vel[0])+(iJ[6]*carts_vel[1])+(iJ[10]*carts_vel[2])+(iJ[14]*carts_vel[3]);
  wq[3]=(iJ[3]*carts_vel[0])+(iJ[7]*carts_vel[1])+(iJ[11]*carts_vel[2])+(iJ[15]*carts_vel[3]);
  
  //imposing physical limits
  if (act_q[0]>=2*pi && wq[0]>0) wq[0]=0.0; 
  else if (act_q[0]<=0 && wq[0]<0) wq[0]=0.0;
  if (act_q[1]>=3.21 && wq[1]>0) wq[1]=0.0; 
  else if (act_q[1]<=-0.07 && wq[1]<0) wq[1]=0.0;
  if (act_q[2]>=2.08 && wq[2]>0) wq[2]=0.0;
  else if (act_q[2]<=-2.08 && wq[2]<0) wq[2]=0.0;
  if (act_q[3]>=1.73 && wq[3]>0) wq[3]=0.0; 
  else if (act_q[3]<=1.73 && wq[3]<0) wq[3]=0.0;
}

#endif
