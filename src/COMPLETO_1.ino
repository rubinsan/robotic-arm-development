#include "inverse_kinematics.h"
#include "foward_kinematics.h"

//#include <ros.h>
//#include <std_msgs/UInt8.h>

#define PWM0            2       //PWM motor0 pin
#define PWM1            3       //PWM motor3 pin
#define PWM2            4       //PWM motor2 pin
#define PWM3            5       //PWM motor3 pin
#define PWM_GRIP        6       //PWM gripper pin
#define DIR0            22      //Direction motor0 pin
#define DIR1            23      //Direction motor0 pin
#define DIR2            24      //Direction motor2 pin
#define DIR3            25      //Direction motor3 pin
#define DIR_G_1         26      //Gripper direction pin 1
#define DIR_G_2         27      //Gripper direction pin 2
#define MOTOR0_ENC_A    32      //Channel A of incremental encoder 0
#define MOTOR0_ENC_B    33      //Channel B of incremental encoder 0
#define MOTOR1_ENC_A    34      //Channel A of incremental encoder 1
#define MOTOR1_ENC_B    35      //Channel B of incremental encoder 1
#define MOTOR2_ENC_A    36      //Channel A of incremental encoder 2
#define MOTOR2_ENC_B    37      //Channel B of incremental encoder 2
#define MOTOR3_ENC_A    38      //Channel A of incremental encoder 3
#define MOTOR3_ENC_B    39      //Channel B of incremental encoder 3
#define BASE_REF_0      44      //Base optical limit switch 
#define LS_LINK1_CCW    46      //Limit switch link 1 CCW
#define LS_LINK2_CW     49      //Limit switch link 2 CW
#define LS_LINK3_CCW    50      //Limit switch link 3 CCW

#define PHYSICAL_RANGE_3  200     //Physical range between link 2 and link 3
#define ANGULAR_CONST_3   10650   //pulses/rad
#define PHYSICAL_RANGE_2  240     //Physical range between link 2 and link 1
#define ANGULAR_CONST_2   9071    //pulses/rad
#define PHYSICAL_RANGE_1  190     //Physical range between link 1 and link 0
#define ANGULAR_CONST_1   18874   //pulses/rad
#define PHYSICAL_RANGE_0  360     //Physical range between link 0 and base
#define ANGULAR_CONST_0   20915   //pulses/rad

#define l1 0.205
#define l2 0.257
#define l3 0.265
#define l4 0.220
#define pi 3.1416

//#define USE_USBCON              //For communicate with DUE properly

//ros::NodeHandle  nh;

volatile int pulses_0=0;                    //Output pulses of each link
volatile int pulses_1=-100;                 //
volatile int pulses_2=-100;                 //
volatile int pulses_3=-100;                 //
int vel_0=1000;                             //PWM values
int vel_1=500;                              //
int vel_2=400;                              //
int vel_3=600;                              //
float wq_3=0;                               //Link 3 control variables
float err_3=0;                              //
float prev_err_3=0;                         //
float int_err_3=0;                          //
int Kp_3=800;                               //
int Ki_3=350;                               //
int Kd_3=300;                               //
int pulses_3_prev=18590;                    //if its the 1st time to call control vel function (with vel objt 0.0)
float wq_2=0;                               //Link 2 control variables
float err_2=0;                              //
float prev_err_2=0;                         //
float int_err_2=0;                          //
int Kp_2=350;                               //
int Ki_2=150;                               //
int Kd_2=130;                               //
int pulses_2_prev=19000;                    //
float wq_1=0;                               //Link 1 control variables
float err_1=0;                              //
float prev_err_1=0;                         //
float int_err_1=0;                          //
int Kp_1=350;                               //
int Ki_1=150;                               //
int Kd_1=130;                               //
int pulses_1_prev=31290;                    //
float wq_0=0;                               //Link 0 control variables
float err_0=0;                              //
float prev_err_0=0;                         //
float int_err_0=0;                          //
int Kp_0=350;                               //
int Ki_0=150;                               //
int Kd_0=130;                               //
int pulses_0_prev=65705;                    //
boolean init_link0=false;                   //Confirmation for init on each link 
boolean init_link1=false;                   //
boolean init_link2=false;                   //
boolean init_link3=false;                   //
boolean initialization=false;               //
boolean initialization_order=false;         //For control initilization by ROS
char arm_config='0';                        //
volatile boolean range0_completed=false;    //Aux var
volatile boolean range1_completed=false;    //Aux var
volatile boolean range2_completed=false;    //Aux var
volatile boolean range3_completed=false;    //Aux var
const float pulses_for_angle_3=185.9;       //
const float pulses_for_angle_2=158.3;       //
const float pulses_for_angle_1=329.4;       //
const float pulses_for_angle_0=365.0;       //

float wq[4]={0.0};                          // Articular velocities commands
float act_q[4];                             // Articular positions
float carts_vel[4]={0.0};                   // Cartesian velocities {m/s, m/s, m/s, rad/s}
float pose[4];                              // Cartesian pose {X, Y, Z, PHI(rad)}
boolean initial_pose=false;                 //

volatile unsigned long enc_3_riseTime=0;    //Speed measurement
volatile int pulse_3_width=0;               //
volatile unsigned long enc_2_riseTime=0;    //
volatile int pulse_2_width=0;               //
volatile unsigned long enc_1_riseTime=0;    //
volatile int pulse_1_width=0;               //
volatile unsigned long enc_0_riseTime=0;    //
volatile int pulse_0_width=0;               //

boolean fin=false;
int count_interrupt=0;

unsigned int vel_crtl_counter=0;

static volatile unsigned long debounce=0; //For avoiding noise on the sensors.

/*
void init(const std_msgs::UInt8& arm_orders)
{
  switch(arm_orders.data)
  {
    case 1: 
      initialization_order=true;
      break;
    case 2: 
      arm_config='A';
      break;
    case 3: 
      arm_config='B';
      break; 
    case 4: 
      arm_config='R';
      break;
    case 7: 
      arm_config='O'; //Close gripper movement
      break;
    case 8: 
      arm_config='S'; //Stop gripper movement
      break; 
    case 9: 
      arm_config='K'; //Open gripper movement
      break;
    default:
      break;     
  }
}

ros::Subscriber<std_msgs::UInt8> sub("arm_control", &init );
*/
void setup() 
{
  //analogWriteResolution(12);
  pinMode(PWM0, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(PWM_GRIP, OUTPUT);
  analogWrite(PWM0, 0);
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);
  analogWrite(PWM3, 0);
  analogWrite(PWM_GRIP, 0);
  
  pinMode(DIR0, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(DIR3, OUTPUT);
  pinMode(DIR_G_1, OUTPUT);
  pinMode(DIR_G_2, OUTPUT);
  digitalWrite(DIR0, LOW);
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, HIGH);
  digitalWrite(DIR3, LOW);
  digitalWrite(DIR_G_1, LOW);
  digitalWrite(DIR_G_2, LOW);

  pinMode(MOTOR0_ENC_A, INPUT_PULLUP);
  pinMode(MOTOR0_ENC_B, INPUT_PULLUP);
  pinMode(MOTOR1_ENC_A, INPUT);
  pinMode(MOTOR1_ENC_B, INPUT);
  pinMode(MOTOR2_ENC_A, INPUT);
  pinMode(MOTOR2_ENC_B, INPUT);
  pinMode(MOTOR3_ENC_A, INPUT_PULLUP);
  pinMode(MOTOR3_ENC_B, INPUT_PULLUP);

  pinMode(BASE_REF_0, INPUT);
  pinMode(LS_LINK1_CCW, INPUT_PULLUP);
  pinMode(LS_LINK2_CW, INPUT_PULLUP);
  pinMode(LS_LINK3_CCW, INPUT_PULLUP);

  //attachInterrupt(digitalPinToInterrupt(BASE_REF_0), base_mark, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR0_ENC_A), motor0_enc_A_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LS_LINK1_CCW), link1_mark_CCW, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR1_ENC_A), motor1_enc_A_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LS_LINK2_CW), link2_mark_CW, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_ENC_A), motor2_enc_A_change, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LS_LINK3_CCW), link3_mark_CCW, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR3_ENC_A), motor3_enc_A_change, CHANGE);
  
  Serial.begin(19200);

  //nh.initNode();
  //nh.subscribe(sub);
}

void loop() 
{
  if (initialization==false) initialize();
  else if (initialization==true)
  {
    puls_to_rad(&act_q[0]); //obtaining q positions (rad)
    //Serial.print("q0 pos: ");
    //Serial.println(act_q[0]);
    //Serial.print("q1 pos: ");
    //Serial.println(act_q[1]);
    //Serial.print("q2 pos: ");
    //Serial.println(act_q[2]);
    //Serial.print("q3 pos: ");
    //Serial.println(act_q[3]);

    if (initial_pose==false)
    {
      vel_link0(0.0);
      if (act_q[1]<2*pi/3) vel_link1(0.5); 
      else if (act_q[1]>=2*pi/3) vel_link1(0.0);
      if (act_q[2]>-pi/3) vel_link2(-0.5); 
      else if (act_q[2]<=-pi/3) vel_link2(0.0); 
      if (act_q[3]>-pi/3) vel_link3(-0.5);
      else if (act_q[3]<=-pi/3) vel_link3(0.0);
      if (act_q[1]>=2*pi/3 && act_q[2]<=-pi/3 && act_q[3]<=-pi/3) initial_pose=true;
    }
    else if (initial_pose==true)
    {      
      vel_link0(wq[0]);
      vel_link1(wq[1]);
      vel_link2(wq[2]); 
      vel_link3(wq[3]);

      if (vel_crtl_counter>=5)
      {
        qpos_to_cartesian(&pose[0], act_q); // cartesian pose
        Serial.print("X: ");
        Serial.println(pose[0]);
        Serial.print("Y: ");
        Serial.println(pose[1]);
        Serial.print("Z: ");
        Serial.println(pose[2]);
        Serial.print("Orientation: ");
        Serial.println(pose[3]);
        
        if (pose[2]>0.48) 
        {
          carts_vel[0]=0; //imposed cartesian speeds {m/s, m/s, m/s, rad/s}
          carts_vel[1]=0; 
          carts_vel[2]=-0.1; 
          carts_vel[3]=0; 
        }
        else if (pose[1]>-0.52 && pose[2]<=0.48) 
        {
          carts_vel[0]=0; //imposed cartesian speeds {m/s, m/s, m/s, rad/s}
          carts_vel[1]=-0.1; 
          carts_vel[2]=0; 
          carts_vel[3]=0; 
        }
        else if (pose[1]<=-0.52 && pose[2]>0.17 && pose[2]<=0.48) 
        {
          carts_vel[0]=0; //imposed cartesian speeds {m/s, m/s, m/s, rad/s}
          carts_vel[1]=0; 
          carts_vel[2]=-0.1; 
          carts_vel[3]=0;      
        }
        else if (pose[1]<=-0.52 && pose[1]>-0.70 && pose[2]<=0.17) 
        {
          carts_vel[0]=0; //imposed cartesian speeds {m/s, m/s, m/s, rad/s}
          carts_vel[1]=-0.1; 
          carts_vel[2]=0; 
          carts_vel[3]=0;      
        }
        else if (pose[1]<=-0.70 && pose[2]<=0.17) 
        {
          carts_vel[0]=0; //imposed cartesian speeds {m/s, m/s, m/s, rad/s}
          carts_vel[1]=0; 
          carts_vel[2]=0; 
          carts_vel[3]=0;      
        }
        
        inv_kinema(&wq[0], act_q, carts_vel); //obtaining q speed to follow cart speeds commands
        /*
        Serial.print("wq_0: ");
        Serial.println(wq[0]);
        Serial.print("wq_1: ");
        Serial.println(wq[1]);
        Serial.print("wq_2: ");
        Serial.println(wq[2]);
        Serial.print("wq_3: ");
        Serial.println(wq[3]);
        */
      
      
        /*
        qpos_to_cartesian(&pose[0], act_q); // cartesian pose
      
        Serial.print("X: ");
        Serial.println(pose[0]);
        Serial.print("Y: ");
        Serial.println(pose[1]);
        Serial.print("Z: ");
        Serial.println(pose[2]);
        Serial.print("Orientation: ");
        Serial.println(pose[3]);
        */
      }
      
    }

    //qpos_to_cartesian(&pose[0], act_q); // cartesian pose
    /*
    Serial.print("q0 pos: ");
    Serial.println(act_q[0]);
    Serial.print("q1 pos: ");
    Serial.println(act_q[1]);
    Serial.print("q2 pos: ");
    Serial.println(act_q[2]);
    Serial.print("q3 pos: ");
    Serial.println(act_q[3]);
    

    qpos_to_cartesian(&pose[0], act_q); // cartesian pose

    Serial.print("X: ");
    Serial.println(pose[0]);
    Serial.print("Y: ");
    Serial.println(pose[1]);
    Serial.print("Z: ");
    Serial.println(pose[2]);
    Serial.print("Orientation: ");
    Serial.println(pose[3]);
    */
    
  }
  //nh.spinOnce();
  //delay(1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// CONTROL FUNCTIONS ////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void puls_to_rad(float *actual_pos)
{
  actual_pos[0]=float(pulses_0)/float(ANGULAR_CONST_0);
  actual_pos[1]=(float(pulses_1)-float(pulses_for_angle_1*5))/float(ANGULAR_CONST_1); //See reference system of the arm
  actual_pos[2]=(float(pulses_2)-float(pulses_for_angle_2*120))/float(ANGULAR_CONST_2);
  actual_pos[3]=(float(pulses_3)-float(pulses_for_angle_3*100))/float(ANGULAR_CONST_3);
}

void qpos_to_cartesian(float *pose, const float (&act_q)[4]) 
{ // Convert q-positions to cartesian coordinates (+ gripper orientation (4 DOF))
  
    pose[0]=cos(act_q[0]+pi/2)*(l2*cos(act_q[1])+(l3*cos(act_q[1]+act_q[2]))+(l4*cos(act_q[1]+act_q[2]+act_q[3])));
    pose[1]=sin(act_q[0]+pi/2)*(l2*cos(act_q[1])+(l3*cos(act_q[1]+act_q[2]))+(l4*cos(act_q[1]+act_q[2]+act_q[3])));
    pose[2]=(l1)+(l2*sin(act_q[1]))+(l3*sin(act_q[1]+act_q[2]))+(l4*sin(act_q[1]+act_q[2]+act_q[3]));
    pose[3]=act_q[1]+act_q[2]+act_q[3];
}

void initialize(void)
{
  if (init_link3==false) initialize_link3();
  //else if (init_link3==true) vel_3=0;
  if (init_link2==false) initialize_link2();
  //else if (init_link2==true) vel_2=0;
  if (init_link1==false) initialize_link1();
  //else if (init_link1==true) vel_1=0;
  if (init_link0==false) initialize_link0();
  //else if (init_link0==true) vel_0=0;
  if (init_link3==true && init_link2==true && init_link1==true && init_link0==true) 
  {
    initialization=true;
    delay(1000);
  }
}

void initialize_link0(void)
{
  if (pulses_0>55000) attachInterrupt(digitalPinToInterrupt(BASE_REF_0), base_mark, RISING);
  if (pulses_0<=65705 && range0_completed==true)
  {
    vel_0=0;
    init_link0=true;
    //StartTime=millis();
  }
  analogWrite(PWM0, vel_0);
}

void initialize_link1(void)
{                          
  if (pulses_1<5000 && range1_completed==false) vel_1=500;    
  else if (pulses_1>=5000 && range1_completed==false) vel_1=(float(31300)/float(pulses_1))*80; //
  else if (pulses_1>48600 && range1_completed==true) vel_1=800; 
  else if (pulses_1<=48600 && pulses_1>31290 && range1_completed==true) vel_1=(float(pulses_1)/float(48600))*800; //
  else if (pulses_1<=31290 && range1_completed==true)
  {
    vel_1=0;
    init_link1=true;
  }
  analogWrite(PWM1, vel_1);
}

void initialize_link2(void)
{                           
  if (pulses_2<-90 && pulses_2>-4000 && range2_completed==false) vel_2=600;   
  else if (pulses_2<=-4000 && range2_completed==false) vel_2=(float(19000)/float(abs(pulses_2)))*125;
  else if (range1_completed==true && pulses_2>=-90 && pulses_2<11400 && pulses_1<=50000) vel_2=700;
  else if (pulses_2>=11400 && pulses_2<19000) vel_2=(float(19000)/float(pulses_2))*420; //
  else if (pulses_2>=19000) 
  {
    vel_2=0; //
    init_link2=true;
  }
  analogWrite(PWM2, vel_2);
}

void initialize_link3(void)
{
  if (pulses_3<8000 && range3_completed==false) vel_3=800;
  else if (pulses_3>=8000 && range3_completed==false) vel_3=(float(18590)/float(pulses_3))*345; 
  else if (pulses_3<=38000 && pulses_3>27180 && range2_completed==true && range3_completed==true && pulses_1<=37000 && range1_completed==true) vel_3=1400;          
  else if (pulses_3<=27180 && pulses_3>18590 && range3_completed==true) vel_3=(float(pulses_3)/float(27180))*1400; 
  else if (pulses_3<=18590 && range3_completed==true) 
  {
    vel_3=0;
    init_link3=true;
  }
  analogWrite(PWM3, vel_3);
}

///////////////////////////////////////////////////////////////////////

void vel_link3(float goal_wq_3)
{
  wq_3=(float)(1000000/pulse_3_width)/(float)ANGULAR_CONST_3;
  if(goal_wq_3==0) //position control for wq=0.0
  {
    if (pulses_3>=pulses_3_prev) digitalWrite(DIR3, HIGH); //CCW  
    else if (pulses_3<pulses_3_prev) digitalWrite(DIR3, LOW); //CW
    vel_3=120000*abs(pulses_3_prev-pulses_3)/37180;
    analogWrite(PWM3, vel_3);
  }
  else
  {
    if(goal_wq_3>=0) 
    {
      digitalWrite(DIR3, LOW); //CW
      err_3=goal_wq_3-wq_3;
    }
    else if(goal_wq_3<0) 
    {
      digitalWrite(DIR3, HIGH); //CCW 
      err_3=-(goal_wq_3-wq_3);
    }
    vel_3=Kp_3*err_3+Ki_3*int_err_3+Kd_3*(err_3-prev_err_3);
    if (vel_3>=1600) vel_3=1600;
    else if (vel_3<=0) vel_3=0;
    analogWrite(PWM3, vel_3);
    int_err_3=int_err_3+err_3;
    prev_err_3=err_3;
    pulses_3_prev=pulses_3;
  }
}

void vel_link2(float goal_wq_2)
{
  wq_2=(float)(1000000/pulse_2_width)/(float)ANGULAR_CONST_2;
  if(goal_wq_2==0) //position control for wq=0.0
  {
    if (pulses_2>=pulses_2_prev) digitalWrite(DIR2, HIGH); //CCW  
    else if (pulses_2<pulses_2_prev) digitalWrite(DIR2, LOW); //CW
    vel_2=80000*abs(pulses_2_prev-pulses_2)/38000;
    analogWrite(PWM2, vel_2);
  }
  else
  {
    if(goal_wq_2>0) 
    {
      digitalWrite(DIR2, LOW); //CW
      err_2=goal_wq_2-wq_2;
    }
    else if(goal_wq_2<0) 
    {
      digitalWrite(DIR2, HIGH); //CCW 
      err_2=-(goal_wq_2-wq_2);
    }
    vel_2=Kp_2*err_2+Ki_2*int_err_2+Kd_2*(err_2-prev_err_2);
    if (vel_2>=1200) vel_2=1200;
    else if (vel_2<=0) vel_2=0;
    analogWrite(PWM2, vel_2);
    int_err_2=int_err_2+err_2;
    prev_err_2=err_2;
    pulses_2_prev=pulses_2;
  }
}

void vel_link1(float goal_wq_1)
{
  wq_1=(float)(1000000/pulse_1_width)/(float)ANGULAR_CONST_1;  
  if(goal_wq_1==0) //position control for wq=0.0
  {
    if (pulses_1>=pulses_1_prev) digitalWrite(DIR1, HIGH); //CCW  
    else if (pulses_1<pulses_1_prev) digitalWrite(DIR1, LOW); //CW
    vel_1=120000*abs(pulses_1_prev-pulses_1)/62580;
    analogWrite(PWM1, vel_1);
  }
  else
  {
    if(goal_wq_1>=0) 
    {
      digitalWrite(DIR1, LOW); //CW
      err_1=goal_wq_1-wq_1;
    }
    else if(goal_wq_1<0) 
    {
      digitalWrite(DIR1, HIGH); //CCW 
      err_1=-(goal_wq_1-wq_1);
    }
    vel_1=Kp_1*err_1+Ki_1*int_err_1+Kd_1*(err_1-prev_err_1);
    if (vel_1>=1400) vel_1=1400;
    else if (vel_1<=0) vel_1=0;
    analogWrite(PWM1, vel_1);
    int_err_1=int_err_1+err_1;
    prev_err_1=err_1;
    pulses_1_prev=pulses_1;
  }
}

void vel_link0(float goal_wq_0)
{
  if (vel_crtl_counter>=5)vel_crtl_counter=0;
  wq_0=(float)(1000000/pulse_0_width)/(float)ANGULAR_CONST_0;
  if(goal_wq_0==0) //position control for wq=0.0
  {
    if (pulses_0>=pulses_0_prev) digitalWrite(DIR0, HIGH); //CCW  
    else if (pulses_0<pulses_0_prev) digitalWrite(DIR0, LOW); //CW
    vel_0=120000*abs(pulses_0_prev-pulses_0)/131400;
    analogWrite(PWM0, vel_0);
  }
  else
  {
    if(goal_wq_0>=0) 
    {
      digitalWrite(DIR0, LOW); //CW
      err_0=goal_wq_0-wq_0;
    }
    else if(goal_wq_0<0) 
    {
      digitalWrite(DIR0, HIGH); //CCW 
      err_0=-(goal_wq_0-wq_0);
    }
    vel_0=Kp_0*err_0+Ki_0*int_err_0+Kd_0*(err_0-prev_err_0);
    if (vel_0>=1200) vel_0=1200;
    else if (vel_0<=0) vel_0=0;
    analogWrite(PWM0, vel_0);
    int_err_0=int_err_0+err_0;
    prev_err_0=err_0;
    pulses_0_prev=pulses_0;
  }
  vel_crtl_counter++;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// ISRs /////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void base_mark() //ISR for count the optical limit switch base interrupts
{
  if (digitalRead (BASE_REF_0) && (micros()-debounce > 80000) && digitalRead (BASE_REF_0)) 
  { //Check if the signal is '1' and little span has passed for avoid noise.
    debounce = micros(); 
    range0_completed=true;
    digitalWrite(DIR0, HIGH); //Change to CCW direction
    pulses_0=131410;
    detachInterrupt(BASE_REF_0);
  }
}

void link1_mark_CCW() //ISR for counter-clockwise limit switch of link 1
{
  if (digitalRead (LS_LINK1_CCW) && (micros()-debounce > 100000) && digitalRead (LS_LINK1_CCW)) 
  { //Check if the signal is '1' and little span has passed for avoid noise.
    debounce = micros(); 
    digitalWrite(DIR1, HIGH); //Change to CW direction
    pulses_1=62600;
    range1_completed=true;
    detachInterrupt(LS_LINK1_CCW);
  }
}

void link2_mark_CW() //ISR for clockwise limit switch of link 2
{
  if (digitalRead (LS_LINK2_CW) && (micros()-debounce > 100000) && digitalRead (LS_LINK2_CW))
  { //Check if the signal is '1' and little span has passed for avoid noise.
    debounce = micros(); 
    pulses_2=0; //beginnig of range
    digitalWrite(DIR2, LOW); //Change to CCW direction
    range2_completed=true;
    detachInterrupt(LS_LINK2_CW);
  }
}

void link3_mark_CCW() //ISR for counter-clockwise limit switch of link 3
{
  if (digitalRead (LS_LINK3_CCW) && (micros()-debounce > 100000) && digitalRead (LS_LINK3_CCW)) 
  { //Check if the signal is '1' and little span has passed for avoid noise.
    debounce = micros();
    digitalWrite(DIR3, HIGH); //Change to CW direction 
    pulses_3=37180; //beginnig of range
    range3_completed=true;
    detachInterrupt(LS_LINK3_CCW);
  }
}

////////////////////////////////////////////// Encoders ////////////////////////////////////////////////////////

void motor0_enc_A_change() //Reading function for motor0 pulses.
{ 
  noInterrupts();                                 
  if( digitalRead(MOTOR0_ENC_B) == 0 ) 
  {
    if ( digitalRead(MOTOR0_ENC_A) == 0 ) 
    { //A atrasado (vel positiva CW), baja: TOMAR TIEMPO BAJADA 
      pulse_0_width=micros()-enc_0_riseTime;
      pulses_0++; // moving clockwise direction
    } 
    else 
    { 
      enc_0_riseTime=micros();//A adelantado (vel negativa CCW), sube: TOMAR TIEMPO SUBIDA
      pulses_0--; // moving counter-clockwise direction
    }
  }
  else if (digitalRead(MOTOR0_ENC_B)==1) 
  { //A adelantado (vel negativa CCW), baja: TOMAR TIEMPO BAJADA
    if (digitalRead(MOTOR0_ENC_A)==0) pulse_0_width=-(micros()-enc_0_riseTime);
    else enc_0_riseTime=micros();
    //A atrasado (vel positiva CW), sube: TOMAR TIEMPO SUBIDA
  }
  interrupts(); 
}

void motor1_enc_A_change() //Reading function for motor1 pulses.
{ 
  noInterrupts();                                 
  if(digitalRead(MOTOR1_ENC_B)==0) 
  {
    if (digitalRead(MOTOR1_ENC_A)==0) 
    { //A atrasado (vel negativa CW), baja: TOMAR TIEMPO BAJADA 
      pulse_1_width=-(micros()-enc_1_riseTime);
      pulses_1--;//A fell, B is low -> moving clockwise direction 
    } 
    else 
    {
      enc_1_riseTime=micros();//A adelantado (vel positiva CCW), sube: TOMAR TIEMPO SUBIDA
      pulses_1++; //A rose, B is low -> moving counter-clockwise direction
    }
  }
  else if (digitalRead(MOTOR1_ENC_B)==1) 
  { //A adelantado (vel positiva CCW), baja: TOMAR TIEMPO BAJADA
    if (digitalRead(MOTOR1_ENC_A)==0) pulse_1_width=micros()-enc_1_riseTime;
    else enc_1_riseTime=micros();
    //A atrasado (vel negativa CW), sube: TOMAR TIEMPO SUBIDA
  }
  interrupts(); 
}

void motor2_enc_A_change() //Reading function for motor3 pulses.
{ 
  noInterrupts();                                 
  if (digitalRead(MOTOR2_ENC_B)==0) 
  {
    if (digitalRead(MOTOR2_ENC_A)==0) 
    { //A atrasado (vel negativa CW), baja: TOMAR TIEMPO BAJADA 
      pulse_2_width=-(micros()-enc_2_riseTime);
      pulses_2--;//A fell, B is low -> moving clockwise direction 
    } 
    else 
    { 
      enc_2_riseTime=micros();//A adelantado (vel positiva CCW), sube: TOMAR TIEMPO SUBIDA
      pulses_2++; //A rose, B is low -> moving counter-clockwise direction
    }
  }
  else if (digitalRead(MOTOR2_ENC_B)==1) 
  { //A adelantado (vel positiva CCW), baja: TOMAR TIEMPO BAJADA
    if (digitalRead(MOTOR2_ENC_A)==0) pulse_2_width=micros()-enc_2_riseTime;
    else enc_2_riseTime=micros();
    //A atrasado (vel negativa CW), sube: TOMAR TIEMPO SUBIDA
  }
  interrupts(); 
}

void motor3_enc_A_change() //Reading function for motor3 pulses.
{ 
  noInterrupts();                                 
  if (digitalRead(MOTOR3_ENC_B)==0) 
  {
    if (digitalRead(MOTOR3_ENC_A)==0) 
    { //A atrasado (vel negativa CW), baja: TOMAR TIEMPO BAJADA 
      pulse_3_width=-(micros()-enc_3_riseTime);
      pulses_3--;//A fell, B is low -> moving clockwise direction 
    } 
    else 
    { 
      enc_3_riseTime=micros();//A adelantado (vel positiva CCW), sube: TOMAR TIEMPO SUBIDA
      pulses_3++; //A rose, B is low -> moving counter-clockwise direction
    }
  }
  else if (digitalRead(MOTOR3_ENC_B)==1) 
  { //A adelantado (vel positiva CCW), baja: TOMAR TIEMPO BAJADA
    if (digitalRead(MOTOR3_ENC_A)==0) pulse_3_width=micros()-enc_3_riseTime;
    else enc_3_riseTime=micros();
    //A atrasado (vel negativa CW), sube: TOMAR TIEMPO SUBIDA
  }
  interrupts(); 
}
