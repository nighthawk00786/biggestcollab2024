#include <Arduino.h>
#include "PinMap.h"
#include <micro_ros_platformio.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>

// ROS Client Libraries
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// ROS Message Libraries
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
// ...

// Other Libraries
#include <utilities/rcl_handler.h>
#include <const.cpp>
#include <encoders.cpp>
#include <pinmap.cpp>

//ros client variables
rclc_support_t support;
rclc_allocator_t allocator;

//ros node variables
rclc_node_t node;

//ros executor variables
rclc_executor_t executor;

// ROS publisher
rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry msg_odom;

//ros publisher variables
rclc_publisher_t odom_publisher;
nav_msgs__msg__Odometry msg_odom;

//timer for bno
rcl_timer_t odom_timer;

int dir[3][4]={{1,1,1,1},{-1,1,-1,1},{-1,-1,1,1}};
int direction[4]; //Initializing an array for direction
int pwm[4]; //Initializing an array for PWM
float mot_vel[4]; //Initializing an array for motor speeds
float vx,vy,vw; //Local Velocities
float odom_angle;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); //BNO initialization
// function declarations
void cmdvel_callback(const void *msgin){
vx = (float)msg_cmdvel.linear.x;
vy = (float)msg_cmdvel.linear.y;
vw = (float)msg_cmdvel.angular.z;
}

void multiplication(int vx, int vy, int vw)
{
mot_vel[0]=vx*dir[0][0]+vy*dir[1][0]+vw*dir[2][0];
mot_vel[1]=vx*dir[0][1]+vy*dir[1][1]+vw*dir[2][1];
mot_vel[2]=vx*dir[0][2]+vy*dir[1][2]+vw*dir[2][2];
mot_vel[3]=vx*dir[0][3]+vy*dir[1][3]+vw*dir[2][3];
}
void direction_pwm()
{ 
  for (int i=0;i<4;i++)
  {
    if (mot_vel[i] >0) 
    {direction[i] = 0;}
    else{
      direction[i] = 1;
    }
    pwm[i] = (int)map(abs(mot_vel[i]),0,1,0,100);
  }
}
void odom_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  odom_dx = (cm_per_tick * ((enc_count[0] + enc_count[1]) / 2));                                    // dx with respect to robot frame calculated every loop
  odom_dy = (cm_per_tick * enc_count[2]) + (B * cm_per_tick * ((enc_count[1] - enc_count[0]) / L)); // dy with respect to robot frame calculated every loop
  odom_wf = (360 - orientationData.orientation.x) * (PI / 180);
  odom_dw = odom_wf - odom_wi;
  float avg_w = odom_wi + (odom_dw)/2.0;
  odom_wi = odom_wf;

  odom_vx = (odom_dx * cos(avg_w)) - (odom_dy * sin(avg_w));
  odom_vy = (odom_dx * sin(avg_w)) + (odom_dy * cos(avg_w));
  odom_vw = angVelocityData.gyro.z;

  odom_x += odom_vx; // we are adding a small value delta x = odom_dx*cos(theta)-odom_dy*sin(theta) to x
  odom_y += odom_vy;

  for (int i = 0; i < 3; i++)
  {
    enc_count[i] = 0;
  }

  // junction_counter();
  
  // if (prev_j_counter == -1 && j_counter == 0){
  //   odom_x = 0;
  //   odom_y = 0;
  // }
  // if (prev_j_counter == 0 && j_counter == 1){
  //   odom_x = 1.00;
  //   odom_y = 0;
  // }
  // if (prev_j_counter == 1 && j_counter == 2){
  //   odom_x = 5.15;
  //   odom_y = 0;
  // }
  // if (prev_j_counter == 2 && j_counter == 3){
  //   odom_x = 6.06;
  //   odom_y = 0;
  // }
  // if (prev_j_counter == 3 && j_counter == 4){
  //   odom_x = 6.06;
  //   odom_y = 3.80;
  // }
  // if (prev_j_counter == 4 && j_counter == 5){
  //   odom_x = 8.16;
  //   odom_y = 5.02;
  // }

  // prev_j_counter = j_counter;
  // 
  RCSOFTCHECK(rmw_uros_sync_session(1000));
	int64_t time = rmw_uros_epoch_millis();

  msg_odom.header.stamp.sec = time / 1000000000;      // Convert nanoseconds to seconds
  msg_odom.header.stamp.nanosec = time % 1000000000; 

  // rosidl_runtime_c__String__assign(&msg_odom.header.frame_id, "odom");
  // rosidl_runtime_c__String__assign(&msg_odom.child_frame_id, "base_link");

  msg_odom.pose.pose.position.x = odom_x;
  msg_odom.pose.pose.position.y = odom_y;
  msg_odom.pose.pose.orientation.z = odom_wf;

  msg_odom.twist.twist.linear.x = odom_vx * 10;
  msg_odom.twist.twist.linear.y = odom_vy * 10;
  msg_odom.twist.twist.angular.z = odom_vw;

  RCCHECK(rcl_publish(&odom_publisher, &msg_odom, NULL)); 


}


void setup()
{   pinMode(MD1_DIR,OUTPUT);
    pinMode(MD2_DIR,OUTPUT);
    pinMode(MD3_DIR,OUTPUT);
    pinMode(MD4_DIR,OUTPUT);
    pinMode(MD1_PWM,OUTPUT);
    pinMode(MD2_PWM,OUTPUT);
    pinMode(MD3_PWM,OUTPUT);
    pinMode(MD4_PWM,OUTPUT);
   /* Initialise the sensor */
    if(!bno.begin()){
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1); 
    }
  for (int i = 0; i < 3; i++)
  {
    pinMode(ENCA[i], INPUT_PULLUP);
    pinMode(ENCB[i], INPUT_PULLUP);
  }
  attachInterrupt(digitalPinToInterrupt(ENCA[0]), update_encoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA[1]), update_encoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA[2]), update_encoder3, RISING);

     //Setup scripts for microROS
    set_microros_serial_transports(Serial);
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    //create node 
     RCCHECK(rclc_node_init_default(&node, "lock", "", &support));

    // Create executor (Here 1 is the number of handles. Change if required)
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    //create subscriber
    const char *cmdvel_name = "/cmd_vel";
    const rosidl_message_type_support_t *Twist_type = 
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);

    //create publisher
    const char *odom_name = "/odom";
    const rosidl_message_type_support_t *Odometry_type = 
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry);

    //create cmdvel subscriber
    RCCHECK(rclc_subscription_init_default(&cmdvel_subscriber,&node,Twist_type,cmdvel_name));
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmdvel_subscriber, &msg_cmdvel, &cmdvel_callback, ON_NEW_DATA));

    //create odom pub
    RCCHECK(rclc_publisher_init_default(&odom_publisher, &node, Odometry_type, odom_name));

    float timer_period = RCL_MS_TO_NS(100);
      // timer for odom publisher
    RCCHECK(rclc_timer_init_default(&odom_timer, &support, timer_period, odom_callback));
    // RCCHECK(rclc_executor_init(&odom_executor, &support.context, 1, &allocator));
    rclc_executor_add_timer(&executor, &odom_timer);
    
    bno.setExtCrystalUse(true);
}
void loop()

{
 // Spin the executors
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

    // put your main code here, to run repeatedly:
    //rotation(event.orientation.x); // Remember to comment out this function if calculation happens in ROS already
    multiplication(vx,vy,vw);
    direction_pwm();
    digitalWrite(MD1_DIR,direction[0]);
    digitalWrite(MD2_DIR,direction[1]);
    digitalWrite(MD3_DIR,direction[2]);
    digitalWrite(MD4_DIR,direction[3]);
    analogWrite(MD1_PWM,pwm[0]);
    analogWrite(MD2_PWM,pwm[1]);
    analogWrite(MD3_PWM,pwm[2]);
    analogWrite(MD4_PWM,pwm[3]);
}

