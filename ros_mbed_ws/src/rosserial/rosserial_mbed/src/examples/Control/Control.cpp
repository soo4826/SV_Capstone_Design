#include "mbed.h"
#include <ros.h>
#include <std_msgs/Float32.h>

#define MAX_SPEED 20

#define CW  1
#define CCW 0
#define DISABLE 1
#define ENABLE  0
#define ROT360 200          // sets how many pulses for 360 degrees
#define PULSE_DELAY 3300    // in uSec.

AnalogOut  drive_motor(p15);

DigitalOut steering_enable(p16);
DigitalOut steering_direction(p17);
DigitalOut steering_pulse(p18);

AnalogIn   steering_in(p20);

DigitalOut myled1(LED1);
DigitalOut myled2(LED2);
DigitalOut myled3(LED3);

ros::NodeHandle nh;

float target_speed_ = 0.0;
float target_steering_ = 0.0;

void targetSpeedCallback( const std_msgs::Float32& cmd_msg) {
   target_speed_ = cmd_msg.data;
   myled1 = !myled1;  //toggle led
}

void targetSteeringCallback( const std_msgs::Float32& cmd_msg) {
   target_steering_ = cmd_msg.data;
   myled2 = !myled2;  //toggle led
}


ros::Subscriber<std_msgs::Float32> sub0("target_speed", targetSpeedCallback);
ros::Subscriber<std_msgs::Float32> sub1("target_steering", targetSteeringCallback);

void stepMotorRun(double count, bool dir_local )   {
   
   steering_enable = ENABLE;
   steering_direction = dir_local;

   count = count * ROT360;
   for (int x = 0; x < count; x++)   {
       steering_pulse = 1;
       myled3 = 1;
       wait_us(10);
       steering_pulse = 0;
       myled3 = 0;
       wait_us(PULSE_DELAY);
   } 
   
   steering_enable = DISABLE;
}

void calSteering(){
   if(target_steering_ > 0.0){
       stepMotorRun(0.1,CW);
   }
   else{
       stepMotorRun(0.1,CCW);
   }
}

void pipeline(){
   drive_motor = target_speed_/MAX_SPEED;
}

int main() {

   nh.initNode();
   nh.subscribe(sub0);
   nh.subscribe(sub1);

   while (1) {
       pipeline();    
       nh.spinOnce();
       wait_ms(10);
   }
}