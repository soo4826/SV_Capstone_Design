/* Example for Spektrum serial receiver driver
 * M. Nentwig, 2011 
 * This program is provided "as is", without any express or implied warranty.  */

#include "mbed.h"
#include "PwmIn.h"

// ROS Header
#include <ros.h>
#include <std_msgs/Float32.h>

// Spectrum
#define DUTYMAX 0.0863
#define DUTYMIN 0.0498

#define FORWARD  1
#define BACKWARD 0

Serial PCserial(USBTX,USBRX);

DigitalOut automodeLED(LED1) ;
DigitalOut directionLED(LED2);
DigitalOut brakeLED(LED3);

PwmIn thro(p21);
PwmIn alie(p22);
PwmIn elev(p23);
PwmIn rudd(p24);
PwmIn gear(p25);
PwmIn aux1(p26);

// ROS Serial
ros::NodeHandle nh;
std_msgs::Float32 r_steer;
std_msgs::Float32 r_accel;
ros::Publisher pub_steer("steer", &r_steer);
ros::Publisher pub_accel("accel", &r_accel);


int main(void){
    // Spectrum
    float accel = 0.0;
    float steer = 0.5;
    bool  brake = false;
    bool  automode = false;
    bool  direction = FORWARD;
    
    float throttle = 0.0;
    float aileron = 0.0;
    float rudder = 0.0;
    float elevator = 0.0;
    float gearmode = 0.0;
    float auxmode = 0.0;
    
    // ROS
    nh.initNode();
    nh.advertise(pub_steer);
    nh.advertise(pub_accel);


    while(1){
        throttle = thro.dutycycle();
        aileron  = alie.dutycycle();
        rudder   = rudd.dutycycle();
        elevator = elev.dutycycle();
        gearmode = gear.dutycycle();
        auxmode  = aux1.dutycycle();
        
        throttle = (throttle - DUTYMIN)/(DUTYMAX-DUTYMIN);
        aileron  = (aileron - DUTYMIN)/(DUTYMAX-DUTYMIN);
        rudder   = (rudder - DUTYMIN)/(DUTYMAX-DUTYMIN);
        elevator = (elevator - DUTYMIN)/(DUTYMAX-DUTYMIN);
        gearmode = (gearmode - DUTYMIN)/(DUTYMAX-DUTYMIN);
        auxmode = (auxmode - DUTYMIN)/(DUTYMAX-DUTYMIN);
        
        if(throttle > 1.0){throttle = 1.0;}
        else if(throttle < 0.0){throttle = 0.0;}
        
        if(aileron > 1.0){aileron = 1.0;}
        else if(aileron < 0.0){aileron = 0.0;}
        
        if(rudder > 1.0){rudder = 1.0;}
        else if(rudder < 0.0){rudder = 0.0;}
        
        if(elevator > 1.0){elevator = 1.0;}
        else if(elevator < 0.0){elevator = 0.0;}
        
        if(gearmode > 1.0){gearmode = 1.0;}
        else if(gearmode < 0.0){gearmode = 0.0;}
        
        if(auxmode > 1.0){auxmode = 1.0;}
        else if(auxmode < 0.0){auxmode = 0.0;}
        
//        PCserial.printf("thro = %.2f\t", throttle);
//        PCserial.printf("alie = %.2f\t", aileron);
//        PCserial.printf("elev = %.2f\t", rudder);
//        PCserial.printf("rudd = %.2f\t", elevator);
//        PCserial.printf("gear = %.2f\t", gearmode);
//        PCserial.printf("aux1 = %.2f\n\n\r", auxmode);
////////////////////////////////////////////////////////////////////////////////

        if(auxmode > 0.5){automode = true ;  automodeLED = true ; }
        else              {automode = false; automodeLED = false; }
        
        
        
        if(gearmode > 0.5){direction = FORWARD ; directionLED = false;}
        else              {direction = BACKWARD; directionLED = true;}
        
        if(automode == true){
            if(elevator > 0.5){
                accel = (elevator - 0.5)*2.0;
                brake = false;
                brakeLED = false;
            }
            else if(elevator < 0.5 && elevator > 0.4){
                accel = 0.0;
                brake = false;
                brakeLED = false;
            }
            else{
                accel = 0.0;
                brake = true;
                brakeLED = true;
            }
        }
        else{
            accel = 0.0;
            brake = true;
            brakeLED = true;
        }

        
        steer = 2.0*(rudder-0.5);

   
        PCserial.printf("steer = %.2f\t", steer);
        PCserial.printf("accel = %.2f\n\n\r", accel);

        // Publish ros topic into serial
        r_steer.data = steer;
        pub_steer.publish(&r_steer);
        r_accel.data = accel;
        pub_accel.publish(&r_accel);
        nh.spinOnce();
       
    }
    
}
