/* Example for Spektrum serial receiver driver
 * M. Nentwig, 2011 
 * This program is provided "as is", without any express or implied warranty.  */

#include "mbed.h"
#include "PwmIn.h"
#include "Servo.h"

// ROS Header
// #include <ros.h>
// #include <std_msgs/Float32.h>

#define STEP_THRESH 0.04
#define STEER_MIN 0.1
#define STEER_MAX 0.9

// Spectrum
#define DUTYMAX 0.0863
#define DUTYMIN 0.0498

#define FORWARD  1
#define BACKWARD 0

// Serial PCserial(USBTX,USBRX);

DigitalOut automodeLED(LED1) ;
DigitalOut directionLED(LED2);
DigitalOut brakeLED(LED3);

PwmIn thro(p30);
PwmIn alie(p21);
PwmIn elev(p22);
PwmIn rudd(p23);
PwmIn gear(p24);
PwmIn aux1(p25);
Servo breakServo(p26);

DigitalOut EnaOut(p9);
DigitalOut DirOut(p10);
DigitalOut PulOut(p11);

AnalogIn potentiometer(p17);
AnalogOut accelOut(p18); //모터 스로틀, 초록선, 0~1
DigitalOut brakeOut(p19); //브레이크
DigitalOut directionOut(p12);//전진

Ticker Pulse;
Timeout stepTimeout;

float steering_value = 0.5;

// ROS Serial
// ros::NodeHandle nh;
// std_msgs::Float32 r_steer;
// std_msgs::Float32 r_accel;
// ros::Publisher pub_steer("steer", &r_steer);
// ros::Publisher pub_accel("accel", &r_accel);

char defineStep(float potentiometer_value){
    float difference = potentiometer_value - steering_value;
    bool InRange = abs(difference) <= STEP_THRESH;
    bool Sign = difference <= 0;
    return ((char)InRange<<1)|((char)Sign);
}

void stepSTOP(){
    EnaOut = false;
    DirOut = true;
}

void pulDown(){
    PulOut = false;
}

void stepCW(){
    DirOut = true;
    PulOut = true;
    stepTimeout.attach(&pulDown, 0.0003);
}

void stepCCW(){
    DirOut = false;
    PulOut = true;
    stepTimeout.attach(&pulDown, 0.0003);
}

void step(){
    float potentiometer_value = potentiometer;
    char act = defineStep(potentiometer_value);
    if (act>>1){
        stepSTOP();
    }
    else if(act&1){
        stepCW();
    }
    else{
        stepCCW();
    }
}

float ang2sval(float ang){
    float center = STEER_MIN;
    float coeff = STEER_MAX-STEER_MIN;//1/3600;
    return coeff*ang+center;
}

void stepCon(float Angle){
    steering_value = ang2sval(Angle);
}

void AccCon(float Acc){
    accelOut = Acc;
}

void BreakCon(float breakOn){
    breakServo.calibrate(0.0005, 45.0);
    if(breakOn<-0.1){
        brakeOut = true;
        breakServo = 1.0 + 1.4*breakOn;
    }else{
        brakeOut = false;
        breakServo = 1.0;
    }
}

void DirCon(float direction){
    if (direction > 0.5){
        directionOut = true;
    }else{
        directionOut = false;
    }
}

void RosRun(float *con_data){
    // AccCon(con_data[4]);
    stepCon(con_data[5]);
    BreakCon(con_data[6]);
    DirCon(con_data[7]);
}

void ControllerRun(float *con_data){
    // AccCon(con_data[0]);
    stepCon(con_data[1]);
    BreakCon(con_data[2]);
    DirCon(con_data[3]);
}

int main(void){
    // Spectrum
    bool  brake = false;
    bool  automode = false;
    float  direction = FORWARD;
    
    float throttle = 0.0;
    float aileron = 0.5;
    float rudder = 0.5;
    float elevator = 0.5;
    float gearmode = 0.0;
    float auxmode = 0.0;

    float con_data[8] = {0.0, 0.5, 1.0, FORWARD, 0.0, 0.5, 1.0, FORWARD};//{ControllerAcc, ControllerSteer, ControllerBreak, ControllerDirection, RosAcc, RosSteer, RosBreak, RosDirection}

    
    // ROS
    // nh.initNode();
    // nh.advertise(pub_steer);
    // nh.advertise(pub_accel);

    Pulse.attach(&step,0.0015);

    void (*Running[2])(float*);
    Running[0] = RosRun;
    Running[1] = ControllerRun;

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
                con_data[0] = (elevator - 0.5)*2.0;
                con_data[1] = rudder;
                con_data[2] = elevator-0.5;
                con_data[3] = direction;
                con_data[4] = 0.0;
                con_data[5] = 0.5;
                con_data[6] = true;
                con_data[7] = direction;
                brakeLED = false;
            }
            else if(elevator < 0.5 && elevator > 0.4){
                con_data[0] = 0.0;
                con_data[1] = rudder;
                con_data[2] = elevator-0.5;
                con_data[3] = direction;
                con_data[4] = 0.0;
                con_data[5] = 0.5;
                con_data[6] = true;
                con_data[7] = direction;
                brakeLED = false;
            }
            else{
                con_data[0] = 0.0;
                con_data[1] = rudder;
                con_data[2] = elevator-0.5;
                con_data[3] = direction;
                con_data[4] = 0.0;
                con_data[5] = 0.5;
                con_data[6] = true;
                con_data[7] = direction;
                brakeLED = true;
            }
        }
        else{
            con_data[0] = 0.0;
            con_data[1] = rudder;
            con_data[2] = -1;
            con_data[3] = direction;
            con_data[4] = 0.0;
            con_data[5] = 0.5;
            con_data[6] = -1;
            con_data[7] = FORWARD;
            brakeLED = true;
        }
        float accel = 1.0;
        accelOut = accel;
        // int sensor = 1000*potentiometer;
        // PCserial.printf("sensor = %d\n\r",sensor);


        // Publish ros topic into serial
        // r_steer.data = con_data[1];
        // pub_steer.publish(&r_steer);
        // r_accel.data = con_data[0];
        // pub_accel.publish(&r_accel);
        // nh.spinOnce();

        Running[automode](con_data);
       
    }
    
}
