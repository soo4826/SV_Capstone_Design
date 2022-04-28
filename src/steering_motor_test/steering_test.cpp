#include "mbed.h"

#define STEP_THRESH 1/3600

//serial
Serial pc(USBTX, USBRX);
Ticker serialOut;
float serialPrint = 0.0;
void serialAct(){
    pc.printf("%f\n", serialPrint);
}

AnalogIn potentiometer(p20);
DigitalOut EnaOut(p27);
DigitalOut DirOut(p28);
DigitalOut PulOut(p29);

Ticker Pulse;
Timeout stepTimeout;

float steering_value = 0.5;

char defineStep(float potentiometer_value){
    float difference = potentiometer_value - steering_value;
    bool InRange = abs(difference) <= STEP_THRESH;
    bool Sign = difference >= 0;
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
    float potentiometer_value = potentiometer.read();
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
    float center = 0.5;
    float coeff = 1/3600;
    return coeff*ang+center;
}

void stepCon(float Angle){
    steering_value = ang2sval(Angle);
}

void RosRun(float *con_data){
    stepCon(con_data[3]);
}

void ControllerRun(float *con_data){
    stepCon(con_data[1]);
}

int main(){
    float con_data[4] = {0.0, 20, 0.0, 10};//{ControllerAcc, ControllerSteer, RosAcc, RosSteer}
    bool automode = false;

    serialOut.attach(&serialAct, 0.1);
    
    Pulse.attach(&step,0.0015);

    void (*Running[2])(float*);
    Running[0] = ControllerRun;
    Running[1] = RosRun;

    while(1){
        Running[automode](con_data);
    }
}