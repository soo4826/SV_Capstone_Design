#include "mbed.h"
#include "PwmIn.h"

//DigitalOut motorController(p26); //컨트롤러 스위치얘도 딱히안해도
//AnalogIn hallSensor(p15); //모터 홀센서 할수이/ㅅ는거 음슴

#define STEP_THRESH 0.2

#define DUTYMAX 0.0863
#define DUTYMIN 0.0498

#define FORWARD  1
#define BACKWARD 0

Serial PCserial(USBTX,USBRX);

DigitalOut automodeLED(LED1) ;
DigitalOut directionLED(LED2);
DigitalOut brakeLED(LED3);



PwmIn thro(p26);
PwmIn alie(p21);
PwmIn elev(p22);
PwmIn rudd(p23);
PwmIn gear(p24);
PwmIn aux1(p25);

DigitalOut EnaOut(p27);
DigitalOut DirOut(p28);
DigitalOut PulOut(p29);

AnalogIn steerIn(p17);

AnalogOut accelOut(p18); //모터 스로틀, 초록선, 0~1
DigitalOut brakeOut(p19); //브레이크
DigitalOut directionOut(p20);//전진

Ticker Pulse;
Timeout stepTimeout;

float steering_value = 0.0;

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
    float potentiometer_value = 0;//potentiometer.read();
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
    float center = 0.0;
    float coeff = 1;//1/3600;
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


int main(void){
    
    float accel = 0.0;
    float steer = 0.5;
    bool  brake = false;
    bool  automode = false;
    bool  direction = FORWARD;
    
    float throttle = 0.0;
    float aileron = 0.5;
    float rudder = 0.5;
    float elevator = 0.5;
    float gearmode = 0.0;
    float auxmode = 0.0;

    float con_data[4] = {0.0, 0, 0.0, 0};//{ControllerAcc, ControllerSteer, RosAcc, RosSteer}

    // serialOut.attach(&serialAct, 0.1);
    
    Pulse.attach(&step,0.0015);

    void (*Running[2])(float*);
    Running[0] = ControllerRun;
    Running[1] = RosRun;
    
    while(1){
//         throttle = thro.dutycycle();
//         aileron  = alie.dutycycle();
//         rudder   = rudd.dutycycle();
//         elevator = elev.dutycycle();
//         gearmode = gear.dutycycle();
//         auxmode  = aux1.dutycycle();
        
//         throttle = (throttle - DUTYMIN)/(DUTYMAX-DUTYMIN);
//         aileron  = (aileron - DUTYMIN)/(DUTYMAX-DUTYMIN);
//         rudder   = (rudder - DUTYMIN)/(DUTYMAX-DUTYMIN);
//         elevator = (elevator - DUTYMIN)/(DUTYMAX-DUTYMIN);
//         gearmode = (gearmode - DUTYMIN)/(DUTYMAX-DUTYMIN);
//         auxmode = (auxmode - DUTYMIN)/(DUTYMAX-DUTYMIN);
        
//         if(throttle > 1.0){throttle = 1.0;}
//         else if(throttle < 0.0){throttle = 0.0;}
        
//         if(aileron > 1.0){aileron = 1.0;}
//         else if(aileron < 0.0){aileron = 0.0;}
        
//         if(rudder > 1.0){rudder = 1.0;}
//         else if(rudder < 0.0){rudder = 0.0;}
        
//         if(elevator > 1.0){elevator = 1.0;}
//         else if(elevator < 0.0){elevator = 0.0;}
        
//         if(gearmode > 1.0){gearmode = 1.0;}
//         else if(gearmode < 0.0){gearmode = 0.0;}
        
//         if(auxmode > 1.0){auxmode = 1.0;}
//         else if(auxmode < 0.0){auxmode = 0.0;}
        
// //        PCserial.printf("thro = %.2f\t", throttle);
// //        PCserial.printf("alie = %.2f\t", aileron);
// //        PCserial.printf("elev = %.2f\t", rudder);
// //        PCserial.printf("rudd = %.2f\t", elevator);
// //        PCserial.printf("gear = %.2f\t", gearmode);
// //        PCserial.printf("aux1 = %.2f\n\n\r", auxmode);
// ////////////////////////////////////////////////////////////////////////////////

//         if(auxmode > 0.5){automode = true ;  automodeLED = true ; }
//         else              {automode = false; automodeLED = false; }
        
        
        
//         if(gearmode > 0.5){direction = FORWARD ; directionLED = false;}
//         else              {direction = BACKWARD; directionLED = true;}
        
//         if(automode == false){
//             if(elevator > 0.5){
//                 accel = (elevator - 0.5)*2.0;
//                 brake = false;
//                 brakeLED = false;
//             }
//             else if(elevator < 0.5 && elevator > 0.4){
//                 accel = 0.0;
//                 brake = false;
//                 brakeLED = false;
//             }
//             else{
//                 accel = 0.0;
//                 brake = true;
//                 brakeLED = true;
//             }
//         }
//         else{
//             accel = 0.0;
//             brake = true;
//             brakeLED = true;
//         }

        
//         steer = 2.0*(rudder-0.5);
//         con_data[1]=steer;
        int sensor = 1000*steerIn;
        PCserial.printf("steer = %d\n\r", sensor);

//         accelOut = accel;
//         brakeOut = brake;
//         directionOut = direction;

//         Running[automode](con_data);
    }
    
}







