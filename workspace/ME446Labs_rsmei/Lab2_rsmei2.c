#include "math.h"
#include "F28335Serial.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define GRAV        9.81

// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
float offset_Enc2_rad = -0.454134671;//0;
float offset_Enc3_rad = 0.24347343;//0;


// Your global varialbes.

long mycount = 0;

#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 0.0;

#pragma DATA_SECTION(printthis, ".my_vars")
float printthis = 0.0;

#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[100];

#pragma DATA_SECTION(theta2array, ".my_arrs")
float theta2array[100];

long arrayindex = 0;
int UARTprint = 0;

float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;

float printtheta1DH = 0;
float printtheta2DH = 0;
float printtheta3DH = 0;

float printtheta1_IK = 0;
float printtheta2_IK = 0;
float printtheta3_IK = 0;

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;

// Coefficient of DH params from motor
float C1 = 0;
float C2 = 1;
float C3 = PI/2.0;

// position of end-effector
float X =0;
float Y =0;
float Z =0;
float temp_theta3 = 0;

float Kp1 = 20;
float Kp2 = 20;
float Kp3 = 20;

float Kd1 = 1.5;
float Kd2 = 1.5;
float Kd3 = 1.5;

float error1 = 0;
float error1_old = 0;

float error2 = 0;
float error2_old = 0;

float error3 = 0;
float error3_old = 0;

float theta1_desire = 0;
float theta2_desire = 0;
float theta3_desire = 0;

float theta1_old = 0;
float theta2_old = 0;
float theta3_old = 0;

float theta1_dot = 0;
float theta1_dot_f = 0;
float theta1_dot_old = 0;
float theta1_dot_oldold = 0;

float theta2_dot = 0;
float theta2_dot_f = 0;
float theta2_dot_old = 0;
float theta2_dot_oldold = 0;

float theta3_dot = 0;
float theta3_dot_f = 0;
float theta3_dot_old = 0;
float theta3_dot_oldold = 0;


// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {


//    *tau1 = 0;
//    *tau2 = 0;
//    *tau3 = 0;

    if ((mycount%1000)==0) {
            if(theta1_desire > 0.1)
                theta1_desire = 0;
            else
                theta1_desire = 0.25;

            if(theta2_desire > 0.1)
                theta2_desire = 0;
            else
                theta2_desire = 0.25;

            if(theta3_desire > 0.1)
                theta3_desire = 0;
            else
                theta3_desire = 0.25;

        }


    //Motor torque limitation(Max: 5 Min: -5)

    // save past states
    if ((mycount%50)==0) {

        theta1array[arrayindex] = theta1motor;
        theta2array[arrayindex] = theta2motor;

        if (arrayindex >= 100) {
            arrayindex = 0;
        } else {
            arrayindex++;
        }
    }

    if ((mycount%500)==0) {
        UARTprint = 1;
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
        GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box
    }

    // state
    theta2_dot = (theta2motor - theta2_old) / 0.001;
    theta2_dot_f = (theta2_dot + theta2_dot_old + theta2_dot_oldold)/3.0;
    theta2_dot_oldold = theta2_dot_old;
    theta2_dot_old = theta2_dot_f;
    theta2_old = theta2motor;


    theta2_dot = (theta2motor - theta2_old) / 0.001;
    theta2_dot_f = (theta2_dot + theta2_dot_old + theta2_dot_oldold)/3.0;
    theta2_dot_oldold = theta2_dot_old;
    theta2_dot_old = theta2_dot_f;
    theta2_old = theta2motor;

    theta3_dot = (theta3motor - theta3_old) / 0.001;
    theta3_dot_f = (theta3_dot + theta3_dot_old + theta3_dot_oldold)/3.0;
    theta3_dot_oldold = theta3_dot_old;
    theta3_dot_old = theta3_dot_f;
    theta3_old = theta3motor;




    error2 = theta2_desire - theta2motor;
    *tau2 = Kp2 * error2 - Kd2 * theta2_dot_f;

    error3 = theta3_desire - theta3motor;
    *tau3 = Kp3 * error3 - Kd3 * theta3_dot_f;




    printtheta1motor = theta1motor;
    printtheta2motor = theta2motor;
    printtheta3motor = theta3motor;

    // DH angles
    printtheta1DH = theta1motor;
    printtheta2DH = theta2motor - PI/2.0;
    printtheta3DH = theta3motor - theta2motor + PI/2.0;

    Simulink_PlotVar1 = theta1motor;
    Simulink_PlotVar2 = theta2motor;
    Simulink_PlotVar3 = theta3motor;
    Simulink_PlotVar4 = theta2_desire;

    float theta_1 = printtheta1DH;
    float theta_2 = printtheta2DH;
    float theta_3 = printtheta3DH;

    // forward kinematics
    X = (127*cos(theta_1)*(cos(theta_2 + theta_3) + cos(theta_2)))/500.0;
    Y = (127*sin(theta_1)*(cos(theta_2 + theta_3) + cos(theta_2)))/500.0;
    Z = 127/500.0 - (127*sin(theta_2))/500.0 - (127*sin(theta_2 + theta_3))/500.0;

    // inverse kinematic for DH angle
    float theta1_DH = atan2(Y,X);
    float theta2_DH;
    float D = (X*X + Y*Y + (Z - 0.254)*(Z - 0.254) - 2*0.254*0.254)/(2*0.254*0.254);
    float theta3_DH;


    // choose elbow-up solution
    temp_theta3 = -atan2(sqrt(1-D*D),D);                                        // first solution, using +sin(-theta3DH)
    if (temp_theta3 > 0)                                                        // if the first solution we found is a positive angle
        theta3_DH = temp_theta3;                                                // let our IK solution to be the first
    else
        theta3_DH = -atan2(-sqrt(1-D*D),D);                                     // second solution, using -sin(-theta3DH)
    theta2_DH = -(atan2(Z-0.254,sqrt(X*X+Y*Y)) + atan2(0.254*sin(theta3_DH),0.254*cos(theta3_DH)+0.254));

    // transition from DH angle to motor angle
    printtheta1_IK = theta1_DH;
    printtheta2_IK = theta2_DH + PI/2.0;
    printtheta3_IK = theta3_DH + printtheta2_IK - PI/2.0;

    mycount++;
}


void printing(void){
    serial_printf(&SerialA, "Motor        %6.2f %6.2f %6.2f   \n\r \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI);
//    serial_printf(&SerialA, "DH           %6.2f %6.2f %6.2f   \n\r \n\r",printtheta1DH*180/PI,printtheta2DH*180/PI,printtheta3DH*180/PI);
    serial_printf(&SerialA, "motor_IK     %6.2f %6.2f %6.2f   \n\r \n\r",printtheta1_IK*180/PI,printtheta2_IK*180/PI,printtheta3_IK*180/PI);
    serial_printf(&SerialA, "End-Effector %6.2f %6.2f %6.2f   \n\r \n\r",X,Y,Z);
}

