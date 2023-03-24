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

float Kp1 = 45;
float Kp1s = 30;
float Kp2 = 2000;
float Kp2s = 40;
float Kp3 = 2500;
float Kp3s = 100;

float Kd1 = 1.5;
float Kd2 = 50;
float Kd3 = 50;

float Kd1s = 2.5;
float Kd2s = 1.5;
float Kd3s = 1.5;
 //Kp3s: short move

float KI1 = 450;
float KI2 = 250;
float KI3 = 400;


float error1 = 0;
float error1_old = 0;
float I1 = 0;
float I1_update = 0;

float error2 = 0;
float error2_old = 0;
float I2 = 0;
float I2_update = 0;

float error3 = 0;
float error3_old = 0;
float I3 = 0;
float I3_update = 0;

float J1 = 0.0167;
float J2 = 0.03;
float J3 = 0.0128;

float theta1_desire = 0;
float theta2_desire = 0;
float theta3_desire = 0;

float theta1_desire_dot = 0;
float theta2_desire_dot = 0;
float theta3_desire_dot = 0;

float theta1_desire_dotdot = 0;
float theta2_desire_dotdot = 0;
float theta3_desire_dotdot = 0;


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

float ethresh1 = 0.02;
float ethresh2 = 0.02;
float ethresh3 = 0.02;

float tau1_temp = 0;
float tau2_temp = 0;
float tau3_temp = 0;

float u_fric1 = 0;
float u_fric2 = 0;
float u_fric3 = 0;

float min_V1 = 0.1;
float min_V2 = 0.05;
float min_V3 = 0.05;

float Viscous_positive1 = 0.18;
float Viscous_positive2 = 0.18;
float Viscous_positive3 = 0.23;

float Viscous_negative1 = 0.16;
float Viscous_negative2 = 0.17;
float Viscous_negative3 = 0.25;

float Coulomb_positive1 = 0.3637;
float Coulomb_positive2 = 0.3;
float Coulomb_positive3 = 0.3;

float Coulomb_negative1 = -0.45;
float Coulomb_negative2 = -0.35;
float Coulomb_negative3 = -0.3;

float slope_bet_min1 = 3.6;
float slope_bet_min2 = 3.6;
float slope_bet_min3 = 3.6;

float p1 = 0.03;
float p2 = 0.0128;
float p3 = 0.0076;
float p4 = 0.0753;
float p5 = 0.0298;

float sin32 = 0;
float cos32 = 0;
float sin2 = 0;
float sin3 = 0;
float cos3 = 0;

float a2 = 0;
float a3 = 0;


void cubic(float t)
{
    float a0 = 0;
    float a1 = 0;
    float a2 = 0;
    float a3 = 0;
    if (t > 0)
    {
        a0 = 0;
        a1 = 0;
        a2 = 1.5;
        a3 = -1;
    }
    if (t > 1)
    {
        a0 = -2;
        a1 = 6;
        a2 = -4.5;
        a3 = 1;
    }

    theta2_desire = a0 + a1*t + a2*t*t +  a3*t*t*t;
    theta2_desire_dot = a1 + 2*a2*t + 3*a3*t*t;
    theta2_desire_dotdot = 2*a2 + 6*a3*t;

    theta1_desire = theta2_desire;
    theta1_desire_dot = theta2_desire_dot;
    theta1_desire_dotdot = theta3_desire_dotdot;

    theta3_desire = theta2_desire;
    theta3_desire_dot = theta2_desire_dot;
    theta3_desire_dotdot = theta3_desire_dotdot;

    if(t > 2)
    {
        theta1_desire = 0;
        theta1_desire_dot = 0;
        theta1_desire_dotdot = 0;

        theta2_desire = 0;
        theta2_desire_dot = 0;
        theta2_desire_dotdot = 0;

        theta3_desire = 0;
        theta3_desire_dot = 0;
        theta3_desire_dotdot = 0;
    }

}

void fun(float t)
{
    float a = 0.15;
    float X = 0.26;
    float Y = 0;
    float Z = 0;

    Y = a*cos(t)/(1+(sin(t)*sin(t)));
    Z = a*(sin(t)*cos(t))/(1+sin(t)*sin(t)) + 0.4;

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

    theta1_desire = theta1_DH;
    theta2_desire = theta2_DH + PI/2.0;
    theta3_desire = theta3_DH + theta2_desire - PI/2.0;

}

// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {


//    *tau1 = 0;
//    *tau2 = 0;
//    *tau3 = 0;

//    if ((mycount%1000)==0) {
//            if(theta1_desire > 0.05)
//                theta1_desire = 0;
//            else
//                theta1_desire = 0.52;
//
//            if(theta2_desire > 0.1)
//                theta2_desire = 0;
//            else
//                theta2_desire = 0.52;// 0.25;
//
//            if(theta3_desire > 0.1)
//                theta3_desire = 0;
//            else
//                theta3_desire = 0.52;//0.25;
//
//        }


//    fun( (mycount%10000)*0.001*2*PI/10 );

    cubic(mycount%2000/1000.0);
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
    theta1_dot = (theta1motor - theta1_old) / 0.001;
    theta1_dot_f = (theta1_dot + theta1_dot_old + theta1_dot_oldold)/3.0;
    theta1_dot_oldold = theta1_dot_old;
    theta1_dot_old = theta1_dot_f;
    theta1_old = theta1motor;


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

//    error1 = theta1_desire - theta1motor;


    error1 = theta1_desire - theta1motor;
//    I1_update = I1 + 0.001*(error1 + error1_old)/2.0;
    if (fabs(error1) <  ethresh1)
    {

//        tau1_temp = Kp1s * error1 - Kd1s * theta1_dot_f;     // + KI1*I1_update;
        tau1_temp = J1*theta1_desire_dotdot + Kp1s * (theta1_desire - theta1motor) + Kd1s*(theta1_desire_dot-theta1_dot_f); // + KI1*I1;
    }
    else
    {
//        tau1_temp = Kp1 * error1 - Kd1 * theta1_dot_f;
        tau1_temp = J1*theta1_desire_dotdot + Kp1s * (theta1_desire - theta1motor) + Kd1s*(theta1_desire_dot-theta1_dot_f);
        I1 = 0;
        I1_update = 0;
    }

    if(theta1_dot_f > min_V1)
    {
        u_fric1 = Viscous_positive1*theta1_dot_f+Coulomb_positive1;
    }
    else if (theta1_dot_f < -min_V1)
    {
        u_fric1 = Viscous_negative1*theta1_dot_f+Coulomb_negative1;
    }
    else
    {
        u_fric1 = slope_bet_min1*theta1_dot_f;
    }

    tau1_temp += u_fric1;


    if (tau1_temp > 5)
    {
        tau1_temp = 5;
        I1_update = I1;
    }
    if (tau1_temp < -5)
    {
        tau1_temp = -5;
        I1_update = I1;
    }
    *tau1 = tau1_temp;
    error1_old = error1;
    I1 = I1_update;


    error2 = theta2_desire - theta2motor;
    I2_update = I2 + 0.001*(error2 + error2_old)/2.0;
//    if (fabs(error2) <  ethresh2)
//    {
////        tau2_temp = Kp2s * error2 - Kd2s * theta2_dot_f + KI2*I2_update;
////        *tau2 = J2*theta2_desire_dotdot + Kp2s * (theta2_desire - theta2motor) + Kd2s*(theta2_desire_dot-theta2_dot_f) + KI2*I2;
//    }
//    else
//    {
//        tau2_temp = Kp2 * error2 - Kd2 * theta2_dot_f;
////        *tau2 = J2*theta2_desire_dotdot + Kp2s * (theta2_desire - theta2motor) + Kd2s*(theta2_desire_dot-theta2_dot_f);
//        I2 = 0;
//        I2_update = 0;
//    }

    a2 = theta2_desire_dotdot + Kp2*(theta2_desire - theta2motor) + Kd2*(theta2_desire_dot-theta2_dot_f);
    a3 = theta3_desire_dotdot + Kp3*(theta3_desire - theta3motor) + Kd3*(theta3_desire_dot-theta3_dot_f);
    sin32 = sin(theta3motor - theta2motor);
    cos32 = cos(theta3motor - theta2motor);
    sin2 = sin(theta2motor);
    sin3 = sin(theta3motor);
    cos3 = cos(theta3motor);

    tau2_temp = p1*a2 - p3*sin32*a3 + 0 - p3*cos32*theta3_dot_f*theta3_dot_f - p4*GRAV*sin2;
    if(theta2_dot_f > min_V2)
    {
        u_fric2 = Viscous_positive2*theta2_dot_f+Coulomb_positive2;
    }
    else if (theta2_dot_f < -min_V2)
    {
        u_fric2 = Viscous_negative2*theta2_dot_f+Coulomb_negative2;
    }
    else
    {
        u_fric2 = slope_bet_min2*theta2_dot_f;
    }

    tau2_temp += u_fric2;

    if (tau2_temp > 5)
    {
        tau2_temp = 5;
        I2_update = I2;
    }
    if (tau2_temp < -5)
    {
        tau2_temp = -5;
        I2_update = I2;
    }
    *tau2 = tau2_temp;
    error2_old = error2;
    I2 = I2_update;


    error3 = theta3_desire - theta3motor;
//    I3_update = I3 + 0.001*(error3 + error3_old)/2.0;
//    if (fabs(error3) <  ethresh3)
//    {
//        tau3_temp = Kp3s * error3 - Kd3s * theta3_dot_f + KI3*I3_update;
////        *tau3 = J3*theta3_desire_dotdot + Kp3s * (theta3_desire - theta3motor) + Kd3s*(theta3_desire_dot-theta3_dot_f) + KI3*I3;
//    }
//    else
//    {
//        tau3_temp = Kp3 * error3 - Kd3 * theta3_dot_f;
////        *tau3 = J3*theta3_desire_dotdot + Kp3s * (theta3_desire - theta3motor) + Kd3s*(theta3_desire_dot-theta3_dot_f);
//        I3 = 0;
//        I3_update = 0;
//    }

    tau3_temp = -p3*sin32*a2 + p2*a3 + p3*cos32*theta2_dot_f*theta2_dot_f + 0 - p5*GRAV*cos3;

    if(theta3_dot_f > min_V3)
    {
        u_fric3 = Viscous_positive3*theta3_dot_f+Coulomb_positive3;
    }
    else if (theta3_dot_f < -min_V3)
    {
        u_fric3 = Viscous_negative3*theta3_dot_f+Coulomb_negative3;
    }
    else
    {
        u_fric3 = slope_bet_min3*theta3_dot_f;
    }


    tau3_temp += u_fric3;

    if (tau3_temp > 5)
    {
        tau3_temp = 5;
        I3_update = I3;
    }
    if (tau3_temp < -5)
    {
        tau3_temp = -5;
        I3_update = I3;
    }
    *tau3 = tau3_temp;
    error3_old = error3;
    I3 = I3_update;



    printtheta1motor = theta1motor;
    printtheta2motor = theta2motor;
    printtheta3motor = theta3motor;

    // DH angles
    printtheta1DH = theta1motor;
    printtheta2DH = theta2motor - PI/2.0;
    printtheta3DH = theta3motor - theta2motor + PI/2.0;

    Simulink_PlotVar1 = theta3motor; //
    Simulink_PlotVar2 = theta3_desire;
    Simulink_PlotVar3 = theta2motor;
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

