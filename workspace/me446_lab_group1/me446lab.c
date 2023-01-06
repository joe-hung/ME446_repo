#include "math.h"
#include "F28335Serial.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define GRAV        9.81

// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
float offset_Enc2_rad = -0.37;
float offset_Enc3_rad = 0.27;


long mycount = 0;

long arrayindex = 0;
int UARTprint = 0;

//// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;
long errorcount = 0;
long norm_count = 0;

#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[500];

#pragma DATA_SECTION(danvar, ".my_vars")
float danvar = 0;

float xk_1 = 10;
float yk_1 = 0;
float zk_1 = 20;

extern float Main_Kp4; // = 40;
extern float Main_Kp5; // = 40;

extern float Main_Kd4; //  = 1.5;
extern float Main_Kd5; //   = 1.5;

float cosq1 = 0;
float sinq1 = 0;
float cosq2 = 0;
float sinq2 = 0;
float cosq3 = 0;
float sinq3 = 0;

float xk = 0;
float xkdot = 0;
float xkdot_1 = 0;
float xkdot_2 = 0;
float xkdot_3 = 0;
float xkdot_4 = 0;

float yk = 0;
float ykdot = 0;
float ykdot_1 = 0;
float ykdot_2 = 0;
float ykdot_3 = 0;
float ykdot_4 = 0;

float zk = 0;
float zkdot = 0;
float zkdot_1 = 0;
float zkdot_2 = 0;
float zkdot_3 = 0;
float zkdot_4 = 0;

float xdesired = 0.0;
float ydesired = 0.0;
float zdesired = 0.0;

float Fx = 0;
float Fy = 0;
float Fz = 0;

float Kpx = 1;
float Kdx = 0.05;

float Kpy = 1;
float Kdy = 0.05;

float Kpz = 1;
float Kdz = 0.05;
// p = 0.02, d = 0.001 is a proper "weak" value


float JT_11 = 0;
float JT_12 = 0;
float JT_13 = 0;
float JT_21 = 0;
float JT_22 = 0;
float JT_23 = 0;
float JT_31 = 0;
float JT_32 = 0;
float JT_33 = 0;

// Friction Comp
float thetamotor1_old = 0;
float v1 = 0;
float v1old1 = 0;
float v1old2 = 0;

float thetamotor2_old = 0;
float v2 = 0;
float v2old1 = 0;
float v2old2 = 0;

float thetamotor3_old = 0;
float v3 = 0;
float v3old1 = 0;
float v3old2 = 0;

float apos1 = .2513;
float bpos1 = .3637;
float aneg1 = .2477;
float bneg1 = -.2948;
float az1   =3.6;
float u1_comp = 0;

float apos2 = .25*0.85; // Did some tuning?
float bpos2 = .4759*0.85;
float aneg2 = .287*0.85;
float bneg2 = -.5031*0.85;
float az2   =3.6;
float u2_comp = 0;

float apos3 = .1922*0.85;
float bpos3 = .5339*0.85;
float aneg3 = .2132*0.85;
float bneg3 = -.519*0.85;
float az3   =3.6;
float u3_comp = 0;

extern float Main_thetaDH1;
extern float Main_thetaDH2;
extern float Main_thetaDH3;

extern float Main_Enc1_rad;
extern float Main_Enc2_rad;
extern float Main_Enc3_rad;

extern int Main_safetymode;




//trajectory
float vel_desire = 1.0; // inches per second
float dist = 0;
float t_total = 0;

float xdot_desired = 0;
float xddot_desired = 0;

float ydot_desired = 0;
float yddot_desired = 0;

float zdot_desired = 0;
float zddot_desired = 0;

//36 cutoff freq
//float butterden[3] = {1.0000000000e+00,
//  -1.6822731848e+00,
//  7.2624607052e-01};
//
//float butternum[3] = {    1.0993221439e-02,
//  2.1986442878e-02,
//  1.0993221439e-02};


//70 cutoff freq
//float butterden[3]={  1.0000000000e+00,
//  -1.3908952814e+00,
//  5.3719462480e-01};
//
//float butternum[3]={  3.6574835844e-02,
//  7.3149671688e-02,
//  3.6574835844e-02};

//125 cutoff freq
//float butterden[3]={  1.0000000000e+00,
//  -9.4280904158e-01,
//  3.3333333333e-01};
//float butternum[3]={  9.7631072938e-02,
//  1.9526214588e-01,
//  9.7631072938e-02};

//300 cutoff freq
float butterden[3]={    1.0000000000e+00,
    3.6952737735e-01,
    1.9581571266e-01};
float butternum[3]={    3.9133577250e-01,
    7.8267154500e-01,
    3.9133577250e-01};

float xbutter = 0;
float xbutter1 = 0;
float xbutter2 = 0;
float ybutter = 0;
float ybutter1 = 0;
float ybutter2 = 0;
float zbutter = 0;
float zbutter1 = 0;
float zbutter2 = 0;

typedef struct point_tag {
    float x;
    float y;
    float z;
    float thz;
    int mode;
} point;

#define XYZSTIFF 1
#define ZSTIFF 2
#define XZSTIFF 3

//#define NUM_POINTS 22
//point point_array[NUM_POINTS] = { { 10, 0, 20, 0, XYZSTIFF},  // point 0
//                                {7.88, 9.03, 18.86, 0, XYZSTIFF},   // point 1
//                                {11.85, 9.40, 12, 0, XYZSTIFF},   // point 2
//                                {11.85, 9.40, 7.1, 0, XYZSTIFF}, // point 3   peg hole
//                                {11.85, 9.4, 5.00, 0, ZSTIFF},  // point 4 x and y week  peg hole
//                                {11.85, 9.4, 7.1, 0, ZSTIFF},  // point 5 x and y week   peg hole
//                                {11.85, 9.4, 12, 0, XYZSTIFF},  // point 6 peg hole
//                                {8.93, 6.39, 10.75, 0, XYZSTIFF},  // point 7  avoid obstacle
//                                {9, 3.68, 9.7, 0, XYZSTIFF},  // point 8 avoid obstacle
//                                {9.14, 0.82, 9.7, 0, XYZSTIFF},  // point 9 avoid obstacle
//                                {10.76, -.97, 7.6, 0, XYZSTIFF},  // point 10 begin zig zag
//                                {12.68, -.04, 7.6, PI/4, XZSTIFF},  // point 11  zig zag
//                                {15.27, -1.96, 7.6, -PI/4, XZSTIFF},  // point 12 zig zag
//                                {17.04, -.82, 7.6, PI/4, XZSTIFF},  // point 13 zig zag
//                                {17.04, -.82, 15, 0, XYZSTIFF},  // point 14 up
//                                {13.72, -6.06, 14, 0, XYZSTIFF},  // point 15 top egg
//                                {13.72, -6.06, 11.5, 0, XYZSTIFF},  // point 16 push egg
//                                {13.72, -6.06, 11.5, 0, XYZSTIFF},  // point 17 push egg
//                                {13.72, -6.06, 14, 0, XYZSTIFF},  // point 18 top egg
//                                {10, 0, 20, 0, XYZSTIFF},  // point 19
//                                {10, 0, 20, 0, XYZSTIFF},  // point 20
//                                {10, 0, 20, 0, XYZSTIFF}  // point 21
//                               };

#define NUM_POINTS 24
point point_array[NUM_POINTS] = { { 6.5, 0, 17, 0, XYZSTIFF},  // point 0
        { 10, 0, 20, 0, XYZSTIFF},  // point 0
        {7.88, 9.03, 18.86, 0, XYZSTIFF},   // point 1
        {11.85, 9.40, 12, 0, XYZSTIFF},   // point 2
        {11.85, 9.40, 7.1, 0, XYZSTIFF}, // point 3   peg hole
        {11.85, 9.4, 5.00, 0, XYZSTIFF},  // point 4 x and y week  peg hole
        {11.85, 9.4, 7.1, 0, XYZSTIFF},  // point 5 x and y week   peg hole
        {11.85, 9.4, 12, 0, XYZSTIFF},  // point 6 peg hole
        {8.93, 6.39, 10.75, 0, XYZSTIFF},  // point 7  avoid obstacle
        {9, 3.68, 9.7, 0, XYZSTIFF},  // point 8 avoid obstacle
        {9.14, 0.82, 9.7, 0, XYZSTIFF},  // point 9 avoid obstacle
        {10.76, -.97, 7.6, 0, XYZSTIFF},  // point 10 begin zig zag
        {12.68, -.04, 7.6, PI/4, XYZSTIFF},  // point 11  zig zag
        {15.27, -1.96, 7.6, -PI/4, XYZSTIFF},  // point 12 zig zag
        {17.04, -.82, 7.6, PI/4, XYZSTIFF},  // point 13 zig zag
        {17.04, -.82, 15, 0, XYZSTIFF},  // point 14 up
        {13.72, -6.06, 14, 0, XYZSTIFF},  // point 15 top egg
        {13.72, -6.06, 11.5, 0, XYZSTIFF},  // point 16 push egg
        {13.72, -6.06, 11.5, 0, XYZSTIFF},  // point 17 push egg
        {13.72, -6.06, 14, 0, XYZSTIFF},  // point 18 top egg
        {10, 0, 20, 0, XYZSTIFF},  // point 19
        { 6.5, 0, 17, 0, XYZSTIFF},  // point 0
        { 6.5, 0, 17, 0, XYZSTIFF},  // point 0
        { 6.5, 0, 17, 0, XYZSTIFF},  // point 0
};


//#define NUM_POINTS 24
//point point_array[NUM_POINTS] = { { 10, 0, 20, 0, XYZSTIFF},  // point 0
//                                {7.88, 9.03, 18.86, 0, XYZSTIFF},   // point 1
//                                {11.85, 9.40, 12, 0, XYZSTIFF},   // point 2
//                                {11.85, 9.40, 7.1, 0, XYZSTIFF}, // point 3   peg hole
//                                {11.85, 9.4, 5.00, 0, XYZSTIFF},  // point 4 x and y week  peg hole
//                                {11.85, 9.4, 7.1, 0, XYZSTIFF},  // point 5 x and y week   peg hole
//                                {11.85, 9.4, 12, 0, XYZSTIFF},  // point 6 peg hole
//                                {8.93, 6.39, 10.75, 0, XYZSTIFF},  // point 7  avoid obstacle
//                                {9, 3.68, 9.7, 0, XYZSTIFF},  // point 8 avoid obstacle
//                                {9.14, 0.82, 9.7, 0, XYZSTIFF},  // point 9 avoid obstacle
//                                {10.76, -.97, 7.6, 0, XYZSTIFF},  // point 10 begin zig zag
//                                {12.68, -.04, 7.6, 0, XYZSTIFF},  // point 11  zig zag
//                                {15.27, -1.96, 7.6, 0, XYZSTIFF},  // point 12 zig zag
//                                {17.04, -.82, 7.6, 0, XYZSTIFF},  // point 13 zig zag
//                                {17.04, -.82, 15, 0, XYZSTIFF},  // point 14 up
//                                //{13.72, -6.06, 14, 0, XYZSTIFF},  // point 15 top egg
//                                {13.72, -.82, 15, 0, XYZSTIFF},  // point 15 top egg
//                                {13.72, -.82, 14, 0, XYZSTIFF},  // point 15 top egg
//                                {13.72, -6.06, 14, 0, XYZSTIFF},  // point 15 top egg
//                                {13.72, -6.06, 11.5, 0, XYZSTIFF},  // point 16 push egg
//                                {13.72, -6.06, 11.5, 0, XYZSTIFF},  // point 17 push egg
//                                {13.72, -6.06, 14, 0, XYZSTIFF},  // point 18 top egg
//                                {10, 0, 20, 0, XYZSTIFF},  // point 19
//                                {10, 0, 20, 0, XYZSTIFF},  // point 20
//                                {10, 0, 20, 0, XYZSTIFF}  // point 21
//                               };
float deltax = 0;
float deltay = 0;
float deltaz = 0;
int pointindex = 0;
float time = 0;
int frompoint = 0;
int topoint = 0;
float starttime = 0;
// end trajectory vars


long int deltatime = 0;
long int begin_time = 0;
long int end_time = 0;
long int maxdelta = 0;


float cosz = 0;
float sinz = 0;
float cosx = 0;
float sinx = 0;
float cosy = 0;
float siny = 0;
float thetaz = 0;
float thetax = 0;
float thetay = 0;
float R11 = 0;
float R12 = 0;
float R13 = 0;
float R21 = 0;
float R22 = 0;
float R23 = 0;
float R31 = 0;
float R32 = 0;
float R33 = 0;

float RT11 = 0;
float RT12 = 0;
float RT13 = 0;
float RT21 = 0;
float RT22 = 0;
float RT23 = 0;
float RT31 = 0;
float RT32 = 0;
float RT33 = 0;


// This function is called every 1 ms
void lab(float thetamotor1,float thetamotor2,float thetamotor3,float *tau1,float *tau2,float *tau3, int error) {
    // Calculate Trajectory

//    begin_time = CLK_gethtime();


    time = mycount*0.001;
    if (((time-starttime) + 0.0005 ) >= t_total) {  // 0.0005 added to help with float numerical issues in if statements
        // if true we need to start new trajectory to new points
        // Since there is no/little overshoot, the more  way would be use position feedback as switch
        frompoint = pointindex;
        topoint = pointindex+1;
        if (topoint == NUM_POINTS) {
            topoint = 0;
            pointindex = 0;
        } else {
            pointindex++;
        }

        thetaz = point_array[topoint].thz;

        if (point_array[topoint].mode == ZSTIFF) {

            //Main_Kp4 = 4.0;
            //Main_Kd4 = .15;

            //Main_Kp5 = 4.0;
            //Main_Kd5 = 0.15;

            Kpx = .1;
            Kdx = 0.01;
            Kpy = 0.1;
            Kdy = 0.004;
            Kpz = 1;
            Kdz = .05;
            vel_desire = 0.5;
        } else if (point_array[topoint].mode == XYZSTIFF) {
            vel_desire = 5;
//          Main_Kp4 = 40.0;
//          Main_Kd4 = 1.5;
//          Main_Kp5 = 40.0;
//          Main_Kd5 = 1.5;

            Kpx = 1.5;
            Kdx = 0.025;
            Kpy = 1.5;
            Kdy = 0.025;
            Kpz = 1.5;
            Kdz = 0.025;
        } else if (point_array[topoint].mode == XZSTIFF) {
            Kpx = 1.5;
            Kdx = 0.05;
            Kpy = 0.1;
            Kdy = 0.004;
            Kpz = 1.1;
            Kdz = .05;
            vel_desire = 0.5;
        }
        starttime = time;
        deltax = point_array[topoint].x-point_array[frompoint].x;
        deltay = point_array[topoint].y-point_array[frompoint].y;
        deltaz = point_array[topoint].z-point_array[frompoint].z;
        dist = sqrt(deltax*deltax + deltay*deltay + deltaz*deltaz);
        if (fabs(dist) < 0.01) {
            t_total = 1.0;
            dist = 0.01;
            deltax = 0;
            deltay = 0;
            deltaz = 0;
        } else {
            t_total = dist/vel_desire;
        }
        xdesired = (vel_desire*deltax/dist)*(time-starttime) + point_array[frompoint].x;
        xdot_desired = (vel_desire*deltax/dist);
        ydesired = (vel_desire*deltay/dist)*(time-starttime) + point_array[frompoint].y;
        ydot_desired = (vel_desire*deltay/dist);
        zdesired = (vel_desire*deltaz/dist)*(time-starttime) + point_array[frompoint].z;
        zdot_desired = (vel_desire*deltaz/dist);

    } else {
        xdesired = (vel_desire*deltax/dist)*(time-starttime) + point_array[frompoint].x;
        xdot_desired = (vel_desire*deltax/dist);
        ydesired = (vel_desire*deltay/dist)*(time-starttime) + point_array[frompoint].y;
        ydot_desired = (vel_desire*deltay/dist);
        zdesired = (vel_desire*deltaz/dist)*(time-starttime) + point_array[frompoint].z;
        zdot_desired = (vel_desire*deltaz/dist);
    }
    // End Calculate Trajectory

// remove after debug
//  xdesired = 15;
//  xdot_desired = 0;
//  ydesired = 2;
//  ydot_desired = 0;
//  zdesired = 10;
//  zdot_desired = 0;
//  Main_Kp4 = 40.0;
//  Main_Kd4 = 1.5;
//  Main_Kp5 = 40.0;
//  Main_Kd5 = 1.5;
//  Kpx = 0.05;
//  Kdx = 0.001;
//  Kpy = 0.8;
//  Kdy = 0.03;
//  Kpz = 0.8;
//  Kdz = 0.03;
// remove after debug


    // ---------- Vel. calculation for Motor #1 ----------
    v1 = (thetamotor1 - thetamotor1_old)/.001;
    v1 = (v1 + v1old1 + v1old2)/3.0;

    v1old2 = v1old1;
    v1old1 = v1;
    thetamotor1_old = thetamotor1;

    // ---------- PD Control for Motor #2 ----------
    v2 = (thetamotor2 - thetamotor2_old)/.001;
    v2 = (v2 + v2old2 + v2old2)/3.0; // Averaging filter

    v2old2 = v2old1;
    v2old1 = v2;
    thetamotor2_old = thetamotor2;

    // ---------- PD Control for Motor #3 ----------
    v3 = (thetamotor3 - thetamotor3_old)/.001;
    v3 = (v3 + v3old1 + v3old2)/3.0;

    v3old2 = v3old1;
    v3old1 = v3;
    thetamotor3_old = thetamotor3;


    //-------friction compensation for motor #1----
    if (v1 > 0.1){
        u1_comp = apos1*v1 + bpos1;
    }
    else if(v1<-0.1)
    {
        u1_comp = aneg1*v1 + bneg1;
    }
    else
    {
        u1_comp=az1*v1;
    }

//  if (v1 > 0.0) {
//      u1_comp = apos1*v1 + bpos1;
//  }
//  else {
//      u1_comp = aneg1*v1 + bneg1;
//  }


    //-------friction compensation for motor #2----
    if (v2 > 0.05){
        u2_comp = apos2*v2 + bpos2;
    }
    else if(v2<-0.05)
    {
        u2_comp = aneg2*v2 + bneg2;
    }
    else
    {
        //u2_comp=0;
        u2_comp=az2*v2;
    }

//  //-------friction compensation for motor #2----
//  if (v2 > 0){
//      u2_comp = apos2*v2 + bpos2;
//  }
//  else {
//      u2_comp = aneg2*v2 + bneg2;
//  }


    //-------friction compensation for motor #3----
    if (v3 > 0.05){
        u3_comp = apos3*v3 + bpos3;
    }
    else if(v3<-0.05)
    {
        u3_comp = aneg3*v3 + bneg3;
    }
    else
    {
//      u3_comp=0;
        u3_comp=az3*v3;
    }

//  if (v3 > 0.0){
//      u3_comp = apos3*v3 + bpos3;
//  }
//  else {
//      u3_comp = aneg3*v3 + bneg3;
//  }


    // Rotation zxy
    cosz = cos(thetaz);
    sinz = sin(thetaz);
    cosx = cos(thetax);
    sinx = sin(thetax);
    cosy = cos(thetay);
    siny = sin(thetay);

    RT11 = R11 = cosz*cosy-sinz*sinx*siny;
    RT21 = R12 = -sinz*cosx;
    RT31 = R13 = cosz*siny+sinz*sinx*cosy;
    RT12 = R21 = sinz*cosy+cosz*sinx*siny;
    RT22 = R22 = cosz*cosx;
    RT32 = R23 = sinz*siny-cosz*sinx*cosy;
    RT13 = R31 = -cosx*siny;
    RT23 = R32 = sinx;
    RT33 = R33 = cosx*cosy;



    // Impedance Ctrl
    cosq1 = cos(thetamotor1);
    sinq1 = sin(thetamotor1);
    cosq2 = cos(thetamotor2);
    sinq2 = sin(thetamotor2);
    cosq3 = cos(thetamotor3);
    sinq3 = sin(thetamotor3);

    //Jacobian transpose
//  J_upper = {
//   {-10 Sin[q1] (Cos[q3] + Sin[q2]), 10 Cos[q1] (Cos[q2] - Sin[q3]), -10 Cos[q1] Sin[q3]},
//   {10 Cos[q1] (Cos[q3] + Sin[q2]), 10 Sin[q1] (Cos[q2] - Sin[q3]), -10 Sin[q1] Sin[q3]},
//   {0, -10 (Cos[q3] + Sin[q2]), -10 Cos[q3]}
//  }
    JT_11 = -10*sinq1*(cosq3 + sinq2);
    JT_12 = 10*cosq1*(cosq3 + sinq2);
    JT_13 = 0;
    JT_21 = 10*cosq1*(cosq2 - sinq3);
    JT_22 = 10*sinq1*(cosq2 - sinq3);
    JT_23 = -10*(cosq3 + sinq2);
    JT_31 = -10*cosq1*sinq3;
    JT_32 = -10*sinq1*sinq3;
    JT_33 = -10*cosq3;


    // trajectory of x,y,z desired
    // right now constant

    // forward kinematics
    xk = 10*cosq1*(cosq3 + sinq2);
    yk = 10*sinq1*(cosq3 + sinq2);
    zk = 10*(1+ cosq2 - sinq3);

    // calculate task space velocities
    xkdot = (xk - xk_1)*1000.0;
    //xkdot = (xkdot + xkdot_1 + xkdot_2)/3.0;
    //xkdot = (xkdot + xkdot_1)/2.0;
    //xkdot = (xkdot + xkdot_1 + xkdot_2 + xkdot_3 + xkdot_4)/5.0;

        xbutter = (xkdot*butternum[0] + xkdot_1*butternum[1] + xkdot_2*butternum[2] - xbutter1*butterden[1]-xbutter2*butterden[2]);

        xbutter2 = xbutter1;
        xbutter1 = xbutter;

    ykdot = (yk - yk_1)*1000.0;
    //ykdot = (ykdot + ykdot_1)/2.0;
    //ykdot = (ykdot + ykdot_1 + ykdot_2)/3.0;
    //ykdot = (ykdot + ykdot_1 + ykdot_2 + ykdot_3 + ykdot_4)/5.0;

    ybutter = (ykdot*butternum[0] + ykdot_1*butternum[1] + ykdot_2*butternum[2] - ybutter1*butterden[1]-ybutter2*butterden[2]);

    ybutter2 = ybutter1;
    ybutter1 = ybutter;



    zkdot = (zk - zk_1)*1000.0;
    //zkdot = (zkdot + zkdot_1)/2.0;
    //zkdot = (zkdot + zkdot_1 + zkdot_2)/3.0;
    //zkdot = (zkdot + zkdot_1 + zkdot_2 + zkdot_3 + zkdot_4)/5.0;

    zbutter = (zkdot*butternum[0] + zkdot_1*butternum[1] + zkdot_2*butternum[2] - zbutter1*butterden[1]-zbutter2*butterden[2]);

    zbutter2 = zbutter1;
    zbutter1 = zbutter;









    //Fx = Kpx*(xdesired - xk) + Kdx*(xdot_desired - xkdot);
    //Fy = Kpy*(ydesired - yk) + Kdy*(ydot_desired - ykdot);
    //Fz = Kpz*(zdesired - zk) + Kdz*(zdot_desired - zkdot);


//    Fx = Kpx*( RT11*(xdesired - xk) + RT12*(ydesired - yk) + RT13*(zdesired - zk) )
//      + Kdx*( RT11*(xdot_desired - xkdot) + RT12*(ydot_desired - ykdot) + RT13*(zdot_desired - zkdot) );
//    Fy = Kpy*( RT21*(xdesired - xk) + RT22*(ydesired - yk) + RT23*(zdesired - zk) )
//      + Kdy*( RT21*(xdot_desired - xkdot) + RT22*(ydot_desired - ykdot) + RT23*(zdot_desired - zkdot) );
//  Fz = Kpz*( RT31*(xdesired - xk) + RT32*(ydesired - yk) + RT33*(zdesired - zk) )
//      + Kdz*( RT31*(xdot_desired - xkdot) + RT32*(ydot_desired - ykdot) + RT33*(zdot_desired - zkdot) );


    //Using butterworth
    Fx = Kpx*( RT11*(xdesired - xk) + RT12*(ydesired - yk) + RT13*(zdesired - zk) )
        + Kdx*( RT11*(xdot_desired - xbutter) + RT12*(ydot_desired - ybutter) + RT13*(zdot_desired - zbutter) );
    Fy = Kpy*( RT21*(xdesired - xk) + RT22*(ydesired - yk) + RT23*(zdesired - zk) )
        + Kdy*( RT21*(xdot_desired - xbutter) + RT22*(ydot_desired - ybutter) + RT23*(zdot_desired - zbutter) );
    Fz = Kpz*( RT31*(xdesired - xk) + RT32*(ydesired - yk) + RT33*(zdesired - zk) )
        + Kdz*( RT31*(xdot_desired - xbutter) + RT32*(ydot_desired - ybutter) + RT33*(zdot_desired - zbutter) );

    if (mycount == 0) {
        *tau1 = 0;
        *tau2 = 0;
        *tau3 = 0;

    } else {
        //*tau1 = JT_11*Fx + JT_12*Fy + JT_13*Fz + u1_comp;
        //*tau2 = JT_21*Fx + JT_22*Fy + JT_23*Fz + u2_comp;
        //*tau3 = JT_31*Fx + JT_32*Fy + JT_33*Fz + u3_comp;

        *tau1 = (JT_11*R11+JT_12*R21+JT_13*R31)*Fx + (JT_11*R12+JT_12*R22+JT_13*R32)*Fy + (JT_11*R13+JT_12*R23+JT_13*R33)*Fz + u1_comp;
        *tau2 = (JT_21*R11+JT_22*R21+JT_23*R31)*Fx + (JT_21*R12+JT_22*R22+JT_23*R32)*Fy + (JT_21*R13+JT_22*R23+JT_23*R33)*Fz + u2_comp;
        *tau3 = (JT_31*R11+JT_32*R21+JT_33*R31)*Fx + (JT_31*R12+JT_32*R22+JT_33*R32)*Fy + (JT_31*R13+JT_32*R23+JT_33*R33)*Fz + u3_comp;
    }

    // also add friction comp.

    // Gravity term
    // save past states
    xk_1 = xk;
    xkdot_4 = xkdot_3;
    xkdot_3 = xkdot_2;
    xkdot_2 = xkdot_1;
    xkdot_1 = xkdot;

    yk_1 = yk;
    ykdot_4 = ykdot_3;
    ykdot_3 = ykdot_2;
    ykdot_2 = ykdot_1;
    ykdot_1 = ykdot;

    zk_1 = zk;
    zkdot_4 = zkdot_3;
    zkdot_3 = zkdot_2;
    zkdot_2 = zkdot_1;
    zkdot_1 = zkdot;

    Simulink_PlotVar1 = thetamotor1;
    Simulink_PlotVar2 = thetamotor2;
    Simulink_PlotVar3 = thetamotor3;


    if ((mycount%5)==0) {

        theta1array[arrayindex] = thetamotor1;

        if (arrayindex >= 500) {
            arrayindex = 0;
        } else {
            arrayindex++;
        }

    }

//    end_time = CLK_gethtime();
//    deltatime = end_time - begin_time;
//    if (deltatime > maxdelta) {
//        maxdelta = deltatime;
//    }

    if ((mycount%500)==0) {

        UARTprint = 1;
        //serial_printf(&SerialA, "%.2f %.2f,%.2f\n\r",thetamotor1,thetamotor2,thetamotor3);
        //          serial_printf(&SerialA, "%.2f %.2f,%.2f       x%.2f y%.2f,z%.2f    %ld %ld\n\r",Main_thetaDH1,Main_thetaDH2,Main_thetaDH3,xk,yk,zk,deltatime,maxdelta);
        //          serial_printf(&SerialB, "%.2f %.2f,%.2f       x%.2f y%.2f,z%.2f    %ld %ld\n\r",Main_thetaDH1,Main_thetaDH2,Main_thetaDH3,xk,yk,zk,deltatime,maxdelta);
        //          serial_printf(&SerialC, "%.2f %.2f,%.2f       x%.2f y%.2f,z%.2f    %ld %ld\n\r",Main_thetaDH1,Main_thetaDH2,Main_thetaDH3,xk,yk,zk,deltatime,maxdelta);
        //          serial_printf(&SerialA, "%.2f\n\r",Main_thetaDH1);
        //          serial_printf(&SerialB, "%.2f\n\r",Main_thetaDH1);
        //          serial_printf(&SerialC, "%.2f\n\r",Main_thetaDH1);


        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
        //      GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Control Card
        maxdelta = 0;
    }

    mycount++;
    errorcount++;
    norm_count++;

//  *tau1 = 0;
//  *tau2 = 0;
//  *tau3 = 0;

    if(Main_safetymode == 1)
    {
        if(errorcount == 200)
            GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1;
        if(errorcount == 250)
            GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1;
        if(errorcount == 300)
            GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1;
        if(errorcount >= 350)
        {
            GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1;
            errorcount = 0;
        }

    } else {
        if(norm_count == 700)
            GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1;
        if(norm_count == 750)
            GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1;
        if(norm_count == 800)
            GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1;
        if(norm_count >= 850)
        {
            GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1;
            norm_count = 0;
        }
    }



}

void printing(void){
    serial_printf(&SerialA, "%.2f %.2f,%.2f       x%.2f y%.2f,z%.2f    %ld %ld\n\r",Main_thetaDH1,Main_thetaDH2,Main_thetaDH3,xk,yk,zk,deltatime,maxdelta);
}
