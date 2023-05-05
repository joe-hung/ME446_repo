#include "math.h"
#include "F28335Serial.h"
#include <stdbool.h>
#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define GRAV        9.81


#define NUM_POINTS  20

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


// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;


typedef struct point_struct {
    float x;
    float y;
    float z;
    float v;
    float thz;
    int mode;
} points;

/*
    mode of soft kp kd
    1 -> x y soft
    2 -> y soft
    3 -> z soft
    4 -> all stiff
 */                         //    X          Y           Z        Velocity   rotation(radian)  mode
points waypoints[NUM_POINTS] = {{0.13354,       0,      0.422,      0.15,               0,      4},       // point 1         (home position)
                                {0.14,      0.172,       0.42,      0.15,               0,      4},       // point 1.5
                                {0.03,      0.345,       0.42,      0.15,               0,      4},       // point 2         (above hole)
                                {0.03,      0.345,      0.205,      0.13,               0,      4},       // point 2.5       (above hole)
                                {0.03,      0.345,       0.13,      0.08,               0,      1},       // point 3         (insert into hole)
                                {0.03,      0.345,      0.126,      0.01,               0,      1},       // point 4         (insert into hole)
                                {0.03,      0.345,      0.205,      0.08,               0,      4},       // point 2.5       (above hole)
                                {0.167,     0.139,      0.312,      0.15,               0,      4},       // point 4.5
                                {0.373,     0.114,      0.215,      0.13,               0,      4},       // point 5         (zigzag start)
                                {0.403,      0.06,      0.209,      0.08,    -0.927293432,      2},       // point 5.5(a)
                                {0.391,     0.040,      0.209,      0.08,      0.78539816,      2},       // point 5.5(b)
                                {0.325,     0.050,      0.209,      0.08,     -0.26179939,      2},       // point 6(a)
                                {0.313,     0.034,      0.209,      0.08,      0.78539816,      2},       // point 6(b)
                                {0.375,    -0.043,      0.209,      0.08,    -0.927293432,      2},       // point 7         (zigzag out)
                                {0.375,    -0.043,       0.30,      0.08,               0,      4},       // point 7.5(a)    (above point 7)
                                {0.230,     0.179,       0.34,      0.15,               0,      4},       // point 7.5(b)    (egg above)
                                {0.230,     0.179,      0.286,      0.02,               0,      3},       // point 8         (press egg)
                                {0.230,     0.179,     0.2855,   0.00025,               0,      3},       // point 9         (press egg)
                                {0.230,     0.179,       0.34,      0.15,               0,      4},       // point 10        (egg above)
                                {0.13354,       0,      0.422,      0.15,               0,      4}};      // point 1         (home position)
points prev_point;
points now_point;

// position of end-effector
float X = 0;
float Y = 0;
float Z = 0;

// Kp in world frame
float Kpx = 350;
float Kpy = 350;
float Kpz = 350;

// Kd in world frame
float Kdx = 25;
float Kdy = 25;
float Kdz = 25;

// Kp in rotated frame N
float Kpx_n = 350;
float Kpy_n = 350;
float Kpz_n = 300;


// stiff Kp in rotated frame N
float Kpx_n_h = 450;
float Kpy_n_h = 450;
float Kpz_n_h = 450;

// soft Kp in rotated frame N
float Kpx_n_s = 15;
float Kpy_n_s = 15;
float Kpz_n_s = 50;

// Kd in rotated frame N
float Kdx_n = 25;
float Kdy_n = 25;
float Kdz_n = 25;

// stiff Kd in rotated frame N
float Kdx_n_h = 25;
float Kdy_n_h = 25;
float Kdz_n_h = 25;

// soft Kd in rotated frame N
float Kdx_n_s = 5;
float Kdy_n_s = 5;
float Kdz_n_s = 5;

// desired end-effector position
float x_desire = 0.254;
float y_desire = 0.254;
float z_desire = 0.254;

// desire end-effector velocity
float x_desire_dot = 0;
float y_desire_dot = 0;
float z_desire_dot = 0;

// 1ms ago end-effector position
float x_old = 0;
float y_old = 0;
float z_old = 0;

// un-filtered  end-effector velocity
float x_dot = 0;
float y_dot = 0;
float z_dot = 0;

// filtered  end-effector velocity
float x_dot_f = 0;
float y_dot_f = 0;
float z_dot_f = 0;

// 1ms ago end-effector velocity
float x_dot_old = 0;
float y_dot_old = 0;
float z_dot_old = 0;

// 2ms ago end-effector velocity
float x_dot_oldold = 0;
float y_dot_oldold = 0;
float z_dot_oldold = 0;

// desired joint angle
float theta1_desire = 0;
float theta2_desire = 0;
float theta3_desire = 0;

// desired joint velocity
float theta1_desire_dot = 0;
float theta2_desire_dot = 0;
float theta3_desire_dot = 0;

// desired joint acceleration
float theta1_desire_dotdot = 0;
float theta2_desire_dotdot = 0;
float theta3_desire_dotdot = 0;

// 1ms ago joint angle
float theta1_old = 0;
float theta2_old = 0;
float theta3_old = 0;

// un-filterd joint velocity
float theta1_dot = 0;
float theta2_dot = 0;
float theta3_dot = 0;

// filtered joint velocity
float theta1_dot_f = 0;
float theta2_dot_f = 0;
float theta3_dot_f = 0;

// 1ms ago joint velocity
float theta1_dot_old = 0;
float theta2_dot_old = 0;
float theta3_dot_old = 0;

// 2ms ago joint velocity
float theta1_dot_oldold = 0;
float theta2_dot_oldold = 0;
float theta3_dot_oldold = 0;

// unsaturated torque
float tau1_temp = 0;
float tau2_temp = 0;
float tau3_temp = 0;

// joint friction
float u_fric1 = 0;
float u_fric2 = 0;
float u_fric3 = 0;

// joint friction factor
float u_fric1_adjust = 0.8;
float u_fric2_adjust = 0.8;
float u_fric3_adjust = 0.8;

// joint static friction threshold
float min_V1 = 0.1;
float min_V2 = 0.05;
float min_V3 = 0.05;

// joint Viscous friction
float Viscous_positive1 = 0.18;
float Viscous_positive2 = 0.18;
float Viscous_positive3 = 0.23;

float Viscous_negative1 = 0.16;
float Viscous_negative2 = 0.17;
float Viscous_negative3 = 0.25;

// joint Coulomb friction
float Coulomb_positive1 = 0.3637;
float Coulomb_positive2 = 0.3;
float Coulomb_positive3 = 0.3;

float Coulomb_negative1 = -0.45;
float Coulomb_negative2 = -0.35;
float Coulomb_negative3 = -0.3;

// joint static friction slope
float slope_bet_min1 = 3.6;
float slope_bet_min2 = 3.6;
float slope_bet_min3 = 3.6;


// sinusoidal value of joint angle
float cosq1 = 0;
float sinq1 = 0;
float cosq2 = 0;
float sinq2 = 0;
float cosq3 = 0;
float sinq3 = 0;

// end-effctor Jacobian transpose
float JT_11 = 0;
float JT_12 = 0;
float JT_13 = 0;
float JT_21 = 0;
float JT_22 = 0;
float JT_23 = 0;
float JT_31 = 0;
float JT_32 = 0;
float JT_33 = 0;

// rotation angle from world frame to N frame
float thetaz = -PI/4.0;
float thetax = 0;
float thetay = 0;

// sinusoidal value of rotation
float cosz = 0;
float sinz = 0;
float cosx = 0;
float sinx = 0;
float cosy = 0;
float siny = 0;

// rotation matrix elements frame world frame to N frame
float R11 = 0;
float R12 = 0;
float R13 = 0;
float R21 = 0;
float R22 = 0;
float R23 = 0;
float R31 = 0;
float R32 = 0;
float R33 = 0;

// rotation matrix transpose elements frame world frame to N frame
float RT_11 = 0;
float RT_12 = 0;
float RT_13 = 0;
float RT_21 = 0;
float RT_22 = 0;
float RT_23 = 0;
float RT_31 = 0;
float RT_32 = 0;
float RT_33 = 0;

// length of each link
float L = 0.254;

// force in world frame for part 2
float Fx = 0;
float Fy = 0;
float Fz = 0;

// force in N frame for part 3
float Fx_n = 0;
float Fy_n = 0;
float Fz_n = 0;

// force in world frame for part 3
float Fx_w = 0;
float Fy_w = 0;
float Fz_w = 0;

// desired force of z direction in world frame
float Fz_cmd = 0;

// torque constant
float Kt = 6;

// desired point 1
float p1x = 0.2;
float p1y = 0.2;
float p1z = 0.254;

// desired point 2
float p2x = 0.3;
float p2y = 0.3;
float p2z = 0.254;

// difference between 2 points
float dx = 0;
float dy = 0;
float dz = 0;

// desire velocity for line following
float v_desire = 0.08;

// gravity compensation when pressing egg
float egg_p = 1.1323;
float  g_offset = 0;

// Line trajectory time variable
float t_start = 0;
float t_total = 0;

// current time in Line trajectory
float t = 0;

// flag for selecting Line trajectory
int state = 1;
int prev_state = 0;



/*
    1 -> x y soft
    2 -> y soft
    3 -> z soft
    4 -> all stiff
 */
// mode of soft kp kd
int mode;

// Line trajectory function
void Line(float t)
{
    /*
        Update the desired point and velocity of Line trajectory given current time

        input args:
                     t  : time in second

        Tf required, set the desired point (xb.yb.zb) as new (xa,ya,za) and get new (xb.yb.zb),
        proceed with computing the component-wise differences delta_x, delta_y, delta_z.

        Based on the user-defined total speed v_total, calculate
        the x,y,z velocities and then use the following formula to
        compute the next desired position:

        x = dx/v*t + xa
        y = dy/v*t + ya
        z = dz/v*t + za

        If one of the ends of the straight line is reached,
        the point xa,ya,za and xb,yb,zb must be assigned qith new value.

        To that end, we track the value of an integer variable 'state'.
        we want to go from 'a' (waypoints[prev_state]) to 'b' (waypoints[state])
        if state = NUM_POINTS, we have finished all points, and the robot arm will stay at the last point
        This switching depends on the difference (t-t_start)

    */


    float xa = 0;
    float ya = 0;
    float za = 0;

    float xb = 0;
    float yb = 0;
    float zb = 0;

    // assign desired points according to state
    prev_point = waypoints[prev_state];
    now_point = waypoints[state];

    xa = prev_point.x;
    ya = prev_point.y;
    za = prev_point.z;

    xb = now_point.x;
    yb = now_point.y;
    zb = now_point.z;

    v_desire = now_point.v;
    thetaz = now_point.thz;
    mode = now_point.mode;

    // mode of soft kp kd
    /*
        1 -> x y soft
        2 -> y soft
        3 -> z soft
        4 -> all stiff
     */

    // unless it is at the mode of pressing egg, the gravity compensation will remain 0
    g_offset = 0;

    // assign kp and kd according to mode
    if(mode == 1)
    {
        Kpx_n = Kpx_n_s;
        Kpy_n = Kpy_n_s;
        Kpz_n = Kpz_n_h;

        Kdx_n = Kdx_n_s;
        Kdy_n = Kdy_n_s;
        Kdz_n = Kdz_n_h;
    }
    else if (mode == 2)
    {
        Kpx_n = Kpx_n_h;
        Kpy_n = Kpy_n_s;
        Kpz_n = Kpz_n_h;

        Kdx_n = Kdx_n_h;
        Kdy_n = Kdy_n_s;
        Kdz_n = Kdz_n_h;
    }
    else if(mode == 3)
    {
        Kpx_n = Kpx_n_h;
        Kpy_n = Kpy_n_h;
        Kpz_n = Kpz_n_s;

        Kdx_n = Kdx_n_h;
        Kdy_n = Kdy_n_h;
        Kdz_n = Kdz_n_s;

        // at the mode of pressing egg, assign gravity compenstation
        g_offset = egg_p;
    }
    else
    {
        Kpx_n = Kpx_n_h;
        Kpy_n = Kpy_n_h;
        Kpz_n = Kpz_n_h;

        Kdx_n = Kdx_n_h;
        Kdy_n = Kdy_n_h;
        Kdz_n = Kdz_n_h;
    }


    // calculate difference between point a and b
    dx = xb - xa;
    dy = yb - ya;
    dz = zb - za;

    // calculate t_total through (total distance)/velocity
    t_total = sqrt(dx*dx + dy*dy + dz*dz)/v_desire;


    // if we reach desired point, then current t - t start will reach t_total
    // change state
    if((t - t_start) >= t_total)
    {
        t_start = t;
        prev_state = state;
        state++;
        if (state == NUM_POINTS)
            state = NUM_POINTS - 1;
    }

    // calculate desire x y z and desire x_dot y_dot z_dot
    x_desire = dx*(t - t_start)/t_total + xa;
    x_desire_dot = dx/t_total;

    y_desire = dy*(t - t_start)/t_total + ya;
    y_desire_dot = dy/t_total;

    z_desire = dz*(t - t_start)/t_total + za;
    z_desire_dot = dz/t_total;
}

// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {


    //Motor torque limitation(Max: 5 Min: -5)


    // save past states
    if ((mycount%50)==0) {

        theta1array[arrayindex] = theta1motor;
        theta2array[arrayindex] = theta2motor;

        if (arrayindex >= 99) {
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
    /*
        Calculate current time
        mycount increments every 1ms
        mycount*0.001 = time in sec
    */
    t = mycount*0.001;

    // calculate desire position and velocity
    Line(t);


    // calculating Rotation matrix zxy elements and its Transpose
    cosz = cos(thetaz);
    sinz = sin(thetaz);
    cosx = cos(thetax);
    sinx = sin(thetax);
    cosy = cos(thetay);
    siny = sin(thetay);
    RT_11 = R11 = cosz*cosy-sinz*sinx*siny;
    RT_21 = R12 = -sinz*cosx;
    RT_31 = R13 = cosz*siny+sinz*sinx*cosy;
    RT_12 = R21 = sinz*cosy+cosz*sinx*siny;
    RT_22 = R22 = cosz*cosx;
    RT_32 = R23 = sinz*siny-cosz*sinx*cosy;
    RT_13 = R31 = -cosx*siny;
    RT_23 = R32 = sinx;
    RT_33 = R33 = cosx*cosy;

    cosq1 = cos(theta1motor);
    cosq2 = cos(theta2motor);
    cosq3 = cos(theta3motor);

    sinq1 = sin(theta1motor);
    sinq2 = sin(theta2motor);
    sinq3 = sin(theta3motor);

    // calculating jacobian matrix transpose elements
    JT_11 = -L*sinq1*(cosq3+sinq2);
    JT_12 = L*cosq1*(cosq3+sinq2);
    JT_13 = 0;
    JT_21 = L*cosq1*(cosq2 - sinq3);
    JT_22 = L*sinq1*(cosq2 - sinq3);
    JT_23 = -L*(cosq3 + sinq2);
    JT_31 = -L*cosq1*sinq3;
    JT_32 = -L*sinq1*sinq3;
    JT_33 = -L*cosq3;

    /*
        Forward Kinematics to calculate the current position of
        the end-effector.
    */
    X = L*cosq1*(cosq3 + sinq2);
    Y = L*sinq1*(cosq3 + sinq2);
    Z = L*(1 + cosq2 - sinq3);

    /*
        Since the encoder of the motor has discrete angle reading,
        calculating velocity from the difference between current and previous value can be quite noisy
        using IIR filter can get rid of the noise

        Calculating current end-effector velocity using moving average (IIR)
        un-filtered velocity        :   v_t = (p_t - p_(t-1)) / 0.001
        filtered velocity           :   vf_t = (v_t + v_(t-1) + v_(t-2)) / 3
        update previous velocity    :   v_(t-1) = vf_t, v_(t-2) = v_(t-1)
    */
    // calculate end-effector velocity
    x_dot = (X - x_old)/0.001;
    x_dot_f = (x_dot + x_dot_old + x_dot_oldold)/3.0;
    x_dot_oldold = x_dot_old;
    x_dot_old = x_dot_f;
    x_old = X;

    y_dot = (Y - y_old)/0.001;
    y_dot_f = (y_dot + y_dot_old + y_dot_oldold)/3.0;
    y_dot_oldold = y_dot_old;
    y_dot_old = y_dot_f;
    y_old = Y;

    z_dot = (Z - z_old)/0.001;
    z_dot_f = (z_dot + z_dot_old + z_dot_oldold)/3.0;
    z_dot_oldold = z_dot_old;
    z_dot_old = z_dot_f;
    z_old = Z;

    // calculate joint velocity
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


    //******************************************//
    //******** Part 2 Feedforward Force ********//
    //******************************************//
    // part 2 force in world frame
    /*
        Calculating force in world frame given desired position and velocity
        Feedfward force control     :   F = Kp*(p_desire - p) + Kd*(v_desire - v)

        Calculating joint torque given force in world frame
        Joint torque                :   tau = JT*F

    */
//    Fx = Kpx*(x_desire - X) + Kdx*(x_desire_dot - x_dot_f);
//    Fy = Kpy*(y_desire - Y) + Kdy*(y_desire_dot - y_dot_f);
//    Fz = Kpz*(z_desire - Z) + Kdz*(z_desire_dot - z_dot_f) + g_offset + Fz_cmd / Kt;
//    tau1_temp = JT_11*Fx + JT_12*Fy + JT_13*Fz;
//    tau2_temp = JT_21*Fx + JT_22*Fy + JT_23*Fz;
//    tau3_temp = JT_31*Fx + JT_32*Fy + JT_33*Fz;



    //********************************************//
    //******** Part 3  Impendence Control ********//
    //********************************************//
    /*
        Calculating joint torque that weaken impedence in specific axes of N frame
        W : world frame
        N : rotated N frame
        tau = (JT * R^W_N * (Kp_N * R^N_W * (p^desire_W - p_W) + Kd_N * R^N_W * (v^desire_W - v_W))) + friction compensation

        partA :    (Kp_N * R^N_W * (p^desire_W - p_W) + Kd_N * R^N_W * (v^desire_W - v_W))
        partB :    R^W_N * partA
        partC :    JT    * partB
        partD :    partC + friction compensation
    */

    // partA    :   force in N frame F_n = KP*(R^N_W)*(Desired point - current position) + KD*(R^N_W)*(Desired velocity - current velocity)
    Fx_n = Kpx_n*(RT_11*(x_desire - X) + RT_12*(y_desire - Y) + RT_13*(z_desire - Z)) + Kdx_n*(RT_11*(x_desire_dot - x_dot_f) + RT_12*(y_desire_dot - y_dot_f) + RT_13*(z_desire_dot - z_dot_f));
    Fy_n = Kpy_n*(RT_21*(x_desire - X) + RT_22*(y_desire - Y) + RT_23*(z_desire - Z)) + Kdy_n*(RT_21*(x_desire_dot - x_dot_f) + RT_22*(y_desire_dot - y_dot_f) + RT_23*(z_desire_dot - z_dot_f));
    // add gravity compensation at z direction force in world frame
    Fz_n = Kpz_n*(RT_31*(x_desire - X) + RT_32*(y_desire - Y) + RT_33*(z_desire - Z)) + Kdz_n*(RT_31*(x_desire_dot - x_dot_f) + RT_32*(y_desire_dot - y_dot_f) + RT_33*(z_desire_dot - z_dot_f)) +  g_offset;

    // partB    :   force in world frame F_w = (R^W_N)*(F_n)
    Fx_w = R11*Fx_n + R12*Fy_n + R13*Fz_n;
    Fy_w = R21*Fx_n + R22*Fy_n + R23*Fz_n;
    Fz_w = R31*Fx_n + R32*Fy_n + R33*Fz_n;

    // partC    :  tau = JT*F_w
    tau1_temp = JT_11*Fx_w + JT_12*Fy_w + JT_13*Fz_w;
    tau2_temp = JT_21*Fx_w + JT_22*Fy_w + JT_23*Fz_w;
    tau3_temp = JT_31*Fx_w + JT_32*Fy_w + JT_33*Fz_w;

    // calculate friction
    if(theta1_dot_f > min_V1) {
        u_fric1 = Viscous_positive1*theta1_dot_f+Coulomb_positive1;
    }
    else if (theta1_dot_f < -min_V1){
        u_fric1 = Viscous_negative1*theta1_dot_f+Coulomb_negative1;
    }
    else{
        u_fric1 = slope_bet_min1*theta1_dot_f;
    }

    if(theta2_dot_f > min_V2){
        u_fric2 = Viscous_positive2*theta2_dot_f+Coulomb_positive2;
    }
    else if (theta2_dot_f < -min_V2){
        u_fric2 = Viscous_negative2*theta2_dot_f+Coulomb_negative2;
    }
    else{
        u_fric2 = slope_bet_min2*theta2_dot_f;
    }

    if(theta3_dot_f > min_V3){
        u_fric3 = Viscous_positive3*theta3_dot_f+Coulomb_positive3;
    }
    else if (theta3_dot_f < -min_V3){
        u_fric3 = Viscous_negative3*theta3_dot_f+Coulomb_negative3;
    }
    else{
        u_fric3 = slope_bet_min3*theta3_dot_f;
    }

    //partD     :   friction compensation
    tau1_temp += u_fric1*u_fric1_adjust;
    tau2_temp += u_fric2*u_fric2_adjust;
    tau3_temp += u_fric3*u_fric3_adjust;

    // torque saturation
    if (tau1_temp > 5){
        tau1_temp = 5;
    }
    if (tau1_temp < -5){
        tau1_temp = -5;
    }
    *tau1 = tau1_temp;

    if (tau2_temp > 5){
        tau2_temp = 5;
    }
    if (tau2_temp < -5){
        tau2_temp = -5;
    }
    *tau2 = tau2_temp;

    if (tau3_temp > 5){
        tau3_temp = 5;
    }
    if (tau3_temp < -5){
        tau3_temp = -5;
    }
    *tau3 = tau3_temp;


    printtheta1motor = theta1motor;
    printtheta2motor = theta2motor;
    printtheta3motor = theta3motor;


    Simulink_PlotVar1 = X;
    Simulink_PlotVar2 = Y;
    Simulink_PlotVar3 = Z;
    Simulink_PlotVar4 = theta2_desire;

    mycount++;
}



void printing(void){
    serial_printf(&SerialA, "Motor        %6.2f %6.2f %6.2f   \n\r \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI);
    serial_printf(&SerialA, "End-Effector %6.5f %6.5f %6.5f   \n\r \n\r",X,Y,Z);
}

