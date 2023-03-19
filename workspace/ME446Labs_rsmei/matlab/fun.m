clear all
clc

syms a0 a1 a2 a3 t

eq1 = a0 + a1*t + a2*(t*t) + a3*(t*t*t);
eq2 = a1 + 2*a2*t + 3*a3*(t*t);

theta_t0 = subs(eq1, t, 0) == 0;
theta_t1 = subs(eq1, t, 1) == 0;
thetadot_t0 = subs(eq2, t, 0) == 0;
thetadot_t1 = subs(eq2, t, 1) == 0;

sol = solve([theta_t0, theta_t1, thetadot_t0, thetadot_t1], [a0, a1, a2, a3])


theta_t0 = subs(eq1, t, 1) == 0;
theta_t1 = subs(eq1, t, 2) == pi/10;
thetadot_t0 = subs(eq2, t, 1) == 0;
thetadot_t1 = subs(eq2, t, 2) == 0;

sol = solve([theta_t0, theta_t1, thetadot_t0, thetadot_t1], [a0, a1, a2, a3])
theta_t0 = subs(eq1, t, 2) == pi/10;
theta_t1 = subs(eq1, t, 3) == pi/10;
thetadot_t0 = subs(eq2, t, 2) == 0;
thetadot_t1 = subs(eq2, t, 3) == 0;

sol = solve([theta_t0, theta_t1, thetadot_t0, thetadot_t1], [a0, a1, a2, a3])

theta_t0 = subs(eq1, t, 3) == pi/10;
theta_t1 = subs(eq1, t, 4) == 0;
thetadot_t0 = subs(eq2, t, 3) == 0;
thetadot_t1 = subs(eq2, t, 4) == 0;

sol = solve([theta_t0, theta_t1, thetadot_t0, thetadot_t1], [a0, a1, a2, a3])