a1 = 0;
alpha1 = -sym(pi)/2;
d1 = 0.254;
theta1_dh = 'theta_1';

a2 = 0.254;
alpha2 = 0;
d2 = 0;
theta2_dh = 'theta_2';

a3 = 0.254;
alpha3 = 0;
d3 = 0;
theta3_dh = 'theta_3';

A1 = DH_matrix(a1,alpha1, d1, theta1_dh)
A2 = DH_matrix(a2,alpha2, d2, theta2_dh)
A3 = DH_matrix(a3,alpha3, d3, theta3_dh)
H3 = A1*A2*A3