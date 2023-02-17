clc; % clear the Command Window (delete all printed output)
close all; % close all figure windows (close all plotted output)

syms theta1 theta2 theta3 
syms theta1_dot theta2_dot theta3_dot 
syms theta1_dot_dot theta2_dot_dot theta3_dot_dot 

syms L2 L3 LC LC2 LC3
syms I2xx I2yy I2zz I3xx I3yy I3zz
syms m2 m3
syms g
syms tau2DH tau3DH
syms tau2Motor tau3Motor

q = [theta2;theta3];
q_dot = [theta2_dot; theta3_dot];
q_dot_dot = [theta2_dot_dot; theta3_dot_dot];
%q_dot_dot = vpa(zeros(2,1));

% TO DO 
q_motor = [theta2 + pi/2.0; theta2+theta3];
q_motor_dot = [theta2_dot; theta2_dot + theta3_dot];
q_motor_dot_dot = [theta2_dot_dot; theta2_dot_dot + theta3_dot_dot];

tauDH = vpa(zeros(2,1)); % [tau2DH; tau3DH]
tauMotor = vpa(zeros(2,1));

gv = [0; -g; 0];

p1 = m2*(LC2^2) + m3*(L2^2) + I2zz;
p2 = m3*(LC3^2) + I3zz;
p3 = m3*(L2*LC3);
p4 = m2*LC2 + m3*L2;
p5 = m3*LC3;

u1 = [0;0;1];
u2 = [0;0;1];

I2 = diag([I2xx, I2yy, I2zz]);
I3 = diag([I3xx, I3yy, I3zz]);

%Sencond joint kinematics
R02 = [[cos(theta2) -sin(theta2) 0];
       [sin(theta2) cos(theta2) 0];
       [0 0 1];];
d01 = [0; 0; 0];
H02 = [[R02 d01];
       [0 0 0 1];];

%Third joint kinematics
R23 = [[cos(theta3) -sin(theta3) 0];
       [sin(theta3) cos(theta3) 0];
       [0 0 1];];

d23 = [L2; 0; 0];
H23 = [[R23 d23];
       [0 0 0 1];];

R03 = R02*R23;

r2 = [LC2;0;0;1];
r3 = [LC3;0;0;1];

H03 = H02*H23;

r02 = simplify(H02*r2);
r03 = simplify(H03*r3);

Jv2 = jacobian(r02,q);
Jv3 = jacobian(r03,q);

Jw2 = [u1 [0;0;0]];
Jw3 = [u1 R23*u2];


K = (m2*transpose(q_dot)*transpose(Jv2)*Jv2*q_dot + ...
     transpose(q_dot)*transpose(Jw2)*R02*I2*transpose(R02)*Jw2*q_dot + ...
     m3*transpose(q_dot)*transpose(Jv3)*Jv3*q_dot + ...
     transpose(q_dot)*transpose(Jw3)*R03*I3*transpose(R03)*Jw3*q_dot)/2;

K = simplify(K);

K_r = (theta2_dot^2)*p1/2 + ((theta2_dot^2)/2 + theta2_dot*theta3_dot+(theta3_dot^2)/2)*p2 + ...
      (cos(theta3)*(theta2_dot^2) + cos(theta3)*theta2_dot*theta3_dot)*p3;
K_r = simplify(K_r);

P = m2*transpose(gv)*r02(1:3) + m3*transpose(gv)*r03(1:3);
P = simplify(P);

P_r = -p4*g*sin(theta2) - p5*g*sin(theta2 + theta3);
P_r = simplify(P_r);

G = jacobian(P, q).';

M = m2*transpose(Jv2)*Jv2 + transpose(Jw2)*R02*I2*transpose(R02)*Jw2 + ...
    m3*transpose(Jv3)*Jv3 + transpose(Jw3)*R03*I3*transpose(R03)*Jw3;

%Chistofell symbols
C = vpa(zeros(2,2));
for k=1:2
    for j=1:2
        for i=1:2
            C(k,j) = C(k,j) + 0.5*(diff(M(k,j),q(i)) + ...
                                   diff(M(k,i),q(j)) - ...
                                   diff(M(i,j),q(k)))*q_dot(i);
        end
    end
end

C = simplify(C);

%Simplifying the expressions for each component of the inertia matrix
for i = 1:2
    for j = 1:2
        M(i,j) = simplify(M(i,j));
    end
    G(i,1) = simplify(G(i,1));
end

M_r = [[p1+p2+2*p3*cos(theta3) p2+p3*cos(theta3)];
       [p2+p3*cos(theta3) p2]];
C_r = [[-p3*sin(theta3)*theta3_dot -p3*sin(theta3)*theta3_dot-p3*sin(theta3)*theta2_dot];
       [p3*sin(theta3)*theta2_dot 0]];
G_r = [-p4*g*cos(theta2)-p5*g*cos(theta2+theta3);
       -p5*g*cos(theta2+theta3)];
tauDH = M*q_dot_dot + C*q_dot + G;
tauDH = simplify(tauDH);
tauMotor = subs(tauDH,[q q_dot q_dot_dot], [q_motor q_motor_dot q_motor_dot_dot]);
tauMotor = simplify(tauMotor);


%q_dot_dot = (M^-1)*tau - (M^-1)*C*q_dot - (M^-1)*G;
