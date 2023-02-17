function matrix = B(q, params)
    %Joint angles
    q(1)
    theta1 = sym(q(1,:));
    theta2 = sym(q(2,:));
    theta3 = sym(q(3,:));

    %System Parameters
    L1 = params(1);
    L2 = params(2);
    L3 = params(3);

    %First joint kinematics
    R01 = [[cos(theta1) -sin(theta1) 0];
           [sin(theta1) cos(theta1) 0];
           [0 0 1];];
    d01 = [0; 0; 0];
    H01 = [[R01 d01];
           [0 0 0 1];];

    %Second joint kinematics
    R12 = [[cos(theta2) 0 sin(theta2)];
           [0 1 0];
           [-sin(theta2) 0 cos(theta2)];];
    d12 = [0; 0; L1];
    H12 = [[R12 d12];
           [0 0 0 1];];

    %Third joint kinematics
    R23 = [[cos(theta3) 0 sin(theta3)];
           [0 1 0];
           [-sin(theta3) 0 cos(theta3)];];
    d23 = [0; 0; L2];
    H23 = [[R23 d23];
           [0 0 0 1];];

    %End-effector position with respect to frame 0 (world-fixed)
    H02 = H01*H12;
    H03 = H01*H12*H23;
    matrix = H03*[L3; 0; 0; 1]
end