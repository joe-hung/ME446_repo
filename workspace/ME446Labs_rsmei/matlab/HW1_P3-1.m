% HW1_P3.m
% Written by Hao-Yang Hung
% Modified from HW1_Examples

function HW1_P3 = HW1_P3(q, params)
    clc; % clear the Command Window (delete all printed output)
    close all; % close all figure windows (close all plotted output)

    %Joint angles
    theta1 = q(1);
    theta2 = -q(2);
    theta3 = -q(3);

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
    R12 = [[1 0 0];
           [0  cos(theta2) sin(theta2)];
           [0 -sin(theta2) cos(theta2)];];
    d12 = [0; 0; L1];
    H12 = [[R12 d12];
           [0 0 0 1];];

    %Third joint kinematics
    R23 = [[1 0 0];
           [0  cos(theta3) sin(theta3)];
           [0 -sin(theta3) cos(theta3)];];
    d23 = [0; L2; 0];
    H23 = [[R23 d23];
           [0 0 0 1];];

    %End-effector position with respect to frame 0 (world-fixed)
    H02 = H01*H12;
    H03 = H01*H12*H23;
    p0 = H03*[0; L3; 0; 1];
    
    %% Plotting
    figure(1) % Open figure window 1
    clf % Clear figure window 1 (erase everything in it)
    
    %Sequence of points to plot (base -> joint 1 -> joint 2 -> end-effector)
    P = [[0; 0; 0; 1] H02*[0; 0; 0; 1] H03*[0; 0; 0; 1] p0];
    plot3(P(1,:),P(2,:),P(3,:),'k','LineWidth',3)
    hold on
    plot3(P(1,:),P(2,:),P(3,:),'ok','LineWidth',3)
    axis equal %Trick to mkae three axis proportional
    grid on % Draw a light grid in the background of the figure axes
    xlabel('X_0')
    ylabel('Y_0')
    zlabel('Z_0')
    offset = 0.08;
    text(p0(1)+offset,p0(2)+offset,p0(3)+offset,'p')
    
    %Plotting frame 0 green
    scale = .2;
    quiver3(0,0,0,scale,0,0,'g')
    quiver3(0,0,0,0,scale,0,'g')
    quiver3(0,0,0,0,0,scale,'g')
    
    %Plotting frame 1 blue
    quiver3(H01(1,4),H01(2,4),H01(3,4),scale*H01(1,1),scale*H01(2,1),scale*H01(3,1),'b')
    quiver3(H01(1,4),H01(2,4),H01(3,4),scale*H01(1,2),scale*H01(2,2),scale*H01(3,2),'b')
    quiver3(H01(1,4),H01(2,4),H01(3,4),scale*H01(1,3),scale*H01(2,3),scale*H01(3,3),'b')
    
    %Plotting frame 2 magenta
    quiver3(H02(1,4),H02(2,4),H02(3,4),scale*H02(1,1),scale*H02(2,1),scale*H02(3,1),'m')
    quiver3(H02(1,4),H02(2,4),H02(3,4),scale*H02(1,2),scale*H02(2,2),scale*H02(3,2),'m')
    quiver3(H02(1,4),H02(2,4),H02(3,4),scale*H02(1,3),scale*H02(2,3),scale*H02(3,3),'m')

    %Plotting frame 3 yellow
    quiver3(H03(1,4),H03(2,4),H03(3,4),scale*H03(1,1),scale*H03(2,1),scale*H03(3,1),'color',[0.8500 0.3250 0.0980])
    quiver3(H03(1,4),H03(2,4),H03(3,4),scale*H03(1,2),scale*H03(2,2),scale*H03(3,2),'color',[0.8500 0.3250 0.0980])
    quiver3(H03(1,4),H03(2,4),H03(3,4),scale*H03(1,3),scale*H03(2,3),scale*H03(3,3),'color',[0.8500 0.3250 0.0980])
    
    %Plotting frame attached to the end-effector red
    quiver3(p0(1),p0(2),p0(3),scale*H03(1,1),scale*H03(2,1),scale*H03(3,1),'r')
    quiver3(p0(1),p0(2),p0(3),scale*H03(1,2),scale*H03(2,2),scale*H03(3,2),'r')
    quiver3(p0(1),p0(2),p0(3),scale*H03(1,3),scale*H03(2,3),scale*H03(3,3),'r')
end

