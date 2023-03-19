
result = zeros(3,21);


time = 0:0.1:2;

for index=1:21
    result(:,index) = cubic(time(index));
end


plot(time,result)
% axis equal %Trick to mkae three axis proportional
grid on % Draw a light grid in the background of the figure axes
xlabel('Time [s]')

legend('theta (rad)', 'theta-dot (rad/s)', 'theta-dotdot (rad/s^2)')



function result = cubic(t)
    
    if t < 1
        theta_d = 1.5*(t^2) - t^3;
        theta_d_dot = 3*t - 3*t^2;
        theta_d_dot2 = 3 - 6*t;
    else
        theta_d = -2 + 6*t - 4.5*(t^2) + t^3;
        theta_d_dot = 6 - 9*t + 3*t^2;
        theta_d_dot2 = -9 + 6*t;
    end

    if t > 2
        theta_d = 0;
        theta_d_dot = 0;
        theta_d_dot2 = 0;
    end

    result = [theta_d; theta_d_dot; theta_d_dot2];
    
end