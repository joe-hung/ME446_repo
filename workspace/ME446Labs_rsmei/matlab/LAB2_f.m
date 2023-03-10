function result = LAB2_f(t)
    
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

for t=0:0.1:2
    result(:,idex) = LAB2_f(t)
    idex = idex+1
end