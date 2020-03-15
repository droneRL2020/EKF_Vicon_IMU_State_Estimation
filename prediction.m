function [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt)
    %uPrev - is the mean of the prev state
    %covarPrev - covar of the prev state
    %angVel - angular velocity input at the time step
    %acc - acceleration at the timestep
    %dt - difference in time

      
    %angular velocity vector
    G = [cos(q_y) 0 -(cos(q_x)*sin(q_y)); %G matrix 
        0 1 sin(q_x) ; 
        sin(q_y) 0 (cos(q_x)*cos(q_y))];
    omega = inv(G) * [w_x - bg_x - ng_x;
                      w_y - bg_y - ng_y;
                      w_z - bg_z - ng_z];
    omg_x = omega(1);
    omg_y = omega(2);
    omg_z = omega(3);
    
    %linear acceleration vector
    comp_1 = (cos(q_z)*cos(q_y)) - (sin(q_x)*sin(q_z)*sin(q_y));
    comp_2 = -(cos(q_x)*cos(q_z));
    comp_3 = (cos(q_z)*sin(q_y)) + (cos(q_y)*sin(q_x)*sin(q_z));
    comp_4 = (cos(q_y)*sin(q_z)) + (cos(q_z)*sin(q_x)*sin(q_y));
    comp_5 = cos(q_x)*cos(q_z);
    comp_6 = (sin(q_z)*sin(q_y)) - (cos(q_z)*cos(q_y)*sin(q_x));
    comp_7 = -(cos(q_x)*sin(q_y));
    comp_8 = sin(q_x);
    comp_9 = cos(q_x)*cos(q_y);
    %Rotation matrix (Z-X-Y)
    R = [comp_1 comp_2 comp_3 ; comp_4 comp_5 comp_6 ; comp_7 comp_8 comp_9];
    acc = [ 0 ; 0 ; -9.81] + R*[a_x - ba_x - na_x;
                                a_y - ba_y - na_y;
                                a_z - ba_z - na_z;];
    acc_x = acc(1);
    acc_y = acc(2);
    acc_z = acc(3);
    
    x = [x;y;z;
         q_x;q_y;q_z;
         v_x;v_y;v_z;
         bg_x;bg_y;bg_z;
         ba_x;ba_y;ba_z];
    
    x_dot=[v_x;v_y;v_z;
           omg_x;omg_y;omg_z;
           a_x;a_y;a_z;
           nbg_x;nbg_y;nbg_z;
           nba_x;nba_y;nba_z];
    
    noise = [ng_x;ng_y;ng_z;
             na_x;na_y;na_z;
             nba_x;nba_y;nba_z;
             nbg_x;nbg_y;nbg_z]
             
    
    A = jacobian(x_dot, x);
    U = jacobian(x_dot, noise);
    F = eye(15) + dt*A;
    Q = (1e-3)*eye(12);
    
    uEst = uPrev + dt*x_dot;
    covarEst = F*covarPrev*F.' + U*dt*Q*U.';
end

