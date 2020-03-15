function [uCurr,covar_curr] = update(z_t,covarEst,uEst)
%% BEFORE RUNNING THE CODE CHANGE NAME TO upd_step
    %% Parameter Definition
    %z_t - is the sensor data at the time step
    C = [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 1 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 1 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 1 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 1 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 1 0 0 0 0 0 0 0 0 0];
    
    R = (1e-3)*eye(6);
    K = covarEst*C.'*(inv(C*covarEst*C.' + R));
    
    uCurr = uEst + K*(z_t - C*uEst);
    covar_curr = covarEst - (K*C*covarEst);
    
    
end

