function [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt)

    % uPrev - is the mean of the prev state
    uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1));
    
    %covarPrev - covar of the prev state
    covarPrev = eye(15);
    
    %angVel - angular velocity input at the time step
    omg1 = extractfield(sampledData,'omg');
    t_omg1 = transpose(omg1);
    omg2 = vec2mat(t_omg1,3);
    omg = transpose(omg2);
    angVel = omg;
    
    %acc - acceleration at the timestep
    acc1=extractfield(sampledData,'acc');
    t_acc1 = transpose(acc1);
    acc2 = vec2mat(t_acc1,3);
    acc = transpose(acc2);
    
    %dt - difference in time 
    dt = vertcat(sampledTime(1),sampledTime(2:end)-sampledTime(1:end-1));
    
    %Q - noise covariance
    Q = (1e-3)*eye(12);
    
    uEst=uCurr + 
         
    
    
end

