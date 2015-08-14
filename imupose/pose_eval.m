function residuals = pose_eval(acc_reading, vicon_reading, recovered)

    [R, unR, H, unH] = utils;
    N = size(acc_reading,1);

    vicon_grav = zeros(N,3);
    for i=1:N
        vicon_grav(i,:) = (R(vicon_reading(i,4:6))\[0 0 -9.8]')';
    end
    
    residuals(1) = 0; % TODO
    
    ang_errs = zeros(N, 1);
    for i=1:N
        v1 = recovered{2}*acc_reading(i,:)';
        v2 = vicon_grav(i,:);
        ang_errs(i) = acos(dot(v1, v2)/norm(v1)/norm(v2));
    end
    residuals(2) = mean(ang_errs);

end
