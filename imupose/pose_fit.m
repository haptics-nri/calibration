function recovered = pose_fit(acc_reading, vicon_reading)

    [R, unR, H, unH] = utils;
    N = size(acc_reading,1);

    vicon_grav = zeros(N,3);
    for i=1:N
        vicon_grav(i,:) = (R(vicon_reading(i,4:6))\[0 0 -9.8]')';
    end

    % construct linear regression from N vector correspondences
    % A*x = b
    % A: 3N x 9 (munged components of accelerometer reading)
    % x:  9 x 1 (components of a rotation matrix)
    % b: 3N x 1 (munged components of vicon gravity vector)
    A = zeros(3*N,9);
    b = zeros(3*N,1);
    centered_acc_reading = bsxfun(@minus, acc_reading, mean(acc_reading));
    centered_vicon_grav = bsxfun(@minus, vicon_grav, mean(vicon_grav));
    for i=1:N
        A( ((i-1)*3+1), 1:3 ) = centered_acc_reading(i,:)';
        A( ((i-1)*3+2), 4:6 ) = centered_acc_reading(i,:)';
        A( ((i-1)*3+3), 7:9 ) = centered_acc_reading(i,:)';
        b( ((i-1)*3+1) : (i*3) ) = centered_vicon_grav(i,:)';
    end
    % solve linear regression and force the result to be in SO(3)
    x = A\b;
    rot = [x(1:3)'; x(4:6)'; x(7:9)'];
    [U,~,V] = svd(rot);
    fix = eye(3);
    fix(3,3) = det(U*V');
    rot = U*fix*V';
    recovered = {[0 0 0] rot};
    
end