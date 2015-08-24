function pose_sim(acc_offset)

    [R, unR, H, unH] = utils;

    % ground truth

    vicon_origin = [0 0 0, 0 0 0]; % pose(s) of Vicon origin in global frame
    rig_pose = [4 5 6, 0 0 0
                -3 5 6, 0 0 pi/4
                4 5 6, 0 pi/4 0
                10 5 6, pi/4 0 0
                4 5 7, 0 0 3*pi/4
                1 5 0, 0 3*pi/4 0
                14 5 2, 3*pi/4 0 0
                ]; % pose of rig in global frame
    if ~exist('acc_offset', 'var')
        acc_offset = [1 2 3, -pi/16 0 pi/4]; % pose of accelerometer in rig frame
    end
    N = size(rig_pose,1);

    % real world BS
    vicon_noise = 0.001*eye(6);
    acc_noise = 0.01*eye(3);

    % step 0. visualization: use ground truth to draw a picture

    clf;
    hold on;
    colors = 'rgbcmykrgbcmyk';
    plot_se(H(vicon_origin), 3, 2, colors(1));
    for i=1:N
        Hrig = H(rig_pose(i,:));
        Hcomp = Hrig*H(acc_offset);
        plot_se(Hrig, 2, 2, colors(i+1));
        plot_se(Hcomp, 1, 1, colors(i+1));
        plot3([Hrig(1,4) Hcomp(1,4)], [Hrig(2,4) Hcomp(2,4)], [Hrig(3,4) Hcomp(3,4)], colors(i+1));
    end
    axis([-5 15, -5 15, -5 15]);
    grid on;
    axis vis3d;
    xlabel X; ylabel Y; zlabel Z;

    % step 1. forward simulation: use ground truth to generate sensor readings

    acc_reading = zeros(N,3);
    vicon_reading = zeros(N,6);
    for i=1:N
        acc_reading(i,:)   = (inv(R(rig_pose(i,4:6))*R(acc_offset(4:6)))*[0 0 -9.8]')' + mvnrnd([0 0 0], acc_noise);
        vicon_reading(i,:) = unH(inv(H(vicon_origin)/H(rig_pose(i,:)))) + mvnrnd([0 0 0, 0 0 0], vicon_noise);
    end

    % step 2. recovery: given acc_reading and vicon_reading, find acc_offset
    recovered = pose_fit(acc_reading, vicon_reading);

    fprintf('given R:\t%s\n', mat2str(R(acc_offset(4:6)), 3));
    fprintf('recovered R:\t%s\n', mat2str(recovered{2}, 3));
    fprintf('position error:\t%g m\n', norm(acc_offset(1:3) - recovered{1}));
    fprintf('angular error:\t%g deg\n', norm(logm(recovered{2}\R(acc_offset(4:6))))*180/pi);
    fprintf('residuals\t%s\n', mat2str(pose_eval(acc_reading, vicon_reading, recovered), 3));

end
