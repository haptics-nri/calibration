function pose_sim(acc_offset)

% utility functions

R = @(euler) RPYtoRot_ZXY(euler(1), euler(2), euler(3));
unR = @(mat) RotToRPY_ZXY(mat);
H = @(params) [R(params(4:6)) params(1:3)'; 0 0 0 1];
unH = @(mat) [mat(1:3,4)' unR(mat(1:3,1:3))];

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

fprintf('given R:\t%s\n', mat2str(R(acc_offset(4:6)), 3));
fprintf('recovered R:\t%s\n', mat2str(recovered{2}, 3));
fprintf('position error:\t%g m\n', norm(acc_offset(1:3) - recovered{1}));
fprintf('angular error:\t%g deg\n', norm(logm(recovered{2}\R(acc_offset(4:6))))*180/pi);
