%% simulate

pose_sim([1 2 3, 2 -5 -5*pi/8]);

%% load data

[vt, vr, at, ar] = load_nri('~/Documents/research/nri/data/20150804/spherecalib2', 'Tuesday2');

%% remove data that doesn't overlap in time, and downsample acc

% crazy manual offset (found by grid search)
vto = vt + 11.727;

%offsets = 11.7:.001:11.8;
%results = zeros(size(offsets));
%for o=1:length(offsets)
    
%    fprintf('current offset %g\n', offsets(o));
%    vto = vt + offsets(o);

    on = max([vto(1) at(1)]);
    off = min([vto(end) at(end)]);
    t = vto(vto >= on & vto <= off);

    vrd = vr(vto >= on & vto <= off, :);
    ard = ar(at >= on & at <= off, :);
    ard = resample(ard, length(t), nnz(at >= on & at <= off));

    %% renormalize acc

    for i=1:size(ard,1)
        ard(i,:) = ard(i,:)/norm(ard(i,:))*9.8;
    end

    %% learn parameters

    recovered = pose_fit(-ard, vrd);
    residuals = pose_eval(-ard, vrd, recovered);
    
%    results(o) = residuals(2);
    
%end

%% sphere fitting and plotting

[R, ~, ~, ~] = utils;

vrp = vrd(:,1:3);
vra = vrd(:,4:6);

% convert from euler back to rotation matrix, then take X axis
for i=1:size(vra,1)
    rot = R(vra(i,:));
    vra(i,:) = -rot(:,1);
end

% fit sphere (positions only)
[center, radius, err] = sphereFit(vrp);

clf
hold on
[xs, ys, zs] = sphere;
mesh(xs*radius + center(1), ys*radius + center(2), zs*radius + center(3), 'facecolor', 'none', 'edgecolor', 'g');
quiver3(vrp(:,1), vrp(:,2), vrp(:,3), vra(:,1), vra(:,2), vra(:,3), 'b');
plot3(center(1), center(2), center(3), 'k.', 'markersize', 20);
axis equal vis3d
xlabel X; ylabel Y; zlabel Z
grid on




%% notes

what assumptions are we making?
    - real acceleration (not gravity) is zero
        - look at periods of high acc magnitude and throw those out
