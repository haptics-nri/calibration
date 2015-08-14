%% simulate

pose_sim([1 2 3, 2 -5 -5*pi/8]);

%% load data

[vt, vr, at, ar] = load_nri('~/Documents/research/nri/data/20150804/squaretable', 'Tuesday2');

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
