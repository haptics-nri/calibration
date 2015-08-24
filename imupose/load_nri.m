function [vicon_time, vicon_readings, acc_time, acc_readings] = load_nri(dir, model)

    [R, unR, H, unH] = utils;

    % sanity checks
    
    vicon_filename = [dir '/vicon.csv'];
    acc_filename = [dir '/stb.acc.csv'];
    if ~exist(vicon_filename, 'file')
        error('no vicon.csv in that directory');
    elseif ~exist(acc_filename, 'file')
        error('no stb.acc.csv in that directory');
    end
    
    % load vicon data
    
    vicon = readtable(vicon_filename, 'delimiter','tab');
    t = find(cellfun(@(s) ~isempty(strfind(s, 'Timestamp')), vicon.Properties.VariableNames));
    tx = find(cellfun(@(s) ~isempty(strfind(s, [model '_Root_T_X'])), vicon.Properties.VariableNames));
    ty = find(cellfun(@(s) ~isempty(strfind(s, [model '_Root_T_Y'])), vicon.Properties.VariableNames));
    tz = find(cellfun(@(s) ~isempty(strfind(s, [model '_Root_T_Z'])), vicon.Properties.VariableNames));
    ax = find(cellfun(@(s) ~isempty(strfind(s, [model '_Root_A_X'])), vicon.Properties.VariableNames));
    ay = find(cellfun(@(s) ~isempty(strfind(s, [model '_Root_A_Y'])), vicon.Properties.VariableNames));
    az = find(cellfun(@(s) ~isempty(strfind(s, [model '_Root_A_Z'])), vicon.Properties.VariableNames));
    if isempty(t)
        error('no vicon timestamps');
    elseif isempty([tx ty tz ax ay az])
        error('vicon model not found');
    end
    
    vicon_readings = zeros(length(vicon.(t)), 6);
    vicon_time = vicon.(t);
    vicon_readings(:,1) = vicon.(tx);
    vicon_readings(:,2) = vicon.(ty);
    vicon_readings(:,3) = vicon.(tz);
    vicon_readings(:,4) = vicon.(ax);
    vicon_readings(:,5) = vicon.(ay);
    vicon_readings(:,6) = vicon.(az);
    % rotation vector -> Euler angles
    for i=1:size(vicon_readings,1)
        v = vicon_readings(i,4:6);
        vcross = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
        vicon_readings(i,4:6) = unR(expm(vcross));
    end
    
    % load accelerometer data
    acc = readtable(acc_filename);
    t = find(cellfun(@(s) ~isempty(strfind(s, 'Timestamp')), acc.Properties.VariableNames));
    p = find(cellfun(@(s) ~isempty(strfind(s, 'FIFOPosition')), acc.Properties.VariableNames));
    x = find(cellfun(@(s) ~isempty(strfind(s, 'AccX')), acc.Properties.VariableNames));
    y = find(cellfun(@(s) ~isempty(strfind(s, 'AccY')), acc.Properties.VariableNames));
    z = find(cellfun(@(s) ~isempty(strfind(s, 'AccZ')), acc.Properties.VariableNames));
    if isempty([t p x y z])
        error('acc data misformatted');
    end
    acc_readings = zeros(length(acc.(t)), 3);
    acc_time = acc.(t);
    acc_readings(:,1) = acc.(x);
    acc_readings(:,2) = acc.(y);
    acc_readings(:,3) = acc.(z);
    % fix timestamps
    i = 1;
    while i < size(acc_readings,1)
        % j is the start of the next packet
        j = i + 1;
        while j < size(acc_readings,1) && acc.(p)(j) ~= 0
            j = j + 1;
        end
        
        % interpolate timestamps through the packet
        acc_time(i:j) = linspace(acc_time(i), acc_time(j), j-i+1);
        
        % jump to next packet
        i = j;
    end

end
