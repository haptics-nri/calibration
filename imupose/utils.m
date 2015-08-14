function [R, unR, H, unH] = utils

    R = @(euler) RPYtoRot_ZXY(euler(1), euler(2), euler(3));
    unR = @(mat) RotToRPY_ZXY(mat);
    H = @(params) [R(params(4:6)) params(1:3)'; 0 0 0 1];
    unH = @(mat) [mat(1:3,4)' unR(mat(1:3,1:3))];
    
end
