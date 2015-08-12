function [H, t, r] = plot_se(se, scale, width, color)

    if ~exist('scale', 'var')
        scale = 1;
    end
    if ~exist('width', 'var')
        width = 1;
    end
    if ~exist('color', 'var')
        color = '';
    end

    d = size(se,1) - 1;
    t = se(1:d, d+1);
    r = se(1:d, 1:d);

    H = {plot3(t(1), t(2), t(3), [color '.'], 'MarkerSize', 20, 'LineWidth', width) ...
         quiver3(t(1), t(2), t(3), r(1,1), r(2,1), r(3,1), scale, 'r', 'LineWidth', width) ...
         quiver3(t(1), t(2), t(3), r(1,2), r(2,2), r(3,2), scale, 'g', 'LineWidth', width) ...
         quiver3(t(1), t(2), t(3), r(1,3), r(2,3), r(3,3), scale, 'b', 'LineWidth', width)};
end
