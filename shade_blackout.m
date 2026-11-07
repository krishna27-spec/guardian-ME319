function shade_blackout(bo, h)
% SHADE_BLACKOUT  grey band for the radar blackout interval in a plot
if all(isfinite(bo))
    fill([bo(1) bo(2) bo(2) bo(1)], [-h -h h h], [0.9 0.9 0.9], 'edgecolor', 'none');
end
end
