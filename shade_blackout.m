function shade_blackout(bo, h)
% SHADE_BLACKOUT  grey band for each radar blackout interval (one row of bo each)
for j = 1:size(bo, 1)
    if all(isfinite(bo(j,:)))
        fill(bo(j,[1 2 2 1]), [-h -h h h], [0.9 0.9 0.9], 'edgecolor', 'none');
    end
end
end
