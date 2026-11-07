function ts = settle_time(t, e, tol)
% SETTLE_TIME  First time after which |e| stays within tol for good
% (0 if it never leaves, NaN if it never settles).
i = find(abs(e) > tol, 1, 'last');
if isempty(i), ts = 0; elseif i == numel(t), ts = NaN; else, ts = t(i+1); end
end
