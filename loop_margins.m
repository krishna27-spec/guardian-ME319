function [wc, pm, w180, gm] = loop_margins(Lfun, wlo, whi)
% LOOP_MARGINS  Gain crossover, phase margin, and gain margin at the first
% -180 deg phase crossing above crossover, for a loop L(jw) given as a
% function handle of w (vectorised). No toolbox needed.
w   = logspace(log10(wlo), log10(whi), 6000);
Lw  = Lfun(w);
mag = abs(Lw);
ph  = rad2deg(unwrap(angle(Lw)));
i   = find(mag(1:end-1) >= 1 & mag(2:end) < 1, 1);
wc  = exp(interp1(log(mag(i:i+1)), log(w(i:i+1)), 0));
pm  = 180 + interp1(log(w(i:i+1)), ph(i:i+1), log(wc));
k   = find(w(1:end-1) > wc & ph(1:end-1) > -180 & ph(2:end) <= -180, 1);
if isempty(k)
    w180 = NaN; gm = Inf;
else
    w180 = exp(interp1(ph(k:k+1), log(w(k:k+1)), -180));
    gm = -20*log10(abs(Lfun(w180)));
end
end
