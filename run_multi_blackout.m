% RUN_MULTI_BLACKOUT  Scenario B against MB (the same motion with three radar
% blackouts: 1.0-1.5, 1.6-2.1 and 4.0-4.5 s). Prints the numbers reported in
% the paper (Table II, row MB) and every pulse with its time and miss.
% About 25 s in Octave. B must reproduce figs/results.csv (8/8, first hit 1.953 s).

here = fileparts(mfilename('fullpath')); addpath(here);
cases = {'B', 'MB'};
for c = 1:numel(cases)
    L = simulate_engagement(cases{c});  M = L.metrics;
    fprintf('%s: %d/%d hits, first hit %.3f s, miss mean %.3f m, max %.3f m, sat %.1f %%, est rms %.3f m, ignored edges %d\n', ...
        cases{c}, M.n_hits, M.n_pulses, M.t_first_hit, mean(M.miss), max(M.miss), ...
        100*M.sat_frac, M.est_now_rms, M.ignored);
    for j = 1:numel(L.pulses)
        P = L.pulses(j);
        fprintf('   pulse %d: t = %.3f s, miss = %.3f m, hit = %d, radar lost at that instant = %d\n', ...
            j, P.t, P.miss, P.hit, L.blackout(round(P.t/1e-3) + 1));
    end
end
fprintf('MB blackout intervals [onset end) in s:\n'); disp(L.bo)
