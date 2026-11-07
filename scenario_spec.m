function spec = scenario_spec(scenario)
% SCENARIO_SPEC  Turn the 'scenario' argument into a full description.
%   'A'  straight line, no manoeuvre
%   'B'  same start, smooth bounded manoeuvre in x and y from t = 1 s
%   'C'  scenario A motion + radar blackout 1.0 <= t < 1.5 s
%   'BC' scenario B motion + the same blackout (stress case)
%   N (integer >= 1)  random admissible case number N (manoeuvre + blackout)
%   struct with fields .name (any of the above) and .noise_seed
%        -> same trajectory, different radar noise (Monte Carlo)

noise_seed = 1;
if isstruct(scenario)
    if isfield(scenario, 'noise_seed'), noise_seed = scenario.noise_seed; end
    scenario = scenario.name;
end

spec.noise_seed = noise_seed;
spec.blackout   = [Inf Inf];       % [onset, end) in s; Inf = none
spec.seed       = 0;

if ischar(scenario)
    spec.name = upper(scenario);
    switch spec.name
        case 'A',  spec.kind = 'line';
        case 'B',  spec.kind = 'manoeuvre';
        case 'C',  spec.kind = 'line';      spec.blackout = [1.0 1.5];
        case 'BC', spec.kind = 'manoeuvre'; spec.blackout = [1.0 1.5];
        otherwise, error('scenario_spec: unknown scenario "%s"', scenario);
    end
elseif isnumeric(scenario) && isscalar(scenario) && scenario >= 1
    spec.name = sprintf('R%d', round(scenario));
    spec.kind = 'random';
    spec.seed = round(scenario);
else
    error('scenario_spec: scenario must be ''A'',''B'',''C'',''BC'', a positive integer or a struct');
end
spec.traj_id = spec.name;
end
