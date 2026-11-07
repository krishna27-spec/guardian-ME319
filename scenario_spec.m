function spec = scenario_spec(scenario)
% SCENARIO_SPEC  Turn the 'scenario' argument into a full description.
%   'A'  straight line, no manoeuvre
%   'B'  same start, smooth bounded manoeuvre in x and y from t = 1 s
%   'C'  scenario A motion + radar blackout 1.0 <= t < 1.5 s
%   'BC' scenario B motion + the same blackout (stress case)
%   'MB' scenario B motion + THREE blackouts: 1.0-1.5, 1.6-2.1, 4.0-4.5 s
%        (the brief says "intervals", so more than one per run is possible)
%   N (integer >= 1)  random admissible case number N (manoeuvre + blackout)
%   'W1', 'W2', ...   WILD random case: jinking turns, random speed changes
%        (still inside every limit) and 0-3 random blackouts
%   struct with fields .name (any of the above) and .noise_seed
%        -> same trajectory, different radar noise (Monte Carlo)

noise_seed = 1;
if isstruct(scenario)
    if isfield(scenario, 'noise_seed'), noise_seed = scenario.noise_seed; end
    scenario = scenario.name;
end

spec.noise_seed = noise_seed;
spec.blackout   = [Inf Inf];       % one row [onset, end) in s per blackout; Inf = none
spec.seed       = 0;

if ischar(scenario)
    spec.name = upper(scenario);
    if ~isempty(regexp(spec.name, '^W\d+$', 'once'))
        spec.kind = 'wild'; spec.seed = str2double(spec.name(2:end));
    else
    switch spec.name
        case 'A',  spec.kind = 'line';
        case 'B',  spec.kind = 'manoeuvre';
        case 'C',  spec.kind = 'line';      spec.blackout = [1.0 1.5];
        case 'BC', spec.kind = 'manoeuvre'; spec.blackout = [1.0 1.5];
        case 'MB', spec.kind = 'manoeuvre'; spec.blackout = [1.0 1.5; 1.6 2.1; 4.0 4.5];
        otherwise, error('scenario_spec: unknown scenario "%s"', scenario);
    end
    end
elseif isnumeric(scenario) && isscalar(scenario) && scenario >= 1
    spec.name = sprintf('R%d', round(scenario));
    spec.kind = 'random';
    spec.seed = round(scenario);
else
    error('scenario_spec: scenario must be ''A'',''B'',''C'',''BC'',''MB'', ''W<n>'', a positive integer or a struct');
end
spec.traj_id = spec.name;
% the brief's limits on every blackout: onset >= 1.0 s, length <= 0.5 s
bo = spec.blackout;
if all(isfinite(bo(:))) && (any(bo(:,1) < 1.0 - 1e-9) || any(bo(:,2) - bo(:,1) > 0.5 + 1e-9))
    error('scenario_spec: blackout in %s breaks the brief''s limits (onset >= 1 s, length <= 0.5 s)', spec.name);
end
end
