function headless_plots()
% HEADLESS_PLOTS  Test scripts draw invisible figures and print them to PNG.
% In Octave without a display (e.g. a server), switch to the gnuplot
% toolkit; with a display, and in MATLAB, nothing changes.
if exist('OCTAVE_VERSION', 'builtin') && isempty(getenv('DISPLAY'))
    w = warning('off', 'all');
    graphics_toolkit('gnuplot');
    warning(w);
end
end
