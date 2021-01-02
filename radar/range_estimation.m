% TODO : Find the Bsweep of chirp for 1 m resolution
c = 3 * 10 ^ 8; % The speed of light
delta_r = 1;   % The range resolution

Bsweep = c / (2 * delta_r); 
% TODO : Calculate the chirp time based on the Radar's Max Range
range_max = 300;
Ts = 5.5 * (range_max * 2 / c);

% TODO : define the frequency shifts 
beat_freq = [0 1.1e6 13e6 24e6]'
calculated_range = c * Ts * beat_freq / (2 * Bsweep);

% Display the calculated range
disp(calculated_range);
