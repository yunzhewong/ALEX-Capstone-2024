J = 0.2052;
b = 1.3877;
K_t = 3.563;
F_c = 1.4368;
F_stat = 1.7102;

FILENAME = '../data/batch 6/step0.76A.csv';
data = readmatrix(FILENAME);    
times = data(:, 1);
currents = data(:, 2);

diffs = zeros(1, numel(times) - 1);
for i = 1:numel(diffs)
    diffs(i) = times(i + 1) - times(i);
end
dt = mean(diffs);

EPSILON = 0.005;