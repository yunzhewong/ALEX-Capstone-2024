close all
clear

% sample transer function
% sys = tf(3, [1 0.5 30]);
sys = tf(1, [1 2]);

% the true bode magnitude distribution
figure
bode(sys)

%randomly generated points
POINTS = 10000;
t = linspace(0, 20, POINTS);
chirp_f = linspace(0, 50, POINTS);

% as the fourier transform creates a frequency domain plot from 0Hz to the
% nyquist frequency of the system (e.g. sample_freq / 2), we need to use
% these values to reconstruct the graph
sample_t = t(2) - t(1);
sample_freq = 1 / sample_t;

% construction of the chirp signal
u = sin(2 * pi * chirp_f .* t);
% u = sin(2 * pi * 10 * t);

% simulation of the system under the chirp signal input
y_true = lsim(sys, u, t)';

% fft response, taking the results and frequency-wise taking the ratio
uffted = fft(u);
yffted = fft(y_true);
ratio = yffted ./ uffted;

% reconstruction of the entire FFT domain. From  0 to sample_freq with the
% same number of points as the input signal
full_f = sample_freq / POINTS * (0:POINTS - 1);

% conversion to a single sided amplitude spectrum. There is no need to
% rescale as a ratio is being taken
P2 = abs(ratio);
P1 = P2(1:POINTS/2+1);
half_f = full_f(1:POINTS / 2 + 1);


% plotting omega (2*pi*f) vs the db (20log10(M))
figure
semilogx(2*pi*half_f, 20*log10(P1))

