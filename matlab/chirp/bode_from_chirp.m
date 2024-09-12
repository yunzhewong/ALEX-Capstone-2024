function [omega, M, theta] = bode_from_chirp(t, u, y)
    % as the fourier transform creates a frequency domain plot from 0Hz to the
    % nyquist frequency of the system (e.g. sample_freq / 2), we need to use
    % these values to reconstruct the graph
    sample_time = t(2) - t(1);
    sample_freq = 1 / sample_time;
    total_points = numel(t);

    % vector checks
    assert(size(u, 1) == 1 && size(u, 2) == total_points)
    assert(size(y, 1) == 1 && size(y, 2) == total_points)

    % fft response, taking the results and frequency-wise taking the ratio
    uffted = fft(u);
    yffted = fft(y);
    ratio = yffted ./ uffted;
    
    % reconstruction of the entire FFT domain. From  0 to sample_freq with the
    % same number of points as the input signal
    full_f = sample_freq / total_points * (0:total_points- 1);
    
    % conversion to a single sided amplitude spectrum. There is no need to
    % rescale as a ratio is being taken
    P2 = abs(ratio);
    P1 = P2(1:total_points/2+1);
    half_f = full_f(1:total_points / 2 + 1);
    
    ratio_angle = angle(ratio) / pi * 180;

    % bode is typically plotted as w (rads^-1) vs M (db)
    omega = 2*pi*half_f;
    M = 20*log10(P1);
    theta = ratio_angle(1:total_points / 2 + 1);


    % figure
    % semilogx(omega, M)
    % title("Magnitude Plot")
    % xlabel("Frequency (rads^-1)")
    % ylabel("Magnitude (db)")
    % 
    % figure
    % semilogx(omega, theta);
    % title("Phase Plot vs Frequency")
    % xlabel("Frequency (rads^-1)")
    % ylabel("Phase (degrees)")


end