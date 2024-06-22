function [res, f] = sampled_fft(y, L, Fs)
    ffted  = fft(y);
    
    P2 = abs(ffted/L);
    P1 = P2(1:L/2+1);
    P1(2:end-1) = 2*P1(2:end-1);
    
    f = Fs/L*(0:(L/2));

    res = P1;
end