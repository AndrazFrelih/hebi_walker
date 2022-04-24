function evaluate_spectrum(signal, Tsample)
Fs = 1/Tsample;             % Sampling frequency                    
L = size(signal,2);         % Length of signal
t = (0:L-1)*Tsample;        % Time vector

Y = fft(signal);
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')

end

