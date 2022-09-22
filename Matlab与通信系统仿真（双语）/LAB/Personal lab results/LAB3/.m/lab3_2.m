%% Amplitude modulation.Sampling rate Fs=1500Hz.

clear;
clc;

%% Task(1)

% Initialization.
t0 = 1;                 % Signal duration
Fs = 1500;              % Sampling rate
Ts = 1/Fs;              % Sampling interval
Fc = 20;                % Carrier rate
t = 0:Ts:t0;            % Time vector
N = length(t);          % Length of t
N0 = 0.0005;            % Power spectral density of noise

% Message signal and modulated signal(DSB_AM and USSB_AM).
m = sin(2*pi*t);        % Message signal
c = cos(2*pi*Fc*t);     % Carrier signal
Ac_DSB_AM = 1;
Ac_USSB_AM = sqrt(2);

% DBS_AM modulated signal.
u_dsb = Ac_DSB_AM*m.*c;
% SSB_AM modulated signal.
u_ussb = 1/2*Ac_USSB_AM*real(hilbert(m).*exp(1i*2*pi*Fc*t));
% "equal to " u_ussb = 1/2*Ac_USSB_AM*cos(2*pi*(Fc+1)*t).

% Fourier transform(Time domain to frequency domain).
[~,M] = T2F(t,m); 
[~,U_DSB] = T2F(t,u_dsb);
[df1,U_USSB] = T2F(t,u_ussb);

% ¢Ù Calculate the power of the modulated signal and message signal.
m_power = sum(abs(m).^2)/N
dsb_power = sum(abs(u_dsb).^2)/N
ussb_power = sum(abs(u_ussb).^2)/N

% ¢Ú Plot the modulated signal and the spectrum.
figure(1)
subplot(2,2,1)
plot(t,u_dsb)
title('DSB Signal')
xlabel('Time')
subplot(2,2,2)
plot(t,u_ussb)
title('USSB Signal')
xlabel('Time')
subplot(2,2,3)
plot(df1,abs(U_DSB))
title('Spectrum of the Modulated DSB Signal')
xlabel('Frequency')
subplot(2,2,4)
plot(df1,abs(U_USSB))
title('Spectrum of the Modulated USSB Signal')
xlabel('Frequency')

%% Task(2)

% Demodulate the above signals.
% Coherent carrier mixing.
y_dsb = u_dsb.*c;
y_ussb = u_ussb.*c;

% Fourier transform(Time domain to frequency domain).
[~,Y_DSB]=T2F(t,y_dsb);
[df1,Y_USSB]=T2F(t,y_ussb);

% Initialization of lowpass filter.
H_DSB = zeros(1,length(df1));
H_USSB = zeros(1,length(df1));
for i=1:length(df1)
    if abs(df1(i))<2
        H_DSB(i)=2*Ac_DSB_AM;
        H_USSB(i)=2*Ac_USSB_AM;
    end
end

% Lowpass filter filtering(Spectrum of the filter output).
DEM_DSB=H_DSB.*Y_DSB;
DEM_USSB=H_USSB.*Y_USSB;

% Fourier transform(Frequency domain to time domain).
[~,dem_w_dsb]=F2T(df1,DEM_DSB);
[dt1,dem_w_ussb]=F2T(df1,DEM_USSB);
dem_w_dsb=dem_w_dsb(1:length(t));
dem_w_ussb=dem_w_ussb(1:length(t));

% ¢Ù The pass band of the low-pass filter for DSB_AM and USSB_AM.
passband_dsb = 2*1
passband_ussb = 2*1
% ¢Ú Plot the demodulated signal and the spectrum.
figure(2)
subplot(2,2,1)
plot(dt1,real(dem_w_dsb))
title('Demodulated DSB signal')
xlabel('Time')
subplot(2,2,2)
plot(dt1,real(dem_w_ussb))
title('Demodulated USSB signal')
xlabel('Time')
subplot(2,2,3)
plot(df1,(abs(DEM_DSB)))
title('Spectrum of the Demodulated DSB signal')
xlabel('Frequency')
subplot(2,2,4)
plot(df1,(abs(DEM_USSB)))
title('Spectrum of the Demodulated USSB signal')
xlabel('Frequency')

%% Task(3)

% Add the thermal noise.
noise = randn(1,length(t))*sqrt(N0/2*Fs);
u_n_dsb = u_dsb + noise;
u_n_ussb = u_ussb + noise;

% Coherent carrier mixing.
y_n_dsb = u_n_dsb.*c;
y_n_ussb = u_n_ussb.*c;

% Fourier transform(Time domain to frequency domain).
[~,Y_N_DSB]=T2F(t,y_n_dsb);
[df1,Y_N_USSB]=T2F(t,y_n_ussb);

% Lowpass filter filtering(Spectrum of the filter output).
% Filters have been intitializated already.
DEM_N_DSB=H_DSB.*Y_N_DSB;
DEM_N_USSB=H_USSB.*Y_N_USSB;

% Fourier transform(Frequency domain to time domain).
[~,dem_n_dsb]=F2T(df1,DEM_N_DSB);
[dt1,dem_n_ussb]=F2T(df1,DEM_N_USSB);

% ¢ÙPlot the modulated signal and demodulated signal(time domain).
figure(3)
subplot(2,2,1)
plot(t,u_n_dsb)
title('Modulated DSB signal with noise')
xlabel('Time')
subplot(2,2,2)
plot(t,u_n_ussb)
title('Modulated USSB signal with noise')
xlabel('Time')
subplot(2,2,3)
plot(dt1,real(dem_n_dsb))
title('Demodulated DSB signal with noise')
xlabel('Time')
subplot(2,2,4)
plot(dt1,real(dem_n_ussb))
title('Demodulated USSB signal with noise')
xlabel('Time')

%% Task(4)

% What is the disadvantage
% if we apply a low-pass filter with a band much wider than necessary?

% Initialization of lowpass filter much wider than necessary.
H_W_DSB = zeros(1,length(df1));
H_W_USSB = zeros(1,length(df1));
for i=1:length(df1)
    if abs(df1(i))<200      % 200 >> 2
        H_W_DSB(i)=2*Ac_DSB_AM;
        H_W_USSB(i)=2*Ac_USSB_AM;
    end
end

% Lowpass filter filtering(Spectrum of the filter output).
DEM_W_DSB=H_W_DSB.*Y_DSB;
DEM_W_USSB=H_W_USSB.*Y_USSB;

% Fourier transform(Frequency domain to time domain).
[~,dem_w_dsb]=F2T(df1,DEM_W_DSB);
[dt1,dem_w_ussb]=F2T(df1,DEM_W_USSB);
dem_w_dsb=dem_w_dsb(1:length(t));
dem_w_ussb=dem_w_ussb(1:length(t));

% The demodulated signal and the spectrum.
figure(4)
subplot(2,2,1) 
plot(dt1,real(dem_w_dsb))
title('Demodulated DSB signal with a much wider band')
xlabel('Time')
subplot(2,2,2)
plot(dt1,real(dem_w_ussb))
title('Demodulated USSB signal with a much wider band')
xlabel('Time')
subplot(2,2,3)
plot(df1,(abs(DEM_DSB)))
title('Spectrum of the Demodulated DSB signal with a much wider band')
xlabel('Frequency')
subplot(2,2,4)
plot(df1,(abs(DEM_USSB)))
title('Spectrum of the Demodulated USSB signal with a much wider band')
xlabel('Frequency')