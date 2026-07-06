fs    = 8000;
R_ref = 1000;
R_dut = 470;
C_dut = 1e-6;

% logarithmically spaced frequencies, 10 Hz to 20 kHz
% but cap at fs/2 = 4000 Hz to stay below Nyquist
freqs = logspace(log10(10), log10(fs/2), 100);

N = 8000;
t = (0:N-1)'/fs;

function X = goertzel(x, omega)
    coeff        = 2*cos(omega);
    phase_factor = exp(-1j*omega);
    w1 = 0; w2 = 0;
    for n = 1:length(x)
        w0 = x(n) + coeff*w1 - w2;
        w2 = w1;
        w1 = w0;
    end
    X = w1 - phase_factor*w2;
end

Z_meas = zeros(size(freqs));

for k = 1:length(freqs)
    f0    = freqs(k);
    omega = 2*pi*f0/fs;

    % true impedance of series RC at this frequency
    Z_true = R_dut + 1/(1j*2*pi*f0*C_dut);

    % voltage divider signals
    H_ref = R_ref / (Z_true + R_ref);
    H_dut = Z_true / (Z_true + R_ref);

    x_ref = real(H_ref * exp(1j*2*pi*f0*t));
    x_dut = real(H_dut * exp(1j*2*pi*f0*t));

    X_ref = goertzel(x_ref, omega);
    X_dut = goertzel(x_dut, omega);

    Z_meas(k) = (X_dut / X_ref) * R_ref;
end

% Bode plot
figure;
subplot(2,1,1);
loglog(freqs, abs(Z_meas), 'b', freqs, abs(R_dut + 1./(1j*2*pi*freqs*C_dut)), 'r--');
ylabel('|Z| (ohm)');
legend('measured', 'true');
grid on;

subplot(2,1,2);
semilogx(freqs, angle(Z_meas)*180/pi, 'b', freqs, angle(R_dut + 1./(1j*2*pi*freqs*C_dut))*180/pi, 'r--');
ylabel('phase (deg)');
xlabel('frequency (Hz)');
legend('measured', 'true');
grid on;