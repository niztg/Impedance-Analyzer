fs    = 8000;
f0    = 1000;
N     = 8000;
R_ref = 1000;

% random complex impedances — real part 100-2000 ohm, imaginary part -500 to 500 ohm
n_tests = 10;
Z_true = (100 + 1900*rand(n_tests,1)) + 1j*(rand(n_tests,1)-0.5)*1000;

omega        = 2*pi*f0/fs;
coeff        = 2*cos(omega);
phase_factor = exp(-1j*omega);
t            = (0:N-1)'/fs;

function X = goertzel(x, coeff, phase_factor)
    w1 = 0; w2 = 0;
    for n = 1:length(x)
        w0 = x(n) + coeff*w1 - w2;
        w2 = w1;
        w1 = w0;
    end
    X = w1 - phase_factor*w2;
end

for k = 1:n_tests
    Z = Z_true(k);
    phase_shift = angle(Z / (Z + R_ref));
    amplitude   = abs(Z / (Z + R_ref));

    V_in = 1;
    H = Z / (Z + R_ref);          % complex voltage divider, no separation needed
    x_ref = real(R_ref/(Z+R_ref) * exp(1j*2*pi*f0*t));
    x_dut = real(H * exp(1j*2*pi*f0*t));

    X_ref = goertzel(x_ref, coeff, phase_factor);
    X_dut = goertzel(x_dut, coeff, phase_factor);

    Z_meas = (X_dut / X_ref) * R_ref;

    fprintf('true: %6.1f + %6.1fj   measured: %6.1f + %6.1fj   err: %.4f%%\n', ...
        real(Z), imag(Z), real(Z_meas), imag(Z_meas), ...
        100*abs(Z_meas - Z)/abs(Z));
end