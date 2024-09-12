b_2 = -13.8341;
b_1 = 51.0314;
estimated_b = b_1 + 2.5 * b_2;

Kt_b = 0.124 * 120 / estimated_b;
J_b = 9.1617 / estimated_b;

sys = tf(Kt_b, [J_b 1]);

[times, currents, velocities, positions] = read_data("./data/chirp0to25at2.5A.csv");
[omega, m, theta] = bode_from_chirp(times, currents', velocities');

[omega_sim, m_sim, theta_sim] = bode_from_chirp(out.current.Time, out.current.Data', out.velocity.Data');

[sysM, sysTheta] = bode(sys, omega);
sysM = 20*log10(reshape(sysM(1,1, :), [], 1)');

figure
semilogx(omega, m)
hold on
semilogx(omega, sysM)
semilogx(omega_sim, m_sim)
legend("Measured Data", "Expected First Order System", "Simulated System", "Location", "southwest")
title("Magnitude Plot")
xlabel("Frequency (rads^-1)")
ylabel("Magnitude (db)")

figure
semilogx(omega, theta);
hold on
semilogx(omega, reshape(sysTheta(1,1, :), [], 1)')
semilogx(omega_sim, theta_sim)
legend("Measured Data", "Expected First Order System", "Simulated System", "Location", "southwest")
title("Phase Plot vs Frequency")
xlabel("Frequency (rads^-1)")
ylabel("Phase (degrees)")


function newTs = interpolateTimeseries(ts, spacing)
    time = ts.Time;
    data = ts.Data;
    
    tt = timetable(milliseconds(time * 1000), data);
    newtimes = time(1):spacing:time(end)
    ttnew = retime(tt, milliseconds(newtimes*1000), 'linear');
    newTs = timeseries(ttnew.data, seconds(ttnew.Time));
end