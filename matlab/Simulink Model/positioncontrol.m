pos = out.posresp;
ref = out.ref;

times = ref.Time;
reference = ref.Data;
positions = pos.Data;

figure
plot(times, reference)
hold on
plot(times, positions)

legend("Reference Positions", "Measured Positions", "Location", "southeast")

xlabel("Time (s)")
ylabel("Position (rad)")
title("Position vs Time")