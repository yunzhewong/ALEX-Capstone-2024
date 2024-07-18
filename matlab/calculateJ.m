radius = 0.06 / 2; % used with incorrect
length = 0.055;
density = 7850;

volume = pi * radius * radius * length;
mass = volume * density;

J = 0.5 * mass * radius * radius