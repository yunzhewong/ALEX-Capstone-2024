close all

data = readmatrix("./data/batch 1/step1.00A.csv");
    
times = data(:, 1);
corrected_times = times - times(1);
currents = data(:, 2);

frequency_plot(corrected_times, currents, "Currents 1A")