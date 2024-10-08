% Define the folder where your CSV files are located
folder = './delta_error';

% List all CSV files in the folder
files = dir(fullfile(folder, '*.csv'));

% Preallocate a cell array to store data
data = cell(length(files), 1);

% Preallocate an array to store the mean current values
mean_current = zeros(length(files), 1);

% Preallocate an array to store the mean velocity values
mean_velocity = zeros(length(files), 1);

% Initialize an empty array for mean_list_current
mean_list_current = [];

% Initialize an empty array for mean_list_velocity
mean_list_velocity = [];

% Loop through each file and load the data
for i = 1:length(files)
    filename = fullfile(folder, files(i).name);
    % Use 'VariableNamingRule', 'preserve' to retain original column names
    data{i} = readtable(filename, 'VariableNamingRule', 'preserve');

    % Adjust the time column to start from 0
    data{i}.Time = data{i}.Time - data{i}.Time(1);

    % Filter the data to include only rows where Time >= 0.11
    data{i} = data{i}(data{i}.Time >= 0.11, :);

    % Calculate the mean value of the current
    mean_current(i) = mean(data{i}.('10.10.10.30 Current'));

    % Calculate the mean value of the velocity
    mean_velocity(i) = mean(data{i}.('10.10.10.30 Velocity'));

    % Create mean_list_current for this file and concatenate with the existing array
    mean_list_current(i) = mean(data{i}.('10.10.10.30 Current'));

    % Create mean_list_velocity for this file and concatenate with the existing array
    mean_list_velocity(i) = mean(data{i}.('10.10.10.30 Velocity'));
end

% % Create mean_list_current for this file and concatenate with the existing array
% mean_list_current = [mean_list_current; mean_current(i) * ones(size(data{i}.('10.10.10.30 Current')))];
% 
% 
% % Create mean_list_velocity for this file and concatenate with the existing array
% mean_list_velocity = [mean_list_velocity; mean_velocity(i) * ones(size(data{i}.('10.10.10.30 Velocity')))];

% Initialize a figure for plotting all files
figure;

delta = (0.1:0.1:1.8)';

% Loop through each file to plot its data
for i = 1:length(files)
    % Extract the x-axis (Time) data
    x = data{i}.Time;

    % Access the relevant columns directly
    current = data{i}.('10.10.10.30 Current');
    velocity = data{i}.('10.10.10.30 Velocity');
    position = data{i}.('10.10.10.30 Position');

    % Ensure all vectors have the same length
    length_x = length(x);
    length_current = length(current);
    length_velocity = length(velocity);
    length_position = length(position);
    length_mean_current = length(mean_list_current);
    length_mean_velocity = length(mean_list_velocity);

    % Find the minimum length
    min_length = min([length_x, length_current, length_velocity, length_position, length_mean_current, length_mean_velocity]);

    % Trim the vectors to the minimum length
    x = x(1:min_length);
    current = current(1:min_length);
    velocity = velocity(1:min_length);
    position = position(1:min_length);
    mean_list_current = mean_list_current(1:min_length);
    mean_list_velocity = mean_list_velocity(1:min_length);

    % Plot the data for each file, using a different color/style
    plot(delta, mean_current, 'DisplayName', ['Current - File ' num2str(i)]);
    hold on;
    plot(delta, mean_velocity,'DisplayName', ['Velocity - File ' num2str(i)]);
    hold on;
    %'o',
end
figure;
plot(delta, velocity);
hold on;
plot(delta, mean_current);

% Finalize the plot
hold off;
grid on;

xlabel('Delta');
ylabel('Angular Velocity/Current');
title('Plot of Delta vs Angular Velocity/Current');
legend ('delta vs angular velocity', 'delta vs current average');


figure
plot(delta, mean_velocity)
xlabel("Delta")
ylabel("Steady-State Velocity (rads^{-1})")
title("Delta vs Measured Steady State Angular Velocity")

data = readmatrix("./delta_error/delta1.80.csv");
times = data(:, 1);
velocities = data(:, 3);
figure
plot(times, velocities)

p = polyfit(delta, mean_velocity, 1)

