close all


files = dir(fullfile("./data", '*.csv'));


figure
hold on
for i = 1:length(files)
    filename = files(i).name;

    data = readmatrix("./data/" + filename);
    
    times = data(:, 1);
    currents = data(:, 2);
    velocities = data(:, 3);

    amperage = mean(currents);

    plot(times, velocities, '-', "DisplayName", amperage + "A")
end

legend("show")

dcm = datacursormode(gcf);
set(dcm, 'UpdateFcn', @myUpdateFcn);

% Custom update function to display plot name
function output_txt = myUpdateFcn(~, event_obj)
    % Get the target (the plot line)
    target = get(event_obj, 'Target');
    
    % Get the display name
    displayName = get(target, 'DisplayName');
    
    % Get the position of the data cursor
    pos = get(event_obj, 'Position');
    
    % Construct the output text
    output_txt = {['X: ', num2str(pos(1))], ...
                  ['Y: ', num2str(pos(2))], ...
                  ['Name: ', displayName]};
end
