close all


batch_names = ["batch 6"];

for b = 1:numel(batch_names)
    batch_name = batch_names(b);
    files = dir(fullfile("./data/" + batch_name, '*.csv'));

    figure
    hold on
    for i = 1:numel(files)
        filename = files(i).name;
    
        data = readmatrix("./data/" + batch_name ...
           + "/" + filename);
        
        times = data(:, 1);
        startTime = times(1);
        currents = data(:, 2);
        velocities = data(:, 3);
    
        amperage = mean(currents);
    
        plot(times - startTime, velocities, '-', "DisplayName", filename)
    end
    
    dcm = datacursormode(gcf);
    set(dcm, 'UpdateFcn', @myUpdateFcn);
    
end


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
