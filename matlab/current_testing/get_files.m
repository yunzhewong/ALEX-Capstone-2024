function [names, count] = get_files(directory)
    files = dir(string(directory));

    % first two are . and .., ignore
    count = numel(files) - 2;

    for i = 1:count
        names(i) = "./" + string(directory) + "/" + files(i + 2).name;
    end
end