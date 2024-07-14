function [match, index] = closest_match(values, searchValue) 
    [match, index] = min(abs(values - searchValue));
end
