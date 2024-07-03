
FILE_NAME = "../data/chirp0to5.csv";
DATA_COLUMN = 3;

table = readmatrix(FILE_NAME);

times = table(:, 1);
data = table(:, DATA_COLUMN);

length = numel(data);


startIndex = -1;
while startIndex == -1
    figure
    plot(1:length, data);
    hold on

    tempIndex = input("Select a start index (1:" + length + "): ");
    xline(tempIndex);

    txt = input("Confirm? (y/n): ", 's');

    if txt == "y"
        startIndex = tempIndex;
    end
end


endIndex = -1;
while endIndex == -1
    figure
    plot(1:length, data);
    hold on
    xline(startIndex)

    tempIndex = input("Select an end index (1:" + length + "): ");
    xline(tempIndex);

    txt = input("Confirm? (y/n): ", 's');

    if txt == "y"
        endIndex = tempIndex;
    end
end

slicedTimes = times(startIndex:endIndex);
firstTime = slicedTimes(1);

slicedTimes = slicedTimes - firstTime;
slicedData = data(startIndex:endIndex);


figure
plot(slicedTimes, slicedData)




