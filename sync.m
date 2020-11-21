function [data_1_sync, timestamp_1_sync, data_2_sync, timestamp_2_sync] = sync(data_1, timestamp_1, data_2, timestamp_2, threshold)
data_1_sync = [];
data_2_sync = [];
timestamp_1_sync = [];
timestamp_2_sync = [];
[m, ~] = size(data_1);
[n, ~] = size(data_2);
index = 1;
last_index = 1;
for i = 1 : m
    [ind, err] = findNearest(timestamp_1(i), timestamp_2(last_index : n));
    if err < threshold
        data_1_sync(index, :) = data_1(i, :);
        data_2_sync(index, :) = data_2(last_index + ind - 1, :);
        timestamp_1_sync(index, 1) = timestamp_1(i);
        timestamp_2_sync(index, 1) = timestamp_2(last_index + ind - 1);
%         fprintf('%f\t%f \n',timestamp_1(i), timestamp_2(last_index + ind
%         - 1)) % Print Synchronization Result
        index = index + 1;
        last_index = last_index + ind;
    else
        continue
    end
end