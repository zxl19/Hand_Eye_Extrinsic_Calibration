function [index, error] = findNearest(element, sequence)
absError = abs(sequence - element);
[~, index] = min(absError);
error = absError(index);
end