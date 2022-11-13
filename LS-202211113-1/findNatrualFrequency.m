function [minFreq] = findNatrualFrequency(dampingRatio, settingTime)
    minFreq = 4/(dampingRatio*settingTime);
end