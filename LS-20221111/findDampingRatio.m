function [x1, x2] = findDampingRatio(overshoot)
    delta = (pi^4 + 4*(log(overshoot)^4))^(1/2);
    aBy2 = 2*log(overshoot)^2;
    minusB = -pi^2;
    x1 = (minusB-delta)/(aBy2);
    x2 = (minusB+delta)/(aBy2);
    if(x1 <= 0)
        x1 = 0;
    end
    if(x2 >= 1)
        x2 = 1;
    end
end
