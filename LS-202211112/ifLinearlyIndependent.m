function [isIndependent] = ifLinearlyIndependent(A, B)
    isIndependent =  rank(A) ~= rank([A, B]);
end