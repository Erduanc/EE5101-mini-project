function [Abar, Bbar, T] = turnToControllableCanonicalForm(A, B)
    % nByn = size(A);
    nBym = size(B);
    % disp(nByn);
    n = nBym(1);
    m = nBym(2);
    % 1. transform the original system to controllable cannonical form
    Wc = []; % controllability matrix
    i = 0;
    while (i < n)
        tempA = A^i;
        Wc = [Wc, tempA * B];
        i = i+1;
    end
    
    chosenColsNum = zeros(1, n); chosenColsNum(1) = 1;
    C = Wc(1:n, 1);
    chooseFromIndex = 1; j = 1; 
    while(chooseFromIndex <= n*m && j <= n)
        chooseCol = Wc(1:n, chooseFromIndex);
        if(ifLinearlyIndependent(C, chooseCol))
            chosenColsNum(j+1) = chooseFromIndex;
            C = [C, chooseCol];
            j = j+1;
        end
        chooseFromIndex = chooseFromIndex + 1;
    end
    disp(chosenColsNum);
    disp(C);
    cTemp = zeros(n, n*m);
    cTempIndecies = zeros(1, n);
    CArranged = zeros(n, n);
    cCounts = ones(1, m); i = 1;
    while( i <= n)
        relatedBIndex = mod(chosenColsNum(i), m);
        if(relatedBIndex == 0)
            relatedBIndex = m;
        end
        cTempIndex = cCounts(relatedBIndex)+m*(relatedBIndex-1);
        cTempIndecies(i) = cTempIndex;
        cTemp(1:n, cTempIndex) = C(1:n, i);
        cCounts(relatedBIndex) =  cCounts(relatedBIndex) + 1;
        i = i+1;
    end
    disp(cTemp);
    i = 1; 
    cTempIndecies = sort(cTempIndecies);
    while( i <= n)
        CArranged(1:n, i) = cTemp(1:n, cTempIndecies(i));
        i = i+1;
    end
    disp(CArranged);
    CInv = inv(CArranged);
    disp(CInv);
    
    T = [];
    i = 1; qIndex = 0; totalRowsNum = 1;
    while (i <= m)
        di = cCounts(i)-1;
        qIndex = qIndex + di;
        j = 1; 
        while( j <= di)
            TRow = CInv(qIndex, 1:n)*A^(j-1);
            T = [T, TRow];
            j = j+1;
            totalRowsNum = totalRowsNum+1;
        end
        i = i+1;
    end
    T = reshape(T, n,[]).';
    disp(T);
%     TInv = inv(T);
%     disp(TInv)
    Abar = T*A/T;
    Bbar = T*B;
    disp(Abar); disp(Bbar);
end
%[x0, A, B, C] = buildStateSpaceModel();
%disp(x0); disp(A); disp(B); disp(C);