function [laplacian,numElements] = CyclicHierarchyLaplacian(structure)
% if there are 3 layers, structure will be like, {[2]; [2,3]; [1,2,1,3,2]}
% if there are 2 layers, structure will be like this {[3];[3,3,3]}
    % find length of longest vector 
    maxLength = max(cellfun(@length, structure));
    % pad all vectors with 0 to fit this length
    A = cellfun(@(a) padarray(a, [0, maxLength - length(a)], 0, 'post'), structure, 'UniformOutput', false);
    % turn into matrix and perform operations
    structureMat = cell2mat(A);
    numElements = sum(structureMat,'all') + 1;

    cellArrMat = {};
    indexI = 0;
    levelI = 1;
    horzIdx = 1;

    [~,cellArrMatF] = recurse (indexI, cellArrMat, levelI, structureMat, horzIdx);
       
    finalMatrix = blkdiag(cellArrMatF{:});

    count = 0;
    idxArr = zeros(1,length(cellArrMatF)-1);
    for k=1:length(cellArrMatF)-1
        count = count + length(cellArrMatF{k});
        idxArr(k) = count;
    end

    for j = 1:length(idxArr)
        if j == length(idxArr)
            k = idxArr(1);
        else
            k = j+1;
        end
        % Add -1
        finalMatrix(idxArr(j)+1,1) = -1;
        % Add 1
        
        finalMatrix(idxArr(k)+1,idxArr(j)+1) = 1;
    end

    laplacian = finalMatrix;
%     buildArray = zeros(elements);
%     indexCount = elements;
%     i = layers
% %     for i = layers:-1:1
%     preLeaderCount = structureMat(i-1,end)
%     for j = length(structureMat(i,:)):-1:1
%         N = structureMat(i,j)
%         smallMat = diag(ones(N-1,1),-1);
%         smallMat(1,N) = 1;
%     
%         bigMat = [zeros(1,N+1);[[-1*ones(N,1)],smallMat]]
%         
%         buildArray(indexCount - (N):indexCount,indexCount - (N):indexCount) = bigMat;
%         indexCount = indexCount - (N+1)
% 
% 
%     end
% %     end

end


function [indexF,cellArrMatF] = recurse (indexI, cellArrMat, levelI, structureMat, horzIdx)
% if there are 3 layers, structure will be like, {[2]; [2,3]; [1,2,1,3,2]}
    indexI = indexI + 1;
    cellArrMat{indexI} = [0];
    if levelI < size(structureMat,1)
        for horzii = 1:structureMat(levelI,horzIdx)
            levelF = levelI + 1;
            [indexI,cellArrMat] = recurse(indexI, cellArrMat, levelF, structureMat, horzii + sum(structureMat(levelI,1:horzIdx-1))); 
        end
    else
        N = structureMat(levelI,horzIdx);
        smallMat = diag(ones(N-1,1),-1);
        smallMat(1,N) = 1;
    
%         indexI = indexI + 1;
        bigMat = [zeros(1,N+1);[[-1*ones(N,1)],smallMat]];
        cellArrMat{indexI} = bigMat;
    end
    indexF = indexI;
    cellArrMatF = cellArrMat;
end
