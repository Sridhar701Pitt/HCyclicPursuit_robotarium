function [L_new] = AddHierarchyLaplacian(L, new, node)
% add the node k to leader j

% update the leader/follower network based on the modification requirement
L(node, new) = -1; % add node k to j

% find the row idx of all the rows with -1
% for finding the blk of cyclic matrix
blkNew = find(ismember(L(:,new),-1))';

% Generate cyclic matrix for filling up the updated node layout
A = eye(size(blkNew,2));
for i = 1:size(A)
    A(i,i) = 0;
    A(mod((i),size(A))+1,i) = 1;
end
A = reshape(A',1,numel(A));

% update the blk matrix of the leader with the standard cyclic one
idx = 1;
for a = blkNew(1,:)
    for b = blkNew(1,:)
        L(a,b) = A(idx);
        idx = idx+1;
    end
end

L_new = L;
end
