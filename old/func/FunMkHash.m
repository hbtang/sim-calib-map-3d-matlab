function [ mark ] = FunMkHash( mark )
%FUNMKHASH generate hash table according to mark id
% mkhash: n*2 matrix, col(1) mark id, col(2) continuous orders

mkHash = unique(mark.id);
if size(mkHash,1) == 1
    mkHash = mkHash.';
end
numId = numel(mkHash);
mkHash(:,2) = (1:numId).';

mark.mkHash = mkHash;
mark.numId = numId;

end

