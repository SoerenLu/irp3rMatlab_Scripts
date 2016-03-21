% Make sure the particles are within the range
function pos =findrange(pos,range)

% dim is the dimension of search space
dim = size( pos, 1);
% s is the swarmsize
n = size(pos, 2);

for i=1:dim,
    for j=1:n,
        if pos(i,j)<=range{i}(1), pos(i,j)=range{i}(1); end
        if pos(i,j)>=range{i}(2), pos(i,j)=range{i}(2); end
    end
end
end