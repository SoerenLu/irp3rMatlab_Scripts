% Make sure the particles are within the range
function pos =findrange(pos,range)
nn = size( pos, 1);
dim = size(pos, 2);

for i=1:nn,
    for j=1:dim,
        if pos(i,j)<=range(1), pos(i,j)=range(1); end
        if pos(i,j)>=range(2), pos(i,j)=range(2); end
    end
end
end