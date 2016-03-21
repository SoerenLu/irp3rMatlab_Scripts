% Initial locations of n particles
function pos =init_pso(dim, n, range)
    for i=1:dim
        drange=range{i}(2)-range{i}(1);
        pos(i,:)=rand(1,n)*drange+range{i}(1);
    end
end