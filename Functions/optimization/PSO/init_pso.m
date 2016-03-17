% Initial locations of n particles
    function pos =init_pso(dim, n,range)
        drange=range(2)-range(1);
        pos=rand(dim,n)*drange+range(1);
    end