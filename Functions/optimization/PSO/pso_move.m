    % Move all the particles toward (xo, yo)
    function pos = pso_move(pos,poso,a,b,range)
        %pos = matrix, where the columns are the position vectors
        %poso = posvector of current global minimum
        nn=size(pos,2); %a=alpha, b=beta
        dim = size(pos,1);
        for i = 1 : nn
            pos(:,i) = pos(:,i).*(1-b)+poso.*b+a.*(rand(dim,1)-0.5);
        end
        pos = findrange(pos,range);
    end