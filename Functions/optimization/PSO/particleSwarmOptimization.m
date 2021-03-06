% The Accelerated Particle Swarm Optimization
% (written by X S Yang, Cambridge University)
% Usage: pso(number_of_particles, Num_iterarions)
% eg: best = pso_demo(20, 10);
% where best = [xbest ybest zbest] %an n by 3 matrix
% xbest(i)/ybest(i) are the best ith iteration

% Modified by S�ren Langhorst, IRP TU Braunschweig 2016

function [alltimeBestPos, alltimeMinval] = particleSwarmOptimization(objFun, dim, n , Num_iterations, Num_iterationsWithoutImprovement, range)
% objFun= objective Function
% dim = dimension of search space (number of Waypoints)
% n=number of particles
% Num_iterations=total_number of iterations

global PSO_alpha;
global PSO_beta;

% return: 
% poso = positionvector of minimum value
% minval = value of objective fuction at poso
if nargin<6  
    range = {[0,1]};
    for i=2:dim
        range(i) = {[0,1]};
    end
end
if nargin<5,    Num_iterations=10;  end
if nargin<4,    n=20;   end
% range = [xmin xmax];


%% Start Particle Swarm Optimization 
% generation the initial locations of n particles in dim dimensions
% pos is a matrix, storing the positionvectors of particles in its columns
pos = init_pso(dim, n ,range);

%Plot the minvalue
figure(1);
alltimeMinval = inf;

% Start iterations
i=1;
noImprovementCounter=0;
while( i<=Num_iterations && noImprovementCounter < Num_iterationsWithoutImprovement)
    % Find the current best location
    zn = zeros(n,1);
    
    tic
    parfor j=1:n
        % objFun of j-th particle   
        zn(j) = objFun(pos(:,j));
        %objectiveFunction(pos(:,j), objFunData{1}, objFunData{2}, objFunData{3}, objFunData{4}, objFunData{5}, objFunData{6});
    end
    
    %determine value of current global minimum
    currentMinval=min(zn);
    %determine position of current global minimum
    for j=1:n
        if zn(j)==currentMinval
            currentBestPos = pos(:,j);
        end
    end

    % The accelerated PSO with alpha=gamme^t
    %gamma=0.7; 
    %alpha=gamma.^i;
    
    % Move all the particles to new locations
    pos = pso_move(pos,currentBestPos, PSO_alpha, PSO_beta, range);
    
    %Only temporarily!!!!!!
    warning('Remove this code after testing');
    %choose n random particles:
    nRandom = 10;
    for k =1:nRandom
        pick = unidrnd(n);
        fminPos = zeros(dim, 1);
        fminPos = pos(:, pick );
        [tMin, minVal] = fminsearch(@(x)objFun(x),  fminPos ); 
        pos(:,pick) = tMin;
    end
    
    %Check if a better minimum could be found
    if currentMinval < alltimeMinval
        alltimeMinval = currentMinval;
        alltimeBestPos = currentBestPos;
        noImprovementCounter = 0;
    else
        noImprovementCounter = noImprovementCounter + 1;
    end
    
    disp(['Time for this Iteration: ', num2str(toc),'sec; Current obj.fun.: ', num2str(alltimeMinval),...
        '; No change since ', num2str(noImprovementCounter), ' Iterations; Iteration No: ', num2str(i)]); 

    hold on; plot(i, currentMinval,'go',i,alltimeMinval,'rx');
    %hold off; plot(pos(1,:),pos(2,:),'.',pos_min(1),pos_min(2),'*');
    drawnow;
    
    i=i+1;
end %end of iterations

    
end