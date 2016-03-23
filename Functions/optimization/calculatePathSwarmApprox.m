function [sliderPos, t] = calculatePathSwarmApprox( robot , qStart, qTarget, pointArray, energyType)
%Calculates the path of minimal objective function for selfmotion trajectories in pointArray
%   q_start is the starting point of robot
%   q_target is the target point
%   if qTarget = NaN, the last point in pointArray is the Targetpoint.
%   in this case qTarget has not yet been determined.
global PSO_swarmsize;
global PSO_maxIterations;
global PSO_maxIterationsWithoutImprovement;

global springConstant       %value of Spring Constant
global torsionSpringConstant %value of Torsion Spring Constant
global springEnergyWeight   %Weight of spring energy in objective function
global manipWeight          %Weight of manipulability in objective function
global torsionEnergyWeight  %Weight of torsion springs in objective function  

%% Values
nWaypoint = length(pointArray);       %how many waypoints are there?
dim = robot.numberOfJoints;             %dimension of the jointspace

%% Calcute the minimum Energy for each Parallel Trajectory Sequence
%  and find the sequence with the lowest Value(Numerically)
%  Description:
%   When there is more than one selfmotiontrajectory for a cartesian point,
%   there are more than one possible ways to go from start to target (i.e.
%   you can take the first, second, third...parallel trajectory).

%%Find all the possible sequences of parallel trajectories
%1. Find number of parallel trajectories at every cartesian point
nParallel=ones(nWaypoint,1);
for i=1:nWaypoint
    nParallel(i) = length(pointArray(i).selfTrajArray);
end

%2. Find all sequences (different sequeces of parallel Trajectories)
sequences=findTrajectorySequences(nParallel);
nSequences = size(sequences,1); % number of sequences

%Display the number of parallel Trajectories, so the user can estimate the
%amount of necessary calculation time
if nSequences==1
    disp('Only one possible Trajectory-Sequence has been found (no parallel Trajectories).')
else
    disp([num2str(nSequences),' different Trajectory-Sequences have been found.'])
end

%% select a sequence and calculate the corresponding energyminimum

minval=inf; %initial value of minimal objective function

for i=1:size(sequences,1)%for all sequences
    disp(['Sequence No ', num2str(i), ' of ', num2str(size(sequences,1))]);
  
    %select a sequence and save its trajectories in currentTrayArray
    currentTrajArray = SelfTraj; %declaration of currentTrajArray
    for j=1:nWaypoint
        currentTrajArray(j) = pointArray(j).selfTrajArray(sequences(i,j));
        %initial point for x in fminsearch
    end
    
    nParticles = 10000;
    maxIterations = 100;
    
    % Gather the data for the MEX-file (which cannot handle function
    % handles)
    % 
    %SelfTrajFourierCoefficientsQ1 = [ a0, a1, b1, a2, b2, a3, b3, a4, b4,..;
    %                               a0, a1, b1, a2, b2, a3, b3, a4, b4,..;
    %                               a0, a1, b1, a2, b2, a3, b3,...;
    %                                   ....]
    % coefficients for first joint and each trajectory:
    % line1 = selftraj1, line2 = selftraj2, line3 = selftraj3...
%     for i=1:nWaypoint %for all selfTrajectories
%         disp('Warning: assuming approximation with Fourier6');
%         selfTrajFourierCoefficientsQ1(i,:) = [  currentTrajArray(i).q1Approx.a0, currentTrajArray(i).q1Approx.a1, currentTrajArray(i).q1Approx.b1];
%     end

    parameterStruct={springConstant,torsionSpringConstant,springEnergyWeight,manipWeight,torsionEnergyWeight};
    fun = @(x)objectiveFunction(x, robot, qStart, qTarget, currentTrajArray, energyType, true, parameterStruct );
    %objFunData = {robot, qStart, qTarget, currentTrajArray, energyType, true,parameterStruct};
    [t_min_parallel, minval_parallel] = particleSwarmOptimization(fun,nWaypoint,PSO_swarmsize,PSO_maxIterations, PSO_maxIterationsWithoutImprovement);


        %if the energy for this sequence and this initial values is lower, take it.
        if minval_parallel<minval
            minval=minval_parallel;
            t_min=t_min_parallel;
            bestTrajArray = currentTrajArray;
        end
end

%check if t is out of boundaries
t_min = mod(t_min, 1);

sliderPos=zeros(1,dim);
for i=1:nWaypoint
       sliderPos(i,:) = bestTrajArray(i).qApprox(t_min(i));
end

t = t_min;

end

