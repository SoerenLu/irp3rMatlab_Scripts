function sliderPos = calculatePathSwarmApprox( robot , qStart, qTarget, pointArray )
%Calculates a random path for selfmotion trajectories in pointArray
%   q_start is the starting point of robot
%   q_target is the target point
%   if qTarget = NaN, the last point in pointArray is the Targetpoint.
%   in this case qTarget has not yet been determined.


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

i=unidrnd(size(sequences,1));%for all sequences
    
    %select a sequence and save its trajectories in currentTrayArray
    currentTrajArray = SelfTraj; %declaration of currentTrajArray
    for j=1:nWaypoint
        currentTrajArray(j) = pointArray(j).selfTrajArray(sequences(i,j));
        %initial point for x in fminsearch
    end


%get a random parameter for the trajectories
t_min = rand(nWaypoint,1);

sliderPos=zeros(1,dim);
for i=1:nWaypoint
       sliderPos(i,:) = currentTrajArray(i).qApprox(t_min(i));
end

end

