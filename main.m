%This script controls the sequence and execution of the algorithms.
%The script has following structure:
% - setting global values/ parameters
% - description of the problem, including a robotmodel and a cartesian path
% - calculation of selfmotion trajectories for each point in cartesian path
% - planning of trajectory in jointSpace by optimizing the objective function
% - plots

% @author S�ren Langhorst - IRP TU BS 2016
% @email soeren.langhorst@gmail.com

close all;

addpath(genpath('Classes'));
addpath(genpath('Functions'));

%% Global Values declaration
global springConstant       %value of Spring Constant
global torsionSpringConstant %value of Torsion Spring Constant
global springEnergyWeight   %Weight of spring energy in objective function
global manipWeight          %Weight of manipulability in objective function
global torsionEnergyWeight  %Weight of torsion springs in objective function
global numberOfInitialValues_globalSearch %Number of guesses for the Initial value of fminsearch
global odeSolver_MaxStep;
global odeSolver_RelTol;
global genAlgo_elitism;
global genAlgo_mutationRate;
global genAlgo_populationSize;
global genAlgo_maxGenerations;
global genAlgo_maxConvergence; %number of maximum consecutive generations with the same best fitness
global genAlgo_newIndividualsPerGeneration; %number of completely new individuals each generation
global PSO_alpha;
global PSO_beta;
global PSO_swarmsize;
global PSO_maxIterations;
global PSO_maxIterationsWithoutImprovement;

%% Global Values initialization

% Optimization
springConstant = 1;
torsionSpringConstant = 1;
springEnergyWeight = 1; 
manipWeight = 0.2;
torsionEnergyWeight = 1;

%Selfmotion Integration
odeSolver_MaxStep = 0.1;
odeSolver_RelTol = 1e-3; %1e-3 ist default value

%Global Optimum Search
numberOfInitialValues_globalSearch = 1;

%Genetic Algorithm
genAlgo_elitism = true;
genAlgo_mutationRate = 0.3;
genAlgo_populationSize = 100; %1000
genAlgo_maxGenerations = 2000; %2000
genAlgo_maxConvergence = 10;
genAlgo_newIndividualsPerGeneration = 0;%round(genAlgo_populationSize/10);

%Particle Swarm Optimization
PSO_alpha = 0.025;   %weight of random movement
PSO_beta = 0.5;    %weight of movement to current optimum
PSO_swarmsize = 10000;
PSO_maxIterations = Inf;
PSO_maxIterationsWithoutImprovement = 10; 

%% Simulation Parameters
nWaypoints=5;                      %number of points in cartesian Space
useRestingLengths = false;          %
standardPathNo = 2;    % 1/2/3/4/'randomLinear'
deleteConfiguration = 'elbowDown';  %'elbowUp'/ 'elbowDown'
globalOptimMethod = 'particleSwarm';   %'fminsearch'/ 'genetic'/ 'geneticAndFminsearch'/ 'particleSwarm'/ 'randomPath'

%% Robot Description
robot3R = Robot;
robot3R.segmentLengths = [ 1.5, 1, 0.5 ];
robot3R.jointMaxLimits = [ pi, pi, pi ];
robot3R.jointMinLimits = [ -pi, -pi, -pi];
robot3R.numberOfJoints = 3;

%The Paths are created Randomly. Sometimes, this causes errors. They have
%to be catched.
unsuccessful = true;
while(unsuccessful)
    try
        %% Path of the Robot in cartesian Space
        [robot3R, qStart] = robot3R.setStandardMovement(standardPathNo, nWaypoints); %initilizes robot.pointArray
        qTarget = []; %if qTarget is empty, it will be determined by the optimization algorithm

        %% Calculate Selfmotion Trajectories
        tic; disp(' # Calculate Selfmotion');
        for i=1:length(robot3R.pointArray) % for every Cartesian Point in robot.pointArray
            robot3R.pointArray(i) = robot3R.pointArray(i).calculateTrimRemodelAndFitSelfmotion(robot3R, deleteConfiguration, 'fourier8', 0.2);
        end
        %qTarget = optimizePoseManipulability(robot3R, robot3R.pointArray(end), true);

        toc; disp(' ');
        if(useRestingLengths)
            tic;
            disp(' # Calculate Shortest Common Normals');
            robot3R.pointArray = robot3R.pointArray.calculateShortestCommonNormal();
            toc; disp(' ');
        else
            robot3R.pointArray = robot3R.pointArray.setShortestCommonNormalZero();
        end
        
        unsuccessful = false;
    catch exception
        %try again
        msgString = getReport(exception);
        disp(msgString);
    end
end

%% Optimize Trajectory in Jointspace
robot3R = robot3R.findPath(qStart, qTarget, globalOptimMethod);

%% Plots
%Plotter.compareHeuristicOptimal(robot3R, qStart, qTarget);
Plotter.standardPlots(robot3R, 1);
%Plotter.saveMovieAnimateRobot(robot3R, 1, 30)

returnRobot = robot3R;
