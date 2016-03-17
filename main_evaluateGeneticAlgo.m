%This script controls the sequence and execution of the algorithms.
%The script has following structure:
% - setting global values/ parameters
% - description of the problem, including a robotmodel and a cartesian path
% - calculation of selfmotion trajectories for each point in cartesian path
% - planning of trajectory in jointSpace by optimizing the objective function
% - plots


close all;

addpath(genpath('Classes'));
addpath(genpath('Functions'));

javaaddpath('Functions/xlwrite/jxl.jar');
javaaddpath('Functions/xlwrite/MXL.jar');

 import mymxl.*;
 import jxl.*;   

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

%% Global Values initialization

% Optimization
springConstant = 1;
torsionSpringConstant = 0;
springEnergyWeight = 0; 
manipWeight = 1;
torsionEnergyWeight = 1;

%Selfmotion Integration
odeSolver_MaxStep = 0.1;
odeSolver_RelTol = 1e-3; %1e-3 ist default value

%Global Optimum Search
numberOfInitialValues_globalSearch = 10;

%Genetic Algorithm
genAlgo_elitism = true;
genAlgo_mutationRate = 0.2;
genAlgo_populationSize = 800; %1000
genAlgo_maxGenerations = 100; %2000
genAlgo_maxConvergence = 10;
genAlgo_newIndividualsPerGeneration = round(genAlgo_populationSize/10);

%Particle Swarm Optimization
PSO_alpha = 0.01;   %weight of random movement
PSO_beta = 0.05;    %weight of movement to current optimum
PSO_swarmsize = 10;
PSO_maxIterations = 100;

%% Simulation Parameters
nWaypoints=5;                      %number of points in cartesian Space
useRestingLengths = false;          %
standardPathNo = 'randomLinear';                         %1-4 / 'randomLinear'
deleteConfiguration = 'elbowDown';  %'elbowUp'/ 'elbowDown'
globalOptimMethod = 'geneticAndFminsearch';   %'fminsearch'/ 'genetic'/ 'geneticAndFminsearch' / 'particleSwarm'

%% Robot Description
robot3R = Robot;
robot3R.segmentLengths = [ 1.5, 1, 0.5 ];
robot3R.jointMaxLimits = [ pi, pi, pi ];
robot3R.jointMinLimits = [ -pi, -pi, -pi];
robot3R.numberOfJoints = 3;


pathNo=1;
while pathNo<=2
    try
tStart = tic;

%% Path of the Robot in cartesian Space
[robot3R, qStart] = robot3R.setStandardMovement(standardPathNo, nWaypoints); %initilizes robot.pointArray
qTarget = []; %if qTarget is empty, it will be determined by the optimization algorithm

%% Calculate Selfmotion Trajectories
tic; disp(' # Calculate Selfmotion');
for i=1:length(robot3R.pointArray) % for every Cartesian Point in robot.pointArray
    robot3R.pointArray(i) = robot3R.pointArray(i).calculateTrimRemodelAndFitSelfmotion(robot3R, deleteConfiguration, 'fourier6', 0.2);
end
toc; disp(' ');
if(useRestingLengths)
    tic;
    disp(' # Calculate Shortest Common Normals');
    robot3R.pointArray = robot3R.pointArray.calculateShortestCommonNormal();
    toc; disp(' ');
else
    robot3R.pointArray = robot3R.pointArray.setShortestCommonNormalZero();
end

    %Optimize the Trajectory for the current Path in cartesian Space i times
    %and safe the results each time in an excel spreadsheet
    i=1;
    while i<=5
        try
            disp(['Approach Run Number: ', num2str(i),'  Path Number: ', num2str(pathNo)]);

    %% Optimize Trajectory in Jointspace
    robot3R = robot3R.findPath(qStart, qTarget, globalOptimMethod);

    %% Evaluation of genetic algorithm
    %safe value of objective function after genetic algorithm
    objectiveGenetic = objectiveFunction2( robot3R, [], [], robot3R.geneticPath, 'linear' );
    %safe value of objective function after genetic algorithm and fminsearch
    objectiveGeneticAndFminsearch = objectiveFunction2( robot3R, [], [], robot3R.optimalPath, 'linear' );
    objectiveHeuristic = objectiveFunction2( robot3R, [], [], robot3R.heuristicPath, 'linear' );
    
    %% Put the data in a nice format
    dataTable = {'i','bisection','after gen. algorithm', 'after fminsearch', [], [],'PathNo', 'total elapsed Time [min]';...
                i,objectiveHeuristic,objectiveGenetic, objectiveGeneticAndFminsearch,[], [], pathNo, round(toc(tStart)/60)};

    %% Save in an Excel Spreadsheet
    filename = ['evaluate',globalOptimMethod,'_autogenerated.xlsx'];
    OS = system_dependent('getos');
    if(strcmp(OS(1:5),'Linux'))
        xlwrite(filename,dataTable,pathNo,['B', num2str(lineInExcelSheet)]);
    end

    i=i+1;
    
        catch exception
            %try again
            msgString = getReport(exception);
            disp(msgString);
        end
        
    end %iteration over optimisation attempts

    %% Save Plots
        %Plot 1 - Jointspace
            Plotter.selfmotionTrajectories(robot3R);                %Selfmotion Trajectories
            Plotter.jointLimitCube(robot3R);                        %the joint Limits as a red cube
            %plot all solutions (global, genetic and heuristic)
            %global
                plotNo(1)=Plotter.springsInJointSpace(robot3R.optimalPath, 1); %Springs in JointSpace
            %heuristic
                plotNo(2)=Plotter.springsInJointSpace( robot3R.heuristicPath, 2); %Springs in JointSpace
                legendEntries = { 'fminsearch', 'Bisection'};
            %genetic
                if (strcmp(globalOptimMethod, 'genetic') || strcmp(globalOptimMethod, 'geneticAndFminsearch'))
                    plotNo(3)=Plotter.springsInJointSpace( robot3R.geneticPath, 3); %Springs in JointSpace
                    legendEntries = { 'Genetic & fminsearch', 'Bisection','Genetic'};
                end
            %legend
                legend(plotNo,legendEntries); %add Legend
        %safe
        savefig(['_SelfmotionJointSpace_'...
            'PathNo', num2str(pathNo)]);
        close all;
    
        %Plot 2 - Cartesian Space
            Plotter.animateRobotStatic (robot3R , 1);
        print(['_SelfmotionCartesianSpace_'...
            'PathNo', num2str(pathNo)],...
            '-dpng');
        close all;
        
    pathNo=pathNo+1;
    catch exception
        %try again
        msgString = getReport(exception);
        disp(msgString);
    end
end %iteration over random cartesian paths

%% Safe Parameters
     dataTable = {...
            'Simulation Parameters', [],[],[],[];...
            [],'Number of Waypoints',nWaypoints,[],[];...
            [],'use Resting Lengths',useRestingLengths,[],[];...
            [],'delete Configuration',deleteConfiguration,[],[];...
            [],'Global Optimization Method',globalOptimMethod,[],[];...
            [],[],[],[],[];...
            'Manipulator Parameters',[],[],[],[];...
            [],'segmentLengths',robot3R.segmentLengths(1),robot3R.segmentLengths(2),robot3R.segmentLengths(3);...
            [],'jointMinLimits',robot3R.jointMinLimits(1),robot3R.jointMinLimits(2),robot3R.jointMinLimits(3);...
            [],'jointMinLimits',robot3R.jointMinLimits(1),robot3R.jointMinLimits(2),robot3R.jointMinLimits(3);...
            [],[],[],[],[];...
            'Objective Function',[],[],[],[];...
            [],'springConstant',springConstant,[],[];...
            [],'torsionSpringConstant',torsionSpringConstant,[],[];...
            [],'springEnergyWeight',springEnergyWeight,[],[]; ...
            [],'manipWeight',manipWeight,[],[];...
            [],'torsionEnergyWeight',torsionEnergyWeight,[],[];...
            [],[],[],[],[];...
            'Selfmotion Integration',[],[],[],[];...  
            [],'odeSolver_MaxStep', odeSolver_MaxStep,[],[];...
            [],'odeSolver_RelTol', odeSolver_RelTol,[],[];...
            [],[],[],[],[];...
            'Global Optimum Search',[],[],[],[];...
            [],'numberOfInitialValues_globalSearch',numberOfInitialValues_globalSearch,[],[];...
            [],[],[],[],[];...
            'Genetic Algorithm',[],[],[],[];...
            [],'genAlgo_elitism',genAlgo_elitism,[],[];...
            [],'genAlgo_mutationRate',genAlgo_mutationRate,[],[];...
            [],'genAlgo_populationSize',genAlgo_populationSize,[],[];...
            [],'genAlgo_maxGenerations',genAlgo_maxGenerations,[],[];...
            [],'genAlgo_maxConvergence',genAlgo_maxConvergence,[],[];...
            [],'genAlgo_newIndividualsPerGeneration',genAlgo_newIndividualsPerGeneration,[],[];...
              };

        %% Save in an Excel Spreadsheet
        filename = ['evaluate',globalOptimMethod,'_autogenerated.xlsx'];
        lineInExcelSheet = 2;
        OS = system_dependent('getos');
        if(strcmp(OS(1:5),'Linux'))
            xlwrite(filename,dataTable,'Parameters',['B', num2str(lineInExcelSheet)]);
        end
        
%% Plots
%Plotter.compareHeuristicOptimal(robot3R, qStart, qTarget);
Plotter.standardPlots(robot3R, 1);
%Plotter.saveMovieAnimateRobot(robot3R, 1, 30)

returnRobot = robot3R;