clear
load('output/returnRobot_path2.mat')

%saving the optimal path
sliderPos = returnRobot.optimalPath(2:end,:);
qStart = returnRobot.optimalPath(1,:);
%saving a copy of optimal path in heuristic path
returnRobot.heuristicPath = returnRobot.optimalPath;

disp(['Optimal Obj Fun: ', num2str(objectiveFunction2( returnRobot, qStart, [], sliderPos))])

%saving heuristic path in sliderPos
returnRobot.heuristicPath(2:end,:)=returnRobot.heuristicPath(2:end,:)*0.8;
qStart = returnRobot.heuristicPath(1,:);
disp(['Distorted Obj Fun: ', num2str(objectiveFunction2( returnRobot, qStart, [], returnRobot.heuristicPath(2:end,:)))])

Plotter.standardPlots(returnRobot, 1, false);