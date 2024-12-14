%% Final Project Code  
% Team 7 

%LIMITED JOINT MOVEMENT
%[myCobot,arm_info] = importrobot('MyCobot600_Simulink.slx');         
%LESS LIMITED -- currently uploaded on the Pathfeedback Simulink
 [myCobotBU,arm_info2] = importrobot('MyCobot600_BackUp.slx');  

%% Position Adjustments 

%read csv file 
Input = readtable("InterpolationPoints.csv");
X = Input.Var1;
Y = Input.Var2;

%start position 
x_start = X(1);
y_start = Y(1);

points = length(X); %number of interpolation points

% offsets for the maze path
xoff = X' - x_start; %need this to be offset from the point of origin
yoff = Y' - y_start;

time = 0:5:(points-1)*5;

x_Os = timeseries(xoff, time);
y_Os = timeseries(yoff, time);

%% Run Simulink 
stoptim = num2str(5*points);

out = sim('Cobot600_PathFeedback.slx', 'StopTime', stoptim);

%% Reshaping Data from Simulink Twin
IK_input = reshape(out.IK_target,[],3);
IK_Jo = reshape(out.IKJointData,6,[])';
% Correct J2, J4, J5 (J4 swith the J2/3 Interaction)
    IK_Jo(:,4) = -180 - IK_Jo(:,2) - IK_Jo(:,3);
    IK_Jo(:,2) = IK_Jo(:,2) -90;
    IK_Jo(:,5) = 90;

%% Find angles input with min error 
error = out.IK_error;
etot = sum(abs(error),2);
index = 10;

%Extract correlated angles
SolJoints = zeros(points,6);
posCheck = zeros(points,3);
erIx = zeros(points,1);
Ix = 1:10; % 0.5 step size for 5 second intervals
for i = 1:points
    k = find(etot==min(etot(Ix))); % locates min value 
    erIx(i) = k(1)-1; %gets first min instance, delay of one due to sim delay
    SolJoints(i,:) = IK_Jo(erIx(i),:); % error delay from joint output
    posCheck(i,:)=IK_input(erIx(i),:);
    Ix = Ix + 10;
end
%% Print Joints
disp(SolJoints)

writematrix(SolJoints,'joints','delimiter',',');

%% Get Data from Matlab FK Code --don't need this anymore???
% Initialize 
Code_EE_pos = zeros(points,3);
% Get end effector data from Sim output
for i = 1:points
    the1 = SolJoints(i,1);
    the2 = SolJoints(i,2);
    the3 = SolJoints(i,3);
    the4 = SolJoints(i,4);
    the5 = SolJoints(i,5);
    the6 = SolJoints(i,6);
    [Px, Py, Pz] = MyCobot600_FK_fun(the1,the2,the3,the4,the5,the6);
    Code_EE_pos(i,:) = [Px, Py, Pz];
    if Pz < 20
        disp("Under 20mm check before running... :(")
    end 
end

disp(Code_EE_pos)
code_error = posCheck - Code_EE_pos
%%
check = [X Y] -posCheck(:,1:2);
if sum(check) == 0 
    disp('data matches input!')
else 
    disp('FAIL')
end