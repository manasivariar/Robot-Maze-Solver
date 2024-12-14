%% Final Project Code  
%Ts = 0.001; 
%[myCobot,arm_info] = importrobot('MyCobot600_Simulink.slx');
%[myCobotBU,arm_info2] = importrobot('MyCobot600_BackUp.slx');
%% Position Adjustments 

%read csv file 
Input = [ -180, -240; -220, -240; -220, -300; -220, -300];
X_Dem = Input(:,1);
Y_Dem = Input(:,2); 

%start position 
x_start = X_Dem(1);
y_start = Y_Dem(1);

points = length(X_Dem); %number of interpolation points

% offsets for the maze path
xoffD = X_Dem' - x_start; %need this to be offset from the point of origin
yoffD = Y_Dem' - y_start;

time = 0:5:(points-1)*5;

x_Os = timeseries(xoffD, time);
y_Os = timeseries(yoffD, time);

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

[m,n] = size(IK_input);

%% Find angles input with min error 
error = out.IK_error;
etot = sum(abs(error),2);
index = 10;

%Extract correlated angles
SolJoints = zeros(points,6);
posCheck = zeros(points,3);
erIx = zeros(points,1);
%Ix = 3:13; %offset by one due to error delay
Ix = 1:10; 
for i = 1:points
    k = find(etot==min(etot(Ix))); % locates min value 
    erIx(i) = k(1)-1; %gets first min instance, delay of one due to sim delay
    SolJoints(i,:) = IK_Jo(erIx(i),:); % error delay from joint output
    posCheck(i,:)=IK_input(erIx(i),:);
    Ix = Ix + 10;
end
%% Demo Exec
disp(points)
J1 = SolJoints(:,1);
J2 = SolJoints(:,2);
J3 = SolJoints(:,3);
J4 = SolJoints(:,4);
J5 = SolJoints(:,5);
J6 = SolJoints(:,6);

%% Safety Check 

check = Input-posCheck(:,1:2);
if sum(check) == 0 
    disp('data matches input!')
else 
    disp('FAIL')
end
