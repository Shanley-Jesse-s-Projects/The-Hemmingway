%Calculate the foward dynamics of the system from an angle input
%The purpose of this code is to show fowards kinematics
%import the UR5 model that was taken from the class files
lbr = importrobot('ur5_robot.urdf');
lbr.DataFormat = 'row';
%lbr.Gravity = [0 0 -9.81];
%get the configuration of the robot
q = homeConfiguration(lbr);
gripper = 'ee_link'; %this is the end effector of the robot, this will need to move to new location
%only concerned with the ee_link or end effector link
%input the t values
ti = 0;
tf = 5;
%take the angle inputs from the robot
th1 = 0; % z
th2=0;
th3=0;
th4=0;
th5=0;
th6=0;
%locked variables
al2=90;
al5=90;
al6=-90;
d1=0.089159;
a2=-0.425;
a3=	-0.39225;
d4=.10915;
d5=	0.09465;
d6=0.0823;

%matrices
T01=[cos(th1) -sin(th1) 0 0; sin(th1) cos(th1) 0 0; 0 0 1 d1; 0 0 0 1];
T12=[cos(th2) 0 sin(th2) 0; sin(th2) 0 -cos(th2) 0; 0 1 0 0; 0 0 0 1];
T23=[cos(th3) -sin(th3) 0 a2*cos(th3); sin(th3) cos(th3) 0 a2*sin(th3); 0 0 1 0; 0 0 0 1];
T34=[cos(th4) -sin(th4) 0 a3*cos(th4); sin(th4) cos(th4) 0 a3*sin(th4); 0 0 1 d4; 0 0 0 1];
T45=[cos(th5) 0 sin(th5) 0; sin(th5) 0 -cos(th5) 0; 0 1 0 d5; 0 0 0 1];
T56=[cos(th6) 0 -sin(th6) 0; sin(th6) 0 cos(th6) 0; 0 -1 0 d6; 0 0 0 1];
T06=T01*T12*T23*T34*T45*T56;
disp(T06);
    
xposition=T06(1,4); %desired x
yposition=T06(2,4); %desired y
zposition=T06(3,4); %desired z
  
t = (ti:0.2:tf)'; % Time
count = length(t);
Position = [xposition, yposition, zposition]; %where the x, y , and z need to go
body = rigidBody('None'); %using a rigid body tree
setFixedTransform(body.Joint, trvec2tform(Position));
addBody(lbr, body, lbr.BaseName);
numWaypoints = 5;
qWaypoints = repmat(q, numWaypoints, 1);
gik = generalizedInverseKinematics('RigidBodyTree', lbr, ...
    'ConstraintInputs', {'cartesian','position','aiming','orientation','joint'});
height = constraintCartesianBounds(gripper);
heightAboveTable.Bounds = [-inf, inf; ...
                           -inf, inf; ...
                           0.1, inf];
distance = constraintPositionTarget('None');
distance.ReferenceBody = gripper; %use the refrence as the end effector
align = constraintAiming('ee_link');
align.TargetPoint = [xposition, yposition, zposition];
limitJointChange = constraintJointBounds(lbr); %make sure that the joints are constrained 
fixOrientation = constraintOrientationTarget(gripper);
%Set target position for the end effector
distance.TargetPosition = [0,0,0];
[qWaypoints(2,:),solutionInfo] = gik(q, height, ...
                       distance, align, fixOrientation, ...
                       limitJointChange); 
fixOrientation.TargetOrientation = ...
    tform2quat(getTransform(lbr,qWaypoints(2,:),gripper));
distanceFrom = linspace(intermediateDistance, final, numWaypoints-1);
maxJointChange = deg2rad(5); %how much the joints can move
for k = 3:numWaypoints
    % Update the target position.
    distance.TargetPosition(3) = distanceFrom(k-1);
    % Restrict the joint positions to lie close to their previous values.
    limitJointChange.Bounds = [qWaypoints(k-1,:)' - maxJointChange, ...
                               qWaypoints(k-1,:)' + maxJointChange];
    % Solve for a configuration and add it to the waypoints array.
    [qWaypoints(k,:),solutionInfo] = gik(qWaypoints(k-1,:), ...
                                         height, ...
                                         distance, align, ...
                                         fixOrientation, limitJointChange);
end
framerate = 30; %changes the framerate
%this calculates the different waypoints 
Waypoints = [0,linspace(numWaypoints/2,numWaypoints,size(qWaypoints,1)-1)];
numFrames = FinalWaypoint*framerate;
%this finds the interpolation to each waypoint
Interp = pchip(Waypoints,qWaypoints',linspace(0,FinalWaypoint,numFrames))';
gripperPosition = zeros(numFrames,3);

%show the initial configuration
show(lbr,qWaypoints(1,:));
hold on
%plot the gripper positon
p = plot3(gripperPosition(1,1), gripperPosition(1,2), gripperPosition(1,3));
%Animate the manipulator and plot the gripper
hold on
for k = 1:size(Interp,1)
    show(lbr, Interp(k,:), 'PreservePlot', false);
    p.XData(k) = gripperPosition(k,1);
    p.YData(k) = gripperPosition(k,2);
    p.ZData(k) = gripperPosition(k,3);
    waitfor(r); %this allows for the animation to be seen
end
hold off