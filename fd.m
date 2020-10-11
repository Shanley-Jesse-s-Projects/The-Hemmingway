%The purpose of this code is to find the forward dynamics of the robotic
%arm
%import the ur5 model
lbr = importrobot('ur5_robot.urdf');
lbr.DataFormat = 'row';
q = homeConfiguration(lbr);
%Change the d values for the force on each joint of the system
d0 = 300;
d1 = 0;
d2 = 1000;
d3 = 0;
d4 = 12;
d5 = 0;
wrench = [d0 d1 d2 d3 d4 d5]; %this is the force on the robot
%Change time of the system
ti = 0;
tf = 4;
fext = externalForce(lbr,'ee_link',wrench,q);
qddot = forwardDynamics(lbr,q,[],[],fext);
display(qddot)
FinalPosition = [qddot];
gripper = 'ee_link';
qddot1 = qddot';
figure
show(lbr,q(1,:)); %display the initial position of the robot
hold on
%show new position
for i = 1:size(qddot)
    show(lbr,qddot(1,:));
    p.XData(i) = FinalPosition(i,1); %new x position 
    p.YData(i) = FinalPosition(i,2); %new y position 
    p.ZData(i) = FinalPosition(i,3); %new z postion 
    waitfor(r);
    drawnow
end
hold on
