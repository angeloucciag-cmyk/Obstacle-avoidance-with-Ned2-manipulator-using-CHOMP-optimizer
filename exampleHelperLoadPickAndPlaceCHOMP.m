function [ned2,initialguess,env]=exampleHelperLoadPickAndPlaceCHOMP()
%exampleHelperLoadPickAndPlaceCHOMP Helper that loads a pick and place scenario for CHOMP.

% This function is for internal use only, and maybe removed in the future.

% Copyright 2022-2024 The MathWorks, Inc.

franka=loadrobot("ned2",DataFormat="row");

% The right and left finger of the robot are moving joints. Given that
% these don't move during picking or placing, replace these joints as fixed
% joints.
leftfinger=getBody(franka,"panda_leftfinger");
rightfinger=getBody(franka,"panda_rightfinger");
leftfingerjoint=leftfinger.Joint;
rightfingerjoint=rightfinger.Joint;

% The fingers are 0.04 units apart when they are open.
offset=trvec2tform([0,0.04,0]);
leftfixedjoint=rigidBodyJoint("leftfingerfixedjoint");
setFixedTransform(leftfixedjoint,leftfingerjoint.JointToParentTransform*offset);
rightfixedjoint=rigidBodyJoint("rightfingerfixedjoint");
setFixedTransform(rightfixedjoint,rightfingerjoint.JointToParentTransform/offset);
replaceJoint(franka,leftfinger.Name,leftfixedjoint);
replaceJoint(franka,rightfinger.Name,rightfixedjoint);

% A starting joint configuration of the robot.
startConfig=homeConfiguration(franka);
startConfig(7)=0.65;
startConfig(6)=0.75;

% Create an environment of collision objects.
bench=collisionBox(0.5,0.9,0.05);
belt1=collisionBox(0.3,0.4,0.23);
barricade=collisionBox(1.8,1.03,1.35);
TBench=trvec2tform([1.35,1,1.2]); % posizione base dove stanno i cilindri rispetto a s.d.r inerziale
TBelt1=trvec2tform([0,-0.6,0.177]);
bench.Pose=TBench;
belt1.Pose=TBelt1;
barricade.Pose=trvec2tform([0.3,-0.25,0.375]);
cylinder1=collisionCylinder(0.028,0.1);
cylinder2=collisionCylinder(0.028,0.1);
cylinder3=collisionCylinder(0.028,0.1);
TCyl=trvec2tform([0.5,0.15,0.278]);
TCyl2=trvec2tform([0.52,0,0.278]);
TCyl3=trvec2tform([0.4,-0.1,0.2758]);
cylinder1.Pose=TCyl;
cylinder2.Pose=TCyl2;
cylinder3.Pose=TCyl3;
env={bench,belt1,cylinder1,cylinder2,cylinder3,barricade};
end
