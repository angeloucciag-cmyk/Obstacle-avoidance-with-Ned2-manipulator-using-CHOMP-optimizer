function exampleHelperVisualizePlaceTrajectory(robot,objectToBePicked,optimplaceconfig,rc,p)
%exampleHelperVisualizePlaceTrajectory Visualize place trajectory of robot

% This function is for internal use only, and maybe removed in the future.

% Copyright 2022 The MathWorks, Inc.

p.FaceAlpha = 0.25;
[~,patchOfObjectToBePicked]=show(objectToBePicked);
patchOfObjectToBePicked.FaceColor=[0,0,1];
hgtform=hgtransform();
patchOfObjectToBePicked.Parent=hgtform;
pickConfig=optimplaceconfig(1,:);
Tcanwrtee=getTransform(robot,pickConfig,robot.BodyNames{end})\objectToBePicked.Pose;
for i = 1:size(optimplaceconfig,1)
    config=optimplaceconfig(i,:);
    eePose=getTransform(robot,config,robot.BodyNames{end});
    show(robot,config,...
        Collisions="on",Visuals="off",PreservePlot=false,FastUpdate=true);
    hgtform.Matrix=((eePose*Tcanwrtee)/objectToBePicked.Pose);
    waitfor(rc);
end
end
