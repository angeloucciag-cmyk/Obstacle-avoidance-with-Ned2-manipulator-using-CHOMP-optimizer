function exampleHelperVisualizePickTrajectory(robot,optimpickconfig,rc)
%exampleHelperVisualizePickTrajectory Helper to visualize the picking trajectory

% This function is for internal use only, and maybe removed in the future.

% Copyright 2022 The MathWorks, Inc.

for i = 1:size(optimpickconfig,1)
    show(robot,optimpickconfig(i,:),...
        Collisions="on",Visuals="on",PreservePlot=false,FastUpdate=true);
    waitfor(rc);
end
end
