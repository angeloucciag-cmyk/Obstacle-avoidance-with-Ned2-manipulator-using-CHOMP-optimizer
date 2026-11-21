function sph=exampleHelperApproximateCollisionCylinderSpheres(cyl,ratio)
%exampleHelperApproximateCollisionCylinderSpheres Approximate spheres for a collision cylinder

% This function is for internal use only, and maybe removed in the future.

% Copyright 2022 The MathWorks, Inc.

fitcapsule=fitCollisionCapsule(cyl);
fitcapsule.Length=fitcapsule.Length-2*fitcapsule.Radius;
collspheres=genspheres(fitcapsule,ratio);
sph=[];
for ii=1:length(collspheres)
    sph=[sph,[collspheres{ii}.Radius;tform2trvec(collspheres{ii}.Pose)']];
end
end
