function [ned2_robot, numJoints] = ned2_import()
% NED2_IMPORT Summary of this function goes here
% Fabio Mastromarino - Date modified 19/01/2024
    % This function can import robots that have .stl files associated with the
    % Unified Robot Description format (URDF) file to describe the visual
    % geometries of the robot. Each rigid body has an individual visual
    % geometry specified. The importrobot function parses the URDF file to get
    % the robot model and visual geometries.
    % The manipulator's structure would be affected if you edit the file
    % 'new_ned2.sdf', so avoid doing so.
    ned2_robot = importrobot("new_ned2.sdf");
    
    % Joints limit by Niryo Ned Studio [degree] ---     DON'T CHANGE IT
    limit_low_shoulder = -166.15776058793872;
    limit_up_shoulder = 166.15776058793872;
    limit_low_arm = -99.12169855763241;
    limit_up_arm = 29.220847551671984;
    limit_low_elbow = -73.6770;
    limit_up_elbow = 84.22479588423101;
    limit_low_forearm = -114.01860123103383;
    limit_up_forearm = 114.01860123103383;
    limit_low_wrist = -104.27831871380982;
    limit_up_wrist = 104.45020605234907;
    limit_low_hand = -139.22874421679006;
    limit_up_hand = 139.22874421679006;
    
    % To ensure greater safety, the margin further reduces manipulator limits
    safety_pos_margin = 0.0;

    % Create the lower limit vector for the six revolute jointsCreate the lower
    % limit vector for the six revolute joints in radians
    joints_low_limit_deg = [limit_low_shoulder, limit_low_arm, limit_low_elbow, limit_low_forearm, limit_low_wrist, limit_low_hand];
    joints_low_limit_rad = deg2rad(joints_low_limit_deg + safety_pos_margin);

    % Create the upper limit vector for the six revolute jointsCreate the lower
    % limit vector for the six revolute joints in radians
    joints_up_limit_deg = [limit_up_shoulder, limit_up_arm, limit_up_elbow, limit_up_forearm, limit_up_wrist, limit_up_hand];
    joints_up_limit_rad = deg2rad(joints_up_limit_deg - safety_pos_margin);

    % Joints_home represents the initial position of the joints after the
    % manipulator calibration phase is complete
    numJoints = 6;
    joints_home = [0.0, 0.499, -1.251, 0.0, 0.0, 0.002];
    
    % Checking that ned2.Bodies is a cell_array 
    if iscell(ned2_robot.Bodies)
        for i = 1:numJoints
            %     For each of the six revolute joints, assign the initial
            %     position as these characteristic  isn't directly obtainable
            %     from the file .sdf
            ned2_robot.Bodies{1,i+1}.Joint.HomePosition = joints_home(i);

            %     Assign movement limits to every revolute joint because the
            %     ones in the .sdf file have less stringent safety requirements
            ned2_robot.Bodies{1,i+1}.Joint.PositionLimits = [joints_low_limit_rad(i), joints_up_limit_rad(i)];
        end
    end
end

