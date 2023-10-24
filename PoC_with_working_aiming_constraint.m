    %% Setup 
    % Define our time vector.
    tStart = 0;   % The time at which the simulation starts, in seconds.
    tStep = 0.1; % The simulation's time step, in seconds.
    tEnd = 5;    % The time at which the simulation ends, in seconds.
    t = (tStart:tStep:tEnd);  % The time vector 
    
    GraphingTimeDelay = 0.0001; % The length of time that Matlab should pause between positions when graphing, if at all, in seconds.

    % DH parameters of the 5 DoF robot
    dhparams = [0   	pi/2	0.44   	0;
            1.04	    0       0       0;
            1.04	    0	    0	    0;
            0	    pi/2	    0	    0;
            0   	0	0.96   	0];

    % Initialization of the robot
    numJoints = size(dhparams,1);
    robot = rigidBodyTree;
    bodies = cell(numJoints,1);
    joints = cell(numJoints,1);
    for i = 1:numJoints
        bodies{i} = rigidBody(['body' num2str(i)]);
        joints{i} = rigidBodyJoint(['jnt' num2str(i)],"revolute");
        setFixedTransform(joints{i},dhparams(i,:),"dh");
        bodies{i}.Joint = joints{i};
        if i == 1 % Add first body to base
            addBody(robot,bodies{i},"base")
        else % Add current body to previous body by name
            addBody(robot,bodies{i},bodies{i-1}.Name)
        end
    end
   % make a IK solver for our robot
   gik = generalizedInverseKinematics('RigidBodyTree',robot,'ConstraintInputs',{'position','aiming'});

    % this is the object's trajectory that we want to grasp
    object_trajec = [linspace(1.5,0.1,length(t)); linspace(1.5,0.1,length(t)); linspace(1.5,1,length(t));];
    % we want the end effector (or body 5) to be at the point of interest
    posTgt = constraintPositionTarget("body5"); 

    aimingTgt = constraintAiming("body5");
    
    % This is is the path we want to follow in joint angle terms
    histories = [zeros(5,length(t));];
    ini_guess = homeConfiguration(robot);
    %This loop is not in the main loop to provide a smooth simulation
    for j = 1:length(t)
        if (j==1)
        % target position is where the object is right now
        posTgt.TargetPosition = object_trajec(:,j);
        aimingTgt.TargetPoint = [0, 0, -100];
        [configSol, solInfo] = gik(ini_guess,posTgt,aimingTgt);

        histories(:,j) = [configSol(1).JointPosition configSol(2).JointPosition configSol(3).JointPosition configSol(4).JointPosition configSol(5).JointPosition];
        else 
            % initial guess is the previous position
            ini_guess(1).JointPosition = histories(1,j-1);
            ini_guess(2).JointPosition = histories(2,j-1);
            ini_guess(3).JointPosition = histories(3,j-1);
            ini_guess(4).JointPosition = histories(4,j-1);
            ini_guess(5).JointPosition = histories(5,j-1);
            posTgt.TargetPosition = object_trajec(:,j);
            [configSol, solInfo] = gik(ini_guess,posTgt,aimingTgt);
            histories(:,j) = [configSol(1).JointPosition configSol(2).JointPosition configSol(3).JointPosition configSol(4).JointPosition configSol(5).JointPosition];

        end
    end
    theta1_history = histories(1,:);
    theta2_history = histories(2,:);
    theta3_history = histories(3,:);
    theta4_history = histories(4,:);
    % theta5_history = histories(5,:);
    theta5_history = linspace(1,pi,length(t));
    
    % I think this type of code will be used for positioning 
    % (i could be wrong)
    % aimCon = constraintAiming("body5");
    % aimCon.TargetPoint = [0.0 0.0 0.0];
    % tempo = robot.Bodies(1)
    for i = 1:length(t)
        
        % you go through all the calculated configurations to trace path
        configNow = [theta1_history(i)   theta2_history(i) theta3_history(i) theta4_history(i) theta5_history(i)];
        % Display robot in provided configuration
        config = homeConfiguration(robot);
        
        for j = 1:numJoints
            config(j).JointPosition = configNow(j);
        end
        % frame1 = getTransform(robot,config, "body1");
        % frame2 = getTransform(robot,config, "body2");
        % frame3 = getTransform(robot,config, "body3");
        % frame4 = getTransform(robot,config, "body4");
        % frame5 = getTransform(robot,config, "body5");
        % below code is just how to plot the robot
        clf;
        hold on;
        grid on;

        axis([-3 3 -3 3 -3 3])
        axis vis3d;
        ahem = show(robot,config);
        fig = gcf;

        % Loop through each joint frame and modify its appearance
        % for iii = 1:numel(robot.Bodies)
        %     % Assuming that the joint frame handles are stored in the UserData property
        %     jointFrameHandle = robot.Bodies(iii).UserData;
        % 
        %     if isvalid(jointFrameHandle)
        %         % Modify the appearance of the joint frame as desired
        %         set(jointFrameHandle, 'MarkerSize', 50, 'MarkerEdgeColor', 'r');
        %     end
        % end
        plot3(object_trajec(1,i), object_trajec(2,i), object_trajec(3,i))
        view(-60,30)
        hold off
        pause(GraphingTimeDelay)
    end




% below code is unnecessary












    % Determine the pose of end-effector in provided configuration
    poseNow = getTransform(robot,config,"body4");
    
    % Display position and orientation of end-effector
    % clc;
    % disp('The position of end-effector is:');
    % disp('');
    % disp(['X: ', num2str(poseNow(1,4))]);
    % disp('');
    % disp(['Y: ', num2str(poseNow(2,4))]);
    % disp('');
    % disp(['Z: ', num2str(poseNow(3,4))]);
    % disp(' ');
    % disp(['R: ']);
    % poseNow(1:3,1:3)
    % disp(' ');
    % disp('The orientation angle is given with respect to the x-axis of joint 2:');
    % disp(''); 
    % poseNow01 = getTransform(robot,config,"body1");
    % R14 = poseNow01(1:3,1:3)'*poseNow(1:3,1:3);
    % angle = rad2deg(atan2(R14(2,1),R14(1,1)));
    % disp(['Angle: ',num2str(angle), ' degrees.']);
   