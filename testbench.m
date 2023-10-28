%% Setup 
    % Define our time vector.
    tStart = 0;   % The time at which the simulation starts, in seconds.
    tStep = 0.1; % The simulation's time step, in seconds.
    tEnd = 5;    % The time at which the simulation ends, in seconds.
    t = (tStart:tStep:tEnd);  % The time vector 
    
    % Set whether to animate the robot's movement and how much to slow it down.
    pause on;  % Set this to off if you don't want to watch the animation.
    GraphingTimeDelay = 0.0001; % The length of time that Matlab should pause between positions when graphing, if at all, in seconds.

    dhparams = [0   	pi/2	0.44   	0;
            1.04	    0       0       0;
            1.04	    0	    0	    0;
            0	    pi/2	    0	    0;
            0   	0	0.96   	0];

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
   showdetails(robot)

   configNow = [ 0    2.3567   -1.5719    0.7859    1.0472];
   gik = generalizedInverseKinematics('RigidBodyTree',robot,'ConstraintInputs',{'position'});

    % this is the object that we want to grasp
    % object_trajec = [linspace(1.5,0.1,length(t)); linspace(1.5,0.1,length(t)); linspace(1.5,1,length(t)); linspace(0,-pi/4,length(t)); linspace(0,pi/6,length(t));];
    object_trajec = [linspace(1.5,0.1,length(t)); linspace(1.5,0.1,length(t)); linspace(1.5,1,length(t));];
    posTgt = constraintPositionTarget("body5");
    histories = [zeros(5,length(t));];
    ini_guess = homeConfiguration(robot);
    for j = 1:length(t)
        if (j==1)
        posTgt.TargetPosition = object_trajec(:,j);
        [configSol, solInfo] = gik(ini_guess,posTgt);
        histories(:,j) = [configSol(1).JointPosition configSol(2).JointPosition configSol(3).JointPosition configSol(4).JointPosition configSol(5).JointPosition];
        else 
            ini_guess(1).JointPosition = histories(1,j-1);
            ini_guess(2).JointPosition = histories(2,j-1);
            ini_guess(3).JointPosition = histories(3,j-1);
            ini_guess(4).JointPosition = histories(4,j-1);
            ini_guess(5).JointPosition = histories(5,j-1);
            posTgt.TargetPosition = object_trajec(:,j);
            [configSol, solInfo] = gik(ini_guess,posTgt);
            histories(:,j) = [configSol(1).JointPosition configSol(2).JointPosition configSol(3).JointPosition configSol(4).JointPosition configSol(5).JointPosition];

        end
    end
    theta1_history = histories(1,:);
    theta2_history = histories(2,:);
    theta3_history = histories(3,:);
    theta4_history = histories(4,:);
    theta5_history = histories(5,:);
    % ini_guess = [0.7854    2.2399   -1.5718    0.7859         0];
    
    
    % aimCon = constraintAiming("body5");
    % aimCon.TargetPoint = [0.0 0.0 0.0];
    ini_guess = homeConfiguration(robot);
    [configSol, solInfo] = gik(ini_guess,posTgt)
    theta = configSol(1).JointPosition;
    
    for i = 1:1
        % configNow = [configSol(1).JointPosition configSol(2).JointPosition configSol(3).JointPosition configSol(4).JointPosition configSol(5).JointPosition];
        theta11 = 0.8; % z
        theta44 = 0.0; % y 
        theta55 = 0.9; % x
        
        configNow = [theta11 0 0 (-theta44+pi/2) theta55];
        
        R_y_theta = [cos(theta44) 0 sin(theta44);
                    0 1 0;
                    -sin(theta44) 0 cos(theta44)];
        % configNow(4) = deg2rad(configNow(4)); % This will just be to
        % ensure linking bw IK solution finder 0.3 0.3 1.12

        % Display robot in provided configuration
        config = homeConfiguration(robot);
        
        for j = 1:numJoints
            config(j).JointPosition = configNow(j);
        end
        frame1 = getTransform(robot,config, "body1");
        frame2 = getTransform(robot,config, "body2");
        frame3 = getTransform(robot,config, "body3");
        frame4 = getTransform(robot,config, "body4");
        frame5 = getTransform(robot,config, "body5")
        clf;
        hold on;
        % xlim([-1 4]); ylim([-1 3]);
        grid on;
        % figure(1);
        % drawnow;
        axis([-3 3 -3 3 -3 3])
        % hold on;
        axis vis3d;
        show(robot,config);
        % object_trajec(1,i)
        % object_trajec(2,i)
        % object_trajec(3,i)
        plot3(object_trajec(1,i), object_trajec(2,i), object_trajec(3,i))
        view(-60,30)
        hold off
        pause(GraphingTimeDelay)
        R_z_theta = [cos(theta11) -sin(theta11) 0;
        sin(theta11) cos(theta11) 0;
        0 0 1];

        R_x_theta = [1 0 0; 0 cos(theta55) -sin(theta55); 0 sin(theta55) cos(theta55)];

        og_r_matrix__i_think = [0 0 1;
                                0 -1 0;
                                1 0 0;];
        Test = R_z_theta*R_y_theta*R_x_theta*og_r_matrix__i_think

        % clf(1)
    end

















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
    

