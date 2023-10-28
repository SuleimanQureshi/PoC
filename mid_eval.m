    clc;close all;clear;
    h = load('Mits_hist.mat','histories');
    histories = h.histories;
    theta1_history = histories(1,:);
    theta2_history = histories(2,:);
    theta3_history = histories(3,:);
    theta4_history = histories(4,:);
    
    % Define our time vector.
    tStart = 0;   % The time at which the simulation starts, in seconds.
    tStep = 0.1; % The simulation's time step, in seconds.
    tEnd = 5;    % The time at which the simulation ends, in seconds.
    t = (tStart:tStep:tEnd);  % The time vector 
    % theta5_history = histories(5,:);
    theta5_history = linspace(1,2*pi,length(t));
    
    GraphingTimeDelay = 0.0001; % The length of time that Matlab should pause between positions when graphing, if at all, in seconds.

    % DH parameters of the 5 DoF robot
    dhparams = [0   	-pi/2	3   	0;
            2.5	    0       0       -pi/2;
            1.6	    0	    0	    0;
            0	    -pi/2	    0	    -pi/2;
            0   	0	0.72   	0];

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
   plot(t,theta1_history)
    hold on
    plot(t,theta2_history)
    hold on
    plot(t,theta3_history)
    hold on
    plot(t,theta4_history)
    hold on
    plot(t,theta5_history)
    hold on
    
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

        axis([-5 5 -5 5 -5 5])
        axis vis3d;
        ahem = show(robot,config);
        fig = gcf;
        set(gcf, 'WindowState', 'maximized')
        % plot3(object_trajec(1,i),object_trajec(2,i),object_trajec(3,i))

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
        % plot3(object_trajec(1,i), object_trajec(2,i), object_trajec(3,i))
        view(-60,30)
        hold off
        pause(GraphingTimeDelay)
    end
%%
clc;close all;clear;
    h = load('Habib_hist.mat','histories');
    histories = h.histories;
    theta1_history = histories(1,:);
    theta2_history = histories(2,:);
    theta3_history = histories(3,:);
    theta4_history = histories(4,:);
    
    % Define our time vector.
    tStart = 0;   % The time at which the simulation starts, in seconds.
    tStep = 0.1; % The simulation's time step, in seconds.
    tEnd = 5;    % The time at which the simulation ends, in seconds.
    t = (tStart:tStep:tEnd);  % The time vector 
    % theta5_history = histories(5,:);
    theta5_history = linspace(1,2*pi,length(t));
    
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
        set(gcf, 'WindowState', 'maximized')
        % plot3(object_trajec(1,i),object_trajec(2,i),object_trajec(3,i))

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
        % plot3(object_trajec(1,i), object_trajec(2,i), object_trajec(3,i))
        view(-60,30)
        hold off
        pause(GraphingTimeDelay)
    end