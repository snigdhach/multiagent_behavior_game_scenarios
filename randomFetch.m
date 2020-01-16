%% Fetch Code
% Author: Casillas
% Date: Nov 21, 2018
% Rev: 1.0.0
% Description: Formation Control with leader towards a goal with randomized
% trajectory
%% Constants and Flags
nAgents = 4;
% Time calculation Formula
% Time = 1.1 * maxIterations * timePerIteration
% Time Per Iteration = 0.33 secs (rbtm.set_velocity takes that much time)
% Constant 1.1 reflects Formation + Leader Follower
% Please make the code with timing of the robot in mind.
maxIterations = 5000;
videoFlag = 0;
%% Setting Up Robotarium; Safety and Dynamics Specification
rbtm = Robotarium('NumberOfRobots', nAgents, 'ShowFigure', true);
si_barrier_cert = create_si_barrier_certificate('SafetyRadius', ...
    1.5*rbtm.robot_diameter);
si_to_uni_dyn = create_si_to_uni_mapping2('LinearVelocityGain', 0.5, ... 
    'AngularVelocityLimit', 0.75*rbtm.max_angular_velocity);
si_pos_controller = create_si_position_controller();
%% Initialize Robots, extract Positions
xuni = rbtm.get_poses();                             
x = xuni(1:2,:);                                            
rbtm.set_velocities(1:nAgents, zeros(2,nAgents));
rbtm.step();
%% Initializing System Parameters for Formation
% Laplacian Matrix
L = completeGL(nAgents);
% Weight Matrix
agentSeparation = 2.5 * rbtm.robot_diameter;
% Initializing Velocity
u = zeros(2, nAgents);
%Gains
formationGain = 5;
%% Initializing Video
if videoFlag 
    vid = VideoWriter('Fetch.mp4', 'MPEG-4');
    vid.Quality = 100;
    vid.FrameRate = 72;
    open(vid);
    writeVideo(vid, getframe(gcf));
end
%% Formation Control
% The logic behind forming a formation control first is to bring the robots
% that maybe initially at a position out of defined range. In case you
% don't want to consider such a scenario, comment out this section. This is
% different from the sample code because the sample code assumes the robots
% will meet the leader. In case the leader is far, a lot of time and charge
% will go towards making the robots meet. Thus, it was decided to make them
% meet at mutually convinient location.
% for k = 1:(maxIterations/5)
%     % Reading Robot Position
%     xuni = rbtm.get_poses();
%     x = xuni(1:2,:);
%     % Algorithm
%     for i = 1:nAgents
%         u(:, i) = [0 ; 0];
%         for j = topological_neighbors(L, i)
%             u(:, i) = u(:, i) + ...
%             formationGain * (norm(x(1:2, i) - x(1:2, j))^2 - ...
%             agentSeparation^2) * (x(1:2, j) - x(1:2, i));
%         end;
%     end;
%     % Transform the single-integrator dynamics to unicycle dynamics
%     % using a provided utility function
%     dx = si_barrier_cert(u, xuni);
%     dx = si_to_uni_dyn(u, xuni);  
%     % Set velocities of agents
%     rbtm.set_velocities(1:nAgents, dx);
%     % Send the previously set velocities to the agents
%     rbtm.step();
% end;
%% Initializing System Parameters for Leader Follower
% Laplacian
followerAgents = -completeGL(nAgents - 1);
L = zeros (nAgents, nAgents);
L(2:nAgents, 2:nAgents) = followerAgents;
L(2,2) = L(2,2) + 1;
L (2,1) = -1;
% Initialize Velocity Vector
u = zeros(2,nAgents);


homeFlag = [-1.6; -1];
targetFlag = [1.6;1];
startPoint = [-1.2;-0.8];
endPoint = [1.5 ; 0.9];
proximityRange = 0.01;
xBound = targetFlag(1);
yBound = targetFlag(2);

%% Waypoint
%The information of leader robot's randomized trajectory towards the taget
%flag is embedded in a randomized matrix. 

WayPointsRand = zeros(2,100);

WayPointsRand(:,1) = startPoint;
for i = 95:100 %The leader could wander around the target flag for a little.
    WayPointsRand(:,i) = endPoint;
end

temp = 0;
for ctr = 2:94
    temp = fix(rand * 10);
    if(WayPointsRand(1,ctr - 1) < 0 && WayPointsRand(2,ctr - 1) < 0)
                                     %40 percent chance moving along positive x-axis
        switch temp                  %40 percent chance moving along positive y-axis
            case 0                   %10 percent chance moving along negative x-axis
                directXpositive = 1; %10 percent chance moving along negative y-axis
                directYpositive = 0;
                directXnegative = 0;
                directYnegative = 0;
            case 1
                directXpositive = 1;
                directYpositive = 0;
                directXnegative = 0;
                directYnegative = 0;
            case 2
                directXpositive = 1;
                directYpositive = 0;
                directXnegative = 0;
                directYnegative = 0;
            case 3
                directXpositive = 1;
                directYpositive = 0;
                directXnegative = 0;
                directYnegative = 0;
            case 4
                directXpositive = 0;
                directYpositive = 1;
                directXnegative = 0;
                directYnegative = 0;
            case 5
                directXpositive = 0;
                directYpositive = 1;
                directXnegative = 0;
                directYnegative = 0;
            case 6
                directXpositive = 0;
                directYpositive = 1;
                directXnegative = 0;
                directYnegative = 0;
            case 7
                directXpositive = 0;
                directYpositive = 1;
                directXnegative = 0;
                directYnegative = 0;
            case 8
                directXpositive = 1;
                directYpositive = 0;
                directXnegative = -1;
                directYnegative = 0;
            case 9
                directXpositive = 0;
                directYpositive = 0;
                directXnegative = 0;
                directYnegative = -1;
        end
    else
                                     %30 percent chance moving along positive x-axis
        switch temp                  %30 percent chance moving along positive y-axis
            case 0                   %20 percent chance moving along negative x-axis
                directXpositive = 1; %20 percent chance moving along negative y-axis
                directYpositive = 0;
                directXnegative = 0;
                directYnegative = 0;
            case 1
                directXpositive = 1;
                directYpositive = 0;
                directXnegative = 0;
                directYnegative = 0;
            case 2
                directXpositive = 1;
                directYpositive = 0;
                directXnegative = 0;
                directYnegative = 0;
            case 3
                directXpositive = 0;
                directYpositive = 1;
                directXnegative = 0;
                directYnegative = 0;
            case 4
                directXpositive = 0;
                directYpositive = 1;
                directXnegative = 0;
                directYnegative = 0;
            case 5
                directXpositive = 0;
                directYpositive = 1;
                directXnegative = 0;
                directYnegative = 0;
            case 6
                directXpositive = 0;
                directYpositive = 0;
                directXnegative = -1;
                directYnegative = 0;
            case 7
                directXpositive = 0;
                directYpositive = 0;
                directXnegative = -1;
                directYnegative = 0;
            case 8
                directXpositive = 0;
                directYpositive = 0;
                directXnegative = 0;
                directYnegative = -1;
            case 9
                directXpositive = 0;
                directYpositive = 0;
                directXnegative = 0;
                directYnegative = -1;
        end
    end
    
    GainXpositive = 1/8;        %Futher ensure the trajectory is generally towards the target flag.
    GainYpositive = 1/8;
    GainXnegative = 1/10;
    GainYnegative = 1/10;
    
    WayPointsRand(1,ctr) = WayPointsRand(1,ctr - 1) + directXpositive * GainXpositive + directXnegative * GainXnegative;
    if(WayPointsRand(1,ctr) > xBound - 0.05) %Avoid hit the arena boundary
        WayPointsRand(1,ctr) = xBound - 0.15;
    end
    WayPointsRand(2,ctr) = WayPointsRand(2,ctr - 1) + directYpositive * GainYpositive + directYnegative * GainYnegative;
    if(WayPointsRand(2,ctr) > yBound - 0.05) %Avoid hit the arena boundary
        WayPointsRand(2,ctr) = yBound - 0.15;
    end
end
disp(WayPointsRand)

%% Leader Follower
state = 1;
for k = 1:maxIterations
    x = rbtm.get_poses();
    %% Algorithm
    for i = 2:nAgents
        u(:, i) = [0 ; 0];
        for j = topological_neighbors(L,i)
            u(:,i) = u(:, i) + ...
                formationGain*(norm(x(1:2, j) - x(1:2, i))^2 - ...
                agentSeparation^2)*(x(1:2, j) - x(1:2, i));
        end
    end
    waypoint = WayPointsRand(:, state);
    u(:, 1) = si_pos_controller(x(1:2, 1), waypoint);
    if(mod(k,(maxIterations/50)) == 0) %Specify the waypoints samples to determine the resolution of the trajectory.
        if(state < 99)
            state = state + 2;
        else
            state = 100;
        end
    end

    % To avoid errors, we need to threshold u.
    norms = arrayfun(@(x) norm(u(:, x)), 1:nAgents);
    threshold = rbtm.max_linear_velocity/2;
    to_thresh = norms > threshold;
    u(:, to_thresh) = threshold*u(:, to_thresh)./norms(to_thresh);
    % Transform the single-integrator dynamics to unicycle dynamics
    % using a provided utility function
    u = si_barrier_cert(u, x);
    dx = si_to_uni_dyn(u, x);
    % Sending Velocity to agents
    rbtm.set_velocities(1:nAgents, dx);
    % Writing to Video
    if videoFlag && mod(k,10)                              
            writeVideo(vid, getframe(gcf)); 
    end
    %Iterate
    rbtm.step();
end
for k = 1:maxIterations/10
    x = rbtm.get_poses();
    %% Algorithm
    for i = 2:nAgents
        u(:, i) = [0 ; 0];
        for j = topological_neighbors(L,i)
            u(:,i) = u(:, i) + ...
                formationGain*(norm(x(1:2, j) - x(1:2, i))^2 - ...
                agentSeparation^2)*(x(1:2, j) - x(1:2, i));
        end
    end
    u(:, 1) = si_pos_controller(x(1:2, 1), endPoint);
    % To avoid errors, we need to threshold u.
    norms = arrayfun(@(x) norm(u(:, x)), 1:nAgents);
    threshold = rbtm.max_linear_velocity/2;
    to_thresh = norms > threshold;
    u(:, to_thresh) = threshold*u(:, to_thresh)./norms(to_thresh);
    % Transform the single-integrator dynamics to unicycle dynamics
    % using a provided utility function
    u = si_barrier_cert(u, x);
    dx = si_to_uni_dyn(u, x);
    % Sending Velocity to agents
    rbtm.set_velocities(1:nAgents, dx);
    % Writing to Video
    if videoFlag && mod(k,10)                              
            writeVideo(vid, getframe(gcf)); 
    end
    %Iterate
    rbtm.step();
end
%% End
if videoFlag
    close(vid);
end
rbtm.debug();