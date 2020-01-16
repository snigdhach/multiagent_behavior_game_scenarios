%% Fetch Code
% Author: Harsh Bhate
% Date: Nov 18, 2018
% Rev: 1.0.0
% Description: Formation Control with leader towards a goal
%% Constants and Flags
nAgents = 3;
% Time calculation Formula
% Time = 1.1 * maxIterations * timePerIteration
% Time Per Iteration = 0.33 secs (rbtm.set_velocity takes that much time)
% Constant 1.1 reflects Formation + Leader Follower
% Please make the code with timing of the robot in mind.
maxIterations = 2000;
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
for k = 1:(maxIterations/10)
    % Reading Robot Position
    xuni = rbtm.get_poses();
    x = xuni(1:2,:);
    % Algorithm
    for i = 1:nAgents
        u(:, i) = [0 ; 0];
        for j = topological_neighbors(L, i)
            u(:, i) = u(:, i) + ...
            formationGain * (norm(x(1:2, i) - x(1:2, j))^2 - ...
            agentSeparation^2) * (x(1:2, j) - x(1:2, i));
        end;
    end;
    % Transform the single-integrator dynamics to unicycle dynamics
    % using a provided utility function
    dx = si_barrier_cert(u, xuni);
    dx = si_to_uni_dyn(u, xuni);  
    % Set velocities of agents
    rbtm.set_velocities(1:nAgents, dx);
    % Send the previously set velocities to the agents
    rbtm.step();
end;
%% Initializing System Parameters for Leader Follower
% Laplacian
followerAgents = -completeGL(nAgents - 1);
L = zeros (nAgents, nAgents);
L(2:nAgents, 2:nAgents) = followerAgents;
L(2,2) = L(2,2) + 1;
L (2,1) = -1;
% Initialize Velocity Vector
u = zeros(2,nAgents);
% Leader State
state = 1;
%% Waypoint
% Assuming a grid size of length (1 - (1.25 * agentSeparation)) * (1 -
% (1.25 * agentSeparation)). The grid size is reduced so that the reduced
% random point edge case doesn't cause boundary errors. Currently using
% random walk to generate the waypoint.
waypoints = 0.85*[ 1  1;...
                  -1  1;...
                  -1 -1;...
                   1 -1]';
tmp = size(waypoints);
nStates = tmp(2);
proximityRange = 0.05;

nStates = 4;
waypoints1 = rand(nStates,2);
startPoint = xuni(1:2, 1);
endPoint = [0.7 ; 0.7];
waypoints1(1, :) = startPoint;
waypoints1(nStates, :) = endPoint;
disp(waypoints1);
m = cumsum(waypoints1);
mout = m./m(end,:); 
disp(mout);
%% Leader Follower
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
    %% Waypoint Traversing
    for currState = 1:nStates
        waypoint = waypoints(:, state);
        u(:, 1) = si_pos_controller(x(1:2, 1), waypoint);
        if(norm(x(1:2, 1) - waypoint) < proximityRange)
            state = currState + 1;
            if (state > nStates)
                state = 1;
            end
        end
    end
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