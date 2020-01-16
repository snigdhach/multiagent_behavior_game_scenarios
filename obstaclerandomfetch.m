%% Fetch Code
% Author: Casillas
% Date: Nov 21, 2018
% Rev: 1.0.0
% Description: Formation Control with leader towards a goal with randomized
% trajectory
%% Constants and Flags
clear all, close all, clc
nAgents = 3;
% Time calculation Formula
% Time = 1.1 * maxIterations * timePerIteration
% Time Per Iteration = 0.33 secs (rbtm.set_velocity takes that much time)
% Constant 1.1 reflects Formation + Leader Follower
% Please make the code with timing of the robot in mind.
maxIterations = 4000;
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
E = [1,2; 2,3; 3,1];
% Weight Matrix
agentSeparation = 2.5 * rbtm.robot_diameter;
% Initializing Velocity
u = zeros(2, nAgents);
%Gains
formationGain = 5;
desired_distance = 0.3;
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
endPoint = [1; 0];
proximityRange = 0.01;
xBound = targetFlag(1);
yBound = targetFlag(2);

%% marker for flag
% Target cycle definition
center = endPoint;
radius = 0.01;
% interAgentDistance = radius*0.00001*sin(pi/N);
interAgentDistance = 1*10^-100;
kp1 = 5;
kp2 = 0.01;
flagplot=plot(center(1),center(2),'*','markersize',12);
th = 0 : 2*pi/20 : 2*pi-2*pi/20;
plot(radius.*cos(th)+center(1),radius.*sin(th)+center(2),'b')

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
% disp(WayPointsRand)

%% Leader Follower
state = 1;
for k = 1:maxIterations/15
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
for k = 1:maxIterations/15
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

%% ********** FETCH *********** 
% in case the leader is blocked by an opponnent but at least one follower 
% has the flag within its range, make that follower the leader and go to
% the flag
distToFlag = zeros(1, nAgents);
for i= 1:nAgents
    distToFlag(1,i) = norm(x(1:2,i)-endPoint);
end

[dist,closestAgent] = min(distToFlag);
% redefine L based on the closest agent to flag as the leader
swapRowTemp = L(closestAgent,1:nAgents);
L(1,1:nAgents) = swapRowTemp;
L(closestAgent , 1:nAgents) = zeros(1,nAgents);

if(dist > 0.3)
    for t = 1:maxIterations/15
    x = rbtm.get_poses();
    %% Algorithm
    for i = 1:nAgents
        if(i~=closestAgent)
            %Zero velocity and get the topological neighbors of agent i
            u(:, i) = [0 ; 0];
        
            neighbors = topological_neighbors(L, i);
        
            for j = neighbors
                u(:, i) = u(:, i) + ...
                formationGain*(norm(x(1:2, j) - x(1:2, i))^2 -  desired_distance^2)*(x(1:2, j) - x(1:2, i));
            end
        end
    end
    
    %% Make the leader travel to flag
    
    u(:, 1) = si_pos_controller(x(1:2, 1), endPoint);
            
    %% Avoid errors
    
    % To avoid errors, we need to threshold dxi
    norms = arrayfun(@(x) norm(u(:, x)), 1:nAgents);
    threshold = rbtm.max_linear_velocity/2;
    to_thresh = norms > threshold;
    u(:, to_thresh) = threshold*u(:, to_thresh)./norms(to_thresh);
    
    %% Use barrier certificate and convert to unicycle dynamics
    u = si_barrier_cert(u, x);
    dx = si_to_uni_dyn(u, x);
    
    %% Send velocities to agents
    
    %Set velocities
    rbtm.set_velocities(1:nAgents, dx);
    
    %Iterate experiment
    rbtm.step();
    end
end
%% make randevouzous
% We need (2 * N - 3) edges to ensure that the formation is rigid.
L = cycleGL(nAgents);

% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively.
u = zeros(2, nAgents);
for t = 0:800
    
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = rbtm.get_poses();
    
    %% Algorithm
    
    %This section contains the actual algorithm for formation control!
    
    %Calculate single integrator control inputs using edge-energy consensus
    
    for i = 1:nAgents
        % Initialize velocity to zero for each agent.  This allows us to sum
        % over agent i's neighbors

        u(:, i) = [0 ; 0];
       
        % Get the topological neighbors of agent i from the communication
        % topology
        for j = topological_neighbors(L, i)
                
            % For each neighbor, calculate appropriate formation control term and
            % add it to the total velocity

            u(:, i) = u(:, i) + ...
            10*(norm(x(1:2, i) - endPoint) - 0.4) ... 
            *(endPoint - x(1:2, i));
        end
    end

    %% Avoid errors
    
    % To avoid errors, we need to threshold dxi
    norms = arrayfun(@(x) norm(u(:, x)), 1:nAgents);
    threshold = rbtm.max_linear_velocity/2;
    to_thresh = norms > threshold;
    u(:, to_thresh) = threshold*u(:, to_thresh)./norms(to_thresh);
    
    % Transform the single-integrator dynamics to unicycle dynamics using a provided utility function
    u = si_barrier_cert(u, x);
    dx = si_to_uni_dyn(u, x);  
    
    % Set velocities of agents 1:N
    rbtm.set_velocities(1:nAgents, dx);
    
    % Send the previously set velocities to the agents.  This function must be called!
    rbtm.step();   
end

%% make formation
% We need (2 * N - 3) edges to ensure that the formation is rigid.

rigidity = 2*nAgents -3;
for i = nAgents+1 : rigidity
    for j = 1:nAgents
        if(L(closestAgent, j)==0)
            L(closestAgent, j) = -1;
            break;
        end
    end
end

% The desired inter-agent distance for the formation
d = 0.4; 

% Weight matrix containing the desired inter-agent distances to achieve a
% rectuangular formation
W = zeros(nAgents,nAgents);
for i = 1:nAgents
    for j = topological_neighbors(L,i)
        W(i,j) = d;
    end
end
u = zeros(2, nAgents);
for t = 0:800
    
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = rbtm.get_poses();
    
    %% Algorithm
    
    %This section contains the actual algorithm for formation control!
    
    %Calculate single integrator control inputs using edge-energy consensus
    for i = 1:nAgents
        % Initialize velocity to zero for each agent.  This allows us to sum
        % over agent i's neighbors
        u(:, i) = [0 ; 0];
%         if(i~=closestAgent)
       
        % Get the topological neighbors of agent i from the communication
        % topology
        for j = topological_neighbors(L, i)
                
            % For each neighbor, calculate appropriate formation control term and
            % add it to the total velocity

            u(:, i) = u(:, i) + ...
            10*(norm(x(1:2, i) - x(1:2, j)) - W(i, j)+norm(x(1:2, i) - endPoint)) ... 
            *(x(1:2, j) - x(1:2, i));
%         end 
        end
    end
    
    %% Avoid errors
   
 
    % To avoid errors, we need to threshold dxi
    norms = arrayfun(@(x) norm(u(:, x)), 1:nAgents);
    threshold = rbtm.max_linear_velocity/2;
    to_thresh = norms > threshold;
    u(:, to_thresh) = threshold*u(:, to_thresh)./norms(to_thresh);
    % Transform the single-integrator dynamics to unicycle dynamics using a provided utility function
    u= si_barrier_cert(u, x);
    dx = si_to_uni_dyn(u, x);  
    
    % Set velocities of agents 1:N
    rbtm.set_velocities(1:nAgents, dx);
    
    % Send the previously set velocities to the agents.  This function must be called!
    rbtm.step();   
end
%circular formation around the flag
center = [1;0];
radius = 0.2;
% interAgentDistance = radius*0.00001*sin(pi/N);
interAgentDistance = 1*10^-100;
kp1 = 8;
kp2 = 8;
% plot(center(1),center(2),'*','markersize',12)
th = 0 : 2*pi/20 : 2*pi-2*pi/20;
plot(radius.*cos(th)+center(1),radius.*sin(th)+center(2),'b')

for t = 1:maxIterations/15
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    % x = r.get_poses();
    x = rbtm.get_poses(); 
    xi = x(1:2,:);
    u = zeros(2,nAgents);
    
    % Convert to SI states
    % xi = uni_to_si_states(x);
    
    %% Algorithm
    
    for i = 1:nAgents
        
        % Initialize velocity to zero for each agent.  This allows us to sum
        %over agent i's neighbors
       u(:, i) = [0 ; 0];
        
        % Get the topological neighbors of agent i based on the graph
        %Laplacian L
        %neighbors = topological_neighbors(L, i);
        
        % Iterate through agent i's neighbors
        for j = topological_neighbors(L, i)%neighbors
            
            % For each neighbor, calculate appropriate consensus term and
            %add it to the total velocity
            %theta = pi/N;
            %dxi(:, i) = dxi(:, i) + ((xi(1, j) - xi(1, i))*(cos(theta))+(xi(2, j) - xi(2, i))*(sin(theta)));
            alpha = pi/nAgents+ kp1*(d- norm(xi(:,j)-xi(:,i)) );
                R = [cos(alpha), sin(alpha); -sin(alpha) cos(alpha)];
                %dxi(:,i) = dxi(:,i) + R*( xi(:,j)-xi(:,i) ) - kp2*( xi(:,i) - center );
               u(:,i) = u(:,i) + R*( xi(:,j)-xi(:,i) ) - kp2*( norm(xi(:,j)-xi(:,i)) - d)*(xi(:,i) - center);
        end        
    end
    
    %% Avoid errors
    
    % To avoid errors, we need to threshold dxi
    norms = arrayfun(@(x) norm(u(:, x)), 1:nAgents);
    threshold = rbtm.max_linear_velocity/5;
    to_thresh = norms > threshold;
    u(:, to_thresh) = threshold*u(:, to_thresh)./norms(to_thresh);
    
    %% Utilize barrier certificates
    u = si_barrier_cert(u, x);
    
    % Transform the single-integrator to unicycle dynamics using the the
    % transformation we created earlier
    dx = si_to_uni_dyn(u,x);
    
    %% Send velocities to agents
    
    % Set velocities of agents 1,...,N
    rbtm.set_velocities(1:nAgents, dx);
    
    % Send the previously set velocities to the agents.  This function must be called!    
    rbtm.step();
end

%obstacle detection
P = [1/0.008, 0; 0, 1/0.008];
center = [0.35;0.02];
t = linspace(0,2*pi,30);
patch(1/sqrt( P(1,1))*cos(t) + center(1), 1/sqrt(P(2,2))*sin(t) + center(2),[0.32,0.32,0.32]);

Delta = 1.2;
dt=0.03;
d1=-0.5;
d2=-0.5;
goal=[d1;d2];
tot_constraints = 6;
N=3;% 3 agent-to-obstacle constraints + 3 agent-to-agent constraints
opts = optimoptions(@quadprog,'Display','off');

for iter = 1:maxIterations/10
    x = rbtm.get_poses(); 
    xi = x(1:2,:);
    u = zeros(2,nAgents);
                                            % Initialize velocities to zero         
    for i = 1:N                
        for k =topological_neighbors(L, i)
            if ~isempty(k)
                u(:,i) = u(:,i) + (norm(xi(:,i)-xi(:,k)) - d)*( xi(:,k)-xi(:,i) );
            end
        end
        u(:,i) = u(:,i) + 0.1.*(goal-xi(:,i))./norm(goal-xi(:,i));
    end
     
    
    % Inizialize zero matrices for storing constraints
    Abf = zeros(tot_constraints, 2*N);
    Bbf = zeros(tot_constraints,   1);
    count = 1;
    % Connectivity constraints
    for k = 1:3
        i = E(k,1);
        j = E(k,2);
        h_ij = (Delta)^2 - (xi(:,i)-xi(:,j))'*(xi(:,i)-xi(:,j));            
        Abf(count , 2*i-1:2*i ) =  2*( xi(:,i)-xi(:,j) );
        Abf(count , 2*j-1:2*j ) = -2*( xi(:,i)-xi(:,j) );   
        Bbf(count) = 1e5*h_ij^3;
        count = count+1;
    end
    
    % Obstacle avoidance
    for i = 1:3
        h_obs = (xi(:,i) - center )'*P*(xi(:,i) - center ) - 1.5;
        Abf(count, 2*i-1:2*i) = -2*( xi(:,i) - center )'*P;
        Bbf(count) = 10*h_obs^3;
        count = count + 1;
    end

    % BF Solution
    H = 2*eye(2*N);
    f = -2*reshape(u,[2*N,1]);
    v = quadprog(sparse(H), double(f), Abf, Bbf, [],[], [], [], [], opts);
    v = reshape(v,[2,N]);

   xi=xi+v*200*dt;
   
    u = si_pos_controller(x(1:2,:), xi);
    
    % To avoid errors, we need to threshold u.
    norms = arrayfun(@(xi) norm(u(:, xi)), 1:nAgents);
    threshold = rbtm.max_linear_velocity/2;
    to_thresh = norms > threshold;
    u(:, to_thresh) = threshold*u(:, to_thresh)./norms(to_thresh);
    % Transform the single-integrator dynamics to unicycle dynamics
    % using a provided utility function
    u = si_barrier_cert(u, x);
    dx = si_to_uni_dyn(u, x);
   
        rbtm.set_velocities(1:nAgents, dx);
        delete(flagplot)
    agenPlot=plot(mean(x(1,:)),mean(x(2,:)),'*','markersize',12);
    rbtm.step(); 
delete(agenPlot)

%     set(agenPlot,'xdata',xi(1,:),'ydata',xi(2,:))
%     set(edgePlot,'Xdata',[xi(1,E(:,1)),xi(1,E(:,2))], 'Ydata',[xi(2,E(:,1)),xi(2,E(:,2))])
%     drawnow
end
plot(mean(x(1,:)),mean(x(2,:)),'*','markersize',12);
% d1=2;
% d2=0.5;
% final_conditions = x-[d1 d1 d1 ;d2 d2 d2;0 0 0];

% args = {'PositionError', 0.01, 'RotationError', 50};
% init_checker = create_is_initialized(args{:});
%   while(~init_checker(x, final_conditions))
%  x = rbtm.get_poses();
%     u = si_pos_controller(x(1:2, :), final_conditions(1:2, :));
%     norms = arrayfun(@(x) norm(u(:, x)), 1:nAgents);
%     threshold = rbtm.max_linear_velocity/2;
%     to_thresh = norms > threshold;
%     u(:, to_thresh) = threshold*u(:, to_thresh)./norms(to_thresh);
%     u = si_barrier_cert(u, x); 
%     dx = si_to_uni_dyn(u, x);
%     rbtm.set_velocities(1:nAgents, dx);
% 
%     agenPlot=plot(mean(x(1,:)),mean(x(2,:)),'o','markersize',12,'MarkerFaceColor',[0.12,0.49,0.65],'MarkerEdgeColor','none');
%  set(agenPlot,'xdata',mean(x(1,:)),'ydata',mean(x(2,:)));
%     rbtm.step(); 
% 
%   end
  

if videoFlag
    close(vid);
end
rbtm.debug();