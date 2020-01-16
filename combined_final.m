%% INTEGRATED CODE
% DATE:         NOV 30, 2018
% REV:          2.0.0
% DESCRIPTION:  An integrated version of all the commands
clear all, close all, clc
%% CONSTANTS AND FLAGS
% Insert Constants, flags and other things here.
nAgents = 6;
maxIterations = 10000;
captureDistance = 0.01;
agentRange = 0.3;
% Flags
videoFlag = 0;
isAttackComplete = 0;
isRendezVousComplete = 0;
isFormationComplete = 0;
isFormation2Complete = 0;
isFetchComplete = 0;
%% SETTING UP ROBOTARIUM, SAFETY AND DYNAMICS SPECIFICATIONS
rbtm = Robotarium('NumberOfRobots', nAgents, 'ShowFigure', true);
si_barrier_cert = create_si_barrier_certificate('SafetyRadius', ...
    1.5*rbtm.robot_diameter);
si_to_uni_dyn = create_si_to_uni_mapping2('LinearVelocityGain', 0.5, ... 
    'AngularVelocityLimit', 0.75*rbtm.max_angular_velocity);
si_pos_controller = create_si_position_controller();
%% SETTING UP LOCATIONS FOR HOME FLAG, OBSTACLE AND ADVERSARY FLAG
% Initial Location
initialPoint = [-1.2, -0.8];
% Target Goal definition
targetPoint = [1; 0];
radius = 0.01;
flagplot = plot(targetPoint(1), targetPoint(2), '*', 'markersize', 12, ...
    'MarkerEdgeColor','red');
th = 0 : 2*pi/20 : 2*pi-2*pi/20;
% Home Goal Definition
homePoint = [-1, 0];
radius = 0.2;
plot(homePoint(1), homePoint(2), '*', 'markersize', 12, ...
    'MarkerEdgeColor','green');
% Obstacle Placement
% P = [1/0.008, 0; 0, 1/0.008];
% obstaclePoint = [0.35;-0.1];
% t = linspace(0,2*pi,30);
% patch(1/sqrt( P(1,1))*cos(t) + obstaclePoint(1), ...
%     1/sqrt(P(2,2))*sin(t) + obstaclePoint(2), ...
%     [0.32,0.32,0.32]);
% Homebase Location
basePoint = [-0.75, 0.75];
plot(basePoint(1), basePoint(2), 'o', 'markersize', 5, ...
    'MarkerEdgeColor','blue');
%% WAYPOINT GENERATION
xBound = 1.6; yBound = 1;
WayPointsRand = zeros(2,100);
WayPointsRand(:,1) = initialPoint;
for i = 95:100 %The leader could wander around the target flag for a little.
    WayPointsRand(:,i) = targetPoint;
end
temp = 0;
for ctr = 2:94
    temp = fix(rand * 10);
    if(WayPointsRand(1,ctr - 1) < 0 && WayPointsRand(2,ctr - 1) < 0) 
        switch temp                  
            case 0                   
                directXpositive = 1; 
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
        switch temp                  
            case 0                   
                directXpositive = 1; 
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
    GainXpositive = 1/8;
    GainYpositive = 1/8;
    GainXnegative = 1/10;
    GainYnegative = 1/10; 
    WayPointsRand(1,ctr) = WayPointsRand(1,ctr - 1) + ...
        directXpositive * GainXpositive + directXnegative * GainXnegative;
    if(WayPointsRand(1,ctr) > xBound - 0.05) %Avoid hit the arena boundary
        WayPointsRand(1,ctr) = xBound - 0.15;
    end
    WayPointsRand(2,ctr) = WayPointsRand(2,ctr - 1) + ...
        directYpositive * GainYpositive + directYnegative * GainYnegative;
    if(WayPointsRand(2,ctr) > yBound - 0.05) %Avoid hit the arena boundary
        WayPointsRand(2,ctr) = yBound - 0.15;
    end
end
%% LAPLACIAN, WEIGHTS AND DEFINITIONS
%--------------------------------------------------------------------------
%                                LEADER - FOLLOWER LAPLACIAN
%--------------------------------------------------------------------------
nAttackers = 3;
followerAgents = -completeGL(nAttackers - 1);
L_attack = zeros (nAttackers, nAttackers);
L_attack(2:nAttackers, 2:nAttackers) = followerAgents;
L_attack(2,2) = L_attack(2,2) + 1;
L_attack(2,1) = -1;
% Initialize Velocity Vector
u = zeros(2,nAgents);
% Weight Matrix
agentSeparation = 2.5 * rbtm.robot_diameter;
% Initializing Velocity
u = zeros(2, nAgents);
%Gains
formationGain = 5;
desired_distance = 0.3;
maxAllowableIterations=1000;
closestAgent = 0;
%--------------------------------------------------------------------------
%                             RENDEZ VOUS
%--------------------------------------------------------------------------
currIterationRendezVous = 0;
maxIterationRendezVous = 400;
L_RendezVous = cycleGL(nAttackers);
%--------------------------------------------------------------------------
%                             FORMATION
%--------------------------------------------------------------------------
L_formation = completeGL(nAttackers);
d = agentSeparation;
W_formation = zeros(nAttackers, nAttackers);
for i = 1:nAttackers
    for j = topological_neighbors(L_formation, i)
        W(i,j) = d;
    end
end
currIterationFormation = 0;
maxIterationFormation = 400;
%--------------------------------------------------------------------------
%                               FETCH
%--------------------------------------------------------------------------
currIterationFetch = 0;
maxIterationFetch = 400;
%--------------------------------------------------------------------------
%                               DEFEND
%--------------------------------------------------------------------------
nDefenders = 3;
A = diag(ones(nDefenders-1,1),-1);
A(1,nDefenders) = 1; 
L = diag(sum(A)) - A;     
transformationGain = 0.06;
isInitialPositionReached = 0;
%% MAIN SECTION
state = 1;
minDist = 100;
%--------------------------------------------------------------------------
%                                FOR LOOP
%--------------------------------------------------------------------------
for k = 1:maxIterations

%--------------------------------------------------------------------------
%                                ATTACK
%--------------------------------------------------------------------------    
    if (isAttackComplete == 0)
        x = rbtm.get_poses();
        for i = 2:nAttackers
            u(:, i) = [0 ; 0];
            for j = topological_neighbors(L_attack,i)
                u(:,i) = u(:, i) + ...
                    formationGain*(norm(x(1:2, j) - x(1:2, i))^2 - ...
                        agentSeparation^2)*(x(1:2, j) - x(1:2, i));
            end
        end
        waypoint = WayPointsRand(:, state);
        u(:, 1) = si_pos_controller(x(1:2, 1), waypoint);
        if(mod(k,(maxAllowableIterations/50)) == 0)
            if(state < 99)
                state = state + 2;
            else
                state = 100;
            end
        end
        for i = 1:nAttackers
           distToTarget = norm(x(1:2,i)-targetPoint);
           if (minDist > distToTarget)
               minDist = distToTarget;
               closestAgent = i;
           end
        end
        if (minDist <= agentRange)
            swapRowTemp = L_attack(closestAgent,1:nAttackers);
            L_attack(1,1:nAttackers) = swapRowTemp;
            L_attack(closestAgent , 1:nAttackers) = zeros(1,nAttackers);
        end
        if (minDist <= captureDistance)
            isAttackComplete = 1
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
%--------------------------------------------------------------------------
%                                RENDEZ-VOUS
%--------------------------------------------------------------------------    
    if (isAttackComplete == 1 & isRendezVousComplete == 0)
        if (currIterationRendezVous <= maxIterationRendezVous)
            x = rbtm.get_poses();
            for i = 1:nAgents
                u(:, i) = [0 ; 0];
                for j = topological_neighbors(L_RendezVous, i)
                    u(:, i) = u(:, i) + ...
                        10*(norm(x(1:2, i) - targetPoint) - 0.4) ... 
                        *(targetPoint - x(1:2, i));
                end
            end
            % To avoid errors, we need to threshold dxi
            norms = arrayfun(@(x) norm(u(:, x)), 1:nAgents);
            threshold = rbtm.max_linear_velocity/2;
            to_thresh = norms > threshold;
            u(:, to_thresh) = threshold*u(:, to_thresh)./norms(to_thresh);
            % Transform the single-integrator dynamics to unicycle dynamics
            u = si_barrier_cert(u, x);
            dx = si_to_uni_dyn(u, x);  
            % Set velocities of agents 1:N
            rbtm.set_velocities(1:nAgents, dx);
            rbtm.step();
            currIterationRendezVous = currIterationRendezVous + 1;
        else
            isRendezVousComplete == 1
        end
    end
%--------------------------------------------------------------------------
%                                FORMATION
%--------------------------------------------------------------------------    
    if (isFormationComplete == 0 && isRendezVousComplete == 1)
        if(currIterationFormation <= maxIterationFormation)
            x = rbtm.get_poses();
            for i = 1:nAttackers
                u(:, i) = [0 ; 0];
                for j = topological_neighbors(L_formation, i)
                    u(:, i) = u(:, i) + ...
                    10*(norm(x(1:2, i) - x(1:2, j)) - W(i, j) + ...
                    norm(x(1:2, i) - targetPoint))*(x(1:2, j) - x(1:2, i));
               end 
            end
            norms = arrayfun(@(x) norm(u(:, x)), 1:nAgents);
            threshold = rbtm.max_linear_velocity/2;
            to_thresh = norms > threshold;
            u(:, to_thresh) = threshold*u(:, to_thresh)./norms(to_thresh);
            u= si_barrier_cert(u, x);
            dx = si_to_uni_dyn(u, x);  
            rbtm.set_velocities(1:nAgents, dx);
            rbtm.step();   
            currIterationFormation = currIterationFormation + 1
        else
            isFormationComplete = 1
            currIterationFormation = 0
        end
    end
%--------------------------------------------------------------------------
%                                FORMATION2
%--------------------------------------------------------------------------    
    if (isFormation2Complete == 0 & isFormationComplete == 1)
        if(currIterationFormation <= maxIterationFormation)
            radius = 0.2;
            kp1 = 8;
            kp2 = 8;
            th = 0 : 2*pi/20 : 2*pi-2*pi/20;
            plot(radius.*cos(th)+targetPoint(1), ...
                 radius.*sin(th)+targetPoint(2),'b');           
            x = rbtm.get_poses(); 
            xi = x(1:2,:);
            for i = 1:nAgents
                u(:, i) = [0 ; 0];
                for j = topological_neighbors(L_formation, i)
                    alpha = pi/nAttackers+ kp1*(d- norm(xi(:,j)-xi(:,i)) );
                    R = [cos(alpha), sin(alpha); -sin(alpha) cos(alpha)];
                    u(:,i) = u(:,i) + R*( xi(:,j)-xi(:,i) ) - ...
                        kp2*( norm(xi(:,j)-xi(:,i)) - d)*(xi(:,i) ...
                        - targetPoint);
                end        
            end
            norms = arrayfun(@(x) norm(u(:, x)), 1:nAgents);
            threshold = rbtm.max_linear_velocity/5;
            to_thresh = norms > threshold;
            u(:, to_thresh) = threshold*u(:, to_thresh)./norms(to_thresh);
            u = si_barrier_cert(u, x);
            dx = si_to_uni_dyn(u,x);
            rbtm.set_velocities(1:nAgents, dx);
            rbtm.step();
            currIterationFormation = currIterationFormation + 1;
        else
            isFormation2Complete = 1;
        end
    end
%--------------------------------------------------------------------------
%                                FORMATION2
%--------------------------------------------------------------------------    
    if (isFetchComplete == 0 & isFormation2Complete == 1)
        goal = [-0.5; -0.5];
        if (currIterationFetch <= maxIterationFetch)
            x = rbtm.get_poses(); 
            xi = x(1:2,:);
            for i = 1:N                
                for k =topological_neighbors(L_formation, i)
                    if ~isempty(k)
                    u(:,i) = u(:,i) + (norm(xi(:,i)-xi(:,k)) - d)* ...
                            ( xi(:,k)-xi(:,i) );
                    end
                end
                u(:,i) = u(:,i) + 0.1.*(basePoint-xi(:,i))./...
                       norm(basePoint-xi(:,i));
                norms = arrayfun(@(xi) norm(u(:, xi)), 1:nAgents);
                threshold = rbtm.max_linear_velocity/2;
                to_thresh = norms > threshold;
                u(:, to_thresh) = threshold...
                            *u(:, to_thresh)./norms(to_thresh);
                u = si_barrier_cert(u, x);
                dx = si_to_uni_dyn(u, x); 
                rbtm.set_velocities(1:nAgents, dx);
                delete(flagplot)
                agenPlot=plot(mean(x(1,:)),...
                    mean(x(2,:)),'*','markersize',12);
                rbtm.step(); 
            end
            currIterationFetch = currIterationFetch + 1;
        else
            isFetchComplete = 1;
        end
    end
%--------------------------------------------------------------------------
%                                PROTECT
%--------------------------------------------------------------------------    
   if (isInitialPositionReached == 0)        
       circularTargets = [ cos( 0:2*pi/N:2*pi*(1- 1/N) ) ...
           ; sin( 0:2*pi/N:2*pi*(1- 1/N) ) ];
        errorToInitialPos = xi - circularTargets;                
        errorNorm = [1,1]*(errorToInitialPos.^2);               
        while max( errorNorm ) > 0.0005
            % Update state variables        
            xuni = rbtm.get_poses(); 
            xuni(1:2,1:3) = 0;
            xi = xuni(1:2,4:6);         
            % Update errors
            errorToInitialPos = xi - circularTargets;
            errorNorm = [1,1]*(errorToInitialPos.^2);
            % Conput control inputs
            u = -0.2.*errorToInitialPos;
        d   xi = si_to_uni_dyn(u, xuni);
        % Assing new control inputs to robots
        rbtm.set_velocities(1:N, dxi);                   
        rbtm.step();                                   
    end
 
end
        
%% END
rbtm.debug()