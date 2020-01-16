%% INTEGRATED CODE
% DATE:         NOV 30, 2018
% REV:          2.0.0
% DESCRIPTION:  An integrated version of all the commands
clear all, close all, clc
%% CONSTANTS AND FLAGS
% Insert Constants, flags and other things here.
nAgents = 6;
maxIterations = 5000;
captureDistance = 0.01;
agentRange = 0.3;
% Flags
videoFlag = 0;
isAttackComplete = 0;
isRendezVousComplete = 0;
isFormationComplete = 0;
isFetchComplete = 0;
isInitialPositionReached = 0;
isInitRendezVousComplete = 0;
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
targetPoint = [0.75; 0];
radius = 0.01;
flagplot = plot(targetPoint(1), targetPoint(2), '*', 'markersize', 12, ...
    'MarkerEdgeColor','red');
th = 0 : 2*pi/20 : 2*pi-2*pi/20;
% Home Goal Definition
homePoint = [-0.75; 0];
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
basePoint = [0; 0.65];
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
%                             INIT RV
%--------------------------------------------------------------------------
L_init = completeGL(3);
currIteration = 0;
maxIteration = 500;
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
maxIterationFormation = 500;
%--------------------------------------------------------------------------
%                               FETCH
%--------------------------------------------------------------------------
currIterationFetch = 0;
maxIterationFetch = 400;
%--------------------------------------------------------------------------
%                               DEFEND
%--------------------------------------------------------------------------
nDefenders = 3;
L_defense = completeGL(nDefenders);
currIterationInitPos = 0;
maxIterationInitPos = 500;
isInitialPositionReached = 0;

A = diag(ones(nDefenders-1,1),-1);
A(1,nDefenders) = 1; 
L_cycle = diag(sum(A)) - A;
interAgentDistance = 0.0001;
kp1 = 4
kp2 = 8
%% MAIN SECTION
state = 1;
minDist = 100;
%--------------------------------------------------------------------------
%                                FOR LOOP
%--------------------------------------------------------------------------
for k = 1:maxIterations
    x = rbtm.get_poses();
%--------------------------------------------------------------------------
%                                ATTACK
%--------------------------------------------------------------------------    
    if (isAttackComplete == 0)
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
%         if (minDist <= agentRange)
%             swapRowTemp = L_attack(closestAgent,1:nAttackers);
%             L_attack(1,1:nAttackers) = swapRowTemp;
%             L_attack(closestAgent , 1:nAttackers) = zeros(1,nAttackers);
%         end
        if (minDist <= captureDistance)
            isAttackComplete = 1;
            for i = 1:nAttackers
                u(1:2, i) = [0;0];
            end
        end  
        if videoFlag && mod(k,10)                              
            writeVideo(vid, getframe(gcf)); 
        end
    end
%--------------------------------------------------------------------------
%                                RENDEZ-VOUS
%--------------------------------------------------------------------------    
    if (isAttackComplete == 1 && isRendezVousComplete == 0)
        if (currIterationRendezVous <= maxIterationRendezVous)
            for i = 1:nAttackers
                u(:, i) = [0 ; 0];
                for j = topological_neighbors(L_RendezVous, i)
                    u(:, i) = u(:, i) + ...
                        10*(norm(x(1:2, i) - targetPoint)) ... 
                        *(targetPoint - x(1:2, i));
                end
            end
            currIterationRendezVous = currIterationRendezVous + 1;
        else
            if(isRendezVousComplete == 1)
                continue
            else
                isRendezVousComplete = 1;
                disp('Rendez Vous Complete')
                for i = 1:nAttackers
                    u(1:2, i) = [0;0];
                end
             end
        end
    end
% --------------------------------------------------------------------------
%                                FORMATION
% --------------------------------------------------------------------------    
    if (isFormationComplete == 0 && isRendezVousComplete == 1)
        if(currIterationFormation <= maxIterationFormation)
            for i = 1:nAttackers
                u(:, i) = [0 ; 0];
                for j = topological_neighbors(L_formation, i)
                    if ~isempty(j)
                        u(:, i) = u(:, i) + (norm(x(1:2, i) ...
                            - x(1:2, j)))*(x(1:2, j) - x(1:2, i));
                    end
                end 
                u(:,i) = u(:,i) + ... 
                    5*(basePoint - x(1:2, i))./norm((basePoint - x(1:2, i)));
            end
        else
            isFormationComplete = 1;
            for i = 1:nAttackers
                u(1:2, i) = [0;0];
            end
            break
        end
    end
%--------------------------------------------------------------------------
%                                PROTECT
%--------------------------------------------------------------------------    
   if (isInitialPositionReached == 0)
        if(currIterationInitPos <= maxIterationInitPos)
            for i = 1:nDefenders
                i_offset = i + nDefenders;
                u(:, i) = [0 ; 0];
                for j = topological_neighbors(L_defense, i)
                    j_offset = j + nDefenders;
                    u(:, i_offset) = u(:, i_offset) + ...
                            40*(norm(x(1:2, i_offset) - homePoint))...
                            *(homePoint - x(1:2, i_offset));
                end 
            end 
            currIterationInitPos = currIterationInitPos + 1;
        else
            isInitialPositionReached = 1;
            for i = (nDefenders+1):nAgents
                u(:,i) = [0;0];
            end
        end
   else
       for i_offset = 1:nDefenders
            for j_offset = topological_neighbors(L_cycle, i_offset)
                i = i_offset + nAttackers;
                j = j_offset + nAttackers;
                if ~isempty(j_offset)
                        alpha = pi/(nAgents) + ...
                               kp1*...
                               (interAgentDistance - norm(x(:,j)-x(:,i)) );
                        R = [cos(alpha), sin(alpha);...
                            -sin(alpha) cos(alpha)];
                        u(:,i) = u(:,i) + ...
                            R*( x(1:2,j)-x(1:2,i) ) - ...
                            kp2*(norm(x(1:2,j)-x(1:2,i))-interAgentDistance)...
                            *(x(1:2,i) - homePoint);
                end
           end
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
end
        
%% END
disp(k)
rbtm.debug()