function [ui,persistentMemory] = controller(missionData,persistentMemory,currentTime)
% Decentralized controller for multi-robot search and rescue mission
% 
% Inputs:
%   +missionData struct containining the following fields:
%       -Submission     : current submission ('init', 'room', 'return', 'refuel') 
%       -SensorData     : 8-by-1 vector of distance to objects in sensor range in the 0,45,...,315 degree directions
%       -AgentState     : 2-by-1 position of current agent
%       -AgentID        : current agent id
%       -NeighborState  : 2-by-|Ni| matrix of visible neighbor states
%       -NeighborIDs    : 1-by-|Ni| vector of neighbor id's
%       -MissionInfo    : Struct containing mission info (e.g., target)
%       -MinSafetyDist  : Minimum safety separation
%       -MaxSensCommRng : Maximum sensing/comm distance
%   +persistentMemory   : 1-by-|Ni| vector of auxiliary flags
%   +currentTime        : current mission time
% Outputs:
%   +ui                 : 2-by-1 velocity reference for current agent
%   +persistentMemory   : 1-by-|Ni| vector of persistent memory scalars per neighbor

% Extracting variable info
xi = missionData.AgentState;
xj = missionData.NeighborStates;
ID = missionData.AgentID;
nIDs = missionData.NeighborIDs;
currentMission = missionData.Submission;
Delta = missionData.MaxSensCommRng;
delta = missionData.MinSafetyDist;
A_formation = missionData.FormationSpec;
sensorData = missionData.SensorData;
deltaSensor = delta*0.85/1.25;
numSensors = length(sensorData);
sensorTheta = 0:2*pi/numSensors:2*pi*(1-1/numSensors);
sensorDir = [cos(sensorTheta);sin(sensorTheta)];
targetDataAvailable = isfield(missionData.MissionInfo,'target');
leaderAgent = false;


if targetDataAvailable

    switch currentMission
        case 'refuel'
            % Get the array of feasible target positions
            feasiblePoints = missionData.MissionInfo.feasiblePoints;
            formationCentroidTarget = missionData.MissionInfo.target;
        otherwise
            % Flag to identify as one of two leader agents
            leaderAgent = true;
            % Target waypoint
            xt = missionData.MissionInfo.target;
    end
end
% Initialize control
ui = [0;0];
auxFlags = zeros(6);
% Define the density function phi (customize as needed)

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Modify this block %%%%%%%%%%%%%%%%%%%%%%%%%%%%
switch currentMission
    case 'init'
        % Formation control loop
        for jj = 1:size(xj, 2)
            xjj = xj(:, jj);                      % Neighbor position
            nij = norm(xjj - xi);                 % Distance to neighbor
           
            % Update auxFlags based on distance
            if (persistentMemory(jj) == 0) && (nij < 0.9 * Delta)
                persistentMemory(jj) = 1;
            end
            
            dij = A_formation(jj);                % Desired distance to neighbor
            % Calculate weight wij
            if dij == 0 || persistentMemory(jj) == 0
                wij = (nij^2-Delta*delta) / ((Delta-nij)^3*(nij-delta)^3)*5e-2;
            else
                wij = 5e-1 * ((nij - dij) * (dij * (Delta + delta - 2 * nij)- delta * Delta + nij^2) / (nij * (Delta - nij)^3 * (nij - delta)^3));
            end
    
            % Add to control input
            ui = ui + wij * (xjj - xi);
        end
        % Leader control (if applicable)
        if ID == 6 
            leaderGain = 5; % Adjust gain for leader influence
            ui = 2*ui + leaderGain*(xt - xi) / max(norm(xt - xi), delta);
            % disp(xt);
        end
    
        for k = 1:numSensors
            if ~isinf(sensorData(k))               % Only consider obstacles in range
                dik = sensorData(k);               % Distance to obstacle
                xoMinusxi = dik * sensorDir(:, k); % Vector to obstacle

                distok = norm(xoMinusxi);

                % Repulsive weight wik
                wik = -5e-4 * (Delta - delta) * (Delta - distok) / (distok * (distok - delta)^3);

                % Add repulsion to control input
                ui = ui + wik * (xoMinusxi);
            end
        end

            % Normalize control input to allow higher speeds
        maxVelocity = 5.0; % Increased max speed cap
        ui = ui / max(norm(ui), maxVelocity);     
        
    case 'return'
        
        % Leader control (if applicable)
        for k = 1:numSensors
            if ~isinf(sensorData(k))               % Only consider obstacles in range
                dik = sensorData(k);               % Distance to obstacle
                xoMinusxi = dik * sensorDir(:, k); % Vector to obstacle

                distok = norm(xoMinusxi);

                if ID == 6
                    wik = -5e-4 * (Delta - delta) * (Delta - distok) / (distok * (distok - delta)^2);
                else
                    wik = -3e-4 * (Delta - delta) * (Delta - distok) / (distok * (distok - delta)^2);
                end

                % Add repulsion to control input
                ui = ui + wik * (xoMinusxi);
            end
        end

        % Formation control loop
        for jj = 1:size(xj, 2)
            xjj = xj(:, jj);                      % Neighbor position
            nij = norm(xjj - xi);                 % Distance to neighbor
           
            % Update auxFlags based on distance
            if (persistentMemory(jj) == 0) && (nij < 0.9 * Delta)
                persistentMemory(jj) = 1;
            end
            
            dij = A_formation(jj);                % Desired distance to neighbor
            % Calculate weight wij
            if dij == 0 || persistentMemory(jj) == 0
                wij = -6e-2 * (Delta - delta) * (Delta - nij) / (nij * (nij - delta)^2);
                % wij = (nij^2-Delta*delta) / ((Delta-nij)^3*(nij-delta)^3)*5e-2;
            else
                wij = 4e-1 * ((nij - dij) * (dij * (Delta + delta - 2 * nij)- delta * Delta + nij^2) / (nij * (Delta - nij)^3 * (nij - delta)^3));
            end
    
            % Add to control input
            ui = ui + delta*wij * (xjj - xi)/max(norm(xjj - xi), delta);
        end

        if ID == 6 
            ui = ui + 0.2*(xt - xi) / max(norm(xt - xi), delta);
            % disp(xt);
        end

    case 'room'
        
        % Corners of the room
        beacons = missionData.MissionInfo.domain;
        minX = min(beacons(1,:)); maxX = max(beacons(1,:));
        minY = min(beacons(2,:)); maxY = max(beacons(2,:));
        domWidth = maxX - minX; domHeight = maxY - minY;

            % Centroid of the domain
        c_x = (minX + maxX) / 2;
        c_y = (minY + maxY) / 2;

         % Parameters for density center motion
        omega = 2 * pi / 10; % Angular velocity for spiraling motion
        r_max = max(domWidth, domHeight) / 2; % Max radius of spiral
        r_t = @(t) (r_max / 10) * t; % Radius grows linearly with time
        mu_x = @(t) c_x + r_t(t) * cos(omega * t);
        mu_y = @(t) c_y + r_t(t) * sin(omega * t);
        
        % Compute density center position at the current time
        mu_t = [mu_x(currentTime); mu_y(currentTime)];
        
        % Parameters for density function
        sigma = 2; % Spread of the Gaussian density

        % Generate grid for density computation
        numPoints = 10; % Grid resolution
        [qx, qy] = meshgrid(...
            linspace(minX, maxX, numPoints), ...
            linspace(minY, maxY, numPoints));
        qx = qx(:); % Flatten grid
        qy = qy(:);

        % Define time-variant density function
        phi = exp(-((qx - mu_t(1)).^2 + (qy - mu_t(2)).^2) / (2 * sigma^1));

        % Time derivative of the density
        dmu_dt = [-r_t(currentTime) * omega * sin(omega * currentTime); ...
                   r_t(currentTime) * omega * cos(omega * currentTime)] + ...
                  [r_max / 10 * cos(omega * currentTime); ...
                   r_max / 10 * sin(omega * currentTime)];
        
        dphi_dt = ((qx - mu_t(1)) .* dmu_dt(1) + (qy - mu_t(2)) .* dmu_dt(2)) ./ sigma .* phi;

        % Compute Voronoi cell for the agent
        vcell = missionData.MissionInfo.AgentVCell;
        inPoly = inpolygon(qx, qy, vcell(1,[1:end 1]).', vcell(2,[1:end 1]).');

        % Mass and center of mass in the agent's Voronoi cell
        dArea = (maxX - minX) * (maxY - minY) / (numPoints^2); % Area of grid cells
        mass = sum(phi(inPoly) * dArea);
        cmass = [sum(qx(inPoly) .* phi(inPoly)) * dArea; ...
                 sum(qy(inPoly) .* phi(inPoly)) * dArea] / mass;

        % Time derivative of center of mass
        dcdt = [sum((qx(inPoly) - cmass(1)) .* dphi_dt(inPoly)) * dArea; ...
                sum((qy(inPoly) - cmass(2)) .* dphi_dt(inPoly)) * dArea] / mass;

        % Control law for coverage control
        k = 0.8; % Gain
        ui = k * (cmass - xi) + dcdt;

         % Formation control loop
        for jj = 1:size(xj, 2)
            xjj = xj(:, jj);                      % Neighbor position
            nij = norm(xjj - xi);                 % Distance to neighbor
           
            % Update auxFlags based on distance
            if (persistentMemory(jj) == 0) && (nij < 0.9 * Delta)
                persistentMemory(jj) = 1;
            end
            wij = -2e-3 * (Delta - delta) * (Delta - nij) / (nij * (nij - delta)^2);
            
        
            % Add to control input
            ui = ui + wij * (xjj - xi)/max(norm(xjj - xi), delta);
        end

        for k = 1:numSensors
            if ~isinf(sensorData(k))               % Only consider obstacles in range
                dik = sensorData(k);               % Distance to obstacle
                xoMinusxi = dik * sensorDir(:, k); % Vector to obstacle

                distok = norm(xoMinusxi);

                wik = -1e-3 * (Delta - delta) * (Delta - distok) / (distok * (distok - delta)^2);

                % Add repulsion to control input
                ui = ui + wik * (xoMinusxi);
            end
        end
         % Normalize control input to limit excessive velocities
        ui = ui / max(norm(ui), 1e-1);

    case 'refuel'
        % disp(missionData.MissionInfo.specGraph);
            % Extract refuel mission information
        specGraph = missionData.MissionInfo.specGraph;       % Specification graph (adjacency matrix)
        feasiblePoints = missionData.MissionInfo.feasiblePoints; % Feasible positions (2-by-N matrix)
        refuelCentroid = missionData.MissionInfo.target;     % Refuel platform centroid
        
        % Initialize control input
        ui = [0; 0];
        
        % Calculate current centroid of the agents
        allAgents = [xi, xj]; % Include the current agent and its neighbors
        currentCentroid = mean(allAgents, 2);
        
        % Control to move the centroid towards the refuel platform
        centroidError = refuelCentroid - currentCentroid;
        centroidControl = 10 * centroidError; % Gain for centroid alignment
        ui = ui + centroidControl;
        % Target the specific feasible point corresponding to this agent
        agentTargetPoint = feasiblePoints(:, ID); % Each agent gets its specific point
        targetError = agentTargetPoint - xi;
        targetControl = 5 * targetError; % Gain for agent-target alignment
        ui = ui + targetControl;
        
        % Formation control based on the specification graph
        for jj = 1:size(xj, 2)
            xjj = xj(:, jj);                      % Neighbor position
            nij = norm(xjj - xi);                 % Distance to neighbor

            % Update auxFlags based on distance
            if (persistentMemory(jj) == 0) && (nij < 0.9 * Delta)
                persistentMemory(jj) = 1;
            end

            % Get desired distance from the specification graph
            if specGraph(ID, nIDs(jj)) > 0
                dij = specGraph(ID, nIDs(jj));
            else
                dij = Delta; % Default to max sensing range if no specGraph data
            end

            % Calculate formation control weight
            if dij == 0 || persistentMemory(jj) == 0
                % wij = (nij^2 - Delta * delta) / ((Delta - nij)^3 * (nij - delta)^3) * 1e-2;
                wij = -2e-3 * (Delta - delta) * (Delta - nij) / (nij * (nij - delta)^2);
            else
                wij = 1e-1 * ((nij - dij) * (dij * (Delta + delta - 2 * nij) - delta * Delta + nij^2) / ...
                    (nij * (Delta - nij)^3 * (nij - delta)^3));
            end

            % Add formation control input
            ui = ui + wij * (xjj - xi);
        end
         
        % Obstacle avoidance using sensor data
        for k = 1:numSensors
            if ~isinf(sensorData(k))               % Only consider obstacles in range
                dik = sensorData(k);               % Distance to obstacle
                xoMinusxi = dik * sensorDir(:, k); % Vector to obstacle
                
                distok = norm(xoMinusxi);
                if distok < delta % Add repulsive force
                    repulsion = -5e-4 * (Delta - delta) * (Delta - distok) / ...
                        (distok * (distok - delta)^3);
                    ui = ui + repulsion * (xoMinusxi / distok);
                end
            end
        end
        
        % Normalize control input to prevent excessive velocities
        ui = ui / max(norm(ui), 1e-2);
        
        
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Modify this block %%%%%%%%%%%%%%%%%%%%%%%%%%%%
end