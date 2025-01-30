%% ENME808T Final Project: Multi-Robot Hospital Disinfection
% Main file for multi-robot hospital disinfection mission.
%
% Copyright (c) 2024
% Collaborative Controls and Robotics Laboratory
% University of Maryland, College Park
%
% All rights reserved.
%% Define mission parameters
% NOTE: You must run the file init.m at least once before this script!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set this flag to false when running the code on the Robotarium
simulate_true = true; % <-------- Set to false when running on Robotarium!!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Select the mission scenario
missionChoice = 'full'; % Pick: 'init', 'room', 'return', 'refuel', 'full'
missionList = {'init','room','return','refuel'};
switch missionChoice
    case 'full'
        currentSubmission = 'init';
    otherwise
        currentSubmission = missionChoice;
end
% Load the initial configuration
[numAgents,initPoses,finalFormation,Delta,imPos0] = loadMission(currentSubmission);
% disp((initPoses));
pairwiseDist = pdist(initPoses.');
A_spec = squareform(pairwiseDist);

% Vector of agent IDs
IDs = 1:numAgents;
%%% Do not uncomment
% controller = @controllerSolutionSubmitted;
% getFormationGraph = @getFormationGraphSubmitted;
%% Get Robotarium object used to communicate with the robots/simulator
r = Robotarium('NumberOfRobots', numAgents, 'ShowFigure', true, 'InitialConditions', initPoses);
% Safety distance
delta = 1.25*r.robot_diameter; % Robot diameter plus a buffer
% Domain Boundaries
domBoundary = r.boundaries;
% Max velocities
vmax = 0.75*r.max_linear_velocity;
% Max velocities
wmax = 0.5*r.max_angular_velocity;
%% Initial Connectivity
% Retrieve the most recent poses from the Robotarium.
x = r.get_poses(); r.step();
% Rename for convenience
robotPositions = x(1:2,:);
robotHeadings = x(3,:);
% Get initial connectivity
[A,pairwiseDist] = deltaDisk(robotPositions,Delta);

edgeIndices = getEdgeIndices(A);
edgeCoordinatesX = [robotPositions(1,edgeIndices(:,1));robotPositions(1,edgeIndices(:,2));nan(1,size(edgeIndices,1))];
edgeCoordinatesY = [robotPositions(2,edgeIndices(:,1));robotPositions(2,edgeIndices(:,2));nan(1,size(edgeIndices,1))];
%% Visualization Elements
% Get Robotarium figure handle
hFig = r.figure_handle;
if simulate_true
    % Get current children graphic handles
    hChildren = get(gca,'Children');
    % Get robot handles
    hRobots = hChildren(1:numAgents);
    % Get boundary handle
    hBoundary = hChildren(end);
end
% Load the background image
[hIm,imX,imY,imPos,imCData] = loadBackgroundImage('missionMap.png',domBoundary);
imPos = imPos + imPos0;
set(hIm,'XData',imX+imPos(1),'YData',imY+imPos(2));
% Load obstacle data
plot_obstacles = false;
[obstacleData,hObs] = getObstacleData(imPos,plot_obstacles);
% Load the navigation waypoints
plot_waypoints = false;
[hWaypoints,navWaypoints] = getNavigationWaypoints(imPos,plot_waypoints);
% Load beacons of disinfection region boundaries
plot_beacons = false;
[hBeacons,regionBoundaryBeacons] = getBoundaryBeacons(imPos,plot_beacons);
% Load the regions to disinfect
[hDisinfection,xDisinfect,yDisinfect,xDisinfectMesh,yDisinfectMesh,CDisinfect,percentDisinfected] = getDisinfectionRegion(imPos);
% Plot Final Formation
hFinalFormation = gobjects(numAgents,1);
hFinalFormationLabel = gobjects(numAgents,1);
linpar = linspace(0,1);
for ii = 1:numAgents
    hFinalFormation(ii) = patch(...
        'XData',delta*cos(2*pi*linpar(2:end))+finalFormation(1,ii),...
        'YData',delta*sin(2*pi*linpar(2:end))+finalFormation(2,ii),...
        'LineWidth',2,'EdgeColor','k','FaceColor',0.75*[1,1,1]);
    hFinalFormationLabel(ii) = text(...
        finalFormation(1,ii),finalFormation(2,ii),num2str(ii),...
        'HorizontalAlignment','center');
end
% Plot connectivity edges
hEdges = plot(edgeCoordinatesX(:),edgeCoordinatesY(:),'LineWidth',2);
% Transform from cell to matrix form
obstacleDataMat = getObstacleDataMat(obstacleData);

% Visualize Voronoi cells
plot_tessellation = false;
hVor = gobjects(numAgents,1);
hVorColors = 0.7*hsv(numAgents);
for ii = 1:numAgents
    hVor(ii) = patch('XData',nan,...
        'YData',nan,...
        'LineWidth',2,'EdgeColor','k','FaceColor',hVorColors(ii,:),'FaceAlpha',0.5,'Visible','off');
end

if simulate_true
    % Make sure the robot patches are at the top of the graphic stack
    uistack(hRobots,'top')
end
%% Construct mission specific info
missionInfo = cell(numAgents,1);

switch currentSubmission
    case 'init'
        missionCounter = 1;
        currentWaypoint = 1;
        currentRegion = 1;
    case 'room' %cafe
        missionCounter = 2;
        currentWaypoint = 3; %size(navWaypoints,2)/2+1;
        currentRegion = 1;
    case 'return'
        missionCounter = 3;%navigate corridor
        currentWaypoint = 4;%size(navWaypoints,2)/2+1;
        currentRegion = 1;
    case 'refuel' %deliver
        missionCounter = 4;
        currentWaypoint = size(navWaypoints,2);
        currentRegion = 1;
end
% Store target waypoint
xt = navWaypoints(:,currentWaypoint);
% disp(xt);
% Store beacon locations
beacons = regionBoundaryBeacons{currentRegion};
% Get final formation specification
specGraph = squareform(pdist(finalFormation.'));
switch currentSubmission
    case 'init'
        missionInfo{end-1}.target = xt;
        missionInfo{end}.target = xt;
        set(hWaypoints(currentWaypoint),'Visible','on')
    case 'room'
        for ii = 1:numAgents
            missionInfo{ii}.domain = beacons;
        end
        set(hWaypoints(currentWaypoint),'Visible','off')
    case 'return'
        missionInfo{end-1}.target = xt;
        missionInfo{end}.target = xt;
        set(hWaypoints(currentWaypoint),'Visible','on')
        set(hDisinfection(currentRegion),'Visible','off')
    otherwise
        for ii = 1:numAgents
            missionInfo{ii}.specGraph = specGraph(ii,A(ii,:));
        end
end
% Get formation graph
A_formation = getFormationGraph(numAgents,currentSubmission,currentWaypoint);
% % Get random marker locations
% markers = getSearchMarkers([beacons(1,[1 2]),beacons(2,[2 3])],0.5*Delta,numAgents);
% activeMarkers = false(1,numAgents);
% % Plot the markers
% hMarkers = plot([markers(1,:);nan(size(markers(1,:)))],[markers(2,:);nan(size(markers(2,:)))],'p','MarkerSize',20,'MarkerFaceColor','y','Visible','off');
% % [hMarkers(:).Visible] = deal('off');
%% Main loop
% Number of iterations
iterations = 1e6;
persMem = zeros(numAgents);
dxu = zeros(2,numAgents);
scalingSpeed = 3;
ell = 0.5*delta;
missionTimer = tic;
if simulate_true
    dt = r.time_step;
else
    previousTime = toc(missionTimer);
    dt = previousTime;
end
for k = 1:iterations
    % Retrieve the most recent poses from the Robotarium.
    x = r.get_poses();
    robotPositions = x(1:2,:);
    robotHeadings = x(3,:);
    [vcells,ADelaunay] = boundedVoronoi(robotPositions,beacons);
    % Get a new time reading
    currentTime = toc(missionTimer);
    % Update Delta-disk graph topology
    [ADeltaDisk,pairwiseDist] = deltaDisk(robotPositions,Delta);
    % Get the sensor data for each robot along the 0,45,90,...,315 deg dir
    sensorData = rangeSensor(robotPositions,obstacleDataMat,delta,Delta);
    % Check for collisions
    collisionDetected = checkCollisions(robotPositions,obstacleDataMat,r.robot_diameter);
    if any(pairwiseDist<r.robot_diameter) || collisionDetected
        disp('Collision detected!')
        hText = text(0,0,'Collision Detected!','FontSize',28,'FontWeight','bold','HorizontalAlignment','center','Color',[1 0.25 0.25]);
        pause(5)
        break
    end
    % Get adjacency matrix information
    switch currentSubmission
        case 'room'
            A = ADeltaDisk;
        otherwise
            A = ADeltaDisk;
    end
    % Get the edges to be plotted
    edgeIndices = getEdgeIndices(A);
    edgeCoordinatesX = [robotPositions(1,edgeIndices(:,1));robotPositions(1,edgeIndices(:,2));nan(1,size(edgeIndices,1))];
    edgeCoordinatesY = [robotPositions(2,edgeIndices(:,1));robotPositions(2,edgeIndices(:,2));nan(1,size(edgeIndices,1))];
    set(hEdges,'XData',edgeCoordinatesX(:),'YData',edgeCoordinatesY(:))
    
    % Compute the control at the node-level
    u = zeros(2,numAgents);
    for ii = 1:numAgents
        % Reset mission information
        missionInfo{ii} = [];
        switch lower(currentSubmission)
            case {'init','return'}
                if ii==numAgents || ii==(numAgents-1)
                    missionInfo{ii}.target = xt;
                end
            case 'room'
                % Store beacon locations
                missionInfo{ii}.domain = beacons;
                missionInfo{ii}.AgentVCell = vcells{ii};
                missionInfo{ii}.NeighborVCells = vcells(A(ii,:)&ADelaunay(ii,:));
                missionInfo{ii}.DelaunayNeighbors = A(ii,:).*ADelaunay(ii,:);
                if plot_tessellation
                    set(hVor(ii), 'Visible','on')
                end
            case 'refuel'
                missionInfo{ii}.specGraph = specGraph;
                missionInfo{ii}.feasiblePoints = finalFormation-mean(finalFormation,2);
                missionInfo{ii}.target = mean(finalFormation,2);
        end
        % Store pertinent data
        missionData = struct(...
            'Submission',currentSubmission,...
            'SensorData',sensorData(:,ii),...
            'AgentState',robotPositions(:,ii),...
            'AgentID',ii,...
            'NeighborStates',robotPositions(:,A(ii,:)),...
            'NeighborIDs',IDs(A(ii,:)),...
            'MissionInfo',missionInfo{ii},...
            'MinSafetyDist',delta,...
            'MaxSensCommRng',Delta,...
            'FormationSpec',A_formation(ii,A(ii,:)));
        %%%%%%%%%%%%%%% Modify this controller %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [u(:,ii),persMem(ii,A(ii,:))] = controller(missionData,persMem(ii,A(ii,:)),currentTime);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    % Scale velocities up beyond the actuator bound (we'll scale down again
    % for safety later, but this allows the screen to scroll faster)
    scaledU = scalingSpeed*u;
    maxSpeed = max(vecnorm(scaledU));
    if maxSpeed > scalingSpeed*vmax
        scaledU = scalingSpeed*vmax*scaledU/maxSpeed;
        %vfield = vmax*vfield/maxSpeed*(1-maxSpeed/maxRelativeSpeed);
    end

    % Compute the vector field for scenary motion (using scaled velocity)
    vfield = getVelocityField(robotPositions,domBoundary,scaledU);
    % Center the camera on the centroid of the agents or domain
    switch currentSubmission
        case {'init','return'}
            agentCentroid = mean(robotPositions,2);
            vfield = vfield - vmax*agentCentroid/max(norm(agentCentroid),delta);
        case 'room'
            roomCenter = mean(beacons,2);
            vfield = vfield - 2*vmax*(roomCenter)/max(norm(roomCenter),vmax);
            %vfield = 1.75*vfield;
        case 'refuel'
            formationCentroid = mean(finalFormation,2);
            vfield = vfield - vmax*(formationCentroid)/max(norm(formationCentroid),vmax);
    end
    % Offload scenary transition from agent control (vfield points away
    % from boundary of screen proportional to how close you get)
    relativeU = scaledU + vfield;
    % Scale velocities again, to ensure actuators don't saturate
    maxRelativeSpeed = max(vecnorm(relativeU));
    if maxRelativeSpeed > vmax
        relativeU = vmax*relativeU/maxRelativeSpeed;
        % Scale vfield by same amount
        vfield = vmax*vfield/maxRelativeSpeed;
    end
    
    % Compute the unicycle control
    for ii = 1:numAgents
        theta_i = robotHeadings(ii);
        dxu(:,ii) = diag([1,1/ell])*[cos(theta_i),sin(theta_i);-sin(theta_i),cos(theta_i)]*relativeU(:,ii);
    end
    
    % Ensure actuator limits are met
    maxLinSpeed = max(abs(dxu(1,:)));
    if maxLinSpeed > vmax
        dxu = vmax*dxu/maxLinSpeed;
        vfield = vmax*vfield/maxLinSpeed;
    end
    maxAngSpeed = max(abs(dxu(2,:)));
    if maxAngSpeed > wmax
        dxu = wmax*dxu/maxAngSpeed;
        vfield = wmax*vfield/maxAngSpeed;
    end
    
    % Transition scenary and update graphics
    if ~simulate_true
        dt = currentTime - previousTime;
        previousTime = currentTime;
    end
    % Reshape data to transition scenery
    [XYDisinfectMesh,szeDisinfectMesh] = grid2vec(xDisinfectMesh,yDisinfectMesh);
    [XYDisinfect,szeDisinfect] = grid2vec(xDisinfectMesh,yDisinfectMesh);
    % Transition scenery
    [navWaypoints,imPos,regionBoundaryBeacons{:},beacons,finalFormation,obstacleData{:},XYDisinfectMesh{:},XYDisinfect{:}] = ...
        transitionScenery(r.time_step,vfield,navWaypoints,imPos,regionBoundaryBeacons{:},beacons,finalFormation,obstacleData{:},XYDisinfectMesh{:},XYDisinfect{:});
    xt = navWaypoints(:,currentWaypoint);
    %specGraph = squareform(pdist(finalFormation.'));
    % Reshape data to update graphic handles
    [xDisinfect,yDisinfect] = vec2grid(XYDisinfect,szeDisinfect);
    [xDisinfectMesh,yDisinfectMesh] = vec2grid(XYDisinfectMesh,szeDisinfectMesh);
    obstacleDataMat = getObstacleDataMat(obstacleData);
    for ii = 1:numel(hDisinfection)
        set(hDisinfection(ii),'XData',xDisinfectMesh{ii},'YData',yDisinfectMesh{ii})
    end
    % Get segment of image within robotarium bounds
    imXinBounds = ((imX+imPos(1))>=domBoundary(1))&((imX+imPos(1))<=domBoundary(2));
    imYinBounds = ((imY+imPos(2))>=domBoundary(3))&((imY+imPos(2))<=domBoundary(4));
    set(hIm,'XData',imX(imXinBounds)+imPos(1),'YData',imY(imYinBounds)+imPos(2),'CData',imCData(imYinBounds,imXinBounds,:));%,'CDataMapping', 'scaled');
    set(hWaypoints(currentWaypoint),'XData',[robotPositions(1,end) xt(1)] ,'YData',[robotPositions(2,end) xt(2)]);
    if plot_waypoints
        for ii = 1:size(navWaypoints,2)
            set(hWaypoints(ii,2),'XData',navWaypoints(1,ii),'YData',navWaypoints(2,ii))
        end
    end
    if plot_beacons
        set(hBeacons,'XData',beacons(1,[1:end 1]),'YData',beacons(2,[1:end 1]));
    end
    if plot_tessellation
        for ii = 1:numAgents
            set(hVor(ii), 'XData',vcells{ii}(1,:), 'YData',vcells{ii}(2,:))
        end
    end
    for ii = 1:numAgents
        set(hFinalFormation(ii),...
            'XData',delta*cos(2*pi*linpar(2:end))+finalFormation(1,ii),...
            'YData',delta*sin(2*pi*linpar(2:end))+finalFormation(2,ii));
        set(hFinalFormationLabel(ii),'Position',finalFormation(:,ii));
    end
    if ~isempty(hObs)
        for kk = 1:length(hObs)
            set(hObs(kk),'XData',obstacleData{kk}(1,:),'YData',obstacleData{kk}(2,:));
        end
    end
    % Check if submission is accomplished
    switch currentSubmission
        case {'init','return'}
            if any(vecnorm(robotPositions(:,[end-1 end])-xt,2,1)<delta)
                if currentWaypoint < 3%size(navWaypoints,2)/2
                    disp(['Succesfully reached waypoint ',num2str(currentWaypoint),'!'])
                    set(hWaypoints(currentWaypoint),'Visible','off')
                    currentWaypoint = currentWaypoint + 1;
                    set(hWaypoints(currentWaypoint),'Visible','on')
                    A_formation = getFormationGraph(numAgents,currentSubmission,currentWaypoint);
                    if currentWaypoint == 3%size(navWaypoints,2)/2
                        displayCheckPointMessage = true;
                    end
                else
                    if currentWaypoint == 3%size(navWaypoints,2)/2
                        if displayCheckPointMessage
                            disp(['Succesfully reached waypoint ',num2str(currentWaypoint),'!'])
                            displayCheckPointMessage = false;
                        end
                        if all(inpolygon(robotPositions(1,:),robotPositions(2,:),0.95*(beacons(1,:)-mean(beacons(1,:)))+mean(beacons(1,:)),0.95*(beacons(2,:)-mean(beacons(2,:)))+mean(beacons(2,:))))
                            disp('All agents inside disinfection region!')
                            if strcmp(missionChoice,'full')
                                set(hWaypoints(currentWaypoint),'Visible','off')
                                missionCounter = missionCounter + 1;
                                currentSubmission = missionList{missionCounter};
                                currentWaypoint = currentWaypoint + 1;
                                A_formation = getFormationGraph(numAgents,currentSubmission,currentWaypoint);
                                
                            else
                                for ii = 1:numAgents
                                    set(hVor(ii), 'Visible', 'off')
                                end
                                prompt = 'Save robot configuration for Room submission? [Y/N]: ';
                                userReply = input(prompt,'s');
                                if strcmpi(userReply,'y')
                                    if isfile('roomSaveData.mat')
                                        warning('Save data exists for Room.')
                                        prompt = 'Overwrite saved data? [Y/N]: ';
                                        userReply = input(prompt,'s');
                                        if strcmpi(userReply,'y')
                                            save('roomSaveData.mat','x','imPos')
                                        end
                                    else
                                        save('roomSaveData.mat','x','imPos')
                                    end
                                end
                                break
                            end
                        end
                    else
                        if currentWaypoint < size(navWaypoints,2)
                            set(hWaypoints(currentWaypoint),'Visible','off')
                            currentWaypoint = currentWaypoint + 1;
                            set(hWaypoints(currentWaypoint),'Visible','on')
                            A_formation = getFormationGraph(numAgents,currentSubmission,currentWaypoint);
                            if currentWaypoint == size(navWaypoints,2)
                                displayCheckPointMessage = true;
                            end
                        else
                            if displayCheckPointMessage
                                disp(['Succesfully reached waypoint ',num2str(currentWaypoint),'!'])
                                displayCheckPointMessage = false;
                            end
                            if all(vecnorm(robotPositions-xt)<0.9239)
                                disp('All agents on refuel platform!')
                                if strcmp(missionChoice,'full')
                                    set(hWaypoints(currentWaypoint),'Visible','off')
                                    missionCounter = missionCounter + 1;
                                    currentSubmission = missionList{missionCounter};
                                    A_formation = getFormationGraph(numAgents,currentSubmission,currentWaypoint);
                                else
                                    prompt = 'Save robot configuration for Refuel submission? [Y/N]: ';
                                    userReply = input(prompt,'s');
                                    if strcmpi(userReply,'y')
                                        if isfile('refuelSaveData.mat')
                                            warning('Save data exists for Refuel.')
                                            prompt = 'Overwrite saved data? [Y/N]: ';
                                            userReply = input(prompt,'s');
                                            if strcmpi(userReply,'y')
                                                save('refuelSaveData.mat','x','imPos')
                                            end
                                        else
                                            save('refuelSaveData.mat','x','imPos')
                                        end
                                    end
                                    break
                                end
                            end
                        end
                    end
                end
            end
        case 'room'
            thresholdDist = delta;
            visitedDisinfectionCells = any(((xDisinfect{currentRegion}(:)-robotPositions(1,:)).^2+(yDisinfect{currentRegion}(:)-robotPositions(2,:)).^2) < thresholdDist^2,2);
            maskMat = ones(size(xDisinfect{currentRegion}));
            maskMat(visitedDisinfectionCells) = nan;
            CDisinfect{currentRegion} = bsxfun(@times,CDisinfect{currentRegion},maskMat(1:end-1,1:end-1));
            set(hDisinfection(currentRegion),'CData',[CDisinfect{currentRegion},nan(size(CDisinfect{currentRegion},1),1,3);nan(1,size(CDisinfect{currentRegion},2)+1,3)])
            % Check if threshold for disinfeciton is met
            percentageDisinfected = nnz(isnan(CDisinfect{currentRegion}(:,:,1)))/numel(CDisinfect{currentRegion}(:,:,1));
            if percentageDisinfected > 0.85
                disp('Succesfully disinfected majority of room region!')
                if strcmp(missionChoice,'full')
                    set(hWaypoints(currentWaypoint),'Visible','on')
                    %set(hDisinfection(currentRegion),'Visible','off')
                    currentRegion = min(currentRegion+1,1);
                    set(hDisinfection(currentRegion),'Visible','on')
                    missionCounter = missionCounter + 1;
                    currentSubmission = missionList{missionCounter};
                    A_formation = getFormationGraph(numAgents,currentSubmission,currentWaypoint);
                else
                    prompt = 'Save robot configuration for Return submission? [Y/N]: ';
                    userReply = input(prompt,'s');
                    if strcmpi(userReply,'y')
                        if isfile('returnSaveData.mat')
                            warning('Save data exists for Return.')
                            prompt = 'Overwrite saved data? [Y/N]: ';
                            userReply = input(prompt,'s');
                            if strcmpi(userReply,'y')
                                save('returnSaveData.mat','x','imPos')
                            end
                        else
                            save('returnSaveData.mat','x','imPos')
                        end
                    end
                    break
                end
            end
        case 'refuel'
            finalFormationMean = mean(finalFormation,2);
            currentRobotMean = mean(robotPositions,2);
            currentRobotFormation = squareform(pdist(robotPositions.'));
            % Sort to remove permutations
            sortedRobotFormation = sort(currentRobotFormation,2);
            sortedRobotFormation = sort(sortedRobotFormation,1);
            sortedFinalFormation = sort(specGraph,2);
            sortedFinalFormation = sort(sortedFinalFormation,2);
            if all(abs(sortedRobotFormation(:)-sortedFinalFormation(:))<0.25*delta) && norm(finalFormationMean-currentRobotMean)<0.25*delta
                hText = text(0,0,'Mission Success!','FontSize',28,'FontWeight','bold','HorizontalAlignment','center','Color',[0.25 1 0.5]);
                disp('Succesfully got in formation!')
                pause(5)
                break
            end
    end
    %% Send velocities to agents
    % Set velocities of agents 1,...,N
    r.set_velocities(1:numAgents, dxu);
    % Send the velocities to the agents and update the back-end.
    r.step();
end
% Checks that the simulation passes the safety tests!
r.debug();
% Store data if not a simulation
if ~simulate_true
    file_name = unique_filename('ENME808T_FinalProject');
    save(file_name);
end
%% Auxiliary Functions
% Obtain delta-disk graph adjacency information
function [A,pairwiseDist] = deltaDisk(robotStates,Delta)
pairwiseDist = pdist(robotStates.');
A = squareform(pairwiseDist)<Delta;
A = (A - diag(diag(A)))~=0;
end
% Get edge indices for plotting
function [edgeIndices] = getEdgeIndices(A)
numAgents = size(A,1);
numEdges = sum(A(:))/2;
edgeIndices = zeros(numEdges,2);
edgeCounter = 1;
for ii = 1:numAgents-1
    for jj = ii+1:numAgents
        if A(ii,jj) == 1
            edgeIndices(edgeCounter,1) = ii;
            edgeIndices(edgeCounter,2) = jj;
            edgeCounter = edgeCounter + 1;
        end
    end
end
end
% Choose between hardcoded scenarios
function [numAgents,initPoses,finalFormation,Delta,imPos] = loadMission(missionChoice)
numAgents=6;
% Connectivity distance
Delta = 1.25;
radForm = 0.95*Delta/2;
linpar = linspace(0,1,numAgents+1);
finalFormation(1,:) = radForm*cos(2*pi*linpar(2:end)+2/3*pi);
finalFormation(2,:) = radForm*sin(2*pi*linpar(2:end)+2/3*pi);
initPoses = zeros(3,numAgents);
switch missionChoice
    case 'init'
        initPoses(1:2,:) = finalFormation;
        initPoses(3,:) = 2*pi*linpar(2:end)-1*pi/3;
        imPos = zeros(2,1);
    case 'room'
        if isfile('roomSaveData.mat')
            load('roomSaveData.mat','x','imPos')
            initPoses = x;
            imPos = imPos+[4.97;-2.38];
        else
            initPoses(1,:) = [-1.0084,-0.7666,-0.4666,-0.4363,-0.6806,-0.9431];
            initPoses(2,:) = [-0.1588,-0.5214,-0.7734,-0.4279,-0.1620, 0.3441];
            initPoses(3,:) = [ 1.0324, 3.1000,-2.4952,-1.5358,-0.3991, 1.2034];
            imPos = [8.3919;0.4144];
        end
    case 'return'
        % if isfile('returnSaveData.mat')
        if isfile('returnSaveData.mat')
            load('returnSaveData.mat','x','imPos')
            initPoses = x;
            imPos = imPos+[4.97;-2.38];
        else
            initPoses(1,:) = [-0.7444   -0.3824    0.2190    0.6555   -0.1911   -0.5873];
            initPoses(2,:) = [0.3432   -0.2135   -0.1452   -0.1559    0.5685    0.7347];
            initPoses(3,:) = [2.1877   -1.6631   -1.5546   -1.5782    1.4825    1.8585];
            imPos = [9.3414;-0.5769];
        end
    case 'refuel'
        if isfile('refuelSaveData.mat')
            load('refuelSaveData.mat','x','imPos')
            initPoses = x;
            imPos = imPos+[4.97;-2.38];
        else
            initPoses(1,:) = [0.9153    0.4958    0.1476    0.4965    0.8816    1.4200];
            initPoses(2,:) = [0.2477   -0.0548   -0.3158   -0.3917   -0.0770    0.4257];
            initPoses(3,:) = [1.4977    1.7243   -0.3738    1.2280    1.7462    1.6535];
            imPos = [1.5332;0.4997];
        end
    otherwise
        error('Mission Choices are ''init'', ''room'', ''return'', ''refuel'', or ''full''')
end
finalFormation = finalFormation + imPos;
end

% Transition between scenarios
function [vfield] = getVelocityField(robotPositions,domBoundary,u)
% Find the robot nearest the x and y boundaries
domWidth = diff(domBoundary(1:2));
domHeight = diff(domBoundary(3:4));
closestDist2XBoundary = 0.1*domHeight;
closestDist2YBoundary = 0.1*domHeight;
targetDist2Boundary = 0.2*domHeight;
[distLowerX,lowerXAgent] = min(robotPositions(1,:)-domBoundary(1)-closestDist2XBoundary);
[distUpperX,upperXAgent] = min(domBoundary(2)-robotPositions(1,:)-closestDist2XBoundary);
[distLowerY,lowerYAgent] = min(robotPositions(2,:)-domBoundary(3)-closestDist2YBoundary);
[distUpperY,upperYAgent] = min(domBoundary(4)-robotPositions(2,:)-closestDist2YBoundary);
vfield = zeros(2,1);
if distLowerX < targetDist2Boundary
    vfield(1) = vfield(1) - min(u(1,lowerXAgent),0)*max((targetDist2Boundary-distLowerX)/targetDist2Boundary,0);
end
if distUpperX < targetDist2Boundary
    vfield(1) = vfield(1) - max(u(1,upperXAgent),0)*max((targetDist2Boundary-distUpperX)/targetDist2Boundary,0);
end
if distLowerY < targetDist2Boundary
    vfield(2) = vfield(2) - min(u(2,lowerYAgent),0)*max((targetDist2Boundary-distLowerY)/targetDist2Boundary,0);
end
if distUpperY < targetDist2Boundary
    vfield(2) = vfield(2) - max(u(2,upperYAgent),0)*max((targetDist2Boundary-distUpperY)/targetDist2Boundary,0);
end
%[maxDist2XBoundary,xBoundarySign] = max([distLowerX,distUpperX]);
%[maxDist2YBoundary,yBoundarySign] = max([distLowerY,distUpperY]);
% boundaryDesDist = 0.05*diff(domBoundary(1:2))/2;
% boundaryGain = maxSpeed/sqrt(2*pi);
% vfield = zeros(2,1);
% vfield(1) = max(boundaryGain/boundaryDesDist^3*[-(distUpperX).*exp(-0.5*((distUpperX)/boundaryDesDist).^2),(distLowerX).*exp(-0.5*((distLowerX)/boundaryDesDist).^2)]);
% vfield(2) = max(boundaryGain/boundaryDesDist^3*[-(distUpperY).*exp(-0.5*((distUpperY)/boundaryDesDist).^2),(distLowerY).*exp(-0.5*((distLowerY)/boundaryDesDist).^2)]);
% [~,farthestRobot] = max(vecnorm(robotPositions));
% vfield = -robotPositions(:,farthestRobot)/domBoundary(4);
% vfield = maxSpeed/(2.5*domBoundary(2)-norm(vfield))^6*vfield/norm(vfield);
end

function [hIm,imX,imY,imPos,imCData] = loadBackgroundImage(imName,domBoundary)
backgroundIm = imread(imName);
backgroundIm = flipud(backgroundIm);
imSize = size(backgroundIm);
imScaleRatio = 4;
xTrim = mod(imSize(1),imScaleRatio);
yTrim = mod(imSize(2),imScaleRatio);
backgroundIm = backgroundIm((1:imSize(1)-round(xTrim/2))+ceil(xTrim/2),(1:imSize(2)-round(yTrim/2))+ceil(yTrim/2),:);
imSize = size(backgroundIm);
imX = linspace(-imScaleRatio*diff(domBoundary(1:2))/2,imScaleRatio*diff(domBoundary(1:2))/2,imSize(2));
imY = linspace(-imScaleRatio*diff(domBoundary(3:4))/2,imScaleRatio*diff(domBoundary(3:4))/2,imSize(1));
imPos = -[4.97;-2.38];%[imX(1)+diff(imX([1 end]))*1.5/10;imY(1)+diff(imY([1 end]))*0.5/20];
% imIndx = 1:imSize(1);%/imScaleRatio;
% hIm = image(imX(imIndx)+imPos(1),imY(imIndx)+imPos(2),backgroundIm,'CDataMapping', 'scaled');
hIm = image(imX+imPos(1),imY+imPos(2),backgroundIm,'CDataMapping', 'scaled');
pixels2skip = 3;
imX = imX(1:pixels2skip:end);
imY = imY(1:pixels2skip:end);
imCData=backgroundIm(1:pixels2skip:end,1:pixels2skip:end,:);
uistack(hIm,'bottom')
end
% Returns list of navigation waypoints
function [hTar,xt] = getNavigationWaypoints(imPos,varargin)
if nargin == 2
    plot_waypoints = varargin{1};
else
    plot_waypoints = false;
end
shiftInPos = imPos(:);% + 0.64*[1;1];
% Define the waypoint locations
%{ 
imPosWaypoints = [...
      3.40340, -4.95518, -4.3713, -3.41742,  3.40340,  4.97000;
     -3.61464, -3.61464, -2.3160, -3.61464, -3.61464, -2.38000];
%}
%{
imPosWaypoints = [...
    4.7700, 2.3500, -2.3500, -4.7700, -4.3713, -4.9552, -3.4174,       0,  3.4034,  4.9700;
    0.1200, 0.1800, -0.1800, -0.1200, -2.3160, -3.6146, -3.6146, -3.1500, -3.6146, -2.3800];
%}
%{
 imPosWaypoints = [3.40340  -4.95518  -4.95518  -4.95518  -3.66466   4.2077   4.97000   4.97000;
     -3.61464  -3.61464  -0.70520   1.44509   3.54528   3.3102   0.71869  -2.38000];
%}
 %{
   [3.40340  -4.95518  -4.95518  -4.95518  -4.95518  -3.66466   3.44548   4.97000   4.97000   4.97000;
  -3.61464  -3.61464  -2.20231  -0.70520   1.44509   3.54528   3.54528   3.07514   0.71869  -2.38000];
 %}

%{
imPosWaypoints = [...
   4.7700, 4.8000, 4.9, 2.80, 0, 0, 4.800,4.97;
    0.1200, 1.000,  3.0, 3.75, 3.12, 0, 0,-2.38];  %these work
%}
imPosWaypoints = [...
    4.7700, 4.8000, 4.9, 2.80, -0.2, -0.2, 4.9700,4.97;
    0.1200, 1.000,  3.25, 3.75, 3.12, 0.0, 0.0,-2.38];
%[4.7700, 4.8000, 4.9, 2.80, -0.2, -0.2, 4.9700,4.97;0.1200, 1.000,  3.0, 3.75, 3.12, 0.0, 0.0,-2.38];

xt = imPosWaypoints + shiftInPos;
if plot_waypoints
    hTar = gobjects(size(xt,2),2);
else
    hTar = gobjects(size(xt,2),1);
end
for ii = 1:size(xt,2)
    if plot_waypoints
        hTar(ii,2) = plot(xt(1,ii).',xt(2,ii).','md','LineWidth',2);
    end
    hTar(ii,1) = plot(xt(1,ii).',xt(2,ii).','--og','LineWidth',2,'Visible','off');
end
end
% Returns the boundaries of the search areas
function [hBeacons,beacons] = getBoundaryBeacons(imPos,plot_beacons)
shiftInPos = imPos(:);% + 0.64*[1;1];
% Define the beacons
numBeacons = 1;
beacons = cell(numBeacons,1);
% Bedrooms
%{
beacons{1} = [...
    -3.41742  -5.3251  -5.3251  -3.41742;
    -3.1137   -3.1137   -1.5183   -1.5183];
%}
% beacons{1} = [...
%     -3.41742  -5.3251  -5.3251  -3.41742;
%     -3.91137  -3.91137  -0.72062  -0.72062];
% beacons{1} = [...
%     -3.41742  -6.31934  -6.31934  -3.41742;
%     -3.91137  -3.91137  -0.72062  -0.72062];
% % Nursing Station
% beacons{2} = [...
%     -3.41742  -6.31934  -6.31934  -3.41742;
%     3.90944   3.90944   0.71869   0.71869];
% % Cafe
%{
beacons{1} = [...
    6.31759   3.41567   3.41567   6.31759;
     3.90944   3.90944   0.71869   0.71869];
%}
%{
beacons{1} = [5.8   4.01567   4.01567   5.8;
     3.30944   3.30944   2.31869   2.31869];
%}
beacons{1} = [5.8   4.01567   4.01567   5.8;
     3.30944   3.30944   2.41869   2.41869];

hBeacons = [];

if plot_beacons
    hBeacons = gobjects(numBeacons,1);
    hTestObs = gobjects(1,1);
end
for ii = 1:numBeacons
    for jj = 1:2
        beacons{ii}(jj,:) = beacons{ii}(jj,:) + shiftInPos(jj);
    end
    if plot_beacons
        hBeacons(ii) = plot(beacons{ii}(1,[1:end 1]),beacons{ii}(2,[1:end 1]),'-h','MarkerFaceColor',[0.8500 0.3250 0.0980],'MarkerEdgeColor',[0.8500 0.3250 0.0980],'MarkerSize',20,'LineWidth',2);%,'Visible','off');
    end
end


end
% Return hardcoded obstacle data
function [obstacleData,varargout] = getObstacleData(varargin)
numObstacles = 7;
obstacleData = cell(numObstacles,1);
% Outer Boundary
% obstacleData{1} = [...
%     5.90378   6.31759   6.31759   5.30411   5.30411   5.16033   5.30411   5.30411   5.91956   5.91956   4.27660   4.06269   0.32964   0.32964  -0.32789  -0.52953  -3.78740  -3.78740  -6.04230  -6.15978  -6.15978  -6.31934  -6.31934  -5.32515  -5.32515  -5.16734  -5.32515  -5.32515  -5.78104  -5.77929  -5.32515  -5.32515  -0.40504  -0.40504   0.27003   0.46992   4.06093   4.06093   5.90904   6.40000  6.40000  -6.40000  -6.40000   6.40000   5.90904;
%     -1.43931  -1.43931  -0.72062  -0.72062  -0.17919   0.02119   0.21965   0.71869   0.71869   3.48362   3.48362   3.90944   3.90944   3.57996   3.57996   3.90944   3.90944   3.72447   3.72447   3.61272   1.13680   1.13680   0.71869   0.71869   0.21965   0.02119  -0.15992  -1.80539  -1.80539  -2.83044  -2.83044  -3.91522  -3.91522  -3.57996  -3.57996  -3.91522  -3.91522  -3.41040  -3.41040  -4.00000  4.00000   4.00000  -4.00000  -4.00000  -3.41040];
obstacleData{1} = [...
    5.90378   6.31759   6.31759   5.30411   5.30411   5.16033   5.30411   5.30411   5.91956   5.91956   4.27660   4.06269   0.32964   0.32964  -0.32789  -0.52953  -3.78740  -3.78740  -6.04230  -6.15978  -6.15978  -6.31934  -6.31934  -5.32515  -5.32515  -5.16734  -5.32515  -5.32515  -5.7802  -5.32515  -5.32515  -0.40504  -0.40504   0.27003   0.46992   4.06093   4.06093   5.90904   6.40000  6.40000  -6.40000  -6.40000   6.40000   5.90904;
    -1.43931  -1.43931  -0.72062  -0.72062  -0.17919   0.02119   0.21965   0.71869   0.71869   3.48362   3.48362   3.90944   3.90944   3.57996   3.57996   3.90944   3.90944   3.72447   3.72447   3.61272   1.13680   1.13680   0.71869   0.71869   0.21965   0.02119  -0.15992  -1.80539  -2.3179  -2.83044  -3.91522  -3.91522  -3.57996  -3.57996  -3.91522  -3.91522  -3.41040  -3.41040  -4.00000  4.00000   4.00000  -4.00000  -4.00000  -3.41040];
% Inner Boundary 1
obstacleData{2} = [...
    3.41567   4.42915   4.42915   2.70378   2.70378   2.04800   1.82005   1.02400   0.32088   0.32088   0.65753   3.41567;
    -0.72062  -0.72062  -0.37572  -0.37572  -0.05780  -0.05780  -0.37572  -0.37572  -1.10983  -2.88054  -3.10212  -3.10212];
% Inner Boundary 2
obstacleData{3} = [...
    -0.32789  -0.32789  -0.65900  -1.02575  -4.43090  -4.43090  -3.41742  -3.41742  -0.65929;
    -2.88054  -1.10983  -0.92900  -0.37572  -0.37572  -0.72062  -0.72062  -3.10212  -3.10212];
% Inner Boundary 3
obstacleData{4} = [...
    -0.32789  -0.32789  -0.65578  -1.74115  -1.74115  -2.39868  -2.63014  -3.41742  -3.41742  -4.43090  -4.43090  -2.65118  -2.65118  -1.99540  -1.76044  -1.02575;
    1.15222   2.92293   3.10019   3.10019   3.44316   3.44316   3.10019   3.10019   0.71869   0.71869   0.41619   0.41619   0.08863   0.08863   0.41619   0.41619];
% Inner Boundary 4
obstacleData{5} = [...
    4.42915   3.41567   3.41567   2.56175   2.50564   1.59036   1.53600   0.66104   0.32964   0.32964   0.66104   1.02225   4.42915;
    0.71869   0.71869   3.10019   3.10019   3.43931   3.43931   3.10019   3.10019   2.92293   1.15222   0.96724   0.42197   0.41619];
% Circular Desk
deskCenter = [-4.9148;2.3622];
deskWidth  = 1.0836;
deskHeight = 1.1908;
linpar = linspace(0,1);
obstacleData{6} = [...
    deskCenter(1) + deskWidth/2*cos(2*pi*linpar(1:end-1));
    deskCenter(2) + deskHeight/2*sin(2*pi*linpar(1:end-1))];
% Rectangular Table
%{
obstacleData{7} = [...
    4.87627   4.20121   4.20471   4.87627;
    1.39692   1.39692   2.75723   2.75723];
%}
obstacleData{7} =[4.2205  3.5407 3.5407  4.2205;
     2.20944   2.20944   0.8035  0.8035];

if nargin>0
    imPos = varargin{1};
    if nargin>1
        plot_obstacles = varargin{2};
    else
        plot_obstacles= false;
    end
else
    plot_obstacles = false;
    imPos = zeros(2,1);
end
shiftInPos = imPos(:);% + 0.64*[1;1];
hObstacles = [];
if plot_obstacles
    hObstacles = gobjects(numObstacles,1);
end
for ii = 1:numObstacles
    obstacleData{ii}(1,:) = obstacleData{ii}(1,:) + shiftInPos(1);
    obstacleData{ii}(2,:) = obstacleData{ii}(2,:) + shiftInPos(2);
    if plot_obstacles
        hObstacles(ii) = patch('XData',obstacleData{ii}(1,:),'YData',obstacleData{ii}(2,:),'FaceColor','r','EdgeColor','r','FaceAlpha',0.333,'LineWidth',2);
    end
end
if nargout>1
    varargout{1} = hObstacles;
end
end
% Get obstacle data in stacked matrix form separated by nan's
function obstacleDataMat = getObstacleDataMat(obstacleData)
numObstacles = length(obstacleData);
obstacleDataMat = [obstacleData{1}.';obstacleData{1}(:,1).'];
for kk = 2:numObstacles
    obstacleDataMat = [obstacleDataMat;nan(1,2);obstacleData{kk}.';obstacleData{kk}(:,1).'];
end
end
% Update the input arguments position given a velocity field
function [varargout] = transitionScenery(dt,v,varargin)
assert((nargin-2)==nargout,'Outputs must match extra inputs.')
varargout = cell(nargout,1);
for ii = 1:nargout
    varargout{ii} = varargin{ii} + dt*v;
end
end
% Get range sensor information
function [sensorData] = rangeSensor(robotPositions,obstacleData,delta,Delta)
numRobots = size(robotPositions,2);
numSensors = 8;
minSenRng = 0*delta;
sensorData  = Inf(numSensors,numRobots);
sensorDir = 0:2*pi/(numSensors):2*pi*(numSensors-1)/numSensors;
sensorRaysX = [minSenRng*cos(sensorDir);Delta*cos(sensorDir)];
sensorRaysY = [minSenRng*sin(sensorDir);Delta*sin(sensorDir)];
% robotTheta = 0:2*pi/50:2*pi;
% robotBody = 0.5*delta*[cos(robotTheta);sin(robotTheta)];
% Construct neighboring robots as stacked polygons separated by nan's
% robotPolyX = [robotBody(1,:).'+robotPositions(1,:);nan(1,numRobots)];
% robotPolyY = [robotBody(2,:).'+robotPositions(2,:);nan(1,numRobots)];
for ii = 1:numRobots
    for kk = 1:numSensors
        % Construct the current sensor ray
        sensorRay = [sensorRaysX(:,kk)+robotPositions(1,ii),sensorRaysY(:,kk)+robotPositions(2,ii)];
        % Get neighbor agent polygons
        % neighborsPolyX = robotPolyX(:,(1:numRobots)~=ii);
        % objectPolyX = [neighborsPolyX(:);obstacleData(:,1)];
        objectPolyX = obstacleData(:,1);
        % neighborsPolyY = robotPolyY(:,(1:numRobots)~=ii);
        % objectPolyY = [neighborsPolyY(:);obstacleData(:,2)];
        objectPolyY = obstacleData(:,2);
        % Check for intersections
        [tempX,tempY] = ...
            polyxpoly(sensorRay(:,1),sensorRay(:,2),objectPolyX,objectPolyY);
        % If there are any intersections, select the closest to the robot
        if ~isempty(tempX) && ~isempty(tempY)
            sensorData(kk,ii) = min(vecnorm([tempX.'-robotPositions(1,ii);tempY.'-robotPositions(2,ii)]));
        end
    end
end
end
% Check for collisions
function [collisionDetected] = checkCollisions(robotPositions,obstacleData,delta)
collisionDetected = false;
numRobots = size(robotPositions,2);
robotTheta = 0:2*pi/50:2*pi;
robotBody = 0.5*delta*[cos(robotTheta);sin(robotTheta)];
% Construct neighboring robots as stacked polygons separated by nan's
robotPolyX = [robotBody(1,:).'+robotPositions(1,:);nan(1,numRobots)];
robotPolyY = [robotBody(2,:).'+robotPositions(2,:);nan(1,numRobots)];
[tempX,tempY] = polyxpoly(robotPolyX,robotPolyY,obstacleData(:,1),obstacleData(:,2));
% If there are any intersections, collision detected
if ~isempty(tempX) && ~isempty(tempY)
    collisionDetected = true;
end
end
% Get bounded Voronoi tessellation
function [vcells,ADelaunay] = boundedVoronoi(generatorSeeds,domBounds)
numSeeds = size(generatorSeeds,2);
% Find the domain boundary hyperplanes tangents
t = zeros(size(domBounds));
numBoundaries = size(t,2);
midPts = zeros(size(domBounds));
% Remaining boundaries
for ii = 1:numBoundaries-1
    t(:,ii) = diff(domBounds(:,[ii ii+1]),1,2);
    midPts(:,ii) = mean(domBounds(:,[ii ii+1]),2);
end
% Last boundary
midPts(:,end) = mean(domBounds(:,[end 1]),2);
t(:,end) = diff(domBounds(:,[end 1]),1,2);
% Normalize the tangents
t = t ./ vecnorm(t,2,1);
% Compute the normal vectors
n = [0,1;-1,0]*t;
% Allocate padded seed matrix
paddedSeeds = zeros(2,numSeeds*(numBoundaries+1));
% Interior points
paddedSeeds(:,1:numSeeds) = generatorSeeds;
% Reflected points
for ii = 1:numBoundaries
    vecSeeds2Boundaries = bsxfun(@minus,generatorSeeds,midPts(:,ii));
    paddedSeeds(:,(1:numSeeds)+ii*numSeeds) = generatorSeeds-2*n(:,ii)*(n(:,ii).'*vecSeeds2Boundaries);
end
% Find the tessellation using domain interior and exterior points
[VVert,VIndx] = voronoin(paddedSeeds.');
% Keep only the cells for the interior (real) agents
vcells = cell(numSeeds,1);
for ii = 1:numSeeds
    vcells{ii} = [VVert(VIndx{ii},1),VVert(VIndx{ii},2)].';
end
% Get Delaunay neighbors
dTri = delaunayTriangulation(paddedSeeds.');
ADelaunay = zeros(numSeeds);
for ii = 1:size(dTri.ConnectivityList,1)
    % No neighbors outside domain
    if all(dTri.ConnectivityList(ii,[1 2]) <= numSeeds)
        ADelaunay(dTri.ConnectivityList(ii,1),dTri.ConnectivityList(ii,2)) = 1;
    end
    if all(dTri.ConnectivityList(ii,[1 3]) <= numSeeds)
        ADelaunay(dTri.ConnectivityList(ii,1),dTri.ConnectivityList(ii,3)) = 1;
    end
    if all(dTri.ConnectivityList(ii,[2 3]) <= numSeeds)
        ADelaunay(dTri.ConnectivityList(ii,2),dTri.ConnectivityList(ii,3)) = 1;
    end
end
ADelaunay = (ADelaunay + ADelaunay.')~=0;
end
% Generate the mesh of cells to disinfect
function [hDisinfection,xDisinfect,yDisinfect,xDisinfectMesh,yDisinfectMesh,CDisinfect,percentDisinfected] = getDisinfectionRegion(imPos)
numRegions = 1;
infectionColor = [116,124,49]/255;
hDisinfection = gobjects(numRegions,1);
xDisinfectMesh = cell(numRegions,1);
yDisinfectMesh = cell(numRegions,1);
xDisinfect = cell(numRegions,1);
yDisinfect = cell(numRegions,1);
CDisinfect = cell(numRegions,1);
percentDisinfected = zeros(numRegions,1);
% regionXLim = [...
%     -5.781;
%     -3.4174];
% regionYLim = [...
%     -3.9114;
%     -0.7206];
% regionXLim = [...
%     -5.781,-6.3193,3.4157;
%     -3.4174,-3.4174,5.9196];
% regionYLim = [...
%     -3.9114,0.7187,0.7187;
%     -0.7206,3.9094,3.9094];
numVerticalTiles = 7;%15;%30;%27;
numHorizontalTiles = 14;%round(diff(regionXLim,1,1)./diff(regionYLim,1,1)*numVerticalTiles);
% numHorizontalTiles = [20,25,21];
disinfectionRegion = cell(numRegions,1);
% Bedroom
%{
disinfectionRegion{1} = [...
    -5.140  -3.632  -3.632  -5.140;
    -3.0075   -3.0075   -1.6245   -1.6245];

%}

% disinfectionRegion{1} = [...
%     -5.140  -3.632  -3.632  -5.140;
%     -3.699  -3.699  -0.933  -0.933];
% disinfectionRegion{1} = [...
%     -5.32515  -3.41742  -3.41742  -5.32515;
%     -0.72062  -0.72062  -3.91137  -3.91137];
% disinfectionRegion{1} = [...
%     -5.32515  -5.77929  -5.78104  -5.32515  -5.32515  -3.41742  -3.41742  -5.32515;
%     -2.83044  -2.83044  -1.80539  -1.80539  -0.72062  -0.72062  -3.91137  -3.91137];
% % Nurse Station
% deskCenter = [-4.9148;2.3622];
% deskWidth  = 1.0836;
% deskHeight = 1.1908;
% linpar = linspace(0,1);
% CircularDesk = [...
%     deskCenter(1) + deskWidth/2*cos(2*pi*linpar(1:end-1));
%     deskCenter(2) + deskHeight/2*sin(2*pi*linpar(1:end-1))];
% disinfectionRegion{2} = [...
%     -6.15978  -6.15978  -6.04230  -3.78740  -3.78740  -3.41742  -3.41742  -6.31934  -6.31934  nan  CircularDesk(1,:);
%     1.13680   3.61272   3.72447   3.72447   3.90944   3.90944   0.71869   0.71869   1.13680  nan  CircularDesk(2,:)];
% % Cafe
% disinfectionRegion{3} = [...
%     4.06269   4.27660   5.91956   5.91956   3.41567   3.41567       NaN   4.87627   4.87627   4.20471   4.20121
%     3.90944   3.48362   3.48362   0.71869   0.71869   3.90944       NaN   1.39692   2.75723   2.75723   1.39692];


disinfectionRegion{1} = [5.8   4.01567   4.01567   5.8;
     3.30944   3.30944   2.41869   2.41869];
for ii = 1:numRegions
    xGridRegion = linspace(min(disinfectionRegion{ii}(1,:)),max(disinfectionRegion{ii}(1,:)),numHorizontalTiles(ii)+1)+imPos(1);
    yGridRegion = linspace(min(disinfectionRegion{ii}(2,:)),max(disinfectionRegion{ii}(2,:)),numVerticalTiles+1)+imPos(2);
    [xDisinfectMesh{ii},yDisinfectMesh{ii}] = meshgrid(xGridRegion,yGridRegion);
    CDisinfect{ii} = reshape(infectionColor,1,1,[]).*ones(size(xDisinfectMesh{ii},1)-1,size(xDisinfectMesh{ii},2)-1,3);
    hDisinfection(ii) = surf(xDisinfectMesh{ii},yDisinfectMesh{ii},zeros(size(xDisinfectMesh{ii})),[CDisinfect{ii},nan(size(CDisinfect{ii},1),1,3);nan(1,size(CDisinfect{ii},2)+1,3)],'FaceColor','flat','EdgeColor','none','FaceAlpha',0.5);%,'Visible','on');
    xDisinfect{ii} =  0.5*(xDisinfectMesh{ii}(1:end-1,1:end-1)+xDisinfectMesh{ii}(2:end,2:end));
    yDisinfect{ii} =  0.5*(yDisinfectMesh{ii}(1:end-1,1:end-1)+yDisinfectMesh{ii}(2:end,2:end));
    percentDisinfected(ii) = 0;
    % xGridRegion = linspace(regionXLim(1,ii),regionXLim(2,ii),numHorizontalTiles(ii)+1)+imPos(1);
    % yGridRegion = linspace(regionYLim(1,ii),regionYLim(2,ii),numVerticalTiles+1)+imPos(2);
    % [xDisinfect{ii},yDisinfect{ii}] = meshgrid(xGridRegion,yGridRegion);
    % CDisinfect{ii} = reshape(infectionColor,1,1,[]).*ones(size(xDisinfect{ii},1),size(xDisinfect{ii},2),3);
    % pointsInRegion = double(inpolygon(xDisinfect{ii}(:),yDisinfect{ii}(:),disinfectionRegion{ii}(1,:).'+imPos(1),disinfectionRegion{ii}(2,:).'+imPos(2)));
    % pointsInRegion(~pointsInRegion) = NaN;
    % pointsInRegion = reshape(pointsInRegion,size(xDisinfect{ii}));
    % CDisinfect{ii} = bsxfun(@times,CDisinfect{ii},pointsInRegion);
    % hDisinfection(ii) = surface(xDisinfect{ii},yDisinfect{ii},zeros(size(xDisinfect{ii})),CDisinfect{ii},'FaceColor','interp','EdgeColor','k','FaceAlpha',0.5);%,'Visible','on');
    % percentDisinfected(ii) = nnz(isnan(pointsInRegion))/numel(pointsInRegion);
end
end
% Transform grid of points to two dimensional matrix
function [XYData,sze] = grid2vec(XData,YData)
numCells = numel(XData);
XYData = cell(numCells,1);
sze = zeros(numCells,2);
for ii = 1:numCells
    XYData{ii} = [XData{ii}(:),YData{ii}(:)].';
    sze(ii,:) = size(XData{ii});
end
end
% Transform two dimensional matrix into grid
function [XData,YData] = vec2grid(XYData,sze)
numCells = numel(XYData);
XData = cell(numCells,1);
YData = cell(numCells,1);
for ii = 1:numCells
    XData{ii} = reshape(XYData{ii}(1,:),sze(ii,:));
    YData{ii} = reshape(XYData{ii}(2,:),sze(ii,:));
end
end