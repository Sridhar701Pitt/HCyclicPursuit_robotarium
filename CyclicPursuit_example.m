close all,  clc
%clear all

% Cyclic graph
% A = diag(ones(N-1,1),-1);
% A(1,N) = 1; 
% followers = - diag(sum(A)) + A;
% L = zeros(N+Lead, N+Lead);
% L(2:N+Lead, 2:N+Lead) = followers;
% for i=2:N+Lead
%     L(i,i) = L(i,i) + 1;
%     L(i, 1) = -1;
% end
% Level Cyclic Graphs
structure = {[3];[1,3,4]};
L2_leaders = [2,4,8];
[L,N] = CyclicHierarchyLaplacian(structure);
%N =  Number of agents
dt=0.01;                   % numerical steplength
max_iter = 8000;                           

% Initialize robotarium
circularTargets = [ 0.7*cos( 0:-2*pi/N:-2*pi*(1- 1/N) ); 0.7*sin( 0:-2*pi/N:-2*pi*(1- 1/N) )];
initial_positions = generate_initial_conditions(N, 'Width', 3, 'Height', 1.8,'Spacing', 0.2);
initial_positions(1:2,:) = circularTargets;
rbtm = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);
% rb = RobotariumBuilder();
% rbtm = rb.set_number_of_agents(N).set_save_data(false).build();
si_to_uni_dyn = create_si_to_uni_dynamics();
% Single-integrator barrier certificates
uni_barrier_cert = create_uni_barrier_certificate_with_boundary('SafetyRadius', 0.10,'BarrierGain', 200);

% Initialize robots
xuni = rbtm.get_poses();                                    % States of real unicycle robots
x = xuni(1:2,1:N);                                            % x-y positions only
rbtm.set_velocities(1:N, zeros(2,N));                       % Assign dummy zero velocity
rbtm.step();                                                % Run robotarium step

%% 

% Target cycle definition
center = [0;0];
radiusL1 = 0.6; % Level 1 radius
radiusL2 = 0.3; % Level 2 Radius

kp1 = 0.7; % Gain to maintain circular formation and distance
% kp2 = 1.5; % Gain to follow the centroid
% kp3 = 0.7; % velocity Gain to follow to neighbour (based on distance)
kp2 = 5; % Gain to vary angle based on distance from leader 
kp3 = 0.7; % Gain to maintain distance from the leader
kp4 = 0.7; % Gain to maintain distance between followes
kpL = 1;
Kp1_4 = [0.7 1.0 0.7 0.7;
         0.7 1.0 0.7 1.0]; % Kp1 and kp4 values for different follower count
robot_maxSpeedFactor = 3/4;

%Display iteration and time and title
start_time = tic; %The start time to compute time elapsed.
font_size = determine_font_size(rbtm, 0.03);
iteration_caption = sprintf('Iteration %d', 0);
time_caption = sprintf('Time Elapsed %0.2fs', toc(start_time));
loop_caption = sprintf('Loop Time %0.2fms', 0);
title_caption = sprintf('Hierarchical Cyclic Pursuit \nTest v0.3f - Follower Shift');
iteration_label = text(-1.5, -0.7, iteration_caption, 'FontSize', font_size, 'Color', 'w','FontName','FixedWidth');
time_label = text(-1.5, -0.9, time_caption, 'FontSize', font_size, 'Color', 'w','FontName','FixedWidth');
loop_label = text(-1.5, -0.8, loop_caption, 'FontSize', font_size, 'Color', 'w','FontName','FixedWidth');
title_label = text(-1.5, 0.85, title_caption, 'FontSize', font_size, 'Color', 'w','FontName','FixedWidth','FontWeight','bold');

%DisplayParams
param_label = text(-1.5, 0.3, sprintf(['Parameters\nRadius L1 %0.2f\nRadius L2 %0.2f' ...
    '\nkp1 %0.2f\nkp2 %0.2f\nkp3 %0.2f\nkp4 %0.2f\nkpL %0.2f\nSpeed factor %0.2f' ...
    '\nRobots %d'], radiusL1, radiusL2,kp1,kp2,kp3,kp4,kpL,robot_maxSpeedFactor,N), 'FontSize', font_size, 'Color', 'w','FontName','FixedWidth');

% centerData = plot(center(1),center(2),'*w','markersize',10);
% centeroidData = plot(center(1),center(2),'*y','markersize',10);
% firstRob = plot(x(:,2),'*r','markersize',15);
% secondRob = plot(x(:,3),'*g','markersize',15);
% thirdRob = plot(x(:,4),'*b','markersize',15);

% Draw the orbit 
th = 0 : 2*pi/90 : 2*pi;
plot(radiusL1.*cos(th)+center(1),radiusL1.*sin(th)+center(2),'.','MarkerSize',3,'Color',[0 0 0]+120/256)

% Import and scale the nightsky appropriately.
gt_img = imread('nightsky.png'); % Original input image file
% Display the image with an associated spatial referencing object.
x_img = linspace(-2.0, 2.0, size(gt_img,2));
y_img = linspace(1.5, -1.5, size(gt_img,1)); %Note the 1 to -1 here due to the (1,1) pixel being in the top left corner.
gt_img_handle = image(x_img, y_img, gt_img,'CDataMapping','scaled');

% Import and scale the earth logo appropriately.
wlte_img = imresize(imread('wlte.png'),0.6); % Original input image file

% Display the image with an associated spatial referencing object.
img_scaleFactor = 1.3;
x_img = linspace(-0.1*img_scaleFactor, 0.1*img_scaleFactor, size(wlte_img,2));
y_img = linspace(0.11*img_scaleFactor, -0.11*img_scaleFactor, size(wlte_img,1)); %Note the 1 to -1 here due to the (1,1) pixel being in the top left corner.
wlte_img_handle = image(x_img, y_img, wlte_img,'CDataMapping','scaled');

uistack(wlte_img_handle, 'bottom')
uistack(gt_img_handle, 'bottom')
uistack([iteration_label], 'top'); % Iteration label is on top.
uistack([time_label], 'top'); % Time label is above iteration label.

% Reach initial positions on a circle
if 1        
    circularTargets = [ radiusL1*cos( 0:-2*pi/N:-2*pi*(1- 1/N) ); radiusL1*sin( 0:-2*pi/N:-2*pi*(1- 1/N) )];
    leader_Target = [radiusL1; 0];
    errorToInitialPos = x - circularTargets;                % Error
    errorNorm = [1,1]*(errorToInitialPos.^2);               % Norm of error
    while max( errorNorm ) > 0.02
        % Update state variables        
        xuni = rbtm.get_poses();                            % States of real unicycle robots
        x = xuni(1:2,:);                                    % x-y positions
%         set(firstRob,'XData',x(1,2),'YData',x(2,2))
%         set(secondRob,'XData',x(1,3),'YData',x(2,3))
%         set(thirdRob,'XData',x(1,4),'YData',x(2,4))
        
        % Update errors
        errorToInitialPos = x - circularTargets;
        errorNorm = [1,1]*(errorToInitialPos.^2);
        
        % Conput control inputs
        dxi = -0.3.*errorToInitialPos;
        % To avoid errors, we need to threshold dxi
        norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
        threshold = 2/4*rbtm.max_linear_velocity;
        to_thresh = norms > threshold;
        dxi(:, to_thresh) = threshold*dxi(:, to_thresh)./norms(to_thresh);

        dxu = si_to_uni_dyn(dxi, xuni);
        dxu = uni_barrier_cert(dxu, xuni);
        % Assing new control inputs to robots
        rbtm.set_velocities(1:N, dxu);                       % Assign dummy zero velocity
        
        rbtm.step();                                        % Run robotarium step
    end
    disp('Initial positions reached')
    
end
 loop_time = 0;
 reachedCenter = false;
for k = 1:max_iter
    loop_start_time = tic;
    if k == 4000
        % follower shift between sub leaders
        temp = [0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0;
                -1,	0,	0,	0,	0,	0,	0,	1,	0,	0,	0,	0;
                0,	-1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1;
                -1,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0;
                0,	0,	0,	-1,	0,	0,	1,	0,	0,	0,	0,	0;
                0,	0,	0,	-1,	1,	0,	0,	0,	0,	0,	0,	0;
                0,	0,	0,	-1,	0,	1,	0,	0,	0,	0,	0,	0;
                -1,	0,	0,	1,	0,	0,	0,	0,	0,	0,	0,	0;
                0,	0,	0,	0,	0,	0,	0,	-1,	0,	0,	1,	0;
                0,	0,	0,	0,	0,	0,	0,	-1,	1,	0,	0,	0;
                0,	0,	0,	0,	0,	0,	0,	-1,	0,	1,	0,	0;
                0,	-1,	1,	0,	0,	0,	0,	0,	0,	0,	0,	0];
        L = temp;
    end

%     center = [0.05*cos(k/200);0.05*sin(k/200)];
    center = [0;0];
%     set(centerData,'XData',center(1),'YData',center(2))
    % Get new data and initialize new null velocities
    xuni = rbtm.get_poses();                                % Get new robots' states
    x = xuni(1:2,:);                                        % Extract single integrator states
%     set(firstRob,'XData',x(1,2),'YData',x(2,2))
%     set(secondRob,'XData',x(1,3),'YData',x(2,3))
%     set(thirdRob,'XData',x(1,4),'YData',x(2,4))
    %Find current centroid
    Xc = 1/(N-2)*x(:,3:N)*ones(N-2,1);
%     set(centeroidData,'XData',Xc(1),'YData',Xc(2))
    %Rotate the earth
    if mod(k,2) == 0
        set(wlte_img_handle,'CData', imrotate(wlte_img,k/5,'crop'));
    end
    % Update Iteration and Time marker
    iteration_caption = sprintf('Iteration %d', k);
    time_caption = sprintf('Time Elapsed %0.2fs', toc(start_time));
    loop_caption = sprintf('Loop Time %0.2fms', loop_time);
    
    iteration_label.String = iteration_caption;
    time_label.String = time_caption;
    loop_label.String = loop_caption;

    font_size = determine_font_size(rbtm, 0.04);
    iteration_label.FontSize = font_size;
    time_label.FontSize = font_size;
    loop_label.FontSize = font_size;
    title_label.FontSize = font_size;
    param_label.FontSize = font_size;
    if k >= 900 && k <= 1000
        param_label.Color = [1 1 1]*((1000 - k)/100);
    end
    % Resize Marker Sizes (In case user changes simulated figure window
    % size, this is unnecessary in submission as the figure window 
    % does not change size).
    
    %Actual calculation
    dx = zeros(2,N);                                           % Initialize velocities to zero         
    %Leader follows the center
    
    dx(:,1) = dx(:,1) + kpL*(center - x(:,1));
    if norm(center - x(:,1)) <= 0.005
        reachedCenter = true;
    elseif norm(center - x(:,1)) <= 0.2
        reachedCenter = false;
    end
    
    
    for i = 2:N % loop for all robots except the main leader
        neighbors = topological_neighbors(L, i);
        for j = neighbors
            if ~isempty(j)
                % Find the leader
                leaderidx = find(L(i,:) == -1,1);
                if ismember(i, L2_leaders)
                    radius = radiusL1;
                else
                    radius = radiusL2;
                end
                if L(i,j) == -1
                    distanceToLeaderE = norm(x(:, leaderidx) - x(:,i));
                    beta = pi/2 + kp2*(radius - distanceToLeaderE);
                    beta = min(max(beta,0),pi);
                    R = [cos(beta), sin(beta); -sin(beta) cos(beta)];
                    dx(:,i) = dx(:,i) + kp3*R*(x(:, leaderidx) - x(:,i))/distanceToLeaderE;
                else
                    %Calculate number of followers and find the desired
                    %inter agent distance
                    followerCount = length(find(L(:,leaderidx) == -1));
                    interAgentDistance = radius*2*sin(pi/(followerCount));
                    % Calculate dx
                    alpha = pi/N - Kp1_4(1,followerCount)*(interAgentDistance - norm(x(:,j)-x(:,i)) );
                    alpha = min(max(alpha,-pi/2),pi/2);
                    R = [cos(alpha), sin(alpha); -sin(alpha) cos(alpha)];
                    dx(:,i) = dx(:,i) - Kp1_4(2,followerCount)*((interAgentDistance^2 - norm(x(:,j)-x(:,i))^2)/interAgentDistance).*(1*(x(:,j)-x(:,i)));
                end
            end
        end
    end
    for i = L2_leaders
        dx(:,i) = 0.065*dx(:,i);
    end
    dxi = dx;
    % To avoid errors, we need to threshold dxi
    norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
    threshold = robot_maxSpeedFactor*rbtm.max_linear_velocity;
    to_thresh = norms > threshold;
    dxi(:, to_thresh) = threshold*dxi(:, to_thresh)./norms(to_thresh);

    dxu = si_to_uni_dyn(dxi, xuni);                            % Convert single integrator inputs into unicycle inputs

    if reachedCenter == true
        dxu(:,1) = [0; 0.2];
    end

    dxu = uni_barrier_cert(dxu, xuni);
    rbtm.set_velocities(1:N, dxu); rbtm.step();              % Set new velocities to robots and update
    loop_time = toc(loop_start_time)*1000;
    
end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
rbtm.debug();

% Font Size Helper Function to scale size with figure window
% Input: robotarium instance, desired height of the font in meters
function font_size = determine_font_size(robotarium_instance, font_height_meters)

% Get the size of the robotarium figure window in point units
curunits = get(robotarium_instance.figure_handle, 'Units');
set(robotarium_instance.figure_handle, 'Units', 'Pixels');
cursize = get(robotarium_instance.figure_handle, 'Position');
set(robotarium_instance.figure_handle, 'Units', curunits);

% Determine the ratio of the font height to the y-axis
font_ratio = (font_height_meters)/(robotarium_instance.boundaries(4) -...
    robotarium_instance.boundaries(3));

% Determine the font size in points so it fits the window. cursize(4) is
% the hight of the figure window in points.
font_size = cursize(4) * font_ratio;

end