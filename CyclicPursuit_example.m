close all, clc
%clear all
N=3;                        % Number of agents
Lead = 1;                      % Number of leaders 
dt=0.01;                   % numerical steplength
max_iter = 10000;                           

% Initialize robotarium
initial_positions = generate_initial_conditions(N+Lead, 'Width', 1, 'Height', 1,'Spacing', 0.1);
rbtm = Robotarium('NumberOfRobots', N+Lead, 'ShowFigure', true, 'InitialConditions', initial_positions);
% rb = RobotariumBuilder();
% rbtm = rb.set_number_of_agents(N).set_save_data(false).build();
si_to_uni_dyn = create_si_to_uni_dynamics();
% Single-integrator barrier certificates
uni_barrier_cert = create_uni_barrier_certificate_with_boundary('SafetyRadius', 0.06,'BarrierGain', 300);

% Initialize robots
xuni = rbtm.get_poses();                                    % States of real unicycle robots
x = xuni(1:2,1:N+Lead);                                            % x-y positions only
rbtm.set_velocities(1:N+Lead, zeros(2,N+Lead));                       % Assign dummy zero velocity
rbtm.step();                                                % Run robotarium step

% Cyclic graph
A = diag(ones(N-1,1),-1);
A(1,N) = 1; 
followers = - diag(sum(A)) + A;
L = zeros(N+Lead, N+Lead);
L(2:N+Lead, 2:N+Lead) = followers;
for i=2:N+Lead
    L(i,i) = L(i,i) + 1;
    L(i, 1) = -1;
end

%% 

% Target cycle definition
center = [0;0];
radiusL1 = 0.6; % Level 1 radius
radiusL2 = 0.3; % Level 2 Radius
interAgentDistance = radiusL2*2*sin(pi/(N+Lead - 1));
kp1 = 0.7; % Gain to maintain circular formation and distance
% kp2 = 1.5; % Gain to follow the centroid
% kp3 = 0.7; % velocity Gain to follow to neighbour (based on distance)
kp4 = 5; % Gain to vary angle based on distance from leader 
kp5 = 0.7; % Gain to maintain distance from the leader
kp6 = 0.7; % Gain to maintain distance between followes
kpL = 1;
robot_maxSpeedFactor = 4/4;

%Display iteration and time and title
start_time = tic; %The start time to compute time elapsed.
font_size = determine_font_size(rbtm, 0.04);
iteration_caption = sprintf('Iteration %d', 0);
time_caption = sprintf('Time Elapsed %0.2fs', toc(start_time));
loop_caption = sprintf('Loop Time %0.2fms', 0);
title_caption = sprintf('Hierarchical Cyclic Pursuit \nTest v0.2a - Updated Algorithm');
iteration_label = text(-1.5, -0.7, iteration_caption, 'FontSize', font_size, 'Color', 'w','FontName','FixedWidth');
time_label = text(-1.5, -0.9, time_caption, 'FontSize', font_size, 'Color', 'w','FontName','FixedWidth');
loop_label = text(-1.5, -0.8, loop_caption, 'FontSize', font_size, 'Color', 'w','FontName','FixedWidth');
title_label = text(-1.5, 0.85, title_caption, 'FontSize', font_size, 'Color', 'w','FontName','FixedWidth','FontWeight','bold');

%DisplayParams
param_label = text(-1.5, -0.0, sprintf('Parameters\nRadius L1 %0.2f\nRadius L2 %0.2f\nkp1 %0.2f\nkp4 %0.2f\nkp5 %0.2f\nkp6 %0.2f\nkpL %0.2f\nSpeed factor %0.2f', radiusL1, radiusL2,kp1,kp4,kp5,kp6,kpL,robot_maxSpeedFactor), 'FontSize', font_size, 'Color', 'w','FontName','FixedWidth');

centerData = plot(center(1),center(2),'*w','markersize',10);
centeroidData = plot(center(1),center(2),'*y','markersize',10);
firstRob = plot(x(:,2),'*r','markersize',15);
secondRob = plot(x(:,3),'*g','markersize',15);
thirdRob = plot(x(:,4),'*b','markersize',15);

% Draw the orbit 
th = 0 : 2*pi/50 : 2*pi-2*pi/50;
plot(radiusL1.*cos(th)+center(1),radiusL1.*sin(th)+center(2),'b')

% Import and scale the nightsky appropriately.
gt_img = imread('nightsky.png'); % Original input image file
% Display the image with an associated spatial referencing object.
x_img = linspace(-2.0, 2.0, size(gt_img,2));
y_img = linspace(1.5, -1.5, size(gt_img,1)); %Note the 1 to -1 here due to the (1,1) pixel being in the top left corner.
gt_img_handle = image(x_img, y_img, gt_img,'CDataMapping','scaled');

% Import and scale the earth logo appropriately.
wlte_img = imresize(imread('wlte.png'),0.3); % Original input image file

% Display the image with an associated spatial referencing object.
img_scaleFactor = 1.6;
x_img = linspace(-0.1*img_scaleFactor, 0.1*img_scaleFactor, size(wlte_img,2));
y_img = linspace(0.11*img_scaleFactor, -0.11*img_scaleFactor, size(wlte_img,1)); %Note the 1 to -1 here due to the (1,1) pixel being in the top left corner.
wlte_img_handle = image(x_img, y_img, wlte_img,'CDataMapping','scaled');

uistack(wlte_img_handle, 'bottom')
uistack(gt_img_handle, 'bottom')
uistack([iteration_label], 'top'); % Iteration label is on top.
uistack([time_label], 'top'); % Time label is above iteration label.

% Reach initial positions on a circle
if 1        
    circularTargets = [ radiusL1*cos( 0:-2*pi/N:-2*pi*(1- 1/N) ) + radiusL1 ; radiusL1*sin( 0:-2*pi/N:-2*pi*(1- 1/N) ) + 0 ];
    leader_Target = [radiusL1; 0];
    circularTargets = [leader_Target circularTargets];
    errorToInitialPos = x - circularTargets;                % Error
    errorNorm = [1,1]*(errorToInitialPos.^2);               % Norm of error
    while max( errorNorm ) > 0.02
        % Update state variables        
        xuni = rbtm.get_poses();                            % States of real unicycle robots
        x = xuni(1:2,:);                                    % x-y positions
        set(firstRob,'XData',x(1,2),'YData',x(2,2))
        set(secondRob,'XData',x(1,3),'YData',x(2,3))
        set(thirdRob,'XData',x(1,4),'YData',x(2,4))
        
        % Update errors
        errorToInitialPos = x - circularTargets;
        errorNorm = [1,1]*(errorToInitialPos.^2);
        
        % Conput control inputs
        dxi = -0.3.*errorToInitialPos;
        % To avoid errors, we need to threshold dxi
        norms = arrayfun(@(x) norm(dxi(:, x)), 1:N+Lead);
        threshold = 2/4*rbtm.max_linear_velocity;
        to_thresh = norms > threshold;
        dxi(:, to_thresh) = threshold*dxi(:, to_thresh)./norms(to_thresh);

        dxu = si_to_uni_dyn(dxi, xuni);
        dxu = uni_barrier_cert(dxu, xuni);
        % Assing new control inputs to robots
        rbtm.set_velocities(1:N+Lead, dxu);                       % Assign dummy zero velocity
        
        rbtm.step();                                        % Run robotarium step
    end
    disp('Initial positions reached')
    
end
 loop_time = 0;
for k = 1:max_iter
    loop_start_time = tic;
%%Radius adjust 1
%     if (k == max_iter/4)
%         radius = 0.8;
%         interAgentDistance = radius*2*sin(pi/N);
%     elseif k == 2*max_iter/4
%         radius = 0.3;
%         interAgentDistance = radius*2*sin(pi/N);
%     elseif k == 3*max_iter/4
%         radius = 1;
%         interAgentDistance = radius*2*sin(pi/N);
%     end
%%Radius adjust 2
%     radius = abs(sin(k/100))/2 + 0.3 % radius goes from 0.3 to 0.5+0.3
%     interAgentDistance = radius*2*sin(pi/N);

%%Center adjust 1
%     if (k == max_iter/4)
%         center = [-0.5; 0.0];
%         set(centerData,'XData',center(1),'YData',center(2))
%     elseif k == 2*max_iter/4
%         center = [0.5; 0.0];
%         set(centerData,'XData',center(1),'YData',center(2))
%     elseif k == 3*max_iter/4
%         center = [0.0; -0.5];
%         set(centerData,'XData',center(1),'YData',center(2))
%     end  

    center = [radiusL1*cos(k/800);radiusL1*sin(k/800)];
    set(centerData,'XData',center(1),'YData',center(2))
    % Get new data and initialize new null velocities
    xuni = rbtm.get_poses();                                % Get new robots' states
    x = xuni(1:2,:);                                        % Extract single integrator states
    set(firstRob,'XData',x(1,2),'YData',x(2,2))
    set(secondRob,'XData',x(1,3),'YData',x(2,3))
    set(thirdRob,'XData',x(1,4),'YData',x(2,4))
    %Find current centroid
    Xc = 1/(N+Lead - 1)*x(:,2:N+Lead)*ones(N+Lead - 1,1);
    set(centeroidData,'XData',Xc(1),'YData',Xc(2))
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
    % Resize Marker Sizes (In case user changes simulated figure window
    % size, this is unnecessary in submission as the figure window 
    % does not change size).
    
    %Actual calculation
    dx = zeros(2,N+Lead);                                           % Initialize velocities to zero         
    %Leader follows the center
    
    dx(:,1) = dx(:,1) + kpL*(center - x(:,1));
    
    for i = 2:N+Lead % loop for followers
        neighbors = topological_neighbors(L, i);
        for j = neighbors
            if ~isempty(j)
                
                if j == 1
                    distanceToLeaderE = norm(x(:, 1) - x(:,i));
                    beta = pi/2 + kp4*(radiusL2 - distanceToLeaderE);
                    beta = min(max(beta,0),pi);
                    R = [cos(beta), sin(beta); -sin(beta) cos(beta)];
                    dx(:,i) = dx(:,i) + kp5*R*(x(:, 1) - x(:,i))/distanceToLeaderE;
                else
                    alpha = pi/N - kp1*(interAgentDistance - norm(x(:,j)-x(:,i)) );
                    alpha = min(max(alpha,-pi/2),pi/2);
                    R = [cos(alpha), sin(alpha); -sin(alpha) cos(alpha)];
                    dx(:,i) = dx(:,i) - kp6*((interAgentDistance^2 - norm(x(:,j)-x(:,i))^2)/interAgentDistance).*(1*(x(:,j)-x(:,i)));
                end
            end
        end
    end
    dxi = dx;
    % To avoid errors, we need to threshold dxi
    norms = arrayfun(@(x) norm(dxi(:, x)), 1:N+Lead);
    threshold = robot_maxSpeedFactor*rbtm.max_linear_velocity;
    to_thresh = norms > threshold;
    dxi(:, to_thresh) = threshold*dxi(:, to_thresh)./norms(to_thresh);

    dxu = si_to_uni_dyn(dxi, xuni);                            % Convert single integrator inputs into unicycle inputs
    dxu = uni_barrier_cert(dxu, xuni);
    rbtm.set_velocities(1:N+Lead, dxu); rbtm.step();              % Set new velocities to robots and update
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