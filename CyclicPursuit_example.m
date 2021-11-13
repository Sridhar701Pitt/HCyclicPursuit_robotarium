close all, clc
%clear all
N=3;                        % Number of agents
dt=0.01;                   % numerical steplength
max_iter = 5000;                           

% Initialize robotarium
initial_positions = generate_initial_conditions(N, 'Width', 1, 'Height', 1,'Spacing', 0.1);
rbtm = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);
% rb = RobotariumBuilder();
% rbtm = rb.set_number_of_agents(N).set_save_data(false).build();
si_to_uni_dyn = create_si_to_uni_dynamics();
% Single-integrator barrier certificates
uni_barrier_cert = create_uni_barrier_certificate_with_boundary();

% Initialize robots
xuni = rbtm.get_poses();                                    % States of real unicycle robots
x = xuni(1:2,:);                                            % x-y positions only
rbtm.set_velocities(1:N, zeros(2,N));                       % Assign dummy zero velocity
rbtm.step();                                                % Run robotarium step

% Cyclic graph
A = diag(ones(N-1,1),-1);
A(1,N) = 1; 
L = diag(sum(A)) - A;

% Target cycle definition
center = [0;0];
radiusL1 = 0.4; % Level 1 radius
raduisL2 = 0.4; % Level 2 Radius
interAgentDistance = raduisL2*2*sin(pi/N);
kp1 = 7; % Gain to maintain circular formation and distance
kp2 = 0.7; % Gain to stay close to the centroid
kp3 = 0.7; % velocity Gain to follow to neighbour (based on distance) 
centerData = plot(center(1),center(2),'*','markersize',12);
th = 0 : 2*pi/20 : 2*pi-2*pi/20;
plot(radiusL1.*cos(th)+center(1),radiusL1.*sin(th)+center(2),'b')


% Reach initial positions on a circle
if 1        
    circularTargets = [ raduisL2*cos( 0:2*pi/N:2*pi*(1- 1/N) ) ; raduisL2*sin( 0:2*pi/N:2*pi*(1- 1/N) ) ];
    errorToInitialPos = x - circularTargets;                % Error
    errorNorm = [1,1]*(errorToInitialPos.^2);               % Norm of error
    while max( errorNorm ) > 0.05
        % Update state variables        
        xuni = rbtm.get_poses();                            % States of real unicycle robots
        x = xuni(1:2,:);                                    % x-y positions
        
        % Update errors
        errorToInitialPos = x - circularTargets;
        errorNorm = [1,1]*(errorToInitialPos.^2);
        
        % Conput control inputs
        dxi = -0.3.*errorToInitialPos;
        % To avoid errors, we need to threshold dxi
        norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
        threshold = 4/4*rbtm.max_linear_velocity;
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
  
for k = 1:max_iter
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

    center = [radiusL1*cos(k/1600);radiusL1*sin(k/1600)];
    set(centerData,'XData',center(1),'YData',center(2))
    % Get new data and initialize new null velocities
    xuni = rbtm.get_poses();                                % Get new robots' states
    x = xuni(1:2,:);                                        % Extract single integrator states

    dx = zeros(2,N);                                           % Initialize velocities to zero         
    for i = 1:N                
        for j = find(A(:,i))'
            if ~isempty(j)
                alpha =  pi/N + kp1*(interAgentDistance - norm(x(:,j)-x(:,i)) );
                % limit alpha to -pi/2 to pi/2
                alpha = min(max(alpha,-pi/2),pi/2);
                R = [cos(alpha), sin(alpha); -sin(alpha) cos(alpha)];
                dx(:,i) = dx(:,i) + kp3*R*( x(:,j)-x(:,i) ) + kp2*(center - x(:,i));
            end
        end
    end
    dxi = dx;
    % To avoid errors, we need to threshold dxi
    norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
    threshold = 2/4*rbtm.max_linear_velocity;
    to_thresh = norms > threshold;
    dxi(:, to_thresh) = threshold*dxi(:, to_thresh)./norms(to_thresh);

    dxu = si_to_uni_dyn(dxi, xuni);                            % Convert single integrator inputs into unicycle inputs
    dxu = uni_barrier_cert(dxu, xuni);
    rbtm.set_velocities(1:N, dxu); rbtm.step();              % Set new velocities to robots and update
    
end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
rbtm.debug();
