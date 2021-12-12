close all; clc

max_iter = 3*300;
N = 6;                         % Number of agents
dt = 0.02;

% Agents' Initial Positions
x0 = [0, 0.6, -0.1, -1, -0.5, 0]';          
y0 = [0, 0.8660254,0.5, -1,-1.0, -0.8660254]';

angleShift = pi/N;


%A = [0,1,0,0,0,0; 1,0,0,0,0,0; 1,1,0,0,1,0; 0,1,0,0,0,0; 0,0,1,1,0,0; 0,1,0,1,0,0];
% Cyclic graph
A = diag(ones(N-1,1),-1);
A(1,N) = 1;
R_theta = [cos(angleShift) sin(angleShift); -sin(angleShift) cos(angleShift)];

% Initialize vectors
x = zeros(N,max_iter);
y = zeros(N,max_iter);
xc = zeros(2,max_iter);
x(:,1) = x0;
y(:,1) = y0;
xc(:,1) = (1/N*ones(1,N)*[x0,y0])';

% Initialize plot
figure(1), hold on
set(gca,'xcolor','none','ycolor','none')
agenPlot = plot(x(:,1),y(:,1),'o','markersize',16,'MarkerFaceColor',[0.12,0.49,0.65],'MarkerEdgeColor','none');
xBplot = plot(xc(1,1),xc(2,1),'x','markersize',16,'MarkerFaceColor',[0.62,0.49,0.15],'linewidth',3);
axis([-1,1,-1,1])
axis equal

for k = 1:max_iter-1
    
    % Initialize zero velocities
    ux = zeros(N,1);           
    uy = zeros(N,1);

    dx = zeros(N, 2);
    
    % Compute consensus
    for i = 1:N
        for j = find(A(:,i))
            if ~isempty(j)
                ux(i,1) = ux(i,1) + ( x(j,k)-x(i,k) );
                uy(i,1) = uy(i,1) + ( y(j,k)-y(i,k) );
                
                mat = [x(j,k)-x(i,k); y(j,k)-y(i,k)];
                dx(i,:) = transpose(transpose(dx(i,:)) + R_theta*(mat))
            end
        end
    end
    
    % Integration step
    %x(:,k+1) = x(:,k) + ux.*dt;
    %y(:,k+1) = y(:,k) + uy.*dt;
    
    x(:,k+1) = x(:,k) + dx(:,1).*dt;
    y(:,k+1) = y(:,k) + dx(:,2).*dt;
    % Record centroid position
    xc(:,k+1) = (1/N*ones(1,N)*[x(:,k),y(:,k)])';
    
    % Update plot
    set(agenPlot,'xdata',x(:,k),'ydata',y(:,k))
    set(xBplot,'xdata',xc(1,k),'ydata',xc(2,k))

    drawnow
end

figure, hold on
set(gcf,'color','white')
plot( 1:max_iter , sqrt(xc(1,:).^2 + xc(2,:).^2),'b','linewidth',2 )
plot( 1:max_iter , sqrt(x.^2 + y.^2),'k' )
legend({'$\bar{x}$','$x_i$'},'interpreter','latex','fontsize',16)
xlabel('Iterations','fontsize',16)
ylabel('$\| x \|$','interpreter','latex','fontsize',16)

%Final Plot
% Initialize plot
figure(3), hold on
set(gca)
plot(x(:,1),y(:,1),'o');
agenPlot = plot(transpose(x(:,:)),transpose(y(:,:)),'.');
plot(x(:,max_iter),y(:,max_iter),'x');
title("Cyclic Pursuit theta = pi/N - 0.1")
axis([-1,1,-1,1])
axis equal