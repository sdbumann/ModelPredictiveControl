%% do the ritual matlab preparations
clc;
clear;
close all;


%% add path to the funtion and class files 
addpath("./src");
addpath("./imgs");


%% Declare system 
Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);


%% Design MPC controllers
draw = true;
mpc_x = MPC_Control_x(sys_x, Ts, draw); 
pause(2); % time to plot & save
mpc_y = MPC_Control_y(sys_y, Ts, draw);
pause(2); % time to plot & save
mpc_z = MPC_Control_z(sys_z, Ts, draw);
pause(2); % time to plot & save
mpc_yaw = MPC_Control_yaw(sys_yaw, Ts, draw);
pause(2); % time to plot & save


%% Simulation parameters 
% starting point: 
x_starting_pose = [0; 0; 0; 2];
y_starting_pose = [0; 0; 0; 2];
z_starting_pose = [0; 2];
yaw_starting_pose = deg2rad([0; 45]);

% reference position: 
x_position_reference = 0;
y_position_reference = 0;
z_position_reference = 0;
yaw_position_reference = deg2rad(0);


%% Simulate!
% setup the simulation 
nbSim = 100;
T = Ts*(0:(nbSim-1));

ux = zeros(1,nbSim-1);
uy = zeros(1,nbSim-1);
uz = zeros(1,nbSim-1);
uyaw = zeros(1,nbSim-1);
x  = zeros(4,nbSim);
y  = zeros(4,nbSim);
z  = zeros(2,nbSim);
yaw = zeros(2,nbSim);

x(:,1) = x_starting_pose;
y(:,1) = y_starting_pose;
z(:,1) = z_starting_pose;
yaw(:,1)= yaw_starting_pose;

for n = 1:(nbSim-1)
    % Get control inputs 
    ux(n)   = mpc_x.get_u(x(:,n));
    uy(n)   = mpc_y.get_u(y(:,n));
    uz(n)   = mpc_z.get_u(z(:,n));
    uyaw(n) = mpc_yaw.get_u(yaw(:,n));
    
    % apply control inputs 
    x(:,n+1)= mpc_x.A * x(:,n) + mpc_x.B*ux(n); 
    y(:,n+1)= mpc_y.A * y(:,n) + mpc_y.B*uy(n); 
    z(:,n+1)= mpc_z.A * z(:,n) + mpc_z.B*uz(n); 
    yaw(:,n+1) = mpc_yaw.A * yaw(:,n) + mpc_yaw.B*uyaw(n); 
end

% calculate settling time:
f = 0.01;
idx     = find(abs(x(4,:)) >= abs(f*x(4,1)), 1, 'last') + 1;
idy     = find(abs(y(4,:)) >= abs(f*y(4,1)), 1, 'last') + 1;
idz     = find(abs(z(2,:)) >= abs(f*z(2,1)), 1, 'last') + 1;
idyaw   = find(abs(yaw(2,:)) >= abs(f*yaw(2,1)), 1, 'last') + 1;


%% Display figures
saveimgs = true;

figure
    colororder({'k', 'r'});
    hold on
    yyaxis right
        p1 = plot(T,  x(2,:), '-r', 'linewidth',2);      % pitch
        p5 = plot(T,  0.035*ones(size(T)), '--r');       % pitch constraint
        p6 = plot(T, -0.035*ones(size(T)), '--r');
        ylabel("pitch [rad]");
    yyaxis left
        p2 = plot(T,  x(4,:), '-k', 'linewidth',2);      % position
        p3 = plot(T,  f*x(4,1)*ones(size(T)), '--k');    % settling thresh
        p4 = plot(T, -f*x(4,1)*ones(size(T)), '--k');
        ylabel("x [m]");
    xlabel("time [s]");
    legend([p1, p2, p5, p3], "pitch", "x", "pitch constraint", "settling thresh");
    title(sprintf("x & pitch, T_{settle} = %.1f s", T(idx(1))))
    grid on
    hold off
    if (saveimgs) 
        saveas(gcf, "imgs/1_x_and_pitch.png"); 
    end

figure
    hold on;
    p1 = plot(T(1:(end-1)),  ux, '-b', 'linewidth',2);% ux
    p2 = plot(T(1:(end-1)),  .3*ones(size(T(1:(end-1)))), '--b');% input constraint
    p3 = plot(T(1:(end-1)), -.3*ones(size(T(1:(end-1)))), '--b');
    xlabel("time [s]"), ylabel("moment [N m]")
    legend("M_{pitch}", "input constraint");
    title("M_{pitch}")
    grid on
    hold off;
    if (saveimgs) 
        saveas(gcf, "imgs/1_M_pitch.png"); 
    end
    
figure
    colororder({'k', 'r'});
    hold on
    yyaxis right
        p1 = plot(T,  y(2,:), '-r', 'linewidth',2);      % roll
        p5 = plot(T,  0.035*ones(size(T)), '--r');       % roll constraint
        p6 = plot(T, -0.035*ones(size(T)), '--r');
        ylabel("roll [rad]");
    yyaxis left
        p2 = plot(T,  y(4,:), '-k', 'linewidth',2);      % position
        p3 = plot(T,  f*y(4,1)*ones(size(T)), '--k');    % settling thresh
        p4 = plot(T, -f*y(4,1)*ones(size(T)), '--k');
        ylabel("y [m]");
    xlabel("time [s]");
    legend([p1, p2, p5, p3], "roll", "y", "roll constraint", "settling thresh");
    title(sprintf("y & roll, T_{settle} = %.1f s", T(idy(1))))
    grid on
    hold off;
    if (saveimgs) 
        saveas(gcf, "imgs/2_y_and_roll.png"); 
    end

figure
    hold on;
    p1 = plot(T(1:(end-1)),  uy, '-b', 'linewidth',2);% uy
    p2 = plot(T(1:(end-1)),  .3*ones(size(T(1:(end-1)))), '--b');% input constraint
    p3 = plot(T(1:(end-1)), -.3*ones(size(T(1:(end-1)))), '--b');
    xlabel("time [s]"), ylabel("moment [N m]")
    legend("M_{roll}", "input constraint");
    title("M_{roll}")
    grid on
    hold off;   
    if (saveimgs) 
        saveas(gcf, "imgs/2_M_roll.png"); 
    end
    
figure
    hold on;
    plot(T,  z(2,:), '-k', 'linewidth',2)       % position
    plot(T,  f*z(2,1)*ones(size(T)), '--k');    % settling thresh
    plot(T, -f*z(2,1)*ones(size(T)), '--k');
    xlabel("time [s]"), ylabel("z [m]")
    legend("z", "settling thresh");
    title(sprintf("z, T_{settle} = %.1f s", T(idz(1))))
    grid on
    hold off;
    if (saveimgs) 
        saveas(gcf, "imgs/3_z.png"); 
    end

figure
    hold on;
    p1 = plot(T(1:(end-1)),  uz, '-b', 'linewidth',2);% ux
    p2 = plot(T(1:(end-1)),  .3*ones(size(T(1:(end-1)))), '--b');    % input constraint
    p3 = plot(T(1:(end-1)), -.2*ones(size(T(1:(end-1)))), '--b');
    xlabel("time [s]"), ylabel("force [N]")
    legend("F_z", "input constraint");
    title("F_z")
    grid on
    hold off;
    if (saveimgs) 
        saveas(gcf, "imgs/3_F_z.png"); 
    end
    
figure
    hold on;
    plot(T,  yaw(2,:), '-k', 'linewidth',2)     % yaw
    plot(T,  f*yaw(2,1)*ones(size(T)), '--k');  % settling thresh
    plot(T, -f*yaw(2,1)*ones(size(T)), '--k');
    xlabel("time [s]"), ylabel("yaw [rad]")
    legend("yaw", "settling thresh");
    title(sprintf("yaw, T_{settle} = %.1f s", T(idyaw(1))))
    grid on
    hold off;
    if (saveimgs) 
        saveas(gcf, "imgs/4_yaw.png"); 
    end

figure
    hold on;
    p1 = plot(T(1:(end-1)),  uyaw, '-b', 'linewidth',2);% ux
    p2 = plot(T(1:(end-1)),  .2*ones(size(T(1:(end-1)))), '--b');    % input constraint
    p3 = plot(T(1:(end-1)), -.2*ones(size(T(1:(end-1)))), '--b');
    xlabel("time [s]"), ylabel("moment [N m]")
    legend("M_{yaw}", "input constraint");
    title("M_{yaw}")
    grid on
    hold off;    
    if (saveimgs) 
        saveas(gcf, "imgs/4_M_yaw.png"); 
    end
    
figure 
    hold on;
    plot3(x(4,:), y(4,:), z(2,:), '-k', 'linewidth',2);
    plot3(x_position_reference,y_position_reference,z_position_reference, 'rx');
    plot3(x(4, 1), y(4, 1), z(2, 1), 'ro');
    legend("trajectory", "goal", "start");
    xlabel("x [m]"); ylabel("y [m]"); zlabel("z [m]");
    title("trajectory of the quadcopter");
    axis("equal")
    grid on;
    rotate3d on;
    hold off;    
    %if (saveimgs) 
    %    saveas(gcf, "imgs/trajectory_of_quadcopter.png"); 
    %end

