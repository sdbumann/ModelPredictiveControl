%% do the ritual matlab preparations
clc;
clear all;
close all;


%% add path to the funtion and class files 
addpath("./src");
addpath("./imgs");


%% Declare system 
Ts = 1/5;
quad = Quad(Ts);
CTRL = ctrl_NMPC(quad);


%% Simulate!
sim = quad.sim(CTRL);


%% Plot results
quad.plot(sim);
xlabel("x [m]"), ylabel("y [m]"), zlabel("z [m]");
rotate3d on;
saveas(figure(1), "imgs/reference_tracking_states.png");
saveas(figure(2), "imgs/reference_tracking_trajectory.png");
