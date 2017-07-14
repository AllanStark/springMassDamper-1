%% Spring Mass Damper Step Response
% Both using step() function (continous and discrete) and using for loop
% by Michael Klamkin, 2017

clear all; clc; clf; close all; % clear workspace, command window, figures, and close all windows
totalTime = tic; % timer for entire script
%% Constants and Initialization
tic % for timing

k = 24;% spring constant

c = 8;% damping coefficient % use formula c^2 - 4*m*k = 0 to find critical damping

m = 200;% mass in grams

stpAmp = m/1000 * 9.81; % step amplitude for step() function

setAmp = stepDataOptions('StepAmplitude' ,stpAmp); % stores above as OPT for use in step(sys, OPT)

X = [0; % initial states: position and velocity
     0]; 

display(toc, 'Constants and Initialization'); % for timing
%% Time Variables
tic % for timing

duration = 250; % duration of for loop

Ts = 1/5; % timestep

t = 0:Ts:duration; % time vector

nSmp = length(t); % number of samples

display(toc, 'Time Variables'); % for timing
%% State Space Matrices
tic % for timing

A = [0 ,   1; 
   -k/m, -c/m]; % for statespace

B = [0;
    1/m]; % for statespace

C = [1,0]; % for statespace

D = 0; % for statespace

display(toc, 'State Space Matrices'); % for timing
%% Preallocation
tic % for timing

y = zeros(1, nSmp); % preallocation for speed

u = stpAmp * ones(1, nSmp); % constant force of gravity

x_plot = zeros(2, nSmp); % preallocation for speed

display(toc, 'Preallocation'); % for timing
%% Continous - Automated Matlab Step Response + Plotting
tic % for timing

sysC = ss(A, B, C, D); % establishes continuous time statespace system

figure(1); % new figure
    subplot(2, 2, 1);
    step(sysC, setAmp); % graph step response
    title('Continous Time - Automated'); % change title
    xlim([0, duration]) % sets xlimit to duration
    
display(toc, 'Continous - Automated Matlab Step Response + Plotting'); % for timing
%% Discrete - Automated Matlab Step Response + Plotting
tic % for timing

sysD = c2d(sysC, Ts); % convert sysC(continous) to sysD (discrete)

subplot(2, 2, 2); % new figure
    step(sysD, setAmp); % automated matlab  step response for discrete time conversion (to compare to for loop)
    title('Discrete Time - Automated');
    xlim([0, duration]) % sets xlimit to duration
    
display(toc, 'Discrete - Automated Matlab Step Response + Plotting'); % for timing
%% Manual Spring-Mass-Damper System Calculation
tic % for timing

for i = 1:nSmp % loops nSmp times, setting i to current cycle #
    
    y(i) = sysD.C * X + sysD.D * u(i); % output - [position]
    
    X = sysD.A * X + sysD.B * u(i); % states - [position;velocity]

    x_plot(:,i) = X; % for plotting X later
    
end % terminate for loop

display(toc, 'Manual Spring-Mass-Damper System Calculation'); % for timing
%% Plotting Manual Spring-Mass-Damper System 
tic % for timing

subplot(2, 2, 3); % new figure
    fig3 = plot(t, x_plot(1,:), 'b', t, x_plot(2, :), 'r'); % plots position in blue and velocity in red (y axis) with time on x axis
    leg = legend('Displacement', 'Velocity'); set(leg, 'FontSize', 10.125); % legend - posiiton is blue and velocity is red
    title('Manual For Loop - Auto Discretization'); % figure title
    set(fig3, 'LineWidth', 1.5); set(gca, 'FontName', 'Times New Roman'); set(gca, 'TitleFontSizeMultiplier', 1.25); set(gca, 'FontSize', 10); set(gca, 'LabelFontSizeMultiplier', 1.125); % figure styling
    ylabel('Displacement (m) and Velocity (m/s)'); % y axis label
    xlabel('Time (s)'); % x axis label
    
display(toc, 'Plotting Manual Spring-Mass-Damper System'); % for timing
%% Manual Discretization of State Space
tic
G = expm((A * Ts)); % discretized A
H = (A')*(G - eye(2))*(B); % discretized B
J = C; % discretized C
K = D; % discretized D

x_dis = [0;0];
y_dis = zeros(1, nSmp); % preallocation for speed
x_dis_plot = zeros(2, nSmp);

for i = 1:nSmp % loops nSmp times, setting i to current cycle #
    
    y_dis(i) = J * x_dis + K * u(i); % output - [position]
    
    x_dis = G * x_dis + H * u(i); % states - [position;velocity]

    x_dis_plot(:,i) = x_dis; % for plotting X later
    
end % terminate for loop
display(toc, 'Manual Discretization of State Space');
%% Plotting Manual Discretization of State Space
tic
subplot(2, 2, 4);
    fig4 = plot(t, x_dis_plot(1,:), 'b', t, x_dis_plot(2, :), 'r'); % plots position in blue and velocity in red (y axis) with time on x axis
    leg = legend('Displacement', 'Velocity'); set(leg, 'FontSize', 10.125); % legend - posiiton is blue and velocity is red
    title('Manual For Loop - Manual Discretization'); % figure title
    set(fig4, 'LineWidth', 1.5); set(gca, 'FontName', 'Times New Roman'); set(gca, 'TitleFontSizeMultiplier', 1.25); set(gca, 'FontSize', 10); set(gca, 'LabelFontSizeMultiplier', 1.125); % figure styling
    ylabel('Displacement (m) and Velocity (m/s)'); % y axis label
    xlabel('Time (s)'); % x axis label
    
display(toc, 'Plotting Manual Spring-Mass-Damper System'); % for timing

display(toc(totalTime), 'Total Time'); % final timer
