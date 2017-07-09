%% Spring Mass Damper Step Response
% Both using step() function (continous and discrete) and using for loop
% by Michael Klamkin, 2017

clear all; clc; clf; close all; % clear workspace, command window, figures, and close all windows
totalTime = tic; % timer for entire script
%% Constants and Initialization
tic % for timing

k = 24;% spring constant

b = 8;% damping coefficient

m = 250;% mass in grams

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
   -k/m, -b/m]; % for statespace

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
%% Automated Matlab Step Response Plotting (Continous Time)
tic % for timing

sysC = ss(A, B, C, D); % establishes continuous time statespace system

figure(1); % new figure
    step(sysC, setAmp); % graph step response
    title('Continous Time - Automated'); % change title
    xlim([0, duration]) % sets xlimit to duration
    
display(toc, 'Automated Matlab Step Response + Plotting (continous)'); % for timing
%% Continous Time to Discrete Time Conversion and Plotting
tic % for timing

sysD = c2d(sysC, Ts); % convert sysC(continous) to sysD (discrete)

figure(2); % new figure
    step(sysD, setAmp); % automated matlab  step response for discrete time conversion (to compare to for loop)
    title('Discrete Time - Automated');
    xlim([0, duration]) % sets xlimit to duration
    
display(toc, 'Continous to Discrete Time Conversion + Plotting '); % for timing
%% Manual Spring-Mass-Damper System Calculation
tic % for timing

for i = 1:nSmp % loops nSmp times, setting i to current cycle #
    
    y(i) = sysD.C * X + sysD.D * u(i); % output - [position]
    
    X = sysD.A * X + sysD.B * u(i); % states - [position;velocity]

    x_plot(:,i) = X; % for plotting X later
    
end % terminate for loop

display(toc, 'Manual Spring-Mass-Damper System Calculation'); % for timing
%% Plotting
tic % for timing

figure(3); % new figure
    fig3 = plot(t, x_plot(1,:), 'b', t, x_plot(2, :), 'r'); % plots position in blue and velocity in red (y axis) with time on x axis
    leg = legend('Position', 'Velocity'); set(leg, 'FontSize', 15); % legend - posiiton is blue and velocity is red
    title('Discrete Time - Manual'); % figure title
    set(fig3, 'LineWidth', 1.5); set(gca, 'FontName', 'Times New Roman'); set(gca, 'TitleFontSizeMultiplier', 1.75); set(gca, 'FontSize', 13); set(gca, 'LabelFontSizeMultiplier', 1.5); % figure styling
    ylabel('Displacement (m) and Velocity (m/s)'); % y axis label
    xlabel('Time (s)'); % x axis label
display(toc, 'Plotting - Final'); % for timing
display(toc(totalTime), 'Total Time'); % final timer
