clear all
clc
close all

addpath obsmutoolsfornewermatlabversions -END % required for some new MATLAB versions

global fi_flag_Simulink

newline = sprintf('\n');

%% Trim aircraft to desired altitude and velocity
%%
altitude = 5000 %input('Enter the altitude for the simulation (ft)  :  ');
velocity = 300 %input('Enter the velocity for the simulation (ft/s):  ');

%% Initial guess for trim
%%
thrust = 5000;          % thrust, lbs
elevator = -0.09;       % elevator, degrees
alpha = 8.49;              % AOA, degrees
rudder = -0.01;             % rudder angle, degrees
aileron = 0.01;            % aileron, degrees

%% Find trim for Hifi model at desired altitude and velocity
%%
disp('Trimming Low Fidelity Model:');
fi_flag_Simulink = 0;
[trim_state_lo, trim_thrust_lo, trim_control_lo, dLEF, xu_lo] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude);

%% Find the state space model for the lofi model at the desired alt and vel.
%%
trim_state_lin = trim_state_lo; trim_thrust_lin = trim_thrust_lo; trim_control_lin = trim_control_lo;

pitcha = xu_lo(5)*180/pi;
g_d = 9.80665;
x_a = 0 ;

[A_lo,B_lo,C_lo,D_lo] = linmod('Trim_lin_F16Block', [trim_state_lin; trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3);...
        dLEF; -trim_state_lin(8)*180/pi], [trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3)]);

    
SS_lo = ss(A_lo,B_lo,C_lo,D_lo);
mat_lo = [A_lo B_lo; C_lo D_lo];

A_longitude_lo = mat_lo([3 5 7 8 11 13 14], [3 5 7 8 11 13 14]);
    
%% System dynamics
%%
% Transfer function determination

As = mat_lo([3 7 8 5 11], [3 7 8 5 11]);
Bs = mat_lo([3 7 8 5 11], [14 13]);
Cs = eye(5);
Ds = zeros(5,2);

%% Flare height
%%

hdot = -velocity*sin(3*pi/180);
x2 = hdot/(tan(3*pi/180));
tau = 1000/x2;
hflare = hdot*tau;
rhflare = hflare+2.5*tau;

%%  Flight path reference plot
%%

mdl = 'Chapter4_GS_and_Flare_controllers_model';
load_system(mdl);

simOut = sim(mdl,'SaveOutput','on',...
   'OutputSaveName','yOut',...
   'SaveTime','on',...
   'TimeSaveName','tOut');
y = simOut.get('yOut');

y_alt= get(y,1);
sim_alt = transpose(y_alt.Values.Data);
sim_t = transpose(y_alt.Values.Time);

y_Vel= get(y,2);
sim_Vel = transpose(y_Vel.Values.Data);

Y_error= get(y,3);
sim_error = transpose(Y_error.Values.Data);

sim_dist = sim_Vel .* sim_t;

distance_airfield = 41162:10:50000;
distance_initial = 1:10:3000;
distance_glideslope = 3000:10:41162;

altitude_airfield  = 3000*ones(length(distance_airfield));
altitude_initial = 5000*ones(length(distance_initial));
altitude_glideslope = -0.0524*distance_glideslope+5.1572e+03;


 
figure(1)
axis([0 50000 2500 5500])
hold on

%simulated data
plot(sim_dist,sim_alt)

%Refrence geometry
plot(distance_glideslope,altitude_glideslope,'r--');
plot(distance_initial,altitude_initial,'r--');
plot(distance_airfield,altitude_airfield,'k','LineWidth',4);

grid on
xlabel('Distance [ft]') 
ylabel('Altitude [ft]')
title('Flight path of controlled F16')
legend({'Controlled flight path','Reference flight path'})

figure(2)
plot(sim_dist,sim_error)
grid on
xlabel('Distance [ft]')
ylabel('Flight tracking error [ft]')

