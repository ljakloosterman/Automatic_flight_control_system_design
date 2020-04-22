clear all
clc
close all

addpath obsmutoolsfornewermatlabversions -END % required for some new MATLAB versions

global fi_flag_Simulink

newline = sprintf('\n');

%% Trim aircraft to desired altitude and velocity
%%
altitude = 30000 %input('Enter the altitude for the simulation (ft)  :  ');
velocity = 600 %input('Enter the velocity for the simulation (ft/s):  ');

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

g_d = 3.2808*9.80665;
x_a = 0;

[A_lo,B_lo,C_lo,D_lo] = linmod('Trim_lin_F16Block', [trim_state_lin; trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3);...
            dLEF; -trim_state_lin(8)*180/pi], [trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3)]);
        
 %% Make state space model of LoFi
%%
SS_lo = ss(A_lo,B_lo,C_lo,D_lo);
%% Make MATLAB matrix
%%
mat_lo = [A_lo B_lo; C_lo D_lo];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Longitudal Directional %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Select the components that make up the longitude A and B matrix
%%

A_longitude_original = mat_lo([7 8 5 11 14], [7 8 5 11 14]);
A_lateral_original = mat_lo([9 4 10 12 15 16], [9 4 10 12 15 16]);

A_longitude = mat_lo([7 8 5 11 14], [7 8 5 11 14]);
C_longitude_lo = mat_lo([21 23 25 26 29], [7 8 5 11 14]);

A_lateral = mat_lo([9 4 10 12 15 16], [9 4 10 12 15 16]);
C_lateral_lo = mat_lo([22 24 25 27 28 30], [9 4 10 12 15 16]);


%% Generate A/c matrices and validation
%%
% Remove final row
A_longitude(5,:) = [];

A_ac_long = [A_longitude(:,1),A_longitude(:,2),A_longitude(:,3),A_longitude(:,4)];
B_ac_long = A_longitude(:,5);
C_ac_long = [0,1,0,0];
D_ac_long = [0];

sys_long = ss(A_ac_long,B_ac_long,C_ac_long,D_ac_long);

%% Constructing the short period reduced model
%%
% Obtain angle of attack and pitch rate
A_red = [A_ac_long(2,2),A_ac_long(2,4);A_ac_long(4,2),A_ac_long(4,4)];
B_red = [B_ac_long(2,:);B_ac_long(4,:)];
C_red = [C_ac_long(:,2),C_ac_long(:,4)];
D_red = D_ac_long;

sys_red = ss(A_red,B_red,C_red,D_red);

%% Time responses to step input

t_short = 0:0.01:5;
t_long = 0:0.01:30;

figure(1)
subplot(2,1,1)
u = -5.7*ones(size(t_short));
y_red = lsim(sys_red,u,t_short);
y_long = lsim(sys_long,u,t_short);
hold on
plot(t_short,y_red)
plot(t_short,y_long)
title('Short term pitch rate response to a step input')
xlabel('time [sec]')
ylabel('pitch rate[deg/s]')
legend({'2 state model','4 state model'},'Location','southeast')

subplot(2,1,2)
u = -5.7*ones(size(t_long));
y_red = lsim(sys_red,u,t_long);
y_long = lsim(sys_long,u,t_long);
hold on
plot(t_long,y_red)
plot(t_long,y_long)
title('Long term pitch rate response to a step input')
xlabel('time [sec]')
ylabel('pitch rate[deg/s]')
legend({'2 state model','4 state model'},'Location','southeast')

%% Deriving CAP and Gibson criteria
%%

velocity=velocity*0.3048;

sp_f = 0.03*velocity;% Natural frequency
sp_d = 0.5;                 % Damping ratio
sp_T = 1/(0.75*sp_f);       % Time constant

%% Constructing feedback gain matrix K
%%

elevator_pitch_rate_tf_des = (tf([sp_T 1],[1 2*sp_d*sp_f sp_f^2]))    % Elevator to pitch rate transfer function
P12 = pole(elevator_pitch_rate_tf_des);                                % Poles of transfer function


K = place(A_red,B_red,P12);                                            % Matrix K=[Ka,Kq]

%% Vertical wind gust
%%

AoA_error = atan(4.572/(velocity));          % Angel of attack error [rad]
e_dd = abs(K(1,1)*AoA_error);                       % Demanded elevator deflection [deg]

%% Time constant calc
%%

% Obtain angle of attack and pitch rate
A_q = A_red - B_red*K;
B_q = [0;B_red(2,:)];
C_q = [0,1];
D_q = [0];

s = tf('s');

[num,den] = ss2tf(A_q,B_q,C_q,D_q);
tf_curr = tf(K(1,2)*num,den)

H_Lead_lag = tf([sp_T 1],K(1,2)*num)

tf_new =  (0.243*s+1)/(s^2+5.486*s+30.1)               %H_Lead_lag*tf_curr
tf_final = 30.1*tf_new

[damp,wn]= damp(tf_final)

%% CAP and Gibson Criteria
%%

CAP = (sp_f^2*9.80665*sp_T)/velocity
Gibson = sp_T-((2*sp_d)/sp_f)

t = 0:0.01:4;
u1 = zeros(size(t));
u1(1,1:200)=1;
y1 = lsim(tf_final,u1,t);
qm1=max(y1);
qm1p=qm1*ones(size(t));

u2=cumtrapz(u1)/100;
y2=cumtrapz(y1)/100;
qm2=max(y2);
qm2p=qm2*ones(size(t));

DB=(y2(t==2)-2)/2;

figure(2)
subplot(2,1,1)
hold on
plot(t,y1)
plot(t,u1)
plot(t,qm1p)
legend('pitch rate','elevator input','q max')
xlabel('time [s]')
ylabel ('pitch rate [rad/s]')

subplot(2,1,2)
hold on
plot(t,y2)
plot(t,u2)
plot(t,qm2p)
legend('pitch angle','elevator input','q max')
xlabel('time [s]')
ylabel ('pitch angle [rad]')