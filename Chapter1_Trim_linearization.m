clear all
clc
close all

addpath obsmutoolsfornewermatlabversions -END % required for some new MATLAB versions

global fi_flag_Simulink

newline = sprintf('\n');

%% Trim aircraft to desired altitude and velocity
%%
altitude = 15000 %input('Enter the altitude for the simulation (ft)  :  ');
velocity = 500 %input('Enter the velocity for the simulation (ft/s):  ');

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

%% Find the state space model for the hifi model at the desired alt and vel.
%%
trim_state_lin = trim_state_lo; trim_thrust_lin = trim_thrust_lo; trim_control_lin = trim_control_lo;

s = tf('s');
g_d = 3.2808*9.80665;

x_a = 0;
[A_lo,B_lo,C_lo,D_lo] = linmod('Trim_lin_F16Block', [trim_state_lin; trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3);...
            dLEF; -trim_state_lin(8)*180/pi], [trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3)]);

A_1 = [A_lo(3,:);A_lo(5,:);A_lo(7,:);A_lo(8,:);A_lo(11,:);A_lo(14,:)];
A_2 = horzcat(A_1(:,3), A_1(:,5),A_1(:,7), A_1(:,8),A_1(:,11), A_1(:,14));

B = [B_lo(3,:);B_lo(5,:);B_lo(7,:);B_lo(8,:);B_lo(11,:);B_lo(14,:)];
C = horzcat(C_lo(:,3), C_lo(:,5),C_lo(:,7), C_lo(:,8),C_lo(:,11), C_lo(:,14));
D = D_lo(19,:);

[num, den] = ss2tf(A_2,B(:,2),C(19,:),D(:,2));
sys = tf(num,den);

figure(1)
rlocus(sys)


figure(2)
subplot(2,1,1)
t = 0:0.01:3;
u = -1.*ones(size(t));
y=lsim(sys,u,t);
plot(t,y)
xlabel('Time (s)')
ylabel('a_n (m/s^2)')


subplot(2,1,2)
t = 0:0.01:300;
u = -1.*ones(size(t));
y=lsim(sys,u,t);
plot(t,y)
xlabel('Time (s)')
ylabel('a_n (m/s^2)')
        

x_a_vect = [0,5,5.9,6,7,15];
for i = 1:6

    x_a = x_a_vect(i);

    
    [A_lo,B_lo,C_lo,D_lo] = linmod('Trim_lin_F16Block', [trim_state_lin; trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3);...
            dLEF; -trim_state_lin(8)*180/pi], [trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); trim_control_lin(3)]);


    A_1 = [A_lo(3,:);A_lo(5,:);A_lo(7,:);A_lo(8,:);A_lo(11,:);A_lo(14,:)];
    A_2 = horzcat(A_1(:,3), A_1(:,5),A_1(:,7), A_1(:,8),A_1(:,11), A_1(:,14));

    B = [B_lo(3,:);B_lo(5,:);B_lo(7,:);B_lo(8,:);B_lo(11,:);B_lo(14,:)];
    C = horzcat(C_lo(:,3), C_lo(:,5),C_lo(:,7), C_lo(:,8),C_lo(:,11), C_lo(:,14));
    D = D_lo(19,:);

    [num, den] = ss2tf(A_2,B(:,2),C(19,:),D(:,2));
    sys = tf(num,den);
    
  
    

    % Plotting negative step Response
    
    t = 0:0.001:0.2;
    u = -1.*ones(size(t));
    %u(t>2) = 0;
    y=lsim(sys,u,t) ;
    
    figure(3)
    hold on
    plot(t,y,'DisplayName',strcat('x_a = ',num2str(x_a)))
    xlabel('Time (s)') 
    ylabel('a_n (m/s^2)')
    title('Step Response of elevator to normal acceleration')
    lgd = legend;
    lgd.NumColumns = 2;
end
    
    
    
    