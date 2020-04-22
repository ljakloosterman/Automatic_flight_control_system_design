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
%%s
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
A_lateral(6,:) = [];
A_lateral(5,:) = [];

A_ac_long = [A_longitude(:,1),A_longitude(:,2),A_longitude(:,3),A_longitude(:,4)];
B_ac_long = A_longitude(:,5);
C_ac_long = [0,1,0,0];
D_ac_long = [0];

sys_long = ss(A_ac_long,B_ac_long,C_ac_long,D_ac_long);
pzmap(sys_long)
grid on

A_ac_lat = [A_lateral(:,1),A_lateral(:,2),A_lateral(:,3),A_lateral(:,4)];
B_ac_lat = [A_lateral(:,5),A_lateral(:,6)];
C_ac_lat = [0,1,0,0];
D_ac_lat = [0,0];

sys_lat = ss(A_ac_lat,B_ac_lat,C_ac_lat,D_ac_lat);
figure(2)
pzmap(sys_lat)
grid on
s = tf('s');
a = 20.2;
Hservo = a/(s+a);

% 1 control surface
sys_long_series = series(sys_long,Hservo);

% 2 control surfaces
sys_lat_series = series(series(sys_lat,Hservo),Hservo);

% Pole comparison for validation
p_long = pole(sys_long_series);
e_long = eig(A_longitude_original);

p_lat = pole(sys_lat_series);
e_lat = eig(A_lateral_original);

Pole_comp_long = strcat(num2str(p_long)," | ",num2str(e_long));
Pole_comp_lat = strcat(num2str(p_lat)," | ",num2str(e_lat));

% disp(Pole_comp_long);
% disp(Pole_comp_lat);

%% Time respones

%Symetric motions
Modes_SYM = [{' Short Period'};{' Phugoid'}];
PoleLoc_SYM = [2,3;4,5];

Input_SYM = [-5.7;-5];
Input_SYM = [1;(-5+xu_lo(14))*pi/180];
checkspph = [0:1];
Input_time_SYM = [1,1];
TimeSpan_SYM = [10;400];
y_axis_SYM = [{'q [deg/s]'};{'Velocity[ft/s]'}];
alpha2 = xu_lo(8)*180/pi;

figure(3)
tiledlayout(3,1)

for i_sym = 1:2
    
    % Standard denom tf
    SYM_sys_Den = (s-p_long(PoleLoc_SYM(i_sym,1)))*(s-p_long(PoleLoc_SYM(i_sym,2)));
    SYM_sys_Tf = 1/SYM_sys_Den;
    disp(strcat('The ',Modes_SYM(i_sym),' motion has pole locations at'));
    
    p_long(PoleLoc_SYM(i_sym,1))
    p_long(PoleLoc_SYM(i_sym,2))
    [Wn,Z] = damp(SYM_sys_Tf);
    disp(strcat('The ',Modes_SYM(i_sym),' motion has natural freq =' , num2str(Wn(1)) ,' Damping = ', num2str(Z(1))));
    
    if i_sym == 1
        C_ac_long = [0,0,0,1];
    else
        C_ac_long = [1,0,0,0];
    end 
    
    sys_long_plot= ss(A_ac_long, B_ac_long, C_ac_long, D_ac_long);
    
    %%make eigenmotion time responses
    t = 0:0.01:TimeSpan_SYM(i_sym);
    u = Input_SYM(i_sym).*ones(size(t));
    y = lsim(SYM_sys_Tf+checkspph(i_sym)*velocity/Input_SYM(i_sym),u,t);
    %y = lsim(sys_long_plot,u,t);
    
    
    nexttile
    plot(t,y)
    grid on
    xlabel('Time [s]') 
    ylabel(y_axis_SYM(i_sym))
    title(strcat('Step Response of ',Modes_SYM(i_sym)))
    t2ha = abs(0.693/real(p_long(PoleLoc_SYM(i_sym,1))));
    Wn;
    Period = 2*pi/Wn(1);
    disp(strcat('The ',Modes_SYM(i_sym),' motion time to half amplitude =' , num2str(t2ha) ,' and Period =',num2str(Period)));
    
    if i_sym == 2
        
        % For specific output definition
        C_ac_long = [0, 1, 0, 0];
        sys_long_plot= ss(A_ac_long, B_ac_long, C_ac_long, D_ac_long);
        y_phugoid = lsim(sys_long_plot+alpha2/(Input_SYM(i_sym)*180/pi),u*180/pi,t);
        
        nexttile
        plot(t,y_phugoid)
        grid on
        xlabel('Time [s]') 
        ylabel('AoA[deg]')
        title(strcat('Step Response of ',Modes_SYM(i_sym)))
        
    end 
    
       
end


%% Asymetric motions
Modes_ASYM = [{' Dutch Roll'};{' Aperiodic Roll'} ; {' Spiral'}];
PoleLoc_ASYM = [3,4;5,1;6,1];

Input_ASYM = [5;1;1];
Input_time_ASYM = [1;1;1];
TimeSpan_ASYM = [10;10;200];
y_axis_ASYM = [{'p [deg/s]'};{'p [deg/s]'};{'Roll angle [deg]'}];

figure(4)
tiledlayout(4,1)

for i_asym = 1:3
    
    disp(strcat('The ',Modes_ASYM(i_asym),' motion has pole location(s) at:' ));
    p_lat(PoleLoc_ASYM(i_asym,1))
    if i_asym == 2 || i_asym ==3
        ASYM_sys_Den = (s-p_lat(PoleLoc_ASYM(i_asym,1)));
        t2ha = abs(0.693/real(p_lat(PoleLoc_ASYM(i_asym,1))));
    else
        ASYM_sys_Den = (s-p_lat(PoleLoc_ASYM(i_asym,1)))*(s-p_lat(PoleLoc_ASYM(i_asym,2)));
        t2ha = abs(0.693/real(p_lat(PoleLoc_ASYM(i_asym,1))));
        p_lat(PoleLoc_ASYM(i_asym,2))
    end
    
    
    
    
    
    
    ASYM_sys_Tf = 1/ASYM_sys_Den;
    t2ha = 0.693/abs(p_lat(PoleLoc_ASYM(i_asym,1)));
    [Wn,Z] = damp(ASYM_sys_Tf);
    Period = 2*pi/Wn(1);
    disp(strcat('The ',Modes_ASYM(i_asym),' motion has natural freq =' , num2str(Wn(1)) ,' Damping = ', num2str(Z(1))));
    disp(strcat('The ',Modes_ASYM(i_asym),' motion time to half amplitude =' , num2str(t2ha)));

    if i_asym == 1
        %rudder input 
        B_ac_lat_rudder_specific_input = B_ac_lat(:,2);
        D_ac_lat_rudder_specific_input = D_ac_lat(:,2);
        disp(strcat('The ',Modes_ASYM(i_asym),' motion Period =' , num2str(Period)));
    else
        %aileron input 
        B_ac_lat_rudder_specific_input= B_ac_lat(:,1);
        D_ac_lat_rudder_specific_input = D_ac_lat(:,1);
        
        tc = 1/(Wn*Z);
        disp(strcat('The ',Modes_ASYM(i_asym),' motion Time constant tau =' , num2str(tc)));
    end
    
    
    if i_asym == 3
        C_ac_lat = [0,57.29,0,0];
    else
        C_ac_lat = [0,0,57.29,0];
    end 
    
    sys_lat_plot= ss(A_ac_lat, B_ac_lat_rudder_specific_input, C_ac_lat, D_ac_lat_rudder_specific_input);
    
    %%make eigenmotion time responses
    t = 0:0.01:TimeSpan_ASYM(i_asym);
    u = Input_ASYM(i_asym).*ones(size(t));
    u(t>Input_time_ASYM(i_asym)) = 0;
    y = lsim(sys_lat_plot,u,t) ;
    
    if i_asym == 1
        
        % For specific output definition
        C_ac_lat = [0,0,0,57.29];
        
        sys_lat_plot= ss(A_ac_lat, B_ac_lat_rudder_specific_input, C_ac_lat, D_ac_lat_rudder_specific_input);
        y_dutchroll = lsim(sys_lat_plot,u,t) ;
        
        nexttile
        plot(t,y_dutchroll)
        grid on
        xlabel('Time [s]') 
        ylabel('r[deg/s]')
        title(strcat('Impulse Response of ',Modes_ASYM(i_asym)))
    end 
    
    nexttile
    plot(t,y)
    grid on
    xlabel('Time [s]') 
    ylabel(y_axis_ASYM(i_asym))
    title(strcat('Impulse Response of ',Modes_ASYM(i_asym)))
        
end







