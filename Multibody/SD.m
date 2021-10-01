%%
clear all;
clc;
%% Dimensions&parameters
Rco = 2.71; %cm
Rci = 2.31; %cm
%Rco = 1.5; %cm
%Rci = 0.75; %cm
dr = Rco-Rci; %cm
Lc = 25; %cm 
cyl_xsec = [0 -Lc/2; Rco -Lc/2; 
            Rco Lc/2; Rci Lc/2; 
            Rci -Lc/2+dr; 0 -Lc/2+dr]; %cm
Lp = 15; %cm 
%Lp1 = 18.064; %cm
piston_xsec = [0 -Lp/2; Rci -Lp/2; Rci Lp/2; 0 Lp/2]; %cm (front)
%piston_xsec1 = [0 -Lp1/2; Rci -Lp1/2; Rci Lp1/2; 0 Lp1/2]; %cm (rear)

%
Rp=0.56; %radius of pinion in cm
SA=120; %Steering angle
Del=SA*Rp*pi/180; %Steering angle in radians

%
load('Front_brake.mat');
load('Rear_brake.mat');
load('TPS_0.mat');

%
sim('Multibody_Street_Drone');

%%
figure(1)
plot(tout,Calc_Fz_front,'black',tout,act_Fz_front,'red','LineWidth',1.5);
xlabel('time(s)');
ylabel('N');
legend('Calculated','Actual');
title('Fz-Front');
grid on;
%%
figure(2)
plot(tout,Calc_Fz_rear,'black',tout,act_Fz_rear,'red','LineWidth',1.5);
xlabel('time(s)');
ylabel('N');
legend('Calculated','Actual');
title('Fz-Rear');
grid on;
%%
figure(3)
plot(tout,calc_Fx_RL,'black',tout,actual_Fx_RL,'red','LineWidth',1.5);
xlabel('time(s)');
ylabel('N');
legend('Calculated','Actual');
title('Fx-RL');
grid on;
%%
figure(4)
plot(tout,calc_Fx_RR,'black',tout,actual_Fx_RR,'red','LineWidth',1.5)
xlabel('time(s)');
ylabel('N');
legend('Calculated','Actual')
title('Fx-RR');
grid on;
%%
figure(5)
plot(tout,Calc_Fx_vehicle,'black',tout,actual_Fx_vehicle,'red','LineWidth',1.5);
xlabel('time(s)');
ylabel('N');
legend('Calculated','Actual');
title('Fx-Total');
grid on;

%%
figure(6)
plot(tout,Desired_Vel,'black',tout,Actual_Vel,'red','LineWidth',1.5);
xlabel('sec');
ylabel('m/s');
title('Velocity vs time');
legend('Desired Velocity','Actual Velocity','Location','best');
ylim([-3 12]);
grid on;
%%
figure(7)
plot(tout,tau,'black','LineWidth',1.5);
xlabel('time(s)');
ylabel('Nm');
title('Motor torque vs time');
grid on;
%%
figure(8)
plot(tout,brTau_F,'black',tout,brTau_R,'red','LineWidth',1.5);
xlabel('time(s)');
ylabel('Nm');
legend('Front','Rear','Location','best');
title('Brake torque vs time');
grid on;

%%
figure(9);
yyaxis right
plot(tout,throttle,'red--','LineWidth',1.5)
xlabel('time [s]');ylabel('%')
%ylim([-0.5 1.1])
yyaxis left
plot(tout,tau,'black','LineWidth',1.5)
xlabel('time [s]');ylabel('Nm')
%ylim([-100 80])
legend('Regenerative Motor torque','Throttle position','Location','best')
title('Regenerative Motor torque & TPS vs time')
grid on
%%
figure(10)
yyaxis right
plot(tout,brake_pedal,'green--','LineWidth',1.5)
xlabel('time [s]');ylabel('%')
ylim([-1 0.1])
yyaxis left
plot(tout,brTau_F,'black',tout,brTau_R,'red','LineWidth',1.5);
xlabel('time(s)');
ylabel('Nm');
ylim([-250 25])
legend('Front Brake torque','Rear Brake torque','Brake Pedal Position','Location','best');
title('Brake torque & Brake pedal position vs time');
grid on;
%%
figure(11)
plot(tout,tau,'black',tout,brTau_F,'red--',tout,brTau_R,'green--','LineWidth',1.5);
xlabel('time(s)');
ylabel('Nm');
%ylim([-250 25])
legend('Regenerative motor torque','Front Brake torque','Rear Brake torque','Location','best');
title('Motor & Brake torque vs time');
grid on;

%%
figure(12)
plot(del_hand_wheel,del_road_wheel,'Black','LineWidth',1.5);
xlabel('steering wheel angle (o)');
ylabel('road wheel angle (o)');
%xlim([-530 530]);
ylim([-0.5 4]);
title('Steering Geometry');
grid on;

%%
figure(13)
plot(tout,varinf1(:,3),'black',tout,varinf2(:,3),'red',tout,varinf3(:,3),'green',tout,varinf4(:,3),'magenta','LineWidth',1.5);
xlabel('time(s)');
ylabel('N');
legend('FL','FR','RL','RR','Location','best');
title('Fz');
%xlim([0 102]);
%ylim([1000 2500]);
grid on;

%%
figure(14)
yyaxis left
plot(tout,throttle,'black','LineWidth',2);
xlabel('sec');
ylabel('-');
yyaxis right
plot(tout,brake_pr,'Red','LineWidth',2);
xlabel('sec');
ylabel('bar');
legend('Throttle pedal position','Brake pressure','Location','best');
title('TPS and Brake pr vs time');
%xlim([0 105]);
%ylim([-10 10]);
grid on;
%%
figure(15)
plot(X_SD,Y_SD,'black','LineWidth',2);
xlabel('meters');
ylabel('meters');
title('Street Drone path');
xlim([0 100]);
%ylim([0 16]);
grid on;
%%
figure(16)
d1 = designfilt('lowpassiir','FilterOrder',1, ...
    'HalfPowerFrequency',0.003,'DesignMethod','butter');
y = filtfilt(d1,Act_Ax);
plot(tout,Act_Ax,'red','LineWidth',2);
xlabel('sec');
ylabel('m/s2');
title('Long acc vs time');
%xlim([0 33]);
ylim([-5 5]);
grid on;
%%
figure(17)
plot(tout,Actual_Vel*3.6,'red','LineWidth',2);
xlabel('sec');
ylabel('km/hr');
title('Act Vel vs time');
%xlim([0 105]);
%ylim([-1 60]);
grid on;

%%
figure(18)
plot(tout,ay,'red','LineWidth',2);
xlabel('sec');
ylabel('m/s-2');
title('Lat Acc vs time');
%xlim([0 105]);
ylim([-6 6]);
grid on;
%%
figure(19)
plot(tout,Yaw,'red','LineWidth',2);
xlabel('sec');
ylabel('rad/sec');
title('Yaw rate vs time');
%xlim([0 105]);
ylim([-1.5 1.5]);
grid on;
%%
figure(20)
plot(tout,del_road_wheel,'red','LineWidth',2);
xlabel('sec');
ylabel('degrees');
title('Steering angle vs time');
%xlim([0 105]);
%ylim([-1.5 1.5]);
grid on;