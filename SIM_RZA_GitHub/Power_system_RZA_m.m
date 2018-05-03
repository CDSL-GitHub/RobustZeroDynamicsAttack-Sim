%% Initialization
clear all
close all
clc

%% Set-up: Plant parameters
Klm = 1;
Tlm = 6;
Thn = 4;
Th1 = 5;
Th2 = 6;
Tg = 0.2;
R = 0.05;

max_noise = 0.004;                      % maximum size of measurement noise

%% Set-up: Initial conditions
Delta_f0 = 0;
Delta_P0 = 0;
Delta_X0 = 0.011;

Delta_f0n = 0;
Delta_P0n = 0;
Delta_X0n = 0.01;

%% Set-up: Nominal controller
tauC = 0.01;
numC = [1.8124 -18.8558 0.1523];
denC = conv([1 0],[tauC 1]);
Cs = tf(numC,denC);

%% Attack design: Transform nominal system into Byrnes-Isidori normal form
phin1 = 2*Klm/Tlm/Tg/R - 1/Tlm/Tg -3/Tlm/Thn - 3/Thn/Thn - 3/Thn/Tg;
phin2 = -1/Tlm - 3/Thn - 1/Tg;
psin = Klm/Tlm/Thn + Klm/Tlm/Tg;
gn = -2*Klm/Tlm/Tg;
Sn = 1/Thn;
Gn = 3/Thn/Klm*(-1-Tlm/Thn);

APn = [Sn Gn 0; 0 0 1; psin phin1 phin2];
BPn = [0;0;gn];
CPn = [0 1 0];
DPn = 0;

%% Attack design: Zero-dynamics attack
za0 = 0.001;

%% Attack design: Robust zero-dynamics attack
zna0 = -0.001;
tau = 0.001;
alpha1 = 2;
alpha0 = 1;
alpha = [alpha1;alpha0];
A2 = [0 1;0 0];
B2 = [0;1];
C2 = [1 0];
overline_phin = [phin2; phin1];
underline_Delta = [tau 0;0 tau^2];
overline_sat = 20000;
underline_sat = -20000;
ks = -10;                                % Additional parameter for design of "smooth" saturation function
ooverline_sat = overline_sat -1/ks;
uunderline_sat = underline_sat + 1/ks;

tau1 = 0.05;                                                                
underline_Delta1 = [tau1 0;0 tau1^2];
tau2 = 0.005;
underline_Delta2 = [tau2 0;0 tau2^2];
tau3 = 0.001;
underline_Delta3 = [tau3 0;0 tau3^2];

%% Simulation: Set-up
tsim = 115;                             % Simulation time
ttrig = 60;                             % Initial time of attack

%% Simulation: Zero-dynamics attacks
sim('Power_system_ZA')

fig = figure()
set(fig, 'Position', [0 0 1000 350]);
set(fig, 'PaperSize', [29.7000 21.0000]);
set(fig,'PaperPosition',[1 1 21 7])
plot(t_ZA,y_NU,'--k','linewidth',3);
hold on
plot(t_ZA,y_SU,'-.r','linewidth',3);
hold on
plot(t_ZA,y_LU,'b','linewidth',3);
grid on
axis([0 tsim -0.02 0.02]);
xlabel('Time (sec)','fontsize',18,'Interpreter','latex');
set(gca,'FontSize',18)
saveas(fig,'ZA_Freq_Uncertain','pdf');

fig = figure()
set(fig, 'Position', [0 0 1000 350]);
set(fig, 'PaperSize', [29.7000 21.0000]);
set(fig,'PaperPosition',[1 1 21 7])
plot(t_ZA,valve_NU,'--k','linewidth',3);
hold on
plot(t_ZA,valve_SU,'-.r','linewidth',3);
hold on
plot(t_ZA,valve_LU,'b','linewidth',3);
grid on
axis([0 tsim -0.1 1]);
xlabel('Time (sec)','fontsize',18,'Interpreter','latex');
set(gca,'FontSize',18)
saveas(fig,'ZA_Valve_Uncertain','pdf');

fig = figure()
set(fig, 'Position', [0 0 1000 350]);
set(fig, 'PaperSize', [29.7000 21.0000]);
set(fig,'PaperPosition',[1 1 21 7])
plot(t_ZA,a_NU,'--','Color',[0 0.6 0],'linewidth',3);
grid on
axis([0 tsim -0.2 1]);
xlabel('Time (sec)','fontsize',18,'Interpreter','latex');
set(gca,'FontSize',18)
saveas(fig,'ZA_Attack_Uncertain','pdf');


%% Simulation: Robust zero-dynamics attacks w/ various T_h
sim('Power_system_RZA')

fig = figure()
set(fig, 'Position', [0 0 1000 350]);
set(fig, 'PaperSize', [29.7000 21.0000]);
set(fig,'PaperPosition',[1 1 21 7])
plot(t_RZA,y_NU_RZA,'--k','linewidth',3);
hold on
plot(t_RZA,y_SU_RZA,'-.r','linewidth',3);
hold on
plot(t_RZA,y_LU_RZA,'b','linewidth',3);
grid on
axis([0 tsim -0.02 0.02]);
xlabel('Time (sec)','fontsize',18,'Interpreter','latex');
set(gca,'FontSize',18)
saveas(fig,'RZA_Freq_Uncertain','pdf');

fig = figure()
set(fig, 'Position', [0 0 1000 350]);
set(fig, 'PaperSize', [29.7000 21.0000]);
set(fig,'PaperPosition',[1 1 21 7])
plot(t_RZA,valve_NU_RZA,'--k','linewidth',3);
hold on
plot(t_RZA,valve_SU_RZA,'-.r','linewidth',3);
hold on
plot(t_RZA,valve_LU_RZA,'b','linewidth',3);
grid on
axis([0 tsim -0.1 1]);
xlabel('Time (sec)','fontsize',18,'Interpreter','latex');
set(gca,'FontSize',18)
saveas(fig,'RZA_Valve_Uncertain','pdf');

fig = figure()
set(fig, 'Position', [0 0 1000 350]);
set(fig, 'PaperSize', [29.7000 21.0000]);
set(fig,'PaperPosition',[1 1 21 7])
plot(t_RZA,a_NU_RZA,'--k','linewidth',3);
hold on
plot(t_RZA,a_SU_RZA,'-.r','linewidth',3);
hold on
plot(t_RZA,a_LU_RZA,'b','linewidth',3);
grid on
axis([0 tsim -0.2 1]);
xlabel('Time (sec)','fontsize',18,'Interpreter','latex');
set(gca,'FontSize',18)
saveas(fig,'RZA_Attack_Uncertain','pdf');

%% Simumation: Robust zero-dynamics attack w/ various taus
sim('Power_system_RZA_tau');

fig = figure()
set(fig, 'Position', [0 0 1000 350]);
set(fig, 'PaperSize', [29.7000 21.0000]);
set(fig,'PaperPosition',[1 1 21 7])
plot(t_RZA_tau,y_LU_RZA_tau3,'b','linewidth',3);
hold on
plot(t_RZA_tau,y_LU_RZA_tau2,'-.','Color',[0,0.6,0],'linewidth',3);
grid on
axis([0 140 -0.02 0.02]);
xlabel('Time (sec)','fontsize',18,'Interpreter','latex');
set(gca,'FontSize',18) 
saveas(fig,'LU_RZA_tauVariation','pdf');

fig = figure()
set(fig, 'Position', [0 0 1000 350]);
set(fig, 'PaperSize', [29.7000 21.0000]);
set(fig,'PaperPosition',[1 1 21 7])
plot(t_RZA_tau,valve_LU_RZA_tau3,'b','linewidth',3);
hold on
plot(t_RZA_tau,valve_LU_RZA_tau2,'-.','Color',[0,0.6,0],'linewidth',3);
grid on
axis([0 140 -0.1 1]);
% axis([0 140 -1 0.1]);
xlabel('Time (sec)','fontsize',18,'Interpreter','latex');
set(gca,'FontSize',18) 
saveas(fig,'LU_RZA_Valve_tauVariation','pdf');

%% Simulation: Robust zero-dynamics attack under noisy measurements
sim('Power_system_Noisy');

fig = figure()
set(fig, 'Position', [0 0 1000 350]);
set(fig, 'PaperSize', [29.7000 21.0000]);
set(fig, 'PaperPosition',[1 1 21 7])
plot(t_Noisy,y_RZA_Noisy,'b','linewidth',3);
grid on
axis([0 tsim -0.02 0.02]);
xlabel('Time (sec)','fontsize',18,'Interpreter','latex');
set(gca,'FontSize',18) 
saveas(fig,'LU_Noisy','pdf');
