%% Uebung Regelungssysteme  
% Parameterdatei zur Simulation des RRP-Roboters
%
% Ersteller:    MK, 09.11.2009
% ?nderungen:   BM, AB, 2011
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


clear all
% close all
clc



% Soll-Trajektorie:
parReg.traj_fKreis     = 0.05;
parReg.traj_fz         = 2;
parReg.traj_phi10      = -pi/2; 
parReg.traj_phi20      = 0;
parReg.traj_s0         = 0.5;



% Roboter:
parSys.m1   = 20;
parSys.m2   = 20;
parSys.m3   = 20;
%Echte Masse, nominell =20;
parSys.mL   = 50;
parSys.l1   = 1;
parSys.l2   = 1;
parSys.l3   = 1;
parSys.Ixx1 = 1.7;
parSys.Iyy1 = 1.7;
parSys.Izz1 = 7e-2;
parSys.Ixx2 = 1.7;
parSys.Iyy2 = 7e-2;
parSys.Izz2 = 1.7;
parSys.Ixx3 = 1.7;
parSys.Iyy3 = 7e-2;
parSys.Izz3 = 1.7;
parSys.d1   = 1;
parSys.d2   = 1;
parSys.d3   = 10;
parSys.delta_phi10 = 0;    %pi/4;
parSys.delta_phi20 = 0;    %pi/4;
parSys.delta_s0    = 0;    %0.25;
parSys.phi10 = parReg.traj_phi10 + parSys.delta_phi10;
parSys.phi20 = parReg.traj_phi20 + parSys.delta_phi20; 
parSys.s0    = parReg.traj_s0    + parSys.delta_s0; 

% Reglerparameter
% nominelle Streckenparameter
parReg.parSys = parSys;
parReg.parSys.mL = 20; % nominelle Lastmasse

% PD-Regler:
parReg.PD.Ts = 0.005;

% Kp Berechnung: tau= tau_stoerung , qp=0 (weil stationaer) -> aus
% Regelgesetz für tau (3.2) Kp berechnen ->
parReg.PD.KP = diag([17188,17188,20000]);

% Trial and Error
parReg.PD.KD = diag([5000,5000,5000]);

% Computed-Torque-Regler:
% Pole Fehlersystem bei (s+10)^2 = s^2 + 20s + 100
parReg.CT.Ts  = 0.005;
parReg.CT.K0 = diag([100,100,100]);
parReg.CT.K1 = diag([20,20,20]);

% Computed-Torque-Regler mit Adaptionsregelgesetz:
parReg.CTA.Ts  = 0.001;
parReg.CTA.B = [zeros(3,3);eye(3,3)];
parReg.CTA.QQ = 10.*eye(6);
parReg.CTA.A=[zeros(3,3) eye(3);-parReg.CT.K0 -parReg.CT.K1];
parReg.CTA.P = lyap(parReg.CTA.A',parReg.CTA.QQ);
parReg.CTA.Gamma = 1e-3;

% Sollwertfilter:
parReg.nennerpolynom = conv(conv([1/100 1],[1/100 1]),[1/100 1]);
parReg.par_filt_a0   = parReg.nennerpolynom(4)/parReg.nennerpolynom(1);
parReg.par_filt_a1   = parReg.nennerpolynom(3)/parReg.nennerpolynom(1);
parReg.par_filt_a2   = parReg.nennerpolynom(2)/parReg.nennerpolynom(1);
parReg.par_filt_V    = 1/parReg.nennerpolynom(1);