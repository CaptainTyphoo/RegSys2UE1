%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameterdatei für Backstepping am Rotary Flexible Joint
%
% Autor:    Baumgart Michael/Michel Alexander
% Datum:    01.08.2011
% Projekt:  Übung Regelungssysteme
%
% Änderungen: 01.08.2011 Baumgart
%             04.11.2014 Schausberger
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc; clear all;
%% Parameter des Systems
par_sys.I_a = 2.773278e-3;      %kgm^2
par_sys.d_a = 3.9563e-04;       %Nms/rad
par_sys.c_0 = 0.891256;         %Nm/rad

%% Soll-Parameter des Impedanz-Systems
par_imp.d = 10e-3;              %Nms/rad - par_imp(1);
par_imp.M_max = 0.2;            %Nm - par_imp(2);
par_imp.c = 0.7;                %Nm/rad - par_imp(3); 

%% Regler-Parameter
Ts = 1e-3;                      %s - Abtastzeit des Reglers

par_reg.k0 = 1;
par_reg.k1 = 600;
par_reg.gamma_1 = 10^10;
par_reg.c_0 = par_sys.c_0;
par_reg.d_a = par_sys.d_a;
par_reg.I_a = par_sys.I_a;

par_reg.Ts = Ts;

