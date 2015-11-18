function Roboter_S_m(block)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Beschreibung: Simulationsmodell eines RRP-Robotors.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% inputs:   u1(1)... M1       ( Drehmoment für Stab 1 )
%           u1(2)... M2       ( Drehmoment für Stab 2 )
%           u1(3)... F        ( Kraft      für Stab 3 )
%
%
% states:   x(1)... phi1      ( Winkel Stab 1  )
%           x(2)... phi2      ( Winkel Stab 2  )     
%           x(3)... s         ( Weg    Stab 3  )     
%           x(4)... phi1p     ( Winkelgeschwindigkeit Stab 1 )
%           x(5)... phi2p     ( Winkelgeschwindigkeit Stab 2 )
%           x(6)... sp        ( Geschwindigkeit       Stab 3 )
%
% outputs:  y1(1..3)...q      ( generalisierte Koordinaten )
%           y2(1..3)...qp     ( generalisierte Geschwindigkeiten )
%           y3.........det(D) ( Determinante der Massenmatrix )
%
% parameters:
%           p(1)... m1        ( Masse Stab 1 )
%           p(2)... m2        ( Masse Stab 2 )
%           p(3)... m3        ( Masse Stab 3 )
%           p(4)... mL        ( Last )
%           p(5)... l1        ( Länge Stab 1 )
%           p(6)... l2        ( Länge Stab 2 )
%           p(7)... l3        ( Länge Stab 3 )
%           p(8)... Ixx1      ( Trägheitsmoment x Stab 1 )
%           p(9)... Iyy1      ( Trägheitsmoment y Stab 1 )
%           p(10)...Izz1      ( Trägheitsmoment z Stab 1 )
%           p(11)...Ixx2      ( Trägheitsmoment x Stab 2 )
%           p(12)...Iyy2      ( Trägheitsmoment y Stab 2 )
%           p(13)...Izz2      ( Trägheitsmoment z Stab 2 )
%           p(14)...Ixx3      ( Trägheitsmoment x Stab 3 )
%           p(15)...Iyy3      ( Trägheitsmoment y Stab 3 )
%           p(16)...Izz3      ( Trägheitsmoment z Stab 3 )
%           p(17)...d1        ( Dämpferkonstante Stab 1 )
%           p(18)...d2        ( Dämpferkonstante Stab 2 )
%           p(19)...d3        ( Dämpferkonstante Stab 3 )
%           p(20)...phi10     ( Anfangswinkel Stab 1 )  
%           p(21)...phi20     ( Anfangswinkel Stab 2 )  
%           p(22)...s0        ( Anfangsweg    Stab 3 )
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Sample Time: Continuous
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% created:  Kam, 29.10.2008
% changed:  Kam, 09.11.2009, y3 erstellt 
%           BM, 10.10.2011: Löschen der Berechnung der Systemmatrizen und
%           Beschleunigungen in mdlOutput, da sie nicht benötigt werden
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%           

setup(block);


function setup(block)
  
  % Register number of input and output ports
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 3;
  
  % Register number of continuous states
  block.NumContStates = 6;
  
  % Register dialog parameter
  block.NumDialogPrms = 22; 
  
  % Port dimensions
  block.InputPort(1).Dimensions        = 3;
  block.InputPort(1).SamplingMode = 'Sample';
  block.InputPort(1).DirectFeedthrough = false;
  
  block.OutputPort(1).Dimensions       = 3;
  block.OutputPort(1).SamplingMode = 'Sample';  
  block.OutputPort(2).Dimensions       = 3;
  block.OutputPort(2).SamplingMode = 'Sample';  
  block.OutputPort(3).Dimensions       = 1;
  block.OutputPort(3).SamplingMode = 'Sample';  
    
  % Set block sample time to continuous time
  block.SampleTimes = [0 0];
  
  % Register methods
  block.RegBlockMethod('InitializeConditions',    @InitConditions); 
  block.RegBlockMethod('Outputs',                 @Output);  
  block.RegBlockMethod('Derivatives',             @Derivatives);
  block.RegBlockMethod('Terminate',               @Terminate);


function InitConditions(block)
  phi10 = block.DialogPrm(20).Data;  
  phi20 = block.DialogPrm(21).Data;  
  s0    = block.DialogPrm(22).Data;  
  
  x0(1) = phi10;
  x0(2) = phi20;
  x0(3) = s0;
  x0(4) = 0;
  x0(5) = 0;
  x0(6) = 0;  
  
  block.ContStates.Data=x0;
  
function Output(block)
  x = block.ContStates.Data;
  u1 = block.InputPort(1).Data;
 
  % Variablen definieren für bessere Code-Lesbarkeit
  M1   = u1(1);
  M2   = u1(2);
  F   =  u1(3);

  phi1  = x(1);
  phi2  = x(2);
  s     = x(3);
  phi1p = x(4);
  phi2p = x(5);
  sp    = x(6);  
    
  m1  = block.DialogPrm(1).Data;
  m2  = block.DialogPrm(2).Data;
  m3  = block.DialogPrm(3).Data;
  mL  = block.DialogPrm(4).Data;
  l1  = block.DialogPrm(5).Data;  
  l2  = block.DialogPrm(6).Data;  
  l3  = block.DialogPrm(7).Data;  
  Ixx1 = block.DialogPrm(8).Data;  
  Iyy1 = block.DialogPrm(9).Data;    
  Izz1 = block.DialogPrm(10).Data;  
  Ixx2 = block.DialogPrm(11).Data;    
  Iyy2 = block.DialogPrm(12).Data;    
  Izz2 = block.DialogPrm(13).Data;    
  Ixx3 = block.DialogPrm(14).Data;    
  Iyy3 = block.DialogPrm(15).Data;    
  Izz3 = block.DialogPrm(16).Data;    
  d1   = block.DialogPrm(17).Data;    
  d2   = block.DialogPrm(18).Data;    
  d3   = block.DialogPrm(19).Data;    
  
  g = 9.81;
  
  % Massenmatrix:
%   DD = [m2 * cos(phi2) ^ 2 * l2 ^ 2 / 0.4e1 + cos(phi2) ^ 2 * m3 * l2 ^ 2 + cos(phi2) ^ 2 * mL * l2 ^ 2 - cos(phi2) ^ 2 * m3 * l2 * l3 + 0.2e1 * cos(phi2) ^ 2 * m3 * l2 * s + 0.2e1 * cos(phi2) ^ 2 * l2 * mL * s + cos(phi2) ^ 2 * m3 * l3 ^ 2 / 0.4e1 - cos(phi2) ^ 2 * m3 * s * l3 + cos(phi2) ^ 2 * m3 * s ^ 2 - Iyy3 * cos(phi2) ^ 2 + cos(phi2) ^ 2 * mL * s ^ 2 + cos(phi2) ^ 2 * Izz2 - Iyy2 * cos(phi2) ^ 2 + cos(phi2) ^ 2 * Izz3 + Iyy2 + Iyy3 + Izz1 0 0; 0 m2 * l2 ^ 2 / 0.4e1 + m3 * l2 ^ 2 + 0.2e1 * m3 * l2 * s - m3 * l2 * l3 + m3 * s ^ 2 - m3 * s * l3 + m3 * l3 ^ 2 / 0.4e1 + mL * l2 ^ 2 + 0.2e1 * l2 * mL * s + mL * s ^ 2 + Ixx2 + Ixx3 0; 0 0 m3 + mL;];
  DDdet = (((l2 + s - l3 / 0.2e1) ^ 2 * m3 + (m2 / 0.4e1 + mL) * l2 ^ 2 + 0.2e1 * mL * l2 * s - Iyy3 + mL * s ^ 2 + Izz2 - Iyy2 + Izz3) * cos(phi2) ^ 2 + Iyy2 + Iyy3 + Izz1) * ((l2 + s - l3 / 0.2e1) ^ 2 * m3 + (m2 / 0.4e1 + mL) * l2 ^ 2 + 0.2e1 * mL * l2 * s + mL * s ^ 2 + Ixx2 + Ixx3) * (m3 + mL);

  
  % Ausgangsgrößen:
  y1(1) = phi1;
  y1(2) = phi2;
  y1(3) = s;

  y2(1) = phi1p;
  y2(2) = phi2p;
  y2(3) = sp;  
  
  y3(1) = DDdet;  
  
  block.OutputPort(1).Data = y1;
  block.OutputPort(2).Data = y2;
  block.OutputPort(3).Data = y3;
  



function Derivatives(block)

  x = block.ContStates.Data;
  u1 = block.InputPort(1).Data;

  % Variablen definieren für bessrer Code-Lesbarkeit
  M1   = u1(1);
  M2   = u1(2);
  F   =  u1(3);
   
  phi1  = x(1);
  phi2  = x(2);
  s     = x(3);
  phi1p = x(4);
  phi2p = x(5);
  sp    = x(6);  
  
  m1  = block.DialogPrm(1).Data;
  m2  = block.DialogPrm(2).Data;
  m3  = block.DialogPrm(3).Data;
  mL  = block.DialogPrm(4).Data;
  l1  = block.DialogPrm(5).Data;  
  l2  = block.DialogPrm(6).Data;  
  l3  = block.DialogPrm(7).Data;  
  Ixx1 = block.DialogPrm(8).Data;  
  Iyy1 = block.DialogPrm(9).Data;    
  Izz1 = block.DialogPrm(10).Data;  
  Ixx2 = block.DialogPrm(11).Data;    
  Iyy2 = block.DialogPrm(12).Data;    
  Izz2 = block.DialogPrm(13).Data;    
  Ixx3 = block.DialogPrm(14).Data;    
  Iyy3 = block.DialogPrm(15).Data;    
  Izz3 = block.DialogPrm(16).Data;    
  d1   = block.DialogPrm(17).Data;    
  d2   = block.DialogPrm(18).Data;    
  d3   = block.DialogPrm(19).Data;    
  
  g = 9.81;
  
  % Massenmatrix:
  DD = [m2 * cos(phi2) ^ 2 * l2 ^ 2 / 0.4e1 + cos(phi2) ^ 2 * m3 * l2 ^ 2 + cos(phi2) ^ 2 * mL * l2 ^ 2 - cos(phi2) ^ 2 * m3 * l2 * l3 + 0.2e1 * cos(phi2) ^ 2 * m3 * l2 * s + 0.2e1 * cos(phi2) ^ 2 * l2 * mL * s + cos(phi2) ^ 2 * m3 * l3 ^ 2 / 0.4e1 - cos(phi2) ^ 2 * m3 * s * l3 + cos(phi2) ^ 2 * m3 * s ^ 2 - Iyy3 * cos(phi2) ^ 2 + cos(phi2) ^ 2 * mL * s ^ 2 + cos(phi2) ^ 2 * Izz2 - Iyy2 * cos(phi2) ^ 2 + cos(phi2) ^ 2 * Izz3 + Iyy2 + Iyy3 + Izz1 0 0; 0 m2 * l2 ^ 2 / 0.4e1 + m3 * l2 ^ 2 + 0.2e1 * m3 * l2 * s - m3 * l2 * l3 + m3 * s ^ 2 - m3 * s * l3 + m3 * l3 ^ 2 / 0.4e1 + mL * l2 ^ 2 + 0.2e1 * l2 * mL * s + mL * s ^ 2 + Ixx2 + Ixx3 0; 0 0 m3 + mL;];
  
  % Matrix der Zentrifugal- und Coriolisterme:
  CC = [cos(phi2) * (-phi2p * sin(phi2) * m2 * l2 ^ 2 - 0.4e1 * phi2p * sin(phi2) * m3 * l2 ^ 2 - 0.4e1 * phi2p * sin(phi2) * mL * l2 ^ 2 + 0.4e1 * phi2p * sin(phi2) * m3 * l2 * l3 - 0.8e1 * phi2p * sin(phi2) * m3 * l2 * s - 0.8e1 * phi2p * sin(phi2) * l2 * mL * s - phi2p * sin(phi2) * m3 * l3 ^ 2 + 0.4e1 * phi2p * sin(phi2) * m3 * s * l3 - 0.4e1 * phi2p * sin(phi2) * m3 * s ^ 2 - 0.4e1 * phi2p * sin(phi2) * Izz3 + 0.4e1 * phi2p * sin(phi2) * Iyy3 - 0.4e1 * phi2p * sin(phi2) * Izz2 + 0.4e1 * phi2p * sin(phi2) * Iyy2 - 0.4e1 * phi2p * sin(phi2) * mL * s ^ 2 + 0.4e1 * sp * cos(phi2) * m3 * s - 0.2e1 * sp * cos(phi2) * m3 * l3 + 0.4e1 * sp * cos(phi2) * m3 * l2 + 0.4e1 * sp * cos(phi2) * mL * l2 + 0.4e1 * sp * cos(phi2) * mL * s) / 0.4e1 (-m2 * l2 ^ 2 - 0.4e1 * m3 * l2 ^ 2 - 0.4e1 * mL * l2 ^ 2 + 0.4e1 * m3 * l2 * l3 - 0.8e1 * m3 * l2 * s - 0.8e1 * l2 * mL * s - m3 * l3 ^ 2 + 0.4e1 * m3 * s * l3 - 0.4e1 * m3 * s ^ 2 - 0.4e1 * Izz3 + 0.4e1 * Iyy3 - 0.4e1 * Izz2 + 0.4e1 * Iyy2 - 0.4e1 * mL * s ^ 2) * phi1p * sin(phi2) * cos(phi2) / 0.4e1 cos(phi2) ^ 2 * phi1p * (0.2e1 * m3 * s - m3 * l3 + 0.2e1 * m3 * l2 + 0.2e1 * mL * l2 + 0.2e1 * mL * s) / 0.2e1; -(-m2 * l2 ^ 2 - 0.4e1 * m3 * l2 ^ 2 - 0.4e1 * mL * l2 ^ 2 + 0.4e1 * m3 * l2 * l3 - 0.8e1 * m3 * l2 * s - 0.8e1 * l2 * mL * s - m3 * l3 ^ 2 + 0.4e1 * m3 * s * l3 - 0.4e1 * m3 * s ^ 2 - 0.4e1 * Izz3 + 0.4e1 * Iyy3 - 0.4e1 * Izz2 + 0.4e1 * Iyy2 - 0.4e1 * mL * s ^ 2) * phi1p * sin(phi2) * cos(phi2) / 0.4e1 (0.2e1 * m3 * s - m3 * l3 + 0.2e1 * m3 * l2 + 0.2e1 * mL * l2 + 0.2e1 * mL * s) * sp / 0.2e1 phi2p * (0.2e1 * m3 * s - m3 * l3 + 0.2e1 * m3 * l2 + 0.2e1 * mL * l2 + 0.2e1 * mL * s) / 0.2e1; -cos(phi2) ^ 2 * phi1p * (0.2e1 * m3 * s - m3 * l3 + 0.2e1 * m3 * l2 + 0.2e1 * mL * l2 + 0.2e1 * mL * s) / 0.2e1 -phi2p * (0.2e1 * m3 * s - m3 * l3 + 0.2e1 * m3 * l2 + 0.2e1 * mL * l2 + 0.2e1 * mL * s) / 0.2e1 0;];

  % Gravitationsvektor:
  gg = [0 g * (2 * m3 * s - m3 * l3 + 2 * m3 * l2 + l2 * m2 + 2 * mL * l2 + 2 * mL * s) * cos(phi2) / 0.2e1 g * (m3 + mL) * sin(phi2)]';

  % Rayleigscher Dissipationsvektor:
  rr = [d1 * phi1p d2 * phi2p d3 * sp]';
  
  
  % Bewegungsgleichungen:
  qqp=[phi1p,phi2p,sp]';
  ttau = [M1 M2 F]';
%   qqpp=inv(DD)*(-CC*qqp - gg - rr + ttau);
  qqpp=DD\(-CC*qqp - gg - rr + ttau);

  % Differentialgleichungen:
  dx(1) = phi1p;
  dx(2) = phi2p;
  dx(3) = sp;
  dx(4) = qqpp(1);
  dx(5) = qqpp(2);
  dx(6) = qqpp(3);
   
  block.Derivatives.Data=dx;          

function Terminate(block)

