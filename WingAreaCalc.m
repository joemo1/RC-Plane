% Approximate wing area calculator

% Masses

Motor = 0.062;
Battery = 0.162;
ESC = 0.031;
AB = 0.02*8;
Reciever = 0.02;
Servo = 0.009*4;
F_D = 0.069;
WingPLA = 0.005*20;
WingMount = 0.025;
Oracover = 0.096;
HstabPLA = 0.005*10;
VstabPLA = 0.005*5;
W_D = 0.049*2;
H_D = 0.04;
extra = 0.026;

Avionics = sum([Motor, Battery, ESC, AB, Reciever]);
Wing = sum([WingPLA, WingMount, Oracover, Servo/2, W_D]);
Emp = sum([HstabPLA VstabPLA, H_D, Servo/2]);

M_total = Avionics + Wing + Emp + F_D + extra;

M = M_total;
g = 9.81; 
W = M*g;

Cl = 0.8; % Deisgn lift coefficient
rho = 1.225; % Air density
V = 5:0.1:15; % Cruise velocity

L = W;
AR = 10; % Aspect ratio

S = L./(0.5.*rho.*(V.^2).*Cl);

span = (AR.*S).^0.5;

mean_chord = S./span; 


yyaxis left
plot(V, span, V, mean_chord);
ylabel('Span & Chord (m)');
grid on;

yyaxis right
plot(V, S);
ylabel('Wing Area S (m^2)');


xlabel('Velocity V (m/s)');
title('Wing Geometry v Cruise Velocity');
legend('Wingspan (m)', 'Mean Chord (m)', 'Wing Area (m^2)', 'Location', 'best');