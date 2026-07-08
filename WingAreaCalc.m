% Basic aircraft geometry calculator
% Current data for E392 and NACA 0010
% Assume wing/hstab center of gravity lies on aerodynamic center

% Setup and Constants
x_full = 0.205 + 0.915;
g = 9.81; 
rho = 1.225; % Air density
Cl_w = 0.8;  % Design lift coefficient
AR_w = 10;   % Aspect ratio
V_h = 0.5; % Horizontal stabiliser volume coefficient
x_h = 1.07;  
AR_h = 4; % Horizontal stabiliser aspect ratio
Cm0_w = -0.105; % Wing Aerofoil zero pitching moment coefficient
Cm0_h = 0; % Hstab Aerofoil zero pitching moment coefficient
Cm = -0.1; % Pitching moment coefficient
V_v = 0.03; % Vertical stabiliser volume coefficient
AR_v = 1.8; % Vertical stabiliser aspect ratio

% Masses and Fixed Positions
% Fuselage Matrix: [Mass (kg), X position (m)]
Motor    = [0.062, 0.01]; 
Battery  = [0.162, 0.145]; 
ESC      = [0.031, 0.085]; 
AB       = [0.16,  0.145];
Reciever = [0.02,  0.205]; 
F_D      = [0.069, 0.6625]; 
Fuselage_matrix = [Motor; Battery; ESC; AB; Reciever; F_D];

Fuselage_X = Fuselage_matrix(:, 2); 
Fuselage_M = Fuselage_matrix(:, 1); 
Fuselage_mass_total = sum(Fuselage_M);
Fuselage_CG = sum(Fuselage_M .* Fuselage_X) / Fuselage_mass_total;

% Empennage Matrix: [Mass (kg), X position (m)]
HstabMount = [0.025,    x_h]; 
Servo_h    = [0.009*2,    x_h]; 
HstabPLA   = [0.005*10, x_h]; 
VstabPLA   = [0.005*5,  x_h]; 
H_D        = [0.04,     x_h]; 
Emp_matrix = [Servo_h; HstabMount; HstabPLA; VstabPLA; H_D];

Emp_X = Emp_matrix(:, 2);
Emp_M = Emp_matrix(:, 1);
Emp_mass_total = sum(Emp_M);
Emp_CG = sum(Emp_M .* Emp_X) / Emp_mass_total;

% Wing mass properties independent of position
Servo_w = 0.009*2; 
WingPLA = 0.005*20; 
WingMount = 0.025; 
W_D = 0.049*2; 
Wing_mass_total = sum([Servo_w, WingPLA, WingMount, W_D]);

V_list = 5:0.1:15; 

M_est = Fuselage_mass_total + Emp_mass_total + Wing_mass_total; 
W_est = M_est * g;
S_w_list = W_est ./ (0.5 .* rho .* (V_list.^2) .* Cl_w);
span_list = (AR_w .* S_w_list).^0.5;
mean_chord_list = S_w_list ./ span_list; 

% Display Cruise Velocity Chart
figure(1)

yyaxis left
plot(V_list, span_list, V_list, mean_chord_list, 'LineWidth', 1.5);
ylabel('Span & Chord (m)');
grid on;

yyaxis right
plot(V_list, S_w_list, 'LineWidth', 1.5);
ylabel('Wing Area S (m^2)');
xlabel('Velocity V (m/s)');
title('Wing Geometry vs Cruise Velocity');
legend('Wingspan (m)', 'Mean Chord (m)', 'Wing Area (m^2)', 'Location', 'best');

% Select velocity
V = input('Select cruise velocity: ');

% Get specific chord for chosen velocity
[~, idx] = min(abs(V_list - V));
c = mean_chord_list(idx);
S_w = S_w_list(idx);
span = span_list(idx);

% Combined mass without the wing
M_NoWing = Fuselage_mass_total + Emp_mass_total;
X_NoWing = (sum(Fuselage_M .* Fuselage_X) + sum(Emp_M .* Emp_X)) / M_NoWing;

x_w = X_NoWing + 0.22*c; % Edit multiple of chord to change wing position and static margin
Wing_CG = x_w;

Total_CG = (Fuselage_CG * Fuselage_mass_total + Wing_CG * Wing_mass_total + Emp_CG * Emp_mass_total)/(M_NoWing + Wing_mass_total);
x_CG = Total_CG;

Kn = (x_w - x_CG)/c;

figure(2)
clf;

% Plot Fuselage Components
stem(Fuselage_X, Fuselage_M, 'filled', 'b', 'LineWidth', 1.5, 'DisplayName', 'Fuselage Components');
hold on;

% Plot Tail Components
stem(Emp_X, Emp_M, 'filled', 'g', 'LineWidth', 1.5, 'DisplayName', 'Tail Components');

% Plot Wing Group Center of Mass
stem(Wing_CG, Wing_mass_total, 'filled', 'k', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'Wing Assembly Mass');

% Define Wing Leading Edge so the patch and report know where it starts
X_wing_LE = Wing_CG - 0.25*c; 

% Draw a visual patch representing the wing chord location vertically
y_limits = [0, 0.25]; % Extends up to match your max mass axis limit
patch([X_wing_LE, X_wing_LE+c, X_wing_LE+c, X_wing_LE], ...
      [y_limits(1), y_limits(1), y_limits(2), y_limits(2)], ...
      'cyan', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'DisplayName', 'Wing Chord Footprint');

% Line showing Total CG
xline(Total_CG, '--r', 'LineWidth', 2, 'DisplayName', sprintf('Total CG (%.3f m)', Total_CG));

% Formatting
xlim([0, x_full + 0.1]);
xlabel('X Position from Nose (m)');
ylabel('Mass (kg)');
title(sprintf('Aircraft Configuration Map (V = %.1f m/s)', V));
grid on;
legend('Location', 'northeast');

% Print terminal report
fprintf('\n--- Performance & Placement Design Specifications ---\n');
fprintf('Required Wing Area: %.3f m^2\n', S_w);
fprintf('Required Wingspan:  %.3f m\n', span);
fprintf('Mean Chord Size:    %.3f m\n', c);
fprintf('MOUNT WING LEADING EDGE AT X = %.3f m\n', X_wing_LE);
fprintf('Target Balanced Aircraft CG at X = %.3f m\n', Total_CG);
fprintf('Static margin Kn = %.3f \n', Kn)

% Calculate Horizontal Stabiliser Properties

L_h = x_h - x_w;
S_h = (V_h*S_w*c)/L_h;

span_h = (AR_h * S_h)^0.5;
c_h = S_h / span_h;

fprintf('\n--- Horizontal Stabilizer Physical Dimensions ---\n');
fprintf('Tail Volume Coeff (Vh): %.2f\n', V_h);
fprintf('Calculated Tail Arm (L_h):     %.3f m\n', L_h);
fprintf('Required H-Stab Area (S_h):    %.4f m^2\n', S_h);
fprintf('H-Stab Wingspan:              %.3f m\n', span_h);
fprintf('H-Stab Chord:                 %.3f m\n', c_h);

Cl_h = (Cm - Cm0_w - (x_CG - x_w)*Cl_w - ((S_h*c_h)/(S_w))*Cm0_h)*((S_w*c)/(S_h*(x_CG - x_h)));

fprintf('Horizontal stabiliser lift coefficient: %.3f\n', Cl_h);
fprintf('Consult Airfoiltools.com to achieve required hstab setting angle');
alpha_h = input('Input hstab setting angle in degrees: ');

% Calculate Vertical Stabiliser Properties

L_v = x_h - x_w;
S_v = (V_v * S_w * span) / L_v;
   

% Print V-Stab Sizing Report
fprintf('Target Vertical Tail Coeff (Vv): %.3f\n', V_v);
fprintf('Required V-Stab Area (S_v):     %.4f m^2\n', S_v);
