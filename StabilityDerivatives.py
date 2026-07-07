import numpy
import matplotlib

# Define variables

# Equilibrium conditions

alpha_e = 0 # Equilibrium angle of attack
V_e = 100 # Equilibrium velocity: meters/second
rho_e = 101325 # Air density equilibrium conditions: pascals
g = 9.81

# Derivatives

V_U = 1
V_W = 0
A_U = 0
A_W = 1 / V_e

CL_V = 0 # Partial lift coefficient/ partial velocity
CD_V = 0 # Partial drag coefficient/ partial velocity

T_V = 0 # Partial thrust/ partial velocity
T_A = 0 # Partial thrust/ partial attitude

CM0_V = 0
CMH_A = 0
Kn_V = 0
Kn_A = 0 # 0 for non-stalled state

Epsilon_alpha = 0

# Aircraft parameters

alpha = 0 # Angle of attack
Sref = 100 # Reference area: meters * meters
epsilon = 0 # Engine setting angle
a = 0 # Lift curve slope
d = 0 # Drag curve slope
m = 1000 # Gross mass
T = 1000 # Thrust
Gamma = 0 # Dihederal
Lambda = 0 # Sweep
b = 10 # Span

aH = 0 # Lift curve slope
dH = 0 # Drag curve slope
SH = 100 # Hstab area
etaH = 1 # Hstab efficiency factor


Zt = 0 # Engine vertical position
Zcg = 0 # Cg vertical position
Zac = 0 # Aerodynamic center
ZH = 0 # Hstab position

Xnp = 0 # Neutral point
Xcg = 0 # Cg horizontal position
Xt = 0 # Engine horizontal position
Xac = 0 # Aerodynamic center
XH = 0 # Hstab position

cR = 1 # Mean aerodynamic chord length
cRH = 1 # mean aerodynamic chord length hstab

CM0 = 0 # Zero lift pitching moment coefficient

# Pre calc

A = alpha - alpha_e # Attitude
CL_A = a
CD_A = d

CLH_A = aH
CDH_A = dH

Lt = (Xcg - Xt) * numpy.sin(epsilon + alpha_e) + (Zcg - Zt) * numpy.cos(epsilon + alpha_e)
LH = XH - Xcg
LpH = XH - Xac
hH = ZH - Zcg

Kn = (Xnp - Xcg) / cR

CLe = (m * g) / (0.5 * rho_e * (V_e ** 2))
CDe = T / (0.5 * rho_e * (V_e ** 2))

CLHe = 0

CL = CLe # Lift coefficient
CD = CDe # Drag coefficient

U_Q = -(Zac - Zcg)
W_Q = (Xac - Xcg)

CLH = CLHe
CDH = 0

s = b/2 # Half span

# Wdot stuff

WH_EpsilonH = -V_e
EpsilonH_Wdot = -LH/(V_e^2) * Epsilon_alpha

# Longitudinal derivatives

# X_U
ThrustXV = T_V * numpy.cos(alpha_e + epsilon)
AeroXV = 0.5 * rho_e * (V_e ** 2) * Sref * (CL_V * numpy.sin(A) - CD_V * numpy.cos(A)) + rho_e * V_e * Sref * (CL * numpy.sin(A) - CD * numpy.cos(A))
X_V = ThrustXV + AeroXV

X_U = X_V * V_U # + X_A * A_U

# X_W
ThrustXA = T_A * numpy.cos(alpha_e + epsilon)
AeroXA = 0.5 * rho_e * (V_e ** 2) * Sref * (CL_A * numpy.sin(A) - CD_A * numpy.cos(A) + CL * numpy.cos(A) - CD * numpy.sin(A))
X_A = ThrustXA + AeroXA

X_W = X_A * A_W # + X_V * V_W

# Z_U

ThrustZV = -T_V * numpy.sin(alpha_e + epsilon)
AeroZV = -0.5 * rho_e * (V_e ** 2) * Sref * (CL_V * numpy.cos(A) + CD_V * numpy.sin(A)) - rho_e * V_e * Sref * (CL * numpy.cos(A) + CD * numpy.sin(A))
Z_V = ThrustZV + AeroZV

Z_U = Z_V * V_U # + Z_A * A_U

# Z_W

ThrustZA = -T_A * numpy.sin(alpha_e + epsilon)
AeroZA = -0.5 * rho_e * (V_e ** 2) * Sref * (CL_A * numpy.cos(A) + CD_A * numpy.sin(A) - CL * numpy.sin(A) + CD * numpy.cos(A))
Z_A = ThrustZA + AeroZA

Z_W = Z_A * A_W # + Z_V + V_w

# M_U
ThrustMV = Lt * T_V
AeroMV = rho_e * V_e * Sref * cR * (CM0 - Kn * CL) + (0.5 * rho_e * (V_e ** 2) * Sref * cR) * (CM0_V - Kn_V * CL - Kn * CL_V)
M_V = ThrustMV + AeroMV

M_U = M_V * V_U # + M_A * A_U

# M_W
ThrustMA = Lt * T_V
AeroMA = -0.5 * rho_e * V_e * Sref * cR * (Kn_A * CL + CL_A + Kn)
M_A = ThrustMA + AeroMA

M_W = M_A * A_W # + M_V * V_W

# X_QH
X_WH = 0.5 * rho_e * etaH * V_e * SH * (-CDH_A + CLH)

X_QH = X_WH * W_Q # + X_UH * U_Q

# Z_QH
Z_WH= -0.5 * rho_e * etaH * V_e * SH * (CLH_A + CDH)

Z_QH = Z_WH * W_Q # + X_UH * U_Q

# M_QH
M_WH = 0.5 * rho_e * etaH * V_e * SH * cRH * CMH_A

M_QH = M_WH * W_Q # + M_UH * U_Q

# X_Wdot
X_Wdot = X_WH * WH_EpsilonH * EpsilonH_Wdot

# Z_Wdot
Z_Wdot = Z_WH * WH_EpsilonH * EpsilonH_Wdot

# M_Wdot
M_Wdot = -hH * X_Wdot + LpH * Z_Wdot + (LH/V_e) * Epsilon_alpha * M_WH

# Lateral derivatives

# L_V

L_V = - (Gamma * CL_A * numpy.cos(Lambda) + CL * numpy.tan(Lambda)) * (numpy.cos(Lambda)/(Sref * s))
# N_V

# Y_V

# L_P

# N_P

# Y_P

# L_r

# N_r

# Y_r

