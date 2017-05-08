% *************************** %
% 4. Ground Interaction Model %
% *************************** %

% ----------------------
% 4.1 Vertical component
% ----------------------

m = 3.2;
g = 9.81;
% stiffness of vertical ground interaction
k_gn = m*g/0.001; %[N/m]

% max relaxation speed of vertical ground interaction
v_gn_max = 0.05; %[m/s]

% ------------------------
% 4.2 Horizontal component
% ------------------------

% sliding friction coefficient
mu_slide = 0.8; %0.8

% sliding to stiction transition velocity limit
vLimit = 0.05; %[m/s] 0.01

% stiffness of horizontal ground stiction
k_gt = k_gn; %(2*(m_F + m_S + m_T) + m_HAT)*g/0.1; %[N/m] 0.01

% max relaxation speed of horizontal ground stiction
v_gt_max = 0.03; %[m/s] 0.03

% stiction to sliding transition coefficient
mu_stick = 0.9;

