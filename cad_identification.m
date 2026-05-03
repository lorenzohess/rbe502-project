%% Build parameter vector from URDF/CAD values (no identification)
clc; clear; close all;

% Geometry
a0 = 0.012; d1 = 0.0595; a1 = 0.024;
d2 = 0.128; a2 = 0.124; ell_e = 0.126;

% Masses [kg]
m1 = 0.0791;
m2 = 0.0984;
m3 = 0.1386;
m4 = 0.1326;

% Inertias [kg*m^2]
Ixx1 = 1.2505e-05; Iyy1 = 2.1898e-05; Izz1 = 1.9267e-05;
Ixx2 = 3.4543e-05; Iyy2 = 3.2689e-05; Izz2 = 1.8850e-05;
Ixx3 = 3.3055e-04; Iyy3 = 3.4290e-04; Izz3 = 6.0346e-05;
Ixx4 = 3.0654e-05; Iyy4 = 2.4230e-04; Izz4 = 2.5155e-04;

g = 9.81;

% Build p in the order expected by M_fun, C_fun, G_fun
p = [a0; d1; a1; d2; a2; ell_e; ...
     m1; m2; m3; m4; ...
     Ixx1; Iyy1; Izz1; ...
     Ixx2; Iyy2; Izz2; ...
     Ixx3; Iyy3; Izz3; ...
     Ixx4; Iyy4; Izz4; ...
     g];

% Friction estimate
pf = [0.05; 0.10; 0.05; 0.02];

% Save in a format compatible with the validation script
% The validator unpacks: x_opt_vec(1:16) = inertials, x_opt_vec(17:20) = friction
x_opt_vec = [m1; m2; m3; m4; ...
             Ixx1; Iyy1; Izz1; ...
             Ixx2; Iyy2; Izz2; ...
             Ixx3; Iyy3; Izz3; ...
             Ixx4; Iyy4; Izz4; ...
             pf];

id_info = struct('g', g, 'source', 'URDF/CAD model — no identification');
x0 = x_opt_vec;       % no optimization, "initial" = "final"
id_opts = struct();

save('identification_result.mat', 'x_opt_vec', 'id_info', 'x0', 'id_opts', 'p');

disp('Saved CAD-based parameters to identification_result.mat')
disp('Now run validate_identified_model.m to see how well the URDF model predicts torque.')
