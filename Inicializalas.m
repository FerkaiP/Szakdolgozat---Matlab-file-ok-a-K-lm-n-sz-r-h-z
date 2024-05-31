% Dynamic vehicle model in terms of road-error variables (e1, e2)

%% Paraméterek:

v = 2;
%R = 6;              % az első 2 teszteléshez szükséges
m = 2300;
Iz = 2873;
Cf = 222685.8/2;
Cr = 136242.8/2;
lf = 1236e-3;
lr = 2789e-3 - lf;

% Sampling time
Ts = 0.033; % [s] 

%% Mátrixok:

A = [0  1                           0                       0
     0  -(2*Cf+2*Cr)/(m*v)          (2*Cf+2*Cr)/m           -(2*Cf*lf+2*Cr*lr)/(m*v)
     0  0                           0                       1
     0  -(2*Cf*lf-2*Cr*lr)/(Iz*v)   (2*Cf*lf-2*Cr*lr)/Iz    -(2*Cf*lf^2+2*Cr*lr^2)/(Iz*v)];

B1 = [0; 2*Cf/m; 0; 2*Cf*lf/Iz];

B2 = [0; -(2*Cf*lf-2*Cr*lr)/(m*v)-v; 0; -(2*Cf*lf^2+2*Cr*lr^2)/(Iz*v)];

B = [0 0; 2*Cf/m -(2*Cf*lf-2*Cr*lr)/(m*v)-v; 0 0; 2*Cf*lf/Iz -(2*Cf*lf^2+2*Cr*lr^2)/(Iz*v)];

C = [1 0 0 0
     0 0 1 0];

%% Countinuous to Discrete

sys = ss(A, B, C, 0);
sysd = c2d(sys, Ts);
Ad = get(sysd, 'A'); 
Bd = get(sysd, 'B'); 

B1d = Bd(1:4,1);
B2d = Bd(1:4,2);

%% Kalman filter:

% Process noise covariance
Q1 = 1e-3; 
Q2 = 1e-3;
Qd = [Ts^3*Q1/3 Ts^2*Q1/2   0           0
      Ts^2*Q1/2 Ts*Q1       0           0
      0         0           Ts^3*Q2/3   Ts^2*Q2/2
      0         0           Ts^2*Q2/2   Ts*Q2];

% Measurement noise covariance
R1 = 1e-2; 
R2 = deg2rad(1)^2; 
Rd = [R1 0
      0 R2];  
