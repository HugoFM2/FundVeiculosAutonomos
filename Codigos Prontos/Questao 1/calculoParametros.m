clear all;
% Variaveis gerais
B_table = readtable('Dados_Beta.csv');

m = 6.3;
beta = 0.8;
V0 = 4.89;
T = B_table.ts(end) - 8;
rho = 1.225;

%% Calcula Cd e RX

Af = 1.6+0.00056*(m-765);

num = 2*m*beta*atan(beta);
den = V0*T*rho*Af;

Cd = num/den


% Calcula rx

num = V0*m*atan(beta);
den = beta*T;

Rx = num/den




