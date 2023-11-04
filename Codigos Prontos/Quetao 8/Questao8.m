clc; close all; clear all;


% Variaveis gerais
table = readtable('Q8.csv');


%% Grafico de Velocidade

plot(table.ts,table.vel,LineWidth=2,Color='k')
xlabel("Tempo [s]")
ylabel('Velocidade [m/s]')
ylim([0,2.2])
title("Tempo x Velocidade")

%% Grafico controle lateral
plot(table.ts,table.x,LineWidth=2,Color='k')
xlabel("Tempo [s]")
ylabel('Posição em x [m]')
title("Tempo x Posição")