clc; close all; clear all;


% Variaveis gerais
table = readtable('Q10.csv');


%% Grafico de Velocidade

plot(table.ts,table.vel,LineWidth=2,Color='k')
xlabel("Tempo [s]")
ylabel('Velocidade [m/s]')
ylim([0,1.7])
title("Tempo x Velocidade")