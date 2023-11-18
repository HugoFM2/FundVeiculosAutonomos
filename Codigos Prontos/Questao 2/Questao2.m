clc; close all; clear all;
% Variaveis gerais
table = readtable('Q2.csv');

% Levando o sistema para zero
ts = table.ts - 1.7;
u = table.us;
% u(ts > 0) = 0.2;
% u(ts > 5-1.7) = 0;

% Grafico de velocidade levando a 0
close all;
plot(ts,table.vs,LineWidth=2,Color='blue',DisplayName="Resposta do Sistema");hold on;
plot(ts,u,DisplayName="Degrau=0.2");hold on;
xlabel("Tempo [s]")
ylabel('Velocidade [m/s]')
ylim([0,1.7])
xlim([-0.2,1.5])
title("Tempo x Velocidade") 

% Plota Linha Horizontal
linha_horiz = 0;
line([-5, 20], [linha_horiz, linha_horiz] ,'Color','red','LineStyle','--','HandleVisibility','off');

% Plota Linha Vertical
line([linha_horiz, linha_horiz], [-5, 20] ,'Color','red','LineStyle','--','HandleVisibility','off');


%% Estimando a inclinacao - Parte 2
% Parte cortada
inicio = 70
fim = 250
corte_ts = ts(inicio:fim)
corte_vs = table.vs(inicio:fim)




% Based on https://www.mathworks.com/matlabcentral/answers/89306-tangent-line-to-a-curve-at-a-given-point
dy = diff(corte_vs)./diff(corte_ts);
k = 100;
tang=(ts-ts(k))*dy(k)+table.vs(k)

plot(ts,tang,LineWidth=2,LineStyle="--",DisplayName="Reta Tangente")
legend

%%

tau = 0.05;
K = 6;
G = tf(K,[tau, 1, 0])

opt = stepDataOptions('StepAmplitude',0.25);
[step_x,step_y] = step(G,8,opt);

plot(step_x,step_y,LineWidth=2,LineStyle="--",Color='red',DisplayName="Modelo Estimado")
legend
