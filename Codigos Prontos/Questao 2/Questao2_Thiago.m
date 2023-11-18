clc; close all; clear all;


% Variaveis gerais
table = readtable('Q2_Thiago.csv');

inicio_degrau = 1.7

% Constroi sinal de controle
u = zeros(length(table.ts),1);
u(table.ts >= inicio_degrau) = 0.2;
u(table.ts > 10) = 0;


%% Grafico de Velocidade
close all;
% plot(table.ts,0.25*ones(800));
% plot(table.ts(table.ts >= 1.2)-1.2,table.vs(table.ts >= 1.2),LineWidth=2,Color='k');hold on;
plot(table.ts,table.vs,LineWidth=2,Color='blue');hold on;
% plot(table.ts-1.78,table.vs-0.3,LineWidth=2,Color='k');hold on;
plot(table.ts,u);
xlabel("Tempo [s]")
ylabel('Velocidade [m/s]')
%%ylim([0,1.7])
title("Tempo x Velocidade") 


%%
% Escolher uma faixa de interesse para traçar a reta de mínimos quadrados
% a partir do momento que começa a estabilizar a inclinação
% mas que não seja longa (10 a 20 elementos)

inicio = 85
final = 95
for i=inicio:final
    x(i-(inicio-1),1) = table.ts(i);
    y(i-(inicio-1),1) = table.vs(i);
end

A = [x x.^0];
th = inv(A'*A)*A'*y
%y = Ax + B /// th(1) = A e th(2) = B logo y = th(1)*x +th(2)

%%
% Gerando recorte vetores para plotar regiao de interesse
inicio = 1
for i=inicio:100
    y_ap(i-(inicio-1)) = (th(1,1)+0.015)*(table.ts(i)) + th(2,1) -0.08;
    tempo(i-(inicio-1)) = table.ts(i);
    velocidade(i-(inicio-1)) = table.vs(i);
    sinal(i-(inicio-1)) = u(i);
end


figure
plot(tempo, velocidade,'k',tempo, y_ap,'g', tempo, sinal, 'y')
hold on;
line([inicio_degrau inicio_degrau], ylim, 'Color', 'r', 'LineStyle', '--');
line(xlim, [0, 0], 'Color', 'r', 'LineStyle', '--');
grid on 

%%
% Calculo do ganho e tau

tau = -(-th(2,1))/(th(1,1)) + inicio_degrau
K =( th(1,1)*(inicio_degrau) + th(2,1))/ tau
%%
close all;
%Transladando o degrau para 0

figure
inicio = find(table.ts == inicio_degrau);
for i=inicio:(100+inicio)
    y_ap(i-(inicio-1)) = th(1,1)*(table.ts(i)-inicio_degrau) ;
    tempo(i-(inicio-1)) = table.ts(i)-inicio_degrau;
    velocidade(i-(inicio-1)) = table.vs(i);
    sinal(i-(inicio-1)) = u(i);
end

Acut = [tempo' (tempo.^0)'];
thcut = inv(Acut'*Acut)*Acut'*velocidade';

for i=inicio:(100+inicio)
    tangente(i-(inicio-1)) = (thcut(1,1)+0.015)*(table.ts(i)-inicio_degrau) +thcut(2,1)-0.05;
end


plot(tempo, velocidade,LineWidth=2,Color='blue',DisplayName="Resposta do Sistema");hold on;

plot(tempo,tangente,LineWidth=2,LineStyle="--",DisplayName="Reta Tangente");hold on;
plot(tempo,sinal,LineWidth=2,DisplayName="Degrau=0.2");hold on;
grid on 
legend


%% Estimando a inclinacao
% i_final = 300;
% i_inicial = 290;
% delta_vel = table.vs(i_final)-table.vs(i_inicial);
% delta_t = table.ts(i_final)-table.ts(i_inicial);
% inclinacao = delta_vel/delta_t;
% 
% plot(table.ts+1.2,table.ts*inclinacao,'red');
