clc;close all; clear all;

T = readtable('Dados_Parametros.csv');

% Plot de Tempo x Velocidade
plot(T.t,T.v);


%% Calculo Beta
clc; clear all; close all

% Obtencao da tabela
B_table = readtable('Dados_Beta.csv');


v_v0 = B_table.vs(B_table.ts > 8,:)./B_table.v0(B_table.ts > 8,:);
t_t0 = (B_table.ts(B_table.ts > 8,:) - 8)./ (B_table.ts(end) - 8);

% Plotando dados Beta

figure
plot(B_table.ts,B_table.vs);

figure
plot(t_t0,v_v0);

beta = [0.2 0.4 0.8 1 1.2 1.4];
t=0:0.05:1;
delta_V = zeros(length(t),length(beta));

for c=1:length(beta)
    for i=1:length(t)
        delta_V(i,c) = (1/beta(c)) * tan( (1-t(i)) * atan(beta(c)) );
    end
end

plot(t,delta_V,'LineWidth',2);

ylabel('v/v_0')
xlabel('t/t0')
hold on;

plot(t_t0,v_v0,'LineWidth',2,'LineStyle','--',Color=[0,0,0]);
legend('\beta=0.2','\beta=0.4','\beta=0.8','\beta=1','\beta=1.2','\beta=1.4','Dados Obtidos');