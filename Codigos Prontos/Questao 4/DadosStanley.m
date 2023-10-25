clc; close all; clear all;


% Variaveis gerais
table = readtable('Dados_Stanley.csv');


plot(table.ts,table.x)