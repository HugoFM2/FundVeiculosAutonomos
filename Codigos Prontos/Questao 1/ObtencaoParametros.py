# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2023/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################
import sys
sys.path.append("../../")
sys.path.append("../../coppelia/")
sys.path.append("../../coppeliasim_zmqremoteapi/")
import class_car as cp
import numpy as np
import cv2
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas 


########################################
# GLOBAIS
########################################
# parametros do carro
CAR = {
		'VELMAX'	: 5.0,				# m/s
		'ACCELMAX'	: 0.25, 			# m/s^2
		'STEERMAX'	: np.deg2rad(20.0),	# deg
	}


# mapa de plot
mapa = cv2.cvtColor(cv2.imread('../../coppelia/pista.png'), cv2.COLOR_RGB2BGR)

########################################
# cria comunicação com o carrinho
car = cp. CarCoppelia(mode='zmq')
# car.stopMission()
time.sleep(0.3)


car.sim.setBoolParam(car.sim.boolparam_realtime_simulation,True) # Definir em tempo real (Equivalente ao clicar no "Relogio")
print("Iniciando Missão")
car.startMission()
car.sim.setInt32Param(car.sim.intparam_speedmodifier,6) # Equivalente ao clicar no "Coelho" 6 vezes




####### CALCULA BETA
# FAZER REGRESSAO, MINIMOS QUADRADOS
## Inicializando valores
regVel = False
vs =  []
ts = []
pos_x = pos_y = []

print("Iniciando simulação do calculo de Beta")
while car.t < 600:
	
	# lê sensores
	car.step()

	# Obtem velocidade atual
	v,w = car.getVel()

	# Printa o tempo atual e velocidade Atual
	print(f't:{round(car.t,2)} / vel: {round(v,2)}')

	# Obtem posicao atual
	x,y = car.getPos()

	#Adiciona a velocidade e o tempo em suas listas
	vs.append(v)
	ts.append(round(car.t,2))

	# Seta a direcao do volante para 0 (approx 0.0326)
	car.setSteer(0.0364)

	if v == 0: break # Caso atinja a velocidade 0 parar o programa
		

	if car.t > 8: # Após 5s
		# Seta o torque do veiculo para 0 (Sem velocidade)
		car.setU(0)

	else:
		# Obtem velocidade inicial no momento em que reduz a velocidade a 0
		v0,w0 = car.getVel()

		# Seta o torque do veiculo para uma velocidade qualquer
		car.setU(50)


# Dados
# Steer 0.0364 / Setu 50 / t = 8s // Beta = 0.8


# Salva os dados em um formato csv
import pandas as pd
df = pd.DataFrame({'ts'    : ts,
					'vs'   : vs,
					'v0'   : v0*np.ones(len(ts)) })
df.to_csv('Dados_Beta.csv', index=True)
